# -*- coding: utf-8 -*-
""" Modbus RTU 指令生成器 v4.5 (点动偏移: 0x272B开窗/0x272C关窗) """

def modbus_crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc

SEP = "─" * 52

# 校验规则: regAddr -> (min, max, step, unit, unit_label)
VALIDATION_RULES = {
    0x2714: (250,  270,   2,  0.1, "V"),    # 过压阈值 25.0~27.0V, 步进0.2V
    0x2715: (210,  230,   2,  0.1, "V"),    # 欠压阈值 21.0~23.0V, 步进0.2V
    0x2716: (0,    2500, 50,  1,   "mA"),   # 过流阈值 0~2500mA, 步进50mA  ← 修改这里
    0x271E: (0,    400,  5,   1,   "ms"),   # 判定时间 0~400ms, 步进5ms
    0x3712: (10,  65535,  0,  0.1, ""),     # 减速比 1.0~6553.5, 单位0.1
    0x3713: (1,     100,  0,  1,   ""),     # 极对数 1~100, 不取整
    0x2726: (0,     200,  0,  0.1, "°"),    # 停止阈值 0~20.0°, 步进0.1°
    0x2711: (1,      60,  1,  1,   "r/min"),# 目标转速 1~60 r/min, 步进1 (类型2/3 超时估算基准)
    0x2717: (-1,     20,  1,  1,   "s"),    # 类型1 固定超时 -1~20s, 步进1 (-1=关闭; 0=受理瞬间即判定)
    0x2718: (-1,     20,  1,  1,   "s"),    # 类型2/3 余量 -1~20s, 步进1 (-1=关闭; 0=纯运动时间)
}

def apply_validation(regAddr, raw_value):
    rule = VALIDATION_RULES.get(regAddr)
    if rule is None:
        return raw_value, False
    vmin, vmax, step, unit, label = rule
    result = raw_value
    modified = False
    if result < vmin:
        result = vmin; modified = True
    if result > vmax:
        result = vmax; modified = True
    if step > 0:
        lower = (result // step) * step
        upper = lower + step
        if (result - lower) >= (upper - result):
            rounded = upper
        else:
            rounded = lower
        if rounded != result:
            result = rounded; modified = True
    return result, modified

def fmt_constraint(regAddr):
    rule = VALIDATION_RULES.get(regAddr)
    if rule is None:
        return ""
    vmin, vmax, step, unit, label = rule
    parts = []
    if vmin != 0 or regAddr in (0x2716, 0x271E):
        parts.append(f"最小 {vmin*unit:.1f}{label}" if unit < 1 else f"最小 {vmin}{label}")
    parts.append(f"最大 {vmax*unit:.1f}{label}" if unit < 1 else f"最大 {vmax}{label}")
    if step > 0:
        s = step * unit
        parts.append(f"步进 {s:.1f}{label}" if unit < 1 else f"步进 {s}{label}")
    return "  [" + ", ".join(parts) + "]"

CONFIG_REGS = [
    (0x2710, "设备地址",           None,         "1~247"),
    (0x2714, "过压阈值",           None,         "0.1V"),
    (0x2715, "欠压阈值",           None,         "0.1V"),
    (0x2716, "过流阈值",           None,         "mA"),
    (0x271C, "关窗极限角度",       None,         "0.1°"),
    (0x271D, "开窗极限角度",       None,         "0.1°"),
    (0x271E, "过流判定时间",       None,         "ms"),
    (0x2729, "关窗过流校准上限",   None,         "0.1°"),
    (0x272A, "关窗过流校准下限",   None,         "0.1°"),
]

DEV_REGS = [
    (0x3714, "霍尔脉冲累计", None,         ""),
    (0x3710, "霍尔方向",       ["正常", "反转"], ""),
    (0x3711, "电机方向",       ["正常", "反转"], ""),
    (0x3712, "减速比",         None,         "0.1"),
    (0x3713, "电机极对数",     None,         ""),
    (0x2726, "停止阈值",       None,         "0.1°"),
]

REALTIME_REGS = [
    (0x2730, "实时转速",   "r/min"),
    (0x2731, "实时角度",   "0.1°"),
    (0x2732, "实时电压",   "0.1V"),
    (0x2733, "实时电流",   "mA"),
    (0x2737, "实时方向",   ""),
]

FAULT_BITS = [
    (0x01, "过压"), (0x02, "开窗过流(正转过流)"), (0x04, "关窗过流(反转过流)"), (0x40, "欠压"),
]

DEV_PASSWORD = "5858"
_dev_authenticated = False

def check_dev_password():
    global _dev_authenticated
    if _dev_authenticated:
        return True
    pwd = input("  密码: ").strip()
    if pwd != DEV_PASSWORD:
        print("  密码错误!")
        return False
    _dev_authenticated = True
    return True

def ask_node():
    s = input("设备地址 [1]: ").strip()
    return int(s) if s else 1

def ask_value(prompt="值"):
    s = input(f"{prompt}: ").strip()
    if s.startswith("0x") or s.startswith("0X"):
        return int(s, 16)
    has_hex_letter = any(c in 'abcdefABCDEF' for c in s)
    if has_hex_letter:
        if all(c in '0123456789abcdefABCDEF' for c in s):
            return int(s, 16)
        return None
    try:
        return int(s)
    except:
        return None

def print_cmd(req_data, node, note="", skip_echo=False):
    print(f"\n  {SEP}")
    crc = modbus_crc16(req_data)
    req = ' '.join(f'{b:02X}' for b in req_data)
    print(f"  发送: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
    if not skip_echo:
        func = req_data[1]
        if func == 0x03:
            dlen = req_data[5] * 2
            xx = ' '.join(['XX'] * dlen)
            print(f"  回令: {node:02X} 03 {dlen:02X} {xx} crc_h crc_l")
        elif func == 0x10:
            # 0x10 success: addr + 0x10 + start(2) + count(2) + CRC
            resp_ok = [req_data[0], 0x10, req_data[2], req_data[3], req_data[4], req_data[5]]
            rcrc = modbus_crc16(resp_ok)
            rq = ' '.join(f'{b:02X}' for b in resp_ok)
            print(f"  成功回令: {rq} {rcrc&0xFF:02X} {rcrc>>8&0xFF:02X}")
            # 0x10 failure: addr + 0x90 + 0x04 + CRC (异常04=未解锁或转动中)
            resp_fail = [req_data[0], 0x90, 0x04]
            fcrc = modbus_crc16(resp_fail)
            fq = ' '.join(f'{b:02X}' for b in resp_fail)
            print(f"  失败回令: {fq} {fcrc&0xFF:02X} {fcrc>>8&0xFF:02X}  (未解锁或转动中)")
        else:
            print(f"  回令: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}  (echo)")
    if note: print(f"  {note}")

# ===== 1. 读实时数据 =====
def menu_read_realtime():
    print("\n====== 读实时数据 =====")
    for i, (addr, name, unit) in enumerate(REALTIME_REGS):
        if addr == 0x2731:
            print(f"  {i+1}. {name}（0x{addr:04X}）[单位：{unit}] ❌ 已禁用")
        else:
            u = f" [单位：{unit}]" if unit else ""
            print(f"  {i+1}. {name}（0x{addr:04X}）{u}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(REALTIME_REGS): raise
        addr, name, unit = REALTIME_REGS[idx]
    except: print("无效"); return

    # 禁用实时角度 (0x2731)
    if addr == 0x2731:
        print("\n  ⚠ 此功能已禁用！")
        print("  📌 请到菜单 [9. 关窗基准点] → [4. 读绝对角度(RAM)] 查看角度")
        print("     或 [5. 解读绝对角度RAM] 手动解析回令数据")
        return

    node = ask_node()
    u = f" ({unit})" if unit else ""
    req_data = [node, 0x03, (addr>>8)&0xFF, addr&0xFF, 0x00, 0x01]

    print(f"\n  ▎{name}{u}")
    if addr == 0x2737:
        print(f"  ▎值: 0=停止  1=开窗(逆时针)  2=关窗(顺时针)")

    print_cmd(req_data, node)

# ===== 2. 控制 =====
def menu_control():
    print("\n====== 电机控制 =====")
    print("  1. 控制开启")
    print("  2. 控制关闭（转动时不会停）")
    print("  3. 急停")
    print("  4. 开窗（逆时针）  ⚠需先解锁(选1)")
    print("  5. 关窗（顺时针）  ⚠需先解锁(选1)")
    print("  6. 重启复位")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0' or c == '': return
    cm = {'1':0x0001,'2':0x0002,'3':0x0004,'4':0x0010,'5':0x0020,'6':0x0008}
    val = cm.get(c)
    if val is None: print("无效"); return

    if val & 0x08: print("\n  ⚠ 设备将复位!")
    node = ask_node()
    req_data = [node, 0x06, 0x27, 0x20, (val>>8)&0xFF, val&0xFF]
    print_cmd(req_data, node)

    if val == 0x0010 or val == 0x0020:
        t1 = TIMEOUT_CFG['t1']
        print("\n  ⏱ 超时检测提示（类型1 固定时间）")
        print("    依据 0x2717(T1): 指令被受理后 T1 秒仍未停转 → 报“运动超时”(0x2740 bit7)")
        if t1 is None:
            print("    T1 未知: 可在[时间配置]里读/写，或直接读 0x2717")
        elif t1 < 0:
            print("    当前 T1 = -1 → 关闭类型1 超时检测")
        elif t1 == 0:
            print("    当前 T1 = 0 → 受理瞬间即判定超时")
        else:
            print(f"    当前 T1 = {t1} 秒 → 受理后约 {t1} 秒判定超时")

# ===== 3. 读配置寄存器 =====
def menu_read_config():
    print("\n====== 读配置寄存器 =====")
    for i, (addr, name, opts, unit) in enumerate(CONFIG_REGS):
        u = f" [单位：{unit}]" if unit and unit != "默认1183" and unit != "默认3" else ""
        c = fmt_constraint(addr)
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}{c}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(CONFIG_REGS): raise
        addr, name, opts, unit = CONFIG_REGS[idx]
    except: print("无效"); return

    node = ask_node()
    req_data = [node, 0x03, (addr>>8)&0xFF, addr&0xFF, 0x00, 0x01]

    print(f"\n  ▎{name}（0x{addr:04X}）")
    if opts is not None:
        print(f"  可选值:")
        for vi, opt_name in enumerate(opts):
            print(f"    {vi} = {opt_name}")

    print_cmd(req_data, node)

# ===== 4. 写配置寄存器 =====
def menu_write_config():
    print("\n====== 写配置寄存器 =====")
    for i, (addr, name, opts, unit) in enumerate(CONFIG_REGS):
        u = f" [单位：{unit}]" if unit and unit != "默认1183" and unit != "默认3" else ""
        c = fmt_constraint(addr)
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}{c}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(CONFIG_REGS): raise
        addr, name, opts, unit = CONFIG_REGS[idx]
    except: print("无效"); return

    print(f"\n  ▎{name}（0x{addr:04X}）")
    rule = VALIDATION_RULES.get(addr)

    val = None
    if opts is not None:
        for j, opt_name in enumerate(opts):
            print(f"  {j+1}. {opt_name}")
        print("  0. 返回")
        c2 = input("选择: ").strip()
        if c2 == '0': return
        try:
            vi = int(c2)-1
            if vi < 0 or vi >= len(opts): raise
            val = vi
        except: print("无效"); return
    else:
        us = f" ({unit})" if unit else ""
        if rule:
            vmin, vmax, step, r_unit, r_label = rule
            min_disp = f"{vmin * r_unit:.1f}" if r_unit < 1 else str(vmin)
            max_disp = f"{vmax * r_unit:.1f}" if r_unit < 1 else str(vmax)
            s = step * r_unit
            step_disp = f"{s:.1f}" if r_unit < 1 else str(s)
            print(f"  范围: {min_disp}~{max_disp}{r_label}, 步进{step_disp}{r_label}")
            prompt = f"值（单位：{unit}）" if unit and unit != "1~247" else "值"
        else:
            print(f"  范围:{us}")
            prompt = f"值（单位：{unit}）" if unit and unit != "1~247" else "值"
        val = ask_value(prompt)
        if val is None: print("无效"); return

    node = ask_node()
    req_data = [node, 0x06, (addr>>8)&0xFF, addr&0xFF, (val>>8)&0xFF, val&0xFF]

    if rule:
        result, modified = apply_validation(addr, val)
        if modified:
            vmin, vmax, step, r_unit, r_label = rule
            disp_raw = f"{val * r_unit:.1f}{r_label}" if r_unit < 1 else f"{val}{r_label}"
            disp_res = f"{result * r_unit:.1f}{r_label}" if r_unit < 1 else f"{result}{r_label}"
            note = f"MCU校验后: {disp_raw} → {disp_res}"
        else:
            note = ""
        print_cmd(req_data, node, note, skip_echo=True)
        if modified:
            echo_data = [node, 0x06, (addr>>8)&0xFF, addr&0xFF, (result>>8)&0xFF, result&0xFF]
            crc = modbus_crc16(echo_data)
            echo = ' '.join(f'{b:02X}' for b in echo_data)
            print(f"  预期回令: {echo} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
        else:
            crc = modbus_crc16(req_data)
            req = ' '.join(f'{b:02X}' for b in req_data)
            print(f"  回令: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}  (echo)")
    else:
        print_cmd(req_data, node)

# ===== 时间配置（超时故障检测）=====
# 本工具只生成指令、不回读响应，因此 TIMEOUT_CFG 记录的是"用户在本工具里写入的值"，
# 仅用于后续估算展示，不代表设备实际值（实际值以读 0x2711/0x2717/0x2718 的回令为准）。
TIMEOUT_CFG = {"rpm": None, "t1": None, "margin": None}

TIMEOUT_REGS = [
    (0x2711, "目标转速",    "r/min", "类型2/3 估算基准(1~60)", "rpm"),
    (0x2717, "T1 固定超时", "s",     "类型1 开窗/关窗(-1关/0即判)", "t1"),
    (0x2718, "余量",        "s",     "类型2/3 附加余量(-1关/0纯运动)", "margin"),
]

def timeout_cfg_disp():
    def f(v, u):
        return f"{v}{u}" if v is not None else "未知"
    return (f"转速(0x2711)={f(TIMEOUT_CFG['rpm'], ' r/min')}, "
            f"T1(0x2717)={f(TIMEOUT_CFG['t1'], ' s')}, "
            f"余量(0x2718)={f(TIMEOUT_CFG['margin'], ' s')}")

# --- 类型2/3 配套指令贴心提示（客户可直接复制发送，均含 CRC） ---
def timeout_hint_read_angle(node):
    """【获取当前绝对角度】—— 类型2/3 估算的第一步"""
    print("  【获取当前绝对角度】")
    print_cmd([node, 0x03, 0x27, 0x21, 0x00, 0x02], node,
              note="读 0x2721+0x2722 = 当前绝对角度 (int32, 0.1°)；"
                   "收到回令后把它整帧粘到下面“当前角度”输入框即可自动解析")

def timeout_hint_read_ref(node):
    """【获取关窗基准点角度】—— 类型3 回基准点的目标角度"""
    print("  【获取关窗基准点角度】")
    print_cmd([node, 0x03, 0x27, 0x1C, 0x00, 0x01], node,
              note="读 0x271C 关窗基准点角度 (int16, 0.1°)；作为回基准点的目标角度")

def timeout_hint_goto(node, tgt_x10):
    """【转到目标角度】的指令 —— 这条就是被超时检测的指令"""
    val = max(-2147483648, min(2147483647, int(tgt_x10)))
    lo = val & 0xFFFF
    hi = (val >> 16) & 0xFFFF
    print(f"  【转到目标角度】{val} (0.1°) = {val * 0.1:.1f}°  ← 本条即被超时检测的指令")
    print_cmd([node, 0x10, 0x27, 0x27, 0x00, 0x02, 0x04,
               (lo >> 8) & 0xFF, lo & 0xFF, (hi >> 8) & 0xFF, hi & 0xFF], node,
              note="0x10 写 0x2727+0x2728 (int32 小端: 低16位在前)")

def timeout_calc_type23(cur_x10, tgt_x10):
    """类型2/3: 按 0x2711(输出轴 r/min) + 0x2718(余量) 估算超时时间"""
    rpm = TIMEOUT_CFG.get('rpm')
    if rpm is None:
        rpm = ask_value("目标转速 0x2711 (r/min)")
    if rpm is None:
        print("    未提供转速 → 跳过估算")
        return
    if rpm <= 0:
        print("    0x2711 <= 0 → 无法推算(校验下限为1) → 不启用超时检测")
        return
    margin = TIMEOUT_CFG.get('margin')
    if margin is None:
        m = ask_value("余量 0x2718 (s) [默认0]")
        margin = 0 if m is None else m
    if margin < 0:
        print("    0x2718 = -1 → 关闭类型2/3 超时检测")
        return
    dist = abs(tgt_x10 - cur_x10)
    t_ms = dist * 1000 // (60 * rpm) + margin * 1000
    print(f"    距离 = |{tgt_x10} - {cur_x10}| = {dist} (0.1°) = {dist * 0.1:.1f}°")
    if margin == 0:
        print(f"    估算 = {dist * 0.1:.1f}° ÷ (6 × {rpm}°/s) = 纯运动时间 ≈ {t_ms / 1000:.1f} 秒后判定超时")
    else:
        print(f"    估算 = {dist * 0.1:.1f}° ÷ (6 × {rpm}°/s) + 余量 {margin}s ≈ {t_ms / 1000:.1f} 秒后判定超时")

def menu_timeout_config():
    while True:
        print("\n====== 时间配置（超时故障检测）======")
        print("  类型1(开窗/关窗): T1(0x2717) 固定时间;  类型2/3(到目标/回基准): 转速(0x2711) 估算 + 余量(0x2718)")
        print("  0x2717=-1 关闭类型1 (0=受理瞬间即判定);  0x2718=-1 关闭类型2/3 (0=纯运动时间)")
        for i, (addr, name, unit, desc, _key) in enumerate(TIMEOUT_REGS):
            u = f" [{unit}]" if unit else ""
            print(f"  {i*2+1}. 读 {name}（0x{addr:04X}）{u}{fmt_constraint(addr)}  — {desc}")
            print(f"  {i*2+2}. 写 {name}（0x{addr:04X}）{u}")
        print(f"  本工具已记录: {timeout_cfg_disp()}")
        print("  0. 返回")
        c = input("选择: ").strip()
        if c == '0' or c == '':
            return
        try:
            ci = int(c)
        except:
            print("无效"); continue
        if ci < 1 or ci > len(TIMEOUT_REGS) * 2:
            print("无效"); continue
        idx = (ci - 1) // 2
        is_write = ((ci - 1) % 2) == 1
        addr, name, unit, desc, key = TIMEOUT_REGS[idx]
        node = ask_node()
        if is_write:
            val = ask_value(f"值（单位：{unit}）" if unit else "值")
            if val is None:
                print("无效"); continue
            val, _fixed = apply_validation(addr, val)
            if _fixed:
                print(f"  ⚠ 值已按 MCU 校验修正为 {val}")
            req_data = [node, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, (val >> 8) & 0xFF, val & 0xFF]
            print_cmd(req_data, node)
            TIMEOUT_CFG[key] = val
        else:
            req_data = [node, 0x03, (addr >> 8) & 0xFF, addr & 0xFF, 0x00, 0x01]
            print(f"\n  ▎{name}（0x{addr:04X}）")
            print_cmd(req_data, node)
        input("\n按 Enter 返回...")

# ===== 5. 查看故障 =====
def menu_read_fault():
    node = ask_node()
    req_data = [node, 0x03, 0x27, 0x40, 0x00, 0x01]

    print(f"\n  ▎故障状态（0x2740）")
    print(f"  ▎bit0=过压  bit1=开窗过流(正转)  bit2=关窗过流(反转)  bit6=欠压  bit7=运动超时")
    print(f"  回令解析示例:")
    for lb, v in [("无故障", 0), ("过压", 0x0001), ("开窗过流", 0x0002),
                  ("关窗过流", 0x0004), ("欠压", 0x0040), ("运动超时", 0x0080)]:
        rd = [node, 0x03, 0x02, (v>>8)&0xFF, v&0xFF]
        rcr = modbus_crc16(rd)
        rq = ' '.join(f'{b:02X}' for b in rd)
        print(f"    {lb}: {rq} {rcr&0xFF:02X} {rcr>>8&0xFF:02X}")

    print_cmd(req_data, node)

# ===== 6. 清除故障 =====
def menu_clear_fault():
    print("\n====== 清除故障（0x2740）======")
    print("  ⚠ 清除开窗/关窗过流任一故障时，固件会同时清除两个方向的过流锁")
    for i, (bit, name) in enumerate(FAULT_BITS):
        print(f"  {i+1}. {name}")
    print(f"  {len(FAULT_BITS)+1}. 全部清除(写0)")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        ci = int(c)
        if ci == len(FAULT_BITS)+1:
            val = 0x0000
        elif 1 <= ci <= len(FAULT_BITS):
            val = FAULT_BITS[ci-1][0]
        else:
            print("无效"); return
    except: print("无效"); return

    node = ask_node()
    req_data = [node, 0x06, 0x27, 0x40, (val>>8)&0xFF, val&0xFF]
    print_cmd(req_data, node)

# ===== 7. 心跳包 =====
def menu_heartbeat():
    """读取心跳包: 读 0x271F"""
    print("\n====== 心跳包 =====")
    print("  地址 0x00 = 广播(全呼), 总线上所有从机均回复")
    print("  地址 1~247 = 只查询指定从机")
    print("  ⚠ 广播模式下多个从机同时回复，RS-485 总线会冲突")
    node = ask_node()
    if node == 0:
        print("\n  ▎广播心跳包 — 所有从机将同时回复（可能冲突）")
    else:
        print(f"\n  ▎查询地址 {node} — 仅该从机回复")
    req_data = [node, 0x03, 0x27, 0x1F, 0x00, 0x01]
    print_cmd(req_data, node)
    print("  ▎回复值 = 从机设备地址")

# ===== 8. 关窗基准点 (主菜单, 无密码) =====
def menu_window_zero():
    """主菜单关窗基准点 - 含回基准点和转动到目标角度"""
    while True:
        print("\n====== 关窗基准点 =====")
        print("  1. 设关窗基准点并保存")
        print("  2. 回到关窗基准点")
        print("  3. 转动到目标角度")
        print("  4. 读绝对角度(RAM)")
        print("  5. 解读绝对角度RAM")
        print("  6. 开窗方向点动")
        print("  7. 关窗方向点动")
        print("  0. 返回")
        c = input("选择: ").strip()
        if c == '0':
            return
        elif c == '1':
            print("\n====== 设关窗基准点并保存 =====")
            print("  将当前位置设为关窗基准点并写入Flash")
            print("  基准点角度值 = 0x271C(关窗极限角度) 的值")
            node = ask_node()
            req_data = [node, 0x06, 0x27, 0x25, 0x00, 0x00]
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        elif c == '2':
            print("\n====== 回到关窗基准点 =====")
            print("  电机自动转动到关窗基准点")
            print("  ⚠ 需先在[控制]中'控制开启'解锁，且电机停转")
            node = ask_node()
            req_data = [node, 0x06, 0x27, 0x25, 0x00, 0x01]
            print_cmd(req_data, node)
            print("\n  ⏱ 超时检测提示（类型3 回基准点 · 按 0x2711 转速估算）")
            print("    判定 = 上方【回到关窗基准点】指令被受理后，超过推算时间仍未到位 → 报“运动超时”(0x2740 bit7)")
            timeout_hint_read_angle(node)
            timeout_hint_read_ref(node)
            cur = ask_value("当前角度 (0.1°, 回车跳过估算)")
            if cur is not None:
                ref = ask_value("基准点角度 0x271C (0.1°)")
                if ref is not None:
                    timeout_calc_type23(cur, ref)
            input("\n按 Enter 返回...")
        elif c == '3':
            print("\n====== 转动到目标角度 =====")
            print("  单位 0.1°（如 880 = 88.0°，-15 = -1.5°，36000 = 3600.0°）")
            print("  正 = 开窗方向, 负 = 关窗方向")
            print("  int32 范围: -2147483648 ~ +2147483647（±2.1 亿度，不会回绕）")
            print("  ⚠ 需先在[控制]中'控制开启'解锁，且电机停转")
            print("  （未解锁或转动中发指令，返回异常04，不覆盖寄存器）")
            val = ask_value("目标角度（0.1°单位）")
            if val is None:
                print("无效"); input("\n按 Enter 返回..."); continue
            if val > 2147483647: val = 2147483647
            if val < -2147483648: val = -2147483648
            lo = val & 0xFFFF
            hi = (val >> 16) & 0xFFFF
            node = ask_node()
            print(f"\n  ▎目标角度: {val} (0.1°) = {val*0.1:.1f}°")
            req_data = [node, 0x10, 0x27, 0x27, 0x00, 0x02, 0x04,
                        (lo>>8)&0xFF, lo&0xFF, (hi>>8)&0xFF, hi&0xFF]
            print_cmd(req_data, node)
            print("\n  ⏱ 超时检测提示（类型2 到目标角度 · 按 0x2711 转速估算）")
            print(f"    判定 = 上方【目标角度】指令(目标 {val})被受理后，超过推算时间仍未到位 → 报“运动超时”(0x2740 bit7)")
            timeout_hint_read_angle(node)
            cur = ask_value("当前角度 (0.1°, 回车跳过估算)")
            if cur is not None:
                timeout_calc_type23(cur, val)
            input("\n按 Enter 返回...")
        elif c == '4':
            node = ask_node()
            req_data = [node, 0x03, 0x27, 0x21, 0x00, 0x02]
            print("\n  读绝对角度(RAM) (int32, 0.1°)")
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        elif c == '5':
            menu_parse_abs_angle()
        elif c == '6':
            print("\n====== 开窗方向点动 =====")
            print("  电机从当前位置往开窗方向(正转)移动指定偏移量")
            print("  单位 0.1°（如 10 = 1.0°, 50 = 5.0°, 300 = 30.0°）")
            print("  uint16 范围: 0~65535 (最大 6553.5°)")
            print("  ⚠ 需先在[控制]中'控制开启'解锁，且电机停转")
            val = ask_value("偏移量 (0.1°)")
            if val is None or val < 0:
                print("无效"); input("\n按 Enter 返回..."); continue
            if val > 65535: val = 65535
            node = ask_node()
            print(f"\n  ▎开窗点动: {val} (0.1°) = {val*0.1:.1f}°")
            req_data = [node, 0x06, 0x27, 0x2B, (val>>8)&0xFF, val&0xFF]
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        elif c == '7':
            print("\n====== 关窗方向点动 =====")
            print("  电机从当前位置往关窗方向(反转)移动指定偏移量")
            print("  单位 0.1°（如 10 = 1.0°, 50 = 5.0°, 300 = 30.0°）")
            print("  uint16 范围: 0~65535 (最大 6553.5°)")
            print("  ⚠ 需先在[控制]中'控制开启'解锁，且电机停转")
            val = ask_value("偏移量 (0.1°)")
            if val is None or val < 0:
                print("无效"); input("\n按 Enter 返回..."); continue
            if val > 65535: val = 65535
            node = ask_node()
            print(f"\n  ▎关窗点动: {val} (0.1°) = {val*0.1:.1f}°")
            req_data = [node, 0x06, 0x27, 0x2C, (val>>8)&0xFF, val&0xFF]
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        else:
            print("无效")

def parse_abs_angle_reply(s):
    """解析 0x03 读 0x2721+0x2722 的回令帧 -> (角度0.1°整数值, 错误信息)
       成功: (val, None)；失败: (None, 原因)"""
    parts = s.split()
    if len(parts) < 7:
        return None, f"需要至少7字节，当前 {len(parts)} 字节"
    raw = [int(x, 16) for x in parts]
    # Modbus int32: 低寄存器(0x2721)在前，每寄存器大端
    # 字节序: [reg0_H, reg0_L, reg1_H, reg1_L]
    low16  = (raw[3] << 8) | raw[4]
    high16 = (raw[5] << 8) | raw[6]
    val = (high16 << 16) | low16
    if val > 0x7FFFFFFF:
        val -= 0x100000000
    return val, None

def ask_angle_value(prompt="输入当前角度"):
    """二选一输入角度: 直接输角度(0.1°) 或 粘贴设备回令帧(自动解析)"""
    print(f"  {prompt}")
    print("    方式1: 直接输入角度 (0.1°，如 880 = 88.0°，-15 = -1.5°)")
    print("    方式2: 粘贴设备回令帧 (如 01 03 04 FF F8 FF FF 4A 66)，自动解析")
    s = input("  请输入: ").strip()
    if not s:
        return None
    if len(s.split()) > 1:
        try:
            val, err = parse_abs_angle_reply(s)
        except ValueError:
            print("  解析失败: 包含非法十六进制字符"); return None
        except Exception as e:
            print(f"  解析失败: {e}"); return None
        if err:
            print(f"  解析失败: {err}"); return None
        print(f"  → 已解析回令: {val} (0.1°) = {val*0.1:.1f}°")
        return val
    try:
        return int(s, 16) if s.lower().startswith("0x") else int(s)
    except ValueError:
        print("  无效: 既不是角度数值，也不是回令帧")
        return None

def menu_parse_abs_angle():
    """解读绝对角度RAM - 解析从机返回的4字节数据，循环直到用户选择退出"""
    print("\n====== 解读绝对角度RAM =====")
    print("  请粘贴从机返回的 Modbus 回令帧 (0x03 读 0x2721+0x2722)")
    print("  示例: 01 03 04 FF F8 FF FF 4A 66")
    print("  直接按 Enter = 返回上级菜单")

    while True:
        s = input("\n请输入: ").strip()

        if not s:
            print("  已返回")
            return

        try:
            val, err = parse_abs_angle_reply(s)
            if err:
                print(f"  格式错误: {err}")
                continue
            angle = val * 0.1
            print(f"  → 原始值: {val} (0.1°)")
            print(f"  → 实际角度: {angle:.1f}°")

        except ValueError:
            print("  解析失败: 包含非法十六进制字符")
        except Exception as e:
            print(f"  解析失败: {e}")

# ===== 开发者选项 - 超时时间计算 =====
def menu_timeout_calc_dev():
    print("\n====== 超时时间计算（类型2/3：到目标角度）======")
    print("  用途：先估算“转到目标角度”的耗时，据此配置 0x2711(转速)/0x2718(余量)，避免误报超时")
    node = ask_node()

    timeout_hint_read_angle(node)
    cur = ask_angle_value("输入当前角度（二选一）")
    if cur is None:
        print("无效"); input("\n按 Enter 返回..."); return
    tgt = ask_value("目标角度 (0.1°)")
    if tgt is None:
        print("无效"); input("\n按 Enter 返回..."); return
    rpm = ask_value("转速 0x2711 (r/min)")
    if rpm is None:
        print("无效"); input("\n按 Enter 返回..."); return

    # ---- 到达目标角度的指令 (0x10 写 0x2727+0x2728, int32 小端: lo 先) ----
    print("\n  ▎配套指令")
    timeout_hint_goto(node, tgt)

    # ---- 超时检测时间 (与固件 RunAngle_EstTimeoutMs 同公式) ----
    print("\n  ⏱ 超时检测时间（类型2/3 估算）")
    if rpm <= 0:
        print("    0x2711 <= 0 → 无法推算(校验下限为1) → 不启用超时检测")
        input("\n按 Enter 返回..."); return
    margin = TIMEOUT_CFG.get('margin')
    if margin is None:
        m = ask_value("余量 0x2718 (s) [默认0]")
        margin = 0 if m is None else m
    if margin < 0:
        print("    0x2718 = -1 → 关闭类型2/3 超时检测")
        input("\n按 Enter 返回..."); return
    dist = abs(tgt - cur)
    t_ms = dist * 1000 // (60 * rpm) + margin * 1000
    print(f"    距离 = |{tgt} - {cur}| = {dist} (0.1°) = {dist * 0.1:.1f}°")
    if margin == 0:
        print(f"    估算 = {dist * 0.1:.1f}° ÷ (6 × {rpm}°/s) = 纯运动时间 ≈ {t_ms / 1000:.1f} 秒后判定超时")
    else:
        print(f"    估算 = {dist * 0.1:.1f}° ÷ (6 × {rpm}°/s) + 余量 {margin}s ≈ {t_ms / 1000:.1f} 秒后判定超时")
    input("\n按 Enter 返回...")

# ===== 9. 开发者选项 (需密码) =====
def menu_dev_options():
    if not check_dev_password():
        return
    while True:
        print("\n====== 开发者选项 =====")
        print("  1. 读配置寄存器")
        print("  2. 写配置寄存器")
        print("  3. 计算霍尔脉冲")
        print("  4. 读霍尔脉冲")
        print("  5. 重置霍尔脉冲")
        print("  6. 计算实时角度")
        print("  7. 关窗基准点（高级）")
        print("  8. 关窗过流校准阈值")
        print("  9. 超时时间计算")
        print("  0. 返回")
        c = input("选择: ").strip()
        if c == '0':
            return
        elif c == '1':
            menu_read_dev_regs()
        elif c == '2':
            menu_write_dev_regs()
        elif c == '3':
            menu_calc_hall_pulse()
        elif c == '4':
            menu_read_hall_pulse()
        elif c == '5':
            menu_reset_hall_pulse()
        elif c == '6':
            menu_calc_realtime_angle()
        elif c == '7':
            menu_window_zero_advanced()
        elif c == '8':
            menu_calib_threshold()
        elif c == '9':
            menu_timeout_calc_dev()
        else:
            print("无效")

def menu_read_dev_regs():
    print("\n====== 开发者选项 - 读配置 =====")
    for i, (addr, name, opts, unit) in enumerate(DEV_REGS):
        u = f" [单位：{unit}]" if unit and unit != "默认1183" and unit != "默认3" else ""
        c = fmt_constraint(addr)
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}{c}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(DEV_REGS): raise
        addr, name, opts, unit = DEV_REGS[idx]
    except: print("无效"); return

    node = ask_node()
    nreg = 2 if addr == 0x3714 else 1
    req_data = [node, 0x03, (addr>>8)&0xFF, addr&0xFF, 0x00, nreg]

    print(f"\n  ▎{name}（0x{addr:04X}）")
    if addr == 0x3714:
        print(f"  ▎开窗(逆时针)→加  关窗(顺时针)→减  上电归零")
        print(f"  ▎公式: 角度(°) = 脉冲数 / 12 / 减速比 × 360°")
    if opts is not None:
        print(f"  可选值:")
        for vi, opt_name in enumerate(opts):
            print(f"    {vi} = {opt_name}")

    print_cmd(req_data, node)

def menu_write_dev_regs():
    print("\n====== 开发者选项 - 写配置 =====")
    for i, (addr, name, opts, unit) in enumerate(DEV_REGS):
        u = f" [单位：{unit}]" if unit and unit != "默认1183" and unit != "默认3" else ""
        c = fmt_constraint(addr)
        print(f"  {i+1}. {name}（0x{addr:04X}）{u}{c}")
    print("  0. 返回")
    c = input("选择: ").strip()
    if c == '0': return
    try:
        idx = int(c)-1
        if idx < 0 or idx >= len(DEV_REGS): raise
        addr, name, opts, unit = DEV_REGS[idx]
    except: print("无效"); return

    print(f"\n  ▎{name}（0x{addr:04X}）")
    rule = VALIDATION_RULES.get(addr)

    val = None
    if opts is not None:
        for j, opt_name in enumerate(opts):
            print(f"  {j+1}. {opt_name}")
        print("  0. 返回")
        c2 = input("选择: ").strip()
        if c2 == '0': return
        try:
            vi = int(c2)-1
            if vi < 0 or vi >= len(opts): raise
            val = vi
        except: print("无效"); return
    else:
        us = f" ({unit})" if unit else ""
        if rule:
            vmin, vmax, step, r_unit, r_label = rule
            min_disp = f"{vmin * r_unit:.1f}" if r_unit < 1 else str(vmin)
            max_disp = f"{vmax * r_unit:.1f}" if r_unit < 1 else str(vmax)
            if step > 0:
                s = step * r_unit
                step_disp = f"{s:.1f}" if r_unit < 1 else str(s)
                print(f"  范围: {min_disp}~{max_disp}{r_label}, 步进{step_disp}{r_label}")
            else:
                print(f"  范围: {min_disp}~{max_disp}{r_label}")
        else:
            print(f"  范围:{us}")
        prompt = f"值（单位：{unit}）" if unit and unit != "1~247" else "值"
        val = ask_value(prompt)
        if val is None: print("无效"); return

    node = ask_node()
    req_data = [node, 0x06, (addr>>8)&0xFF, addr&0xFF, (val>>8)&0xFF, val&0xFF]

    if rule:
        result, modified = apply_validation(addr, val)
        if modified:
            vmin, vmax, step, r_unit, r_label = rule
            disp_raw = f"{val * r_unit:.1f}{r_label}" if r_unit < 1 else f"{val}{r_label}"
            disp_res = f"{result * r_unit:.1f}{r_label}" if r_unit < 1 else f"{result}{r_label}"
            note = f"MCU校验后: {disp_raw} → {disp_res}"
        else:
            note = ""
        print_cmd(req_data, node, note, skip_echo=True)
        if modified:
            echo_data = [node, 0x06, (addr>>8)&0xFF, addr&0xFF, (result>>8)&0xFF, result&0xFF]
            crc = modbus_crc16(echo_data)
            echo = ' '.join(f'{b:02X}' for b in echo_data)
            print(f"  预期回令: {echo} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
        else:
            crc = modbus_crc16(req_data)
            req = ' '.join(f'{b:02X}' for b in req_data)
            print(f"  回令: {req} {crc&0xFF:02X} {crc>>8&0xFF:02X}  (echo)")
    else:
        print_cmd(req_data, node)

# ===== 关窗过流校准阈值 (开发者选项内) =====
def menu_calib_threshold():
    """关窗过流校准阈值 - 0x2729(上限) 0x272A(下限)"""
    while True:
        print("\n====== 关窗过流校准阈值 =====")
        print("  校准有效角度区间 [下限, 上限]")
        print("  默认: 上限=-1.0° 下限=-3.0°")
        print("  1. 读上限 (0x2729)")
        print("  2. 读下限 (0x272A)")
        print("  3. 设上限")
        print("  4. 设下限")
        print("  0. 返回")
        c = input("选择: ").strip()
        if c == '0':
            return
        elif c == '1':
            node = ask_node()
            req_data = [node, 0x03, 0x27, 0x29, 0x00, 0x01]
            print("\n  读关窗过流校准上限 (0x2729, int16, 0.1°)")
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        elif c == '2':
            node = ask_node()
            req_data = [node, 0x03, 0x27, 0x2A, 0x00, 0x01]
            print("\n  读关窗过流校准下限 (0x272A, int16, 0.1°)")
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        elif c == '3':
            print("\n====== 设关窗过流校准上限 =====")
            print("  单位 0.1°, int16 范围")
            print("  默认: -10 (即 -1.0°)")
            print("  仅当关窗过流时角度 ≤ 此值才执行校准")
            val = ask_value("上限 (0.1°单位)")
            if val is None:
                print("无效"); input("\n按 Enter 返回..."); continue
            if val > 32767: val = 32767
            if val < -32768: val = -32768
            node = ask_node()
            print(f"\n  ▎上限: {val} (0.1°) = {val*0.1:.1f}°")
            req_data = [node, 0x06, 0x27, 0x29, (val>>8)&0xFF, val&0xFF]
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        elif c == '4':
            print("\n====== 设关窗过流校准下限 =====")
            print("  单位 0.1°, int16 范围")
            print("  默认: -30 (即 -3.0°)")
            print("  仅当关窗过流时角度 ≥ 此值才执行校准")
            val = ask_value("下限 (0.1°单位)")
            if val is None:
                print("无效"); input("\n按 Enter 返回..."); continue
            if val > 32767: val = 32767
            if val < -32768: val = -32768
            node = ask_node()
            print(f"\n  ▎下限: {val} (0.1°) = {val*0.1:.1f}°")
            req_data = [node, 0x06, 0x27, 0x2A, (val>>8)&0xFF, val&0xFF]
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        else:
            print("无效")

def menu_read_hall_pulse():
    print("\n====== 读霍尔脉冲 =====")
    node = ask_node()
    req_data = [node, 0x03, 0x37, 0x14, 0x00, 0x02]
    print_cmd(req_data, node)

def menu_reset_hall_pulse():
    print("\n====== 重置霍尔脉冲 =====")
    print("  ⚠ 将清零霍尔脉冲累计值")
    node = ask_node()
    req_data = [node, 0x06, 0x37, 0x14, 0x00, 0x00]
    print_cmd(req_data, node)

def menu_calc_realtime_angle():
    print("\n====== 计算实时角度 =====")
    print("  粘贴 Modbus 回令帧 (如: 01 03 02 03 93 F8 D9)")
    s = input("  回令: ").strip()
    try:
        parts = s.split()
        if len(parts) < 4:
            print("  格式错误: 至少需要4字节"); return
        raw = [int(x, 16) for x in parts]

        if raw[1] == 0x03:
            byte_count = raw[2]
            if len(raw) < 3 + byte_count:
                print(f"  数据不完整, 需要 {3+byte_count} 字节"); return
            data_bytes = raw[3:3+byte_count]
            val = (data_bytes[0] << 8) | data_bytes[1]
            if val > 32767:
                val = val - 65536
            angle = val * 0.1
            print(f"  实时角度: {angle:.1f}°")
        else:
            print("  不是 0x03 回令帧")
    except Exception as e:
        print(f"  解析失败: {e}")

def menu_calc_hall_pulse():
    print("\n====== 计算霍尔脉冲 → 角度 =====")
    print("  粘贴 Modbus 回令帧 (如: 01 03 04 0D C9 00 00 28 A1)")
    s = input("  回令: ").strip()
    try:
        parts = s.split()
        if len(parts) < 4:
            print("  格式错误: 至少需要4字节"); return
        raw = [int(x, 16) for x in parts]

        if raw[1] == 0x03:
            byte_count = raw[2]
            if len(raw) < 3 + byte_count:
                print(f"  数据不完整, 需要 {3+byte_count} 字节"); return
            data_bytes = raw[3:3+byte_count]
        else:
            data_bytes = raw

        pulse_count = 0
        for i, b in enumerate(data_bytes):
            reg_idx = i // 2
            byte_in_reg = 1 - (i % 2)
            shift = reg_idx * 16 + byte_in_reg * 8
            pulse_count |= (b << shift)

        if pulse_count > 0x7FFFFFFF:
            pulse_count = pulse_count - 0x100000000

        print(f"  脉冲数: {pulse_count}")
    except Exception as e:
        print(f"  解析失败: {e}"); return

    rs = input("  减速比 (单位0.1, 默认11830): ").strip()
    try:
        ratio_raw = int(rs) if rs else 11830
    except:
        print("  无效"); return
    ratio = ratio_raw / 10.0

    ps = input("  一圈几脉冲 [12]: ").strip()
    try:
        pulses_per_rev = int(ps) if ps else 12
    except:
        print("  无效"); return
    if pulses_per_rev <= 0:
        print("  脉冲数必须 > 0"); return

    angle = pulse_count / pulses_per_rev / ratio * 360.0
    print(f"\n  {'='*42}")
    print(f"  脉冲数: {pulse_count}")
    print(f"  减速比: {ratio:.1f}:1")
    print(f"  一圈脉冲: {pulses_per_rev}")
    print(f"  转动角度: {angle:.2f}°")
    print(f"  {'='*42}")

# ===== 关窗基准点高级 (开发者选项内) =====
def menu_window_zero_advanced():
    """关窗基准点高级 - 放在开发者选项内"""
    while True:
        print("\n====== 关窗基准点（高级）======")
        print("  1. 读关窗基准点(Flash)")
        print("  2. 保存基准点到Flash")
        print("  3. 读停止阈值")
        print("  4. 设置停止阈值")
        print("  0. 返回")
        c = input("选择: ").strip()
        if c == '0':
            return
        elif c == '1':
            node = ask_node()
            req_data = [node, 0x03, 0x27, 0x23, 0x00, 0x02]
            print("\n  读关窗基准点(Flash) (int32, 0.1°)")
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        elif c == '2':
            print("\n====== 保存基准点到Flash =====")
            print("  将当前RAM偏移值固化到Flash，掉电后自动恢复")
            node = ask_node()
            req_data = [node, 0x06, 0x27, 0x25, 0x00, 0x02]
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        elif c == '3':
            node = ask_node()
            print("\n  停止阈值 (0x2726, 0.1°)")
            req_data = [node, 0x03, 0x27, 0x26, 0x00, 0x01]
            print_cmd(req_data, node)
            input("\n按 Enter 返回...")
        elif c == '4':
            print("\n====== 设置停止阈值 =====")
            print("  回基准点/回目标的停止判定阈值")
            print("  范围: 0.0°~20.0° (值 0~200, 单位 0.1°)")
            print("  默认: 0.1° (值=1)")
            val = ask_value("阈值 (0.1°)")
            if val is None:
                print("无效"); input("\n按 Enter 返回..."); continue
            node = ask_node()
            req_data = [node, 0x06, 0x27, 0x26, (val>>8)&0xFF, val&0xFF]
            result, modified = apply_validation(0x2726, val)
            if modified:
                note = f"MCU校验后: {val*0.1:.1f}° → {result*0.1:.1f}°"
            else:
                note = ""
            print_cmd(req_data, node, note, skip_echo=True)
            echo_data = [node, 0x06, 0x27, 0x26, (result>>8)&0xFF, result&0xFF]
            crc = modbus_crc16(echo_data)
            echo = ' '.join(f'{b:02X}' for b in echo_data)
            print(f"  回令: {echo} {crc&0xFF:02X} {crc>>8&0xFF:02X}")
            input("\n按 Enter 返回...")
        else:
            print("无效"); input("\n按 Enter 返回...")

# ===== 主菜单 =====
MENU = [
    ("读实时数据",     menu_read_realtime),
    ("控制",           menu_control),
    ("读配置寄存器",   menu_read_config),
    ("写配置寄存器",   menu_write_config),
    ("时间配置",       menu_timeout_config),
    ("查看故障",       menu_read_fault),
    ("清除故障",       menu_clear_fault),
    ("心跳包",         menu_heartbeat),
    ("关窗基准点",     menu_window_zero),
]

def main():
    dev_key = len(MENU) + 1
    while True:
        print("\n" + "=" * 42)
        print("  Modbus RTU 指令生成器 v4.5")
        print("=" * 42)
        for i, (name, _) in enumerate(MENU):
            print(f"  {i+1}. {name}")
        print(f"  {dev_key}. 开发者选项")
        print("  0. 退出")
        print("=" * 42)
        c = input(f"选择 [0-{dev_key}]: ").strip()
        if c == '0':
            print("退出")
            break
        if c == str(dev_key):
            menu_dev_options()
        else:
            try:
                idx = int(c) - 1
                if 0 <= idx < len(MENU):
                    MENU[idx][1]()
                else:
                    print("无效")
            except:
                print("无效")
        input("\n按 Enter 返回...")

if __name__ == '__main__':
    main()