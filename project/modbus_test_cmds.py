# -*- coding: utf-8 -*-
"""
Modbus RTU 指令生成器
用法:
  1. 修改下方 OPERATION / REG_ADDR / WRITE_VALUE 等参数
  2. 运行: py modbus_test_cmds.py
  3. 复制输出的十六进制指令帧（已含 CRC）
=============================================================================
"""

# =============================================================================
# ===== 用户配置 =====
# =============================================================================

# 设备地址 (1~247)
NODE_ID = 1

# 操作类型: "READ" / "WRITE" / "CLEAR_FAULT" / "CTRL_CMD"
OPERATION = "CTRL_CMD"

# 寄存器地址 (WRITE/READ 时有效)
REG_ADDR = 0x3710   # 默认: REG_MOTOR_HALL_DIR

# 写入值 (WRITE 时有效, 负数自动转为 uint16)
WRITE_VALUE = 1

# 控制命令 (CTRL_CMD 时有效)
CTRL_CMD_VALUE = 0x0008   # RESET

# 故障清除 (CLEAR_FAULT 时有效)
CLEAR_FAULT_BITS = 0x0000   # 0=清除全部故障

# =============================================================================
# ===== CRC 计算 =====
# =============================================================================

def modbus_crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def make_frame(data_bytes):
    crc = modbus_crc16(data_bytes)
    frame = list(data_bytes) + [crc & 0xFF, (crc >> 8) & 0xFF]
    return ' '.join(f'{b:02X}' for b in frame)

def to_u16(val):
    if val < 0:
        return val + 65536
    return val

# =============================================================================
# ===== 常用指令速查（含 CRC） =====
# =============================================================================

PRESET_CMDS = {
    # --- 控制命令 REG_CTRL_CMD (0x2720) ---
    "START":        [0x01,0x06,0x27,0x20,0x00,0x01],
    "STOP":         [0x01,0x06,0x27,0x20,0x00,0x02],
    "ESTOP":        [0x01,0x06,0x27,0x20,0x00,0x04],
    "RESET":        [0x01,0x06,0x27,0x20,0x00,0x08],
    "FWD":          [0x01,0x06,0x27,0x20,0x00,0x11],   # START + FWD
    "REV":          [0x01,0x06,0x27,0x20,0x00,0x21],   # START + REV

    # --- 读取实时数据 (0x03) ---
    "READ_SPEED":   [0x01,0x03,0x27,0x30,0x00,0x01],
    "READ_ANGLE":   [0x01,0x03,0x27,0x31,0x00,0x01],
    "READ_VOLTAGE": [0x01,0x03,0x27,0x32,0x00,0x01],
    "READ_CURRENT": [0x01,0x03,0x27,0x33,0x00,0x01],
    "READ_DIR":     [0x01,0x03,0x27,0x37,0x00,0x01],
    "READ_FAULT":   [0x01,0x03,0x27,0x40,0x00,0x01],

    # --- 读取配置参数 (0x03) ---
    "READ_NODE_ID":       [0x01,0x03,0x27,0x10,0x00,0x01],
    "READ_TARGET_SPEED":  [0x01,0x03,0x27,0x11,0x00,0x01],
    "READ_TARGET_ANGLE":  [0x01,0x03,0x27,0x12,0x00,0x01],
    "READ_VOLT_UPPER":    [0x01,0x03,0x27,0x14,0x00,0x01],
    "READ_VOLT_LOWER":    [0x01,0x03,0x27,0x15,0x00,0x01],
    "READ_CURR_UPPER":    [0x01,0x03,0x27,0x16,0x00,0x01],
    "READ_CLOSE_ANGLE":   [0x01,0x03,0x27,0x1C,0x00,0x01],
    "READ_OPEN_ANGLE":    [0x01,0x03,0x27,0x1D,0x00,0x01],
    "READ_CURR_DETECT":   [0x01,0x03,0x27,0x1E,0x00,0x01],

    # --- 霍尔方向 (0x3710) ---
    "HALL_NORMAL":  [0x01,0x06,0x37,0x10,0x00,0x00],
    "HALL_INVERT":  [0x01,0x06,0x37,0x10,0x00,0x01],

    # --- 电机方向 (0x3711) ---
    "MOTOR_NORMAL": [0x01,0x06,0x37,0x11,0x00,0x00],
    "MOTOR_INVERT": [0x01,0x06,0x37,0x11,0x00,0x01],

    # --- 清除故障 (0x2740) ---
    "CLEAR_ALL":    [0x01,0x06,0x27,0x40,0x00,0x00],
}

# 动态计算所有 CRC
PRESET_CMDS = {k: make_frame(v) for k, v in PRESET_CMDS.items()}

print("=" * 60)
print("  Modbus RTU 指令生成器")
print("=" * 60)
print(f"  设备地址: {NODE_ID}")
print(f"  操作:     {OPERATION}")

# =============================================================================
# 生成指令
# =============================================================================

if OPERATION == "READ":
    print(f"  寄存器:   0x{REG_ADDR:04X}")
    print("=" * 60)
    cmd = make_frame([NODE_ID, 0x03, (REG_ADDR >> 8) & 0xFF, REG_ADDR & 0xFF, 0x00, 0x01])
    print(f"\n  发送: {cmd}")
    print(f"  应答: {NODE_ID:02X} 03 02 XX XX CRC  (XX XX = 寄存器值)")

elif OPERATION == "WRITE":
    val = to_u16(WRITE_VALUE)
    print(f"  寄存器:   0x{REG_ADDR:04X}")
    print(f"  写入值:   {WRITE_VALUE} (0x{val:04X})")
    print("=" * 60)
    cmd = make_frame([NODE_ID, 0x06, (REG_ADDR >> 8) & 0xFF, REG_ADDR & 0xFF,
                      (val >> 8) & 0xFF, val & 0xFF])
    print(f"\n  发送: {cmd}")
    print(f"  应答: 回显相同帧")

elif OPERATION == "CLEAR_FAULT":
    val = CLEAR_FAULT_BITS
    print(f"  清除位:   0x{val:04X}")
    print("=" * 60)
    cmd = make_frame([NODE_ID, 0x06, 0x27, 0x40,
                      (val >> 8) & 0xFF, val & 0xFF])
    print(f"\n  发送: {cmd}")

elif OPERATION == "CTRL_CMD":
    val = CTRL_CMD_VALUE
    desc_parts = []
    if val & 0x0001: desc_parts.append("START")
    if val & 0x0002: desc_parts.append("STOP")
    if val & 0x0004: desc_parts.append("ESTOP")
    if val & 0x0008: desc_parts.append("RESET")
    if val & 0x0010: desc_parts.append("FWD")
    if val & 0x0020: desc_parts.append("REV")
    desc = " + ".join(desc_parts) if desc_parts else "NONE"
    print(f"  命令:     0x{val:04X} = {desc}")
    print("=" * 60)
    cmd = make_frame([NODE_ID, 0x06, 0x27, 0x20,
                      (val >> 8) & 0xFF, val & 0xFF])
    print(f"\n  发送: {cmd}")

# =============================================================================
# 常用指令速查表
# =============================================================================
print(f"\n{'='*60}")
print("  常用指令速查 (已含 CRC)")
print(f"{'='*60}")
for name, frame in PRESET_CMDS.items():
    print(f"  {name:<18} {frame}")
print(f"{'='*60}")

input("\n按 Enter 退出...")
