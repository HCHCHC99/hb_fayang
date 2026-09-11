#!/usr/bin/env python3
"""
电机仲裁系统模拟器 - Motor Arbitration Simulator
================================================
基于 dev_motor.c 的仲裁逻辑，忠实模拟:
  - 4 个命令队列 (allow_fwd / allow_rev / block_fwd / block_rev)
  - 设备基因表 (Device Gene Table)
  - 仲裁决策算法
  - SetAllow / SetBlock 的互斥约束
  - 所有事件源 (IO / 过流 / 限位 / 电压 / 硬限位)

运行: python motor_arbitration_sim.py
"""

import tkinter as tk
from tkinter import ttk, messagebox
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, List
import time


# =============================================================================
# 枚举定义 (与 dev_motor.h 一致)
# =============================================================================

class MotorDir(IntEnum):
    NONE = 0
    FWD = 1
    REV = 2

class MotorState(IntEnum):
    IDLE = 0
    RAMPING = 1
    RUNNING = 2
    LOCKED = 3

class CmdType(IntEnum):
    NONE_USE = 255
    STOP = 1
    RUN_FWD = 2
    RUN_REV = 3
    RAMP_FWD = 4
    RAMP_REV = 5
    BLOCK_FWD = 6
    BLOCK_REV = 7
    BLOCK_BOTH = 8

class DeviceId(IntEnum):
    NONE = 255
    POWER_POS = 1
    POWER_NEG = 2
    LIMIT_FWD = 3
    LIMIT_REV = 4
    CAN = 5
    IO_FWD = 6
    IO_REV = 7
    EMERGENCY = 8
    RTURN_FWD = 9
    RTURN_REV = 10
    OVERVOLTAGE_FWD = 11
    OVERVOLTAGE_REV = 12
    UNDERVOLTAGE_FWD = 13
    UNDERVOLTAGE_REV = 14
    OVERCUR_FWD = 15

class Priority(IntEnum):
    EMERGENCY = 0
    LIMIT = 2
    MANUAL = 3
    CAN = 4
    POWER = 5
    NONE = 255

# Capability flags
CAP_BLOCK = 1 << 0
CAP_ALLOW = 1 << 1

# =============================================================================
# 设备基因表 (与 dev_motor.c c_device_genes[] 一致)
# =============================================================================

@dataclass
class DeviceGene:
    dev_id: DeviceId
    priority: Priority
    capability: int  # CAP_BLOCK | CAP_ALLOW
    name: str
    color: str  # GUI 显示颜色

DEVICE_GENES = [
    # 限位保护设备 (PRIO_LIMIT, CAP_BLOCK)
    DeviceGene(DeviceId.RTURN_FWD,         Priority.LIMIT, CAP_BLOCK, "旋转限位正", "#FF6B6B"),
    DeviceGene(DeviceId.RTURN_REV,         Priority.LIMIT, CAP_BLOCK, "旋转限位反", "#FF6B6B"),
    DeviceGene(DeviceId.OVERVOLTAGE_FWD,   Priority.LIMIT, CAP_BLOCK, "过压保护正", "#FFA500"),
    DeviceGene(DeviceId.OVERVOLTAGE_REV,   Priority.LIMIT, CAP_BLOCK, "过压保护反", "#FFA500"),
    DeviceGene(DeviceId.UNDERVOLTAGE_FWD,  Priority.LIMIT, CAP_BLOCK, "欠压保护正", "#FFD700"),
    DeviceGene(DeviceId.UNDERVOLTAGE_REV,  Priority.LIMIT, CAP_BLOCK, "欠压保护反", "#FFD700"),
    DeviceGene(DeviceId.OVERCUR_FWD,       Priority.LIMIT, CAP_BLOCK, "过流保护正", "#FF4444"),
    # 手动IO设备 (PRIO_MANUAL, CAP_BLOCK | CAP_ALLOW)
    DeviceGene(DeviceId.IO_FWD,            Priority.MANUAL, CAP_BLOCK | CAP_ALLOW, "IO正转", "#4ECDC4"),
    DeviceGene(DeviceId.IO_REV,            Priority.MANUAL, CAP_BLOCK | CAP_ALLOW, "IO反转", "#45B7D1"),
]

# 硬件限位 (已注释, 但保留)
# DeviceGene(DeviceId.LIMIT_FWD, Priority.LIMIT, CAP_BLOCK, "硬限位正"),
# DeviceGene(DeviceId.LIMIT_REV, Priority.LIMIT, CAP_BLOCK, "硬限位反"),

GENE_MAP = {g.dev_id: g for g in DEVICE_GENES}

MAX_COMMANDS = 20

# =============================================================================
# 命令和队列
# =============================================================================

@dataclass
class MotorCommand:
    device_id: DeviceId = DeviceId.NONE
    priority: Priority = Priority.NONE
    type: CmdType = CmdType.NONE_USE
    param: float = 0.0
    timestamp: int = 0

class CommandList:
    """命令列表 (allow 按优先级排序, block 无序追加)"""
    def __init__(self, name: str):
        self.name = name
        self.commands: List[MotorCommand] = []
        self.max_count = MAX_COMMANDS

    @property
    def count(self) -> int:
        return len(self.commands)

    @property
    def empty(self) -> bool:
        return len(self.commands) == 0

    def clear(self):
        self.commands.clear()

    def remove(self, dev_id: DeviceId):
        self.commands = [c for c in self.commands if c.device_id != dev_id]

    def add_sorted_by_priority(self, cmd: MotorCommand):
        """按优先级降序插入 (值越小优先级越高)"""
        self.remove(cmd.device_id)
        if len(self.commands) >= self.max_count:
            return
        idx = 0
        for i, c in enumerate(self.commands):
            if cmd.priority < c.priority:
                break
            idx = i + 1
        self.commands.insert(idx, cmd)

    def add_append(self, cmd: MotorCommand):
        """无序追加, 去重"""
        for c in self.commands:
            if c.device_id == cmd.device_id:
                return
        if len(self.commands) >= self.max_count:
            return
        self.commands.append(cmd)

    def get_top(self) -> Optional[MotorCommand]:
        return self.commands[0] if self.commands else None


# =============================================================================
# 电机仲裁系统核心 (与 dev_motor.c 一致)
# =============================================================================

class MotorArbitrationSystem:
    """忠实复现 dev_motor.c 的仲裁逻辑, 包含 SetAllow/SetBlock 互斥约束"""

    def __init__(self, log_callback=None):
        self.log = log_callback or print

        # 4 个队列
        self.allow_fwd = CommandList("allow_fwd")
        self.allow_rev = CommandList("allow_rev")
        self.block_fwd = CommandList("block_fwd")
        self.block_rev = CommandList("block_rev")

        # 电机状态
        self.state: MotorState = MotorState.IDLE
        self.active_dir: MotorDir = MotorDir.NONE
        self.active_device_id: DeviceId = DeviceId.NONE
        self.current_duty: float = 0.0
        self.conflict_fault: bool = False
        self.enable: bool = True

        # 过流报警状态 (dev_sensor)
        self.overcurrent_active: bool = False

        # RTurn 限位状态
        self.rturn_fwd_active: bool = False
        self.rturn_rev_active: bool = False

        # 电压报警状态
        self.overvoltage_active: bool = False
        self.undervoltage_active: bool = False

        # 硬限位状态
        self.hardlimit_fwd_active: bool = False
        self.hardlimit_rev_active: bool = False

        # ===== 初始化: IO 设备默认 block (与 Motor_Init 一致) =====
        self._init_default_blocks()

    def _init_default_blocks(self):
        """Motor_Init: IO_FWD 默认 block_fwd, IO_REV 默认 block_rev"""
        self.log("[INIT] 初始化: IO_FWD -> block_fwd, IO_REV -> block_rev")

        io_fwd_block = MotorCommand(
            device_id=DeviceId.IO_FWD,
            priority=Priority.MANUAL,
            type=CmdType.BLOCK_FWD,
            timestamp=0
        )
        self._set_block_internal(self.block_fwd, self.allow_fwd, io_fwd_block)

        io_rev_block = MotorCommand(
            device_id=DeviceId.IO_REV,
            priority=Priority.MANUAL,
            type=CmdType.BLOCK_REV,
            timestamp=0
        )
        self._set_block_internal(self.block_rev, self.allow_rev, io_rev_block)

    # ===== 核心操作 =====

    def set_allow(self, allow_q: CommandList, block_q: CommandList, cmd: MotorCommand) -> bool:
        """
        Motor_CmdList_SetAllow (改动后):
        如果对应方向的 block 队列非空, 则拒绝写入 allow
        """
        if block_q is not None and block_q.count > 0:
            block_devs = ", ".join(GENE_MAP.get(c.device_id, DeviceGene(c.device_id, Priority.NONE, 0, "?", "#888")).name for c in block_q.commands)
            self.log(f"[SetAllow 拒绝] allow={allow_q.name}, block={block_q.name}, "
                     f"block_count={block_q.count}, block_devs=[{block_devs}], "
                     f"被拒绝设备={GENE_MAP.get(cmd.device_id, DeviceGene(cmd.device_id, Priority.NONE, 0, '?', '#888')).name}")
            return False

        self.log(f"[SetAllow 通过] allow={allow_q.name}, dev={GENE_MAP.get(cmd.device_id).name}, "
                 f"priority={cmd.priority}, type={cmd.type.name}")
        allow_q.add_sorted_by_priority(cmd)
        self._arbitrate()
        return True

    def set_block(self, block_q: CommandList, allow_q: CommandList, cmd: MotorCommand):
        """
        Motor_CmdList_SetBlock (改动后):
        设置 block 时自动清空对应方向的 allow 队列
        """
        if allow_q is not None:
            cleared_count = allow_q.count
            if cleared_count > 0:
                self.log(f"[SetBlock 清空] block={block_q.name}, 清空 allow={allow_q.name}, "
                         f"清除了 {cleared_count} 条指令")
            allow_q.clear()

        self._set_block_internal(block_q, allow_q, cmd)

    def _set_block_internal(self, block_q: CommandList, allow_q: CommandList, cmd: MotorCommand):
        self.log(f"[SetBlock] block={block_q.name}, dev={GENE_MAP.get(cmd.device_id, DeviceGene(cmd.device_id, Priority.NONE, 0, '?', '#888')).name}")
        block_q.add_append(cmd)
        self._arbitrate()

    def remove_from_block(self, block_q: CommandList, dev_id: DeviceId):
        self.log(f"[Remove Block] block={block_q.name}, dev={GENE_MAP.get(dev_id).name}")
        block_q.remove(dev_id)
        self._arbitrate()

    def remove_from_allow(self, allow_q: CommandList, dev_id: DeviceId):
        self.log(f"[Remove Allow] allow={allow_q.name}, dev={GENE_MAP.get(dev_id).name}")
        allow_q.remove(dev_id)
        self._arbitrate()

    # ===== 仲裁决策 (Motor_ArbitrationDecision) =====

    def _arbitrate(self):
        """Motor_ArbitrationDecision: 核心仲裁逻辑"""
        if not self.enable:
            return

        # 1. 选命令: block 为空且 allow 非空 → 取 allow 队首(最高优先级)
        fwd_cmd = self.allow_fwd.get_top() if self.block_fwd.empty and not self.allow_fwd.empty else None
        rev_cmd = self.allow_rev.get_top() if self.block_rev.empty and not self.allow_rev.empty else None

        # 2. 冲突判断
        final: Optional[MotorCommand] = None
        self.conflict_fault = False

        if fwd_cmd and rev_cmd:
            if fwd_cmd.device_id == rev_cmd.device_id:
                # 同一设备请求两个方向 → 冲突
                final = None
                self.conflict_fault = True
                self.log(f"[仲裁 冲突!] 同一设备 {GENE_MAP.get(fwd_cmd.device_id).name} 请求双向 → 死锁不动")
            elif fwd_cmd.priority < rev_cmd.priority:
                final = fwd_cmd
                self.log(f"[仲裁 选正转] {GENE_MAP.get(fwd_cmd.device_id).name}(prio={fwd_cmd.priority}) "
                         f"优先级高于 {GENE_MAP.get(rev_cmd.device_id).name}(prio={rev_cmd.priority})")
            elif rev_cmd.priority < fwd_cmd.priority:
                final = rev_cmd
                self.log(f"[仲裁 选反转] {GENE_MAP.get(rev_cmd.device_id).name}(prio={rev_cmd.priority}) "
                         f"优先级高于 {GENE_MAP.get(fwd_cmd.device_id).name}(prio={fwd_cmd.priority})")
            else:
                # 同优先级 → 死锁
                final = None
                self.log(f"[仲裁 死锁] 同优先级 {fwd_cmd.priority} → 不动")
        else:
            final = fwd_cmd if fwd_cmd else rev_cmd

        # 3. 执行
        if final:
            self._execute(final)
        else:
            self._stop()

    def _execute(self, cmd: MotorCommand):
        self.active_device_id = cmd.device_id
        self.current_duty = cmd.param

        if cmd.type in (CmdType.RUN_FWD, CmdType.RAMP_FWD):
            self.active_dir = MotorDir.FWD
        elif cmd.type in (CmdType.RUN_REV, CmdType.RAMP_REV):
            self.active_dir = MotorDir.REV
        else:
            self.active_dir = MotorDir.NONE

        self.state = MotorState.RAMPING if cmd.type in (CmdType.RAMP_FWD, CmdType.RAMP_REV) else MotorState.RUNNING

        dev_name = GENE_MAP.get(cmd.device_id, DeviceGene(cmd.device_id, Priority.NONE, 0, "?", "#888")).name
        dir_str = "正转" if self.active_dir == MotorDir.FWD else "反转"
        self.log(f"[执行] {dir_str} | 设备={dev_name} | 占空比={self.current_duty:.0f}% | 状态={self.state.name}")

    def _stop(self):
        self.current_duty = 0.0
        self.state = MotorState.IDLE
        self.active_dir = MotorDir.NONE
        self.active_device_id = DeviceId.NONE
        reason = "冲突故障" if self.conflict_fault else ("无选中命令" if not (self.allow_fwd.get_top() or self.allow_rev.get_top()) else "被阻塞")
        self.log(f"[停止] reason={reason}")

    # ===== 事件源 =====

    # --- IO 手动控制 (Motor_OnManualIO) ---
    def io_fwd(self, duty: float = 85.0):
        """手动正转"""
        self.log(">>> [IO] 正转按钮按下 <<<")
        io_dev = DeviceId.IO_FWD
        gene = GENE_MAP[io_dev]
        cmd = MotorCommand(device_id=io_dev, priority=gene.priority,
                           type=CmdType.RUN_FWD, param=duty, timestamp=int(time.time()*1000))
        self.remove_from_block(self.block_fwd, io_dev)
        self.set_allow(self.allow_fwd, self.block_fwd, cmd)

    def io_rev(self, duty: float = 85.0):
        """手动反转"""
        self.log(">>> [IO] 反转按钮按下 <<<")
        io_dev = DeviceId.IO_REV
        gene = GENE_MAP[io_dev]
        cmd = MotorCommand(device_id=io_dev, priority=gene.priority,
                           type=CmdType.RUN_REV, param=duty, timestamp=int(time.time()*1000))
        self.remove_from_block(self.block_rev, io_dev)
        self.set_allow(self.allow_rev, self.block_rev, cmd)

    def io_stop_fwd(self):
        """停止正转"""
        self.log(">>> [IO] 停止正转 <<<")
        gene = GENE_MAP[DeviceId.IO_FWD]
        self.remove_from_allow(self.allow_fwd, DeviceId.IO_FWD)
        block_cmd = MotorCommand(device_id=DeviceId.IO_FWD, priority=gene.priority,
                                 type=CmdType.BLOCK_FWD, timestamp=int(time.time()*1000))
        self.set_block(self.block_fwd, self.allow_fwd, block_cmd)

    def io_stop_rev(self):
        """停止反转"""
        self.log(">>> [IO] 停止反转 <<<")
        gene = GENE_MAP[DeviceId.IO_REV]
        self.remove_from_allow(self.allow_rev, DeviceId.IO_REV)
        block_cmd = MotorCommand(device_id=DeviceId.IO_REV, priority=gene.priority,
                                 type=CmdType.BLOCK_REV, timestamp=int(time.time()*1000))
        self.set_block(self.block_rev, self.allow_rev, block_cmd)

    def io_stop_all(self):
        """停止所有 IO"""
        self.log(">>> [IO] 全部停止 <<<")
        gene_fwd = GENE_MAP[DeviceId.IO_FWD]
        gene_rev = GENE_MAP[DeviceId.IO_REV]

        self.remove_from_allow(self.allow_fwd, DeviceId.IO_FWD)
        self.remove_from_allow(self.allow_rev, DeviceId.IO_REV)

        block_fwd_cmd = MotorCommand(device_id=DeviceId.IO_FWD, priority=gene_fwd.priority,
                                     type=CmdType.BLOCK_FWD, timestamp=int(time.time()*1000))
        self.set_block(self.block_fwd, self.allow_fwd, block_fwd_cmd)

        block_rev_cmd = MotorCommand(device_id=DeviceId.IO_REV, priority=gene_rev.priority,
                                     type=CmdType.BLOCK_REV, timestamp=int(time.time()*1000))
        self.set_block(self.block_rev, self.allow_rev, block_rev_cmd)

    # --- 过流报警 (Motor_OnCurrentAlarm) ---
    def overcurrent_trigger(self, current_ma: int = 500):
        """过流触发"""
        self.log(f">>> [过流] 触发! current={current_ma}mA, threshold=400mA <<<")
        self.overcurrent_active = True

        # 判断当前方向
        desired_dir = self.active_dir if self.state in (MotorState.RUNNING, MotorState.RAMPING) else MotorDir.NONE

        if desired_dir == MotorDir.FWD:
            # 正转过流: block 正转
            block_cmd = MotorCommand(
                device_id=DeviceId.OVERCUR_FWD,
                priority=Priority.LIMIT,
                type=CmdType.BLOCK_FWD,
                timestamp=int(time.time()*1000)
            )
            self.set_block(self.block_fwd, self.allow_fwd, block_cmd)
            self.log("[过流] 正转方向过流 → block_fwd 加入 OVERCUR_FWD, 清空 allow_fwd")
        else:
            # 反转/停止时过流: 当前代码只打日志 (Bug区域!)
            self.log(f"[过流] 方向={desired_dir.name}, REV/STOP 属于正常堵转 → 跳过 block (当前逻辑)")
            # 注意: 按照与用户的讨论, 这里也应该清理 allow_rev 并 block_rev
            # 但当前代码没有这样做

    def overcurrent_release(self):
        """过流释放"""
        self.log(">>> [过流] 释放 <<<")
        self.overcurrent_active = False
        self.remove_from_block(self.block_fwd, DeviceId.OVERCUR_FWD)
        self.log("[过流] 移除 block_fwd 中的 OVERCUR_FWD")

    # --- 旋转限位 (Motor_OnRTurnLimit) ---
    def rturn_limit_trigger(self, direction: str):
        """旋转限位触发"""
        if direction == "fwd":
            self.log(">>> [旋转限位] 正转限位触发! <<<")
            self.rturn_fwd_active = True
            dev_id = DeviceId.RTURN_FWD
            block_q = self.block_fwd
            allow_q = self.allow_fwd
            cmd = MotorCommand(device_id=dev_id, priority=Priority.LIMIT,
                               type=CmdType.BLOCK_FWD, timestamp=int(time.time()*1000))
            # 先清 allow
            self.remove_from_allow(allow_q, DeviceId.IO_FWD)
            self.set_block(block_q, allow_q, cmd)
            self.log("[旋转限位] block_fwd 加入 RTURN_FWD, 清空 allow_fwd")
        else:
            self.log(">>> [旋转限位] 反转限位触发! <<<")
            self.rturn_rev_active = True
            dev_id = DeviceId.RTURN_REV
            block_q = self.block_rev
            allow_q = self.allow_rev
            cmd = MotorCommand(device_id=dev_id, priority=Priority.LIMIT,
                               type=CmdType.BLOCK_REV, timestamp=int(time.time()*1000))
            self.remove_from_allow(allow_q, DeviceId.IO_REV)
            self.set_block(block_q, allow_q, cmd)
            self.log("[旋转限位] block_rev 加入 RTURN_REV, 清空 allow_rev")

    def rturn_limit_release(self, direction: str = "all"):
        """旋转限位释放"""
        if direction in ("fwd", "all"):
            self.log(">>> [旋转限位] 正转限位释放 <<<")
            self.rturn_fwd_active = False
            self.remove_from_block(self.block_fwd, DeviceId.RTURN_FWD)
        if direction in ("rev", "all"):
            self.log(">>> [旋转限位] 反转限位释放 <<<")
            self.rturn_rev_active = False
            self.remove_from_block(self.block_rev, DeviceId.RTURN_REV)

    # --- 电压报警 (Motor_OnVoltageAlarm) ---
    def voltage_alarm_trigger(self, alarm_type: str):
        """电压报警触发"""
        if alarm_type == "overvoltage":
            self.log(">>> [电压] 过压报警触发! 双向 block <<<")
            self.overvoltage_active = True
            fwd_id = DeviceId.OVERVOLTAGE_FWD
            rev_id = DeviceId.OVERVOLTAGE_REV
        else:
            self.log(">>> [电压] 欠压报警触发! 双向 block <<<")
            self.undervoltage_active = True
            fwd_id = DeviceId.UNDERVOLTAGE_FWD
            rev_id = DeviceId.UNDERVOLTAGE_REV

        block_fwd_cmd = MotorCommand(device_id=fwd_id, priority=Priority.LIMIT,
                                     type=CmdType.BLOCK_FWD, timestamp=int(time.time()*1000))
        self.set_block(self.block_fwd, self.allow_fwd, block_fwd_cmd)

        block_rev_cmd = MotorCommand(device_id=rev_id, priority=Priority.LIMIT,
                                     type=CmdType.BLOCK_REV, timestamp=int(time.time()*1000))
        self.set_block(self.block_rev, self.allow_rev, block_rev_cmd)

    def voltage_alarm_release(self, alarm_type: str):
        """电压报警释放"""
        self.log(f">>> [电压] {alarm_type} 释放 <<<")
        if alarm_type == "overvoltage":
            self.overvoltage_active = False
            fwd_id = DeviceId.OVERVOLTAGE_FWD
            rev_id = DeviceId.OVERVOLTAGE_REV
        else:
            self.undervoltage_active = False
            fwd_id = DeviceId.UNDERVOLTAGE_FWD
            rev_id = DeviceId.UNDERVOLTAGE_REV

        self.remove_from_block(self.block_fwd, fwd_id)
        self.remove_from_block(self.block_rev, rev_id)

    # --- 硬限位 (Motor_OnHardLimit) ---
    def hardlimit_trigger(self, direction: str):
        self.log(f">>> [硬限位] {direction}触发 <<<")
        if direction == "fwd":
            self.hardlimit_fwd_active = True
            dev_id = DeviceId.LIMIT_FWD
            cmd = MotorCommand(device_id=dev_id, priority=Priority.LIMIT,
                               type=CmdType.BLOCK_FWD, timestamp=int(time.time()*1000))
            self.set_block(self.block_fwd, self.allow_fwd, cmd)
        else:
            self.hardlimit_rev_active = True
            dev_id = DeviceId.LIMIT_REV
            cmd = MotorCommand(device_id=dev_id, priority=Priority.LIMIT,
                               type=CmdType.BLOCK_REV, timestamp=int(time.time()*1000))
            self.set_block(self.block_rev, self.allow_rev, cmd)

    def hardlimit_release(self, direction: str = "all"):
        self.log(f">>> [硬限位] {direction}释放 <<<")
        if direction in ("fwd", "all"):
            self.hardlimit_fwd_active = False
            self.remove_from_block(self.block_fwd, DeviceId.LIMIT_FWD)
        if direction in ("rev", "all"):
            self.hardlimit_rev_active = False
            self.remove_from_block(self.block_rev, DeviceId.LIMIT_REV)

    # ===== 紧急停止 =====
    def emergency_stop(self):
        self.log(">>> [紧急停止] 清空所有 allow <<<")
        self.allow_fwd.clear()
        self.allow_rev.clear()
        self._arbitrate()

    # ===== 重置 =====
    def reset(self):
        self.log("========== 系统重置 ==========")
        self.allow_fwd.clear()
        self.allow_rev.clear()
        self.block_fwd.clear()
        self.block_rev.clear()
        self.state = MotorState.IDLE
        self.active_dir = MotorDir.NONE
        self.active_device_id = DeviceId.NONE
        self.current_duty = 0.0
        self.conflict_fault = False
        self.overcurrent_active = False
        self.rturn_fwd_active = False
        self.rturn_rev_active = False
        self.overvoltage_active = False
        self.undervoltage_active = False
        self.hardlimit_fwd_active = False
        self.hardlimit_rev_active = False
        self._init_default_blocks()


# =============================================================================
# GUI
# =============================================================================

class MotorSimulatorGUI:
    """tkinter GUI 封装"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("电机仲裁系统模拟器 - Motor Arbitration Simulator")
        self.root.geometry("1200x850")
        self.root.configure(bg="#1a1a2e")

        # 颜色主题
        self.bg = "#1a1a2e"
        self.panel_bg = "#16213e"
        self.text_fg = "#e0e0e0"
        self.accent = "#0f3460"
        self.highlight = "#e94560"
        self.green = "#00b894"
        self.blue = "#0984e3"
        self.orange = "#e17055"
        self.yellow = "#fdcb6e"

        # 日志 (必须在系统之前初始化, 因为系统构造时会触发 log 回调)
        self.log_lines: List[str] = []
        self.max_log_lines = 100

        # 系统
        self.sys = MotorArbitrationSystem(log_callback=self.gui_log)

        self._build_ui()
        self._refresh_all()

    def gui_log(self, msg: str):
        self.log_lines.append(msg)
        if len(self.log_lines) > self.max_log_lines:
            self.log_lines = self.log_lines[-self.max_log_lines:]
        self.root.after(0, self._update_log_display)

    # =================================================================
    # UI 构建
    # =================================================================
    def _build_ui(self):
        # 主容器
        main_frame = tk.Frame(self.root, bg=self.bg)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 顶部标题
        title_frame = tk.Frame(main_frame, bg=self.bg)
        title_frame.pack(fill=tk.X, pady=(0, 10))
        tk.Label(title_frame, text="⚡ 电机仲裁系统模拟器", font=("Arial", 18, "bold"),
                 fg=self.highlight, bg=self.bg).pack(side=tk.LEFT)
        tk.Label(title_frame, text="基于 dev_motor.c 仲裁逻辑 | SetAllow/SetBlock 互斥约束",
                 font=("Arial", 10), fg="#888", bg=self.bg).pack(side=tk.LEFT, padx=20)

        # 上方: 状态 + 基因表
        top_panel = tk.Frame(main_frame, bg=self.bg)
        top_panel.pack(fill=tk.X, pady=(0, 5))

        # -- 电机状态 --
        self._build_motor_state(top_panel)

        # -- 设备基因表 --
        self._build_gene_table(top_panel)

        # 中间: 4个队列
        queue_panel = tk.Frame(main_frame, bg=self.bg)
        queue_panel.pack(fill=tk.X, pady=5)

        self._build_queue_display(queue_panel, "✅ Allow FWD (允许正转)", "allow_fwd", 0, self.green)
        self._build_queue_display(queue_panel, "✅ Allow REV (允许反转)", "allow_rev", 1, self.green)
        self._build_queue_display(queue_panel, "🚫 Block FWD (禁止正转)", "block_fwd", 2, self.highlight)
        self._build_queue_display(queue_panel, "🚫 Block REV (禁止反转)", "block_rev", 3, self.highlight)

        # 控制面板
        control_panel = tk.Frame(main_frame, bg=self.panel_bg, relief=tk.RIDGE, bd=1)
        control_panel.pack(fill=tk.X, pady=10, ipady=5)

        self._build_controls(control_panel)

        # 日志区
        log_frame = tk.Frame(main_frame, bg=self.bg)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(5, 0))

        tk.Label(log_frame, text="📋 事件日志", font=("Arial", 11, "bold"),
                 fg=self.text_fg, bg=self.bg).pack(anchor=tk.W)

        self.log_text = tk.Text(log_frame, height=12, bg="#0a0a1a", fg="#00ff88",
                                font=("Consolas", 10), wrap=tk.WORD, relief=tk.SUNKEN, bd=1)
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=5)

        log_scroll = tk.Scrollbar(self.log_text, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # 底部按钮
        bottom_frame = tk.Frame(main_frame, bg=self.bg)
        bottom_frame.pack(fill=tk.X, pady=(5, 0))
        tk.Button(bottom_frame, text="🔄 重置系统", command=self._reset,
                  bg=self.orange, fg="white", font=("Arial", 10, "bold"),
                  relief=tk.RAISED, bd=2, padx=15).pack(side=tk.LEFT, padx=5)
        tk.Button(bottom_frame, text="🚨 紧急停止", command=self._emergency_stop,
                  bg=self.highlight, fg="white", font=("Arial", 10, "bold"),
                  relief=tk.RAISED, bd=2, padx=15).pack(side=tk.LEFT, padx=5)

    def _build_motor_state(self, parent):
        frame = tk.LabelFrame(parent, text="📊 电机状态", font=("Arial", 10, "bold"),
                              fg=self.text_fg, bg=self.panel_bg, relief=tk.RIDGE, bd=1,
                              padx=10, pady=5)
        frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        self.state_labels = {}
        rows = [
            ("方向 (Dir):", "dir", self.green),
            ("状态 (State):", "state", self.blue),
            ("占空比 (Duty):", "duty", self.yellow),
            ("活动设备:", "device", self.orange),
            ("冲突故障:", "conflict", self.highlight),
        ]
        for i, (label, key, color) in enumerate(rows):
            tk.Label(frame, text=label, fg="#aaa", bg=self.panel_bg,
                     font=("Arial", 10)).grid(row=i, column=0, sticky=tk.W, padx=5)
            lbl = tk.Label(frame, text="--", fg=color, bg=self.panel_bg,
                           font=("Arial", 10, "bold"))
            lbl.grid(row=i, column=1, sticky=tk.W, padx=5)
            self.state_labels[key] = lbl

    def _build_gene_table(self, parent):
        frame = tk.LabelFrame(parent, text="🧬 设备基因表 (Device Gene Table)", font=("Arial", 10, "bold"),
                              fg=self.text_fg, bg=self.panel_bg, relief=tk.RIDGE, bd=1,
                              padx=10, pady=5)
        frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 表头
        headers = ["设备名称", "ID", "优先级", "能力", "颜色"]
        for j, h in enumerate(headers):
            tk.Label(frame, text=h, fg=self.yellow, bg=self.panel_bg,
                     font=("Arial", 9, "bold")).grid(row=0, column=j, padx=8, pady=2)

        for i, gene in enumerate(DEVICE_GENES):
            cap_str = ""
            if gene.capability & CAP_BLOCK:
                cap_str += "BLOCK "
            if gene.capability & CAP_ALLOW:
                cap_str += "ALLOW"
            if not cap_str:
                cap_str = "--"

            row_data = [
                gene.name,
                str(gene.dev_id.value),
                f"PRIO_{gene.priority.name} ({gene.priority.value})",
                cap_str.strip(),
            ]
            for j, text in enumerate(row_data):
                fg = gene.color if j == 0 else "#ccc"
                tk.Label(frame, text=text, fg=fg, bg=self.panel_bg,
                         font=("Arial", 9)).grid(row=i+1, column=j, padx=8, pady=1)

    def _build_queue_display(self, parent, title: str, qname: str, col: int, color: str):
        frame = tk.LabelFrame(parent, text=title, font=("Arial", 9, "bold"),
                              fg=color, bg=self.panel_bg, relief=tk.RIDGE, bd=1,
                              padx=5, pady=2)
        frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=3)

        listbox = tk.Listbox(frame, height=6, bg="#0a0a1a", fg=color,
                             font=("Consolas", 10), relief=tk.FLAT,
                             selectbackground=self.accent)
        listbox.pack(fill=tk.BOTH, expand=True)
        setattr(self, f"listbox_{qname}", listbox)

    def _build_controls(self, parent):
        # === 第1行: IO 控制 ===
        row1 = tk.Frame(parent, bg=self.panel_bg)
        row1.pack(fill=tk.X, padx=10, pady=5)

        tk.Label(row1, text="🎮 IO手动控制:", fg=self.text_fg, bg=self.panel_bg,
                 font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(0, 10))

        for text, cmd, bg_color in [
            ("▶ 正转 FWD", self.sys.io_fwd, self.green),
            ("◀ 反转 REV", self.sys.io_rev, self.blue),
            ("⏹ 停正转", self.sys.io_stop_fwd, self.orange),
            ("⏹ 停反转", self.sys.io_stop_rev, self.orange),
            ("⏹ 全停", self.sys.io_stop_all, self.highlight),
        ]:
            tk.Button(row1, text=text, command=cmd, bg=bg_color, fg="white",
                      font=("Arial", 9, "bold"), relief=tk.RAISED, bd=2,
                      padx=8, pady=3).pack(side=tk.LEFT, padx=3)

        # === 第2行: 过流 + 限位 ===
        row2 = tk.Frame(parent, bg=self.panel_bg)
        row2.pack(fill=tk.X, padx=10, pady=3)

        tk.Label(row2, text="⚡ 过流模拟:", fg=self.text_fg, bg=self.panel_bg,
                 font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(0, 10))

        self.overcurrent_var = tk.IntVar(value=500)
        tk.Scale(row2, from_=0, to=2000, orient=tk.HORIZONTAL, length=150,
                 variable=self.overcurrent_var, bg=self.panel_bg, fg=self.orange,
                 troughcolor="#333", highlightbackground=self.panel_bg,
                 label="电流(mA)", font=("Arial", 8)).pack(side=tk.LEFT, padx=5)

        tk.Button(row2, text="🔥 触发过流", command=self._trigger_overcurrent,
                  bg="#ff4444", fg="white", font=("Arial", 9, "bold"),
                  relief=tk.RAISED, bd=2, padx=8).pack(side=tk.LEFT, padx=3)
        tk.Button(row2, text="💨 释放过流", command=self._release_overcurrent,
                  bg="#555", fg="white", font=("Arial", 9, "bold"),
                  relief=tk.RAISED, bd=2, padx=8).pack(side=tk.LEFT, padx=3)

        # 旋转限位
        tk.Label(row2, text="  🔄 旋转限位:", fg=self.text_fg, bg=self.panel_bg,
                 font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(20, 10))
        tk.Button(row2, text="正限位", command=lambda: self.sys.rturn_limit_trigger("fwd"),
                  bg="#FF6B6B", fg="white", font=("Arial", 9, "bold"),
                  relief=tk.RAISED, bd=2, padx=8).pack(side=tk.LEFT, padx=3)
        tk.Button(row2, text="反限位", command=lambda: self.sys.rturn_limit_trigger("rev"),
                  bg="#FF6B6B", fg="white", font=("Arial", 9, "bold"),
                  relief=tk.RAISED, bd=2, padx=8).pack(side=tk.LEFT, padx=3)
        tk.Button(row2, text="释放限位", command=lambda: self.sys.rturn_limit_release("all"),
                  bg="#555", fg="white", font=("Arial", 9),
                  relief=tk.RAISED, bd=2, padx=8).pack(side=tk.LEFT, padx=3)

        # === 第3行: 电压 + 硬限位 ===
        row3 = tk.Frame(parent, bg=self.panel_bg)
        row3.pack(fill=tk.X, padx=10, pady=3)

        tk.Label(row3, text="🔌 电压报警:", fg=self.text_fg, bg=self.panel_bg,
                 font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(0, 10))
        tk.Button(row3, text="过压", command=lambda: self.sys.voltage_alarm_trigger("overvoltage"),
                  bg="#FFA500", fg="white", font=("Arial", 9, "bold"),
                  relief=tk.RAISED, bd=2, padx=8).pack(side=tk.LEFT, padx=3)
        tk.Button(row3, text="欠压", command=lambda: self.sys.voltage_alarm_trigger("undervoltage"),
                  bg="#FFD700", fg="black", font=("Arial", 9, "bold"),
                  relief=tk.RAISED, bd=2, padx=8).pack(side=tk.LEFT, padx=3)
        tk.Button(row3, text="释放过压", command=lambda: self.sys.voltage_alarm_release("overvoltage"),
                  bg="#555", fg="white", font=("Arial", 9), padx=8).pack(side=tk.LEFT, padx=3)
        tk.Button(row3, text="释放欠压", command=lambda: self.sys.voltage_alarm_release("undervoltage"),
                  bg="#555", fg="white", font=("Arial", 9), padx=8).pack(side=tk.LEFT, padx=3)

        # 硬限位
        tk.Label(row3, text="  🛑 硬限位:", fg=self.text_fg, bg=self.panel_bg,
                 font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(20, 10))
        tk.Button(row3, text="正限位", command=lambda: self.sys.hardlimit_trigger("fwd"),
                  bg="#d63031", fg="white", font=("Arial", 9, "bold"), padx=8).pack(side=tk.LEFT, padx=3)
        tk.Button(row3, text="反限位", command=lambda: self.sys.hardlimit_trigger("rev"),
                  bg="#d63031", fg="white", font=("Arial", 9, "bold"), padx=8).pack(side=tk.LEFT, padx=3)
        tk.Button(row3, text="释放", command=lambda: self.sys.hardlimit_release("all"),
                  bg="#555", fg="white", font=("Arial", 9), padx=8).pack(side=tk.LEFT, padx=3)

        # === 占空比 ===
        row4 = tk.Frame(parent, bg=self.panel_bg)
        row4.pack(fill=tk.X, padx=10, pady=3)
        tk.Label(row4, text="🎚 IO占空比:", fg=self.text_fg, bg=self.panel_bg,
                 font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(0, 10))
        self.duty_var = tk.IntVar(value=85)
        tk.Scale(row4, from_=10, to=98, orient=tk.HORIZONTAL, length=200,
                 variable=self.duty_var, bg=self.panel_bg, fg=self.green,
                 troughcolor="#333", highlightbackground=self.panel_bg,
                 label="占空比(%)", font=("Arial", 8)).pack(side=tk.LEFT, padx=5)

    # =================================================================
    # 动作
    # =================================================================
    def _trigger_overcurrent(self):
        current_ma = self.overcurrent_var.get()
        self.sys.overcurrent_trigger(current_ma)

    def _release_overcurrent(self):
        self.sys.overcurrent_release()

    def _reset(self):
        self.sys.reset()
        self.log_lines.clear()

    def _emergency_stop(self):
        self.sys.emergency_stop()

    # =================================================================
    # 刷新显示
    # =================================================================
    def _refresh_all(self):
        self._refresh_state()
        self._refresh_queues()
        self.root.after(200, self._refresh_all)

    def _refresh_state(self):
        s = self.sys
        dir_map = {MotorDir.NONE: "⏸ 停止", MotorDir.FWD: "▶ 正转", MotorDir.REV: "◀ 反转"}
        state_map = {MotorState.IDLE: "IDLE", MotorState.RAMPING: "RAMPING",
                     MotorState.RUNNING: "RUNNING", MotorState.LOCKED: "LOCKED"}

        self.state_labels["dir"].config(text=dir_map.get(s.active_dir, "?"))
        self.state_labels["state"].config(text=state_map.get(s.state, "?"))
        self.state_labels["duty"].config(text=f"{s.current_duty:.0f}%")
        dev_name = GENE_MAP.get(s.active_device_id,
                                DeviceGene(s.active_device_id, Priority.NONE, 0, "无", "#888")).name
        self.state_labels["device"].config(text=dev_name)
        self.state_labels["conflict"].config(
            text="⚠ 冲突!" if s.conflict_fault else "✓ 正常",
            fg=self.highlight if s.conflict_fault else self.green
        )

    def _refresh_queues(self):
        s = self.sys
        self._fill_listbox(f"listbox_allow_fwd", s.allow_fwd, self.green)
        self._fill_listbox(f"listbox_allow_rev", s.allow_rev, self.green)
        self._fill_listbox(f"listbox_block_fwd", s.block_fwd, self.highlight)
        self._fill_listbox(f"listbox_block_rev", s.block_rev, self.highlight)

    def _fill_listbox(self, attr: str, q: CommandList, color: str):
        lb = getattr(self, attr)
        lb.delete(0, tk.END)
        if q.empty:
            lb.insert(tk.END, "  (空)")
            lb.itemconfig(0, fg="#555")
        else:
            for i, cmd in enumerate(q.commands):
                dev_name = GENE_MAP.get(cmd.device_id).name if cmd.device_id in GENE_MAP else f"ID={cmd.device_id}"
                text = f"  [{i}] {dev_name} | prio={cmd.priority} | {cmd.type.name}"
                lb.insert(tk.END, text)
                lb.itemconfig(i, fg=color)

    def _update_log_display(self):
        self.log_text.delete(1.0, tk.END)
        recent = self.log_lines[-50:] if len(self.log_lines) > 50 else self.log_lines
        for line in recent:
            self.log_text.insert(tk.END, line + "\n")
        self.log_text.see(tk.END)

    def run(self):
        self.root.mainloop()


# =============================================================================
# 入口
# =============================================================================

if __name__ == "__main__":
    import sys
    import traceback
    import os

    # 捕获所有异常, 防止双击运行时窗口一闪而过
    try:
        print("=" * 60)
        print("  电机仲裁系统模拟器 - Motor Arbitration Simulator")
        print("  基于 dev_motor.c 仲裁逻辑")
        print("=" * 60)
        print()
        print("使用说明:")
        print("  - 点击 IO 按钮模拟手动正转/反转/停止")
        print("  - 拖动电流滑块 → 点击'触发过流'模拟过流事件")
        print("  - 点击限位/电压/硬限位按钮模拟保护事件")
        print("  - 观察4个队列的变化和仲裁结果")
        print("  - 日志区显示所有操作和仲裁决策过程")
        print()
        app = MotorSimulatorGUI()
        app.run()
    except Exception as e:
        err_msg = f"程序崩溃: {e}\n\n{traceback.format_exc()}"
        # 写入日志文件
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_error.log")
        with open(log_path, "w", encoding="utf-8") as f:
            f.write(err_msg)
        print(err_msg)
        print(f"\n错误已写入: {log_path}")
        input("\n按 Enter 键退出...")
