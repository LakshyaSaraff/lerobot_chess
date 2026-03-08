 #!/usr/bin/env python3
"""
=============================================================================
 STEP 2 — Multi-Motor Arm Control  (SO-100 / SO-101 style 6-DOF arm)
=============================================================================

WHY THIS FILE?
    motor_control.py talks to ONE servo at a time.  A real robot arm has 6
    servos on the same serial bus.  This file adds:

    1.  A high-level Arm class that groups 6 motors by name + ID.
    2.  SYNC_WRITE — one packet that commands all 6 motors at once.
        (Much faster than 6 individual writes — critical for smooth motion.)
    3.  Bulk read — reads all positions in a tight loop.
    4.  Safety limits so you don't accidentally command impossible angles.

SERVO LAYOUT  (standard SO-100 LeRobot arm):
    ┌─────────────────┬────┬──────────────────────────────────┐
    │ Name            │ ID │ Role                             │
    ├─────────────────┼────┼──────────────────────────────────┤
    │ shoulder_pan     │  1 │ Rotate the whole arm left/right  │
    │ shoulder_lift    │  2 │ Lift the upper arm up/down       │
    │ elbow_flex       │  3 │ Bend at the elbow                │
    │ wrist_flex       │  4 │ Tilt the wrist up/down           │
    │ wrist_roll       │  5 │ Rotate the wrist                 │
    │ gripper          │  6 │ Open/close the fingers           │
    └─────────────────┴────┴──────────────────────────────────┘

    Encoder range: 0 – 4095 (one full 360° revolution)
    Mid-point:  2048  (≈ 180°, usually the "neutral" pose)
=============================================================================
"""

import time
from dataclasses import dataclass, field

# Import everything we built in Step 1
from motor_control import (
    open_port, ping, read_position, set_position, read_status,
    enable_torque, disable_torque, write_register, read_register,
    build_packet, send_and_receive, from_signed, to_signed, checksum,
    INST_SYNC_WRITE, ADDR_GOAL_POSITION, ADDR_GOAL_SPEED,
    ADDR_PRESENT_POSITION, ADDR_PRESENT_LOAD, ADDR_TORQUE_ENABLE,
    ADDR_LOCK, ADDR_ACCELERATION, ADDR_OPERATING_MODE,
    PORT, BAUDRATE,
)

# ─────────────────────────────────────────────────────────────────────────────
# ARM CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class JointConfig:
    """
    Configuration for a single joint (motor).

    WHY have limits?
        Without software limits you could command a position that physically
        jams the arm against itself, stalling the motor and tripping the
        overload protection.  Defining safe ranges here prevents that.
    """
    name: str
    motor_id: int
    min_pos: int = 0        # lower software limit (encoder ticks)
    max_pos: int = 4095     # upper software limit
    home_pos: int = 2048    # "neutral" position


# Default joint layout for a SO-100 style arm
DEFAULT_JOINTS = [
    JointConfig("shoulder_pan",  1, min_pos=500,  max_pos=3500, home_pos=2048),
    JointConfig("shoulder_lift", 2, min_pos=1000, max_pos=3100, home_pos=2048),
    JointConfig("elbow_flex",    3, min_pos=1000, max_pos=3100, home_pos=2048),
    JointConfig("wrist_flex",    4, min_pos=600,  max_pos=3500, home_pos=2048),
    JointConfig("wrist_roll",    5, min_pos=500,  max_pos=3500, home_pos=2048),
    JointConfig("gripper",       6, min_pos=1500, max_pos=3000, home_pos=2048),
]


# ─────────────────────────────────────────────────────────────────────────────
# SYNC_WRITE — the key to smooth multi-motor motion
# ─────────────────────────────────────────────────────────────────────────────

def sync_write_positions(ser, motor_ids: list[int], positions: list[int]):
    """
    Write Goal_Position to many servos in a SINGLE packet.

    WHY sync write?
        If you send 6 individual WRITE commands, there's a small delay
        between each one — the first motor starts moving before the last
        one gets its command.  The arm looks jerky.

        SYNC_WRITE broadcasts one fat packet that contains data for ALL
        motors.  Every servo picks out its own data and starts moving
        simultaneously.  Result: smooth, coordinated motion.

    PACKET STRUCTURE:
        [0xFF][0xFF][0xFE][LEN][SYNC_WRITE][START_ADDR][DATA_LEN]
           [ID_1][data1_lo][data1_hi]
           [ID_2][data2_lo][data2_hi]
           ...
        [CHECKSUM]

        • ID = 0xFE (broadcast — every servo receives this)
        • START_ADDR = 42 (Goal_Position address)
        • DATA_LEN = 2 (each motor gets 2 bytes of data)
        • Each motor's block: [its_ID, low_byte, high_byte]
    """
    start_addr = ADDR_GOAL_POSITION[0]   # 42
    data_per_motor = ADDR_GOAL_POSITION[1]  # 2 bytes

    # Build the parameter block
    params = [start_addr, data_per_motor]
    for mid, pos in zip(motor_ids, positions):
        enc = from_signed(pos, sign_bit=15)
        lo = enc & 0xFF
        hi = (enc >> 8) & 0xFF
        params.extend([mid, lo, hi])

    # SYNC_WRITE uses broadcast ID (0xFE) and expects NO status reply
    pkt = build_packet(0xFE, INST_SYNC_WRITE, params)

    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    # No response — broadcast packets never get a reply


def read_all_positions(ser, motor_ids: list[int]) -> dict[int, int]:
    """
    Read Present_Position from every motor, one at a time.

    NOTE: The SDK supports SYNC_READ (opcode 0x82), but it requires
    careful bus timing.  For learning purposes, sequential reads are
    simpler and perfectly fine at 1 Mbps (each read ≈ 0.3 ms).
    """
    positions = {}
    for mid in motor_ids:
        raw = read_register(ser, mid, *ADDR_PRESENT_POSITION)
        positions[mid] = to_signed(raw, sign_bit=15)
    return positions


# ─────────────────────────────────────────────────────────────────────────────
# ARM CLASS — ties everything together
# ─────────────────────────────────────────────────────────────────────────────

class RobotArm:
    """
    High-level controller for a multi-servo robot arm.

    Usage:
        arm = RobotArm()       # scans & pings all motors
        arm.enable()           # torque on
        arm.go_home()          # move to neutral
        arm.set_joint("elbow_flex", 1500)  # move one joint
        arm.disable()          # torque off
    """

    def __init__(self, port: str = PORT, baudrate: int = BAUDRATE,
                 joints: list[JointConfig] = None):
        self.joints = joints or DEFAULT_JOINTS
        self.ser = open_port(port, baudrate)
        self._verify_motors()
        self._configure_motors()

    # ── discovery ─────────────────────────────────────────────────
    def _verify_motors(self):
        """Ping each motor to make sure it's on the bus."""
        print("Scanning motors...")
        for j in self.joints:
            try:
                model = ping(self.ser, j.motor_id)
                print(f"  ✓ {j.name} (ID {j.motor_id}): model {model}")
            except TimeoutError:
                print(f"  ✗ {j.name} (ID {j.motor_id}): NOT FOUND")
                raise RuntimeError(
                    f"Motor '{j.name}' (ID {j.motor_id}) did not respond.  "
                    "Check power and wiring."
                )

    def _configure_motors(self):
        """
        Apply recommended settings to each motor.

        WHY these values?
        • Acceleration=254: smoothest possible ramp-up — avoids jerky
          starts that could damage the gears or knock the arm over.
        • Operating_Mode=0: position servo mode (the default, but we
          set it explicitly to be safe).
        """
        for j in self.joints:
            write_register(self.ser, j.motor_id, *ADDR_ACCELERATION, 254)
            write_register(self.ser, j.motor_id, *ADDR_OPERATING_MODE, 0)
        print("  Motors configured (accel=254, mode=position)\n")

    # ── torque ────────────────────────────────────────────────────
    def enable(self):
        """Enable torque on all motors (they will hold position)."""
        for j in self.joints:
            enable_torque(self.ser, j.motor_id)

    def disable(self):
        """Disable torque on all motors (they go limp)."""
        for j in self.joints:
            disable_torque(self.ser, j.motor_id)

    # ── read ──────────────────────────────────────────────────────
    def get_positions(self) -> dict[str, int]:
        """
        Returns {joint_name: encoder_position} for every joint.
        """
        raw = read_all_positions(self.ser,
                                 [j.motor_id for j in self.joints])
        return {j.name: raw[j.motor_id] for j in self.joints}

    def get_joint_position(self, name: str) -> int:
        j = self._joint_by_name(name)
        return to_signed(
            read_register(self.ser, j.motor_id, *ADDR_PRESENT_POSITION), 15
        )

    def get_joint_load(self, name: str) -> int:
        """
        Read the current load (torque) from the servo.
        A value of ~0 means no load.
        Positive/Negative values indicate direction of force.
        Range is roughly -1000 to +1000 for normal operation.
        """
        j = self._joint_by_name(name)
        # Load is at address 60, 2 bytes, 10-bit sign-magnitude (bit 10 is sign)
        # However, our read_status helper uses sign_bit=10
        raw = read_register(self.ser, j.motor_id, *ADDR_PRESENT_LOAD)
        return to_signed(raw, sign_bit=10)

    def print_status(self):
        """Pretty-print all joint positions."""
        positions = self.get_positions()
        print("┌─────────────────┬──────────┐")
        print("│ Joint           │ Position │")
        print("├─────────────────┼──────────┤")
        for name, pos in positions.items():
            print(f"│ {name:<15s} │ {pos:>8d} │")
        print("└─────────────────┴──────────┘")

    # ── move ──────────────────────────────────────────────────────
    def set_joint(self, name: str, position: int, speed: int = 0):
        """
        Move a single joint to `position`.

        The position is clamped to the joint's [min_pos, max_pos] so you
        can't accidentally go out of bounds.
        """
        j = self._joint_by_name(name)
        position = max(j.min_pos, min(j.max_pos, position))
        set_position(self.ser, j.motor_id, position, speed)

    def set_all_joints(self, positions: dict[str, int]):
        """
        Move all joints simultaneously using SYNC_WRITE.

        `positions` = {"shoulder_pan": 2048, "elbow_flex": 1500, ...}
        Any joint not in the dict keeps its current goal.
        """
        motor_ids = []
        motor_positions = []
        for j in self.joints:
            if j.name in positions:
                pos = max(j.min_pos, min(j.max_pos, positions[j.name]))
                motor_ids.append(j.motor_id)
                motor_positions.append(pos)
        if motor_ids:
            sync_write_positions(self.ser, motor_ids, motor_positions)

    def go_home(self, speed: int = 300):
        """
        Move every joint to its home (neutral) position.

        WHY have a "home" pose?
            It's a known-safe configuration — no joint is near its limit,
            the arm is roughly centered, and it's a good starting point
            for any task.
        """
        print("Moving to home position...")
        for j in self.joints:
            set_position(self.ser, j.motor_id, j.home_pos, speed)
        # Wait for motion to finish
        time.sleep(2.0)
        self.print_status()

    # ── incremental move (used by keyboard teleop) ────────────────
    def nudge_joint(self, name: str, delta: int):
        """
        Move a joint by `delta` ticks relative to its current position.

        This is the building block for keyboard teleoperation —
        each key press calls nudge_joint with a small positive or
        negative delta.
        """
        current = self.get_joint_position(name)
        self.set_joint(name, current + delta)

    # ── helpers ───────────────────────────────────────────────────
    def _joint_by_name(self, name: str) -> JointConfig:
        for j in self.joints:
            if j.name == name:
                return j
        raise ValueError(
            f"Unknown joint '{name}'.  "
            f"Valid: {[j.name for j in self.joints]}"
        )

    def close(self):
        """Disable torque and close the serial port."""
        try:
            self.disable()
        except Exception:
            pass
        self.ser.close()
        print("Port closed.")


# ═══════════════════════════════════════════════════════════════════
#  MAIN — test the arm
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    arm = RobotArm()

    print("\n" + "=" * 60)
    print(" Current positions:")
    print("=" * 60)
    arm.print_status()

    print("\n" + "=" * 60)
    print(" Enabling torque & going home...")
    print("=" * 60)
    arm.enable()
    arm.go_home()

    print("\n" + "=" * 60)
    print(" Nudging elbow_flex by +200 ticks...")
    print("=" * 60)
    arm.nudge_joint("elbow_flex", 200)
    time.sleep(1)
    arm.print_status()

    print("\n" + "=" * 60)
    print(" Disabling torque...")
    print("=" * 60)
    arm.disable()
    arm.close()
