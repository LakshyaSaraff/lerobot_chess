#!/usr/bin/env python3
"""
=============================================================================
 STEP 1 — Understanding the Feetech STS3215 Serial Protocol (from scratch)
=============================================================================

WHAT IS THE STS3215?
    The Feetech STS3215 is a "smart servo" — it has a motor, a gear-box, a
    12-bit magnetic encoder (4096 steps/revolution), and a tiny MCU all in
    one package.  Unlike a hobby RC servo that only understands a PWM pulse
    width, the STS3215 speaks a *packet-based serial protocol* over a single
    half-duplex UART wire.

    "Half-duplex" means TX and RX share the same physical wire.  The USB
    adapter (the little green board that came with your kit) handles the
    TX/RX switching for you — you just open a normal serial port on your PC.

WHY WRITE RAW PACKETS?
    The official `feetech-servo-sdk` is a thin wrapper around pyserial that
    builds these same packets.  Writing them by hand first gives you a deep
    understanding of *exactly* what travels on the wire, so when something
    breaks you can debug it with a logic analyser or a simple hex dump.

THE PACKET FORMAT (almost identical to Dynamixel Protocol 1.0):
    ┌──────┬──────┬────┬────────┬─────────────┬───────────┬──────────┐
    │ 0xFF │ 0xFF │ ID │ LENGTH │ INSTRUCTION │ PARAMs... │ CHECKSUM │
    └──────┴──────┴────┴────────┴─────────────┴───────────┴──────────┘

    • Header     : always 0xFF 0xFF — lets the servo sync to the start.
    • ID         : which servo (1-252), or 0xFE = broadcast (all servos).
    • LENGTH     : number of remaining bytes = len(params) + 2.
    • INSTRUCTION: what to do (PING=0x01, READ=0x02, WRITE=0x03, …).
    • PARAMs     : depends on the instruction.
    • CHECKSUM   : ~(ID + LENGTH + INSTRUCTION + Σparams) & 0xFF
                   (bitwise NOT of the byte-sum, masked to one byte).

    The servo replies with a STATUS packet that has the same shape, but
    byte 4 is an ERROR bitfield instead of an instruction.
=============================================================================
"""

import serial
import time
import struct

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTS — you'll reuse these throughout every script
# ─────────────────────────────────────────────────────────────────────────────

PORT = "/dev/ttyACM1"          # ← your USB-to-TTL adapter; may be ttyUSB0
BAUDRATE = 1_000_000           # STS3215 ships at 1 Mbps
TIMEOUT = 1.0                  # seconds to wait for a reply

# Instruction opcodes (host → servo)
INST_PING       = 0x01
INST_READ       = 0x02
INST_WRITE      = 0x03
INST_REG_WRITE  = 0x04         # buffered write (not used yet)
INST_ACTION     = 0x05         # trigger buffered writes
INST_SYNC_READ  = 0x82         # read from many servos in one packet
INST_SYNC_WRITE = 0x83         # write to many servos in one packet

# ─────────────────────────────────────────────────────────────────────────────
# STS3215 CONTROL TABLE — think of this as the servo's "register map"
# ─────────────────────────────────────────────────────────────────────────────
# Each entry is (address, size_in_bytes).
# EPROM values survive power-off; SRAM values reset every power cycle.

# --- EPROM (non-volatile) ---
ADDR_MODEL_NUMBER       = (3, 2)    # read-only  — should return 777
ADDR_ID                 = (5, 1)    # default 1
ADDR_BAUD_RATE          = (6, 1)    # 0→1M, 1→500k, 2→250k ...
ADDR_RETURN_DELAY       = (7, 1)    # µs before servo replies
ADDR_MIN_POS_LIMIT      = (9, 2)
ADDR_MAX_POS_LIMIT      = (11, 2)   # default 4095
ADDR_MAX_TEMP_LIMIT     = (13, 1)
ADDR_OPERATING_MODE     = (33, 1)   # 0=pos, 1=vel, 2=pwm, 3=step

# --- SRAM (volatile) ---
ADDR_TORQUE_ENABLE      = (40, 1)   # 0=free, 1=holding
ADDR_ACCELERATION       = (41, 1)   # profile acceleration
ADDR_GOAL_POSITION      = (42, 2)   # where you want it to go (0-4095)
ADDR_GOAL_SPEED         = (46, 2)   # how fast to get there
ADDR_TORQUE_LIMIT       = (48, 2)
ADDR_LOCK               = (55, 1)   # 1 = lock EPROM from accidental writes
ADDR_PRESENT_POSITION   = (56, 2)   # where it actually is right now
ADDR_PRESENT_SPEED      = (58, 2)
ADDR_PRESENT_LOAD       = (60, 2)
ADDR_PRESENT_VOLTAGE    = (62, 1)
ADDR_PRESENT_TEMPERATURE = (63, 1)
ADDR_PRESENT_CURRENT    = (69, 2)


# ─────────────────────────────────────────────────────────────────────────────
# HELPER: compute the checksum
# ─────────────────────────────────────────────────────────────────────────────
def checksum(packet_body: list[int]) -> int:
    """
    The servo's checksum is simply:
        ~(sum of all bytes from ID through the last param) & 0xFF

    `packet_body` should be [ID, LENGTH, INSTRUCTION, param0, param1, ...].
    """
    return (~sum(packet_body)) & 0xFF


# ─────────────────────────────────────────────────────────────────────────────
# HELPER: build an instruction packet
# ─────────────────────────────────────────────────────────────────────────────
def build_packet(servo_id: int, instruction: int, params: list[int] = None) -> bytes:
    """
    Builds a complete instruction packet ready to send over the wire.

    Example — PING servo 1:
        build_packet(1, INST_PING)
        → b'\\xff\\xff\\x01\\x02\\x01\\xfb'
          header  ID  LEN PING CHECKSUM
    """
    if params is None:
        params = []
    length = len(params) + 2                     # +1 instruction, +1 checksum
    body = [servo_id, length, instruction] + params
    chk = checksum(body)
    return bytes([0xFF, 0xFF] + body + [chk])


# ─────────────────────────────────────────────────────────────────────────────
# HELPER: open the serial port
# ─────────────────────────────────────────────────────────────────────────────
def open_port(port: str = PORT, baudrate: int = BAUDRATE) -> serial.Serial:
    """
    Opens the serial port that talks to the servo bus.

    WHY these settings?
      • 8N1 is the only framing the servo understands.
      • timeout=TIMEOUT so reads don't hang forever if a servo doesn't reply.
      • We flush both buffers to start clean.
    """
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT,
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


# ─────────────────────────────────────────────────────────────────────────────
# HELPER: send a packet and read the status reply
# ─────────────────────────────────────────────────────────────────────────────
def send_and_receive(ser: serial.Serial, packet: bytes,
                     expected_params: int = 0) -> dict:
    """
    Sends `packet` and reads the servo's status response.

    The status packet looks like:
        [0xFF][0xFF][ID][LEN][ERROR][param0...paramN][CHECKSUM]

    Returns a dict:
        {"id": int, "error": int, "params": list[int]}

    WHY flush first?
        Half-duplex means the wire was just driven by us.  There may be
        stale echo bytes sitting in the RX buffer.  Flushing avoids
        misaligning our read.
    """
    ser.reset_input_buffer()      # throw away any stale data
    ser.write(packet)
    ser.flush()                   # make sure every byte is on the wire

    # Status packet minimum = 6 bytes (header + ID + len + error + checksum)
    # Extra bytes come from any returned data (e.g., a 2-byte position read)
    expected_length = 6 + expected_params
    response = ser.read(expected_length)

    if len(response) < 6:
        raise TimeoutError(
            f"Expected {expected_length} bytes, got {len(response)}.  "
            "Is the servo powered?  Is the ID correct?"
        )

    # Validate header
    if response[0] != 0xFF or response[1] != 0xFF:
        raise ValueError(f"Bad header: {response[:2].hex()}")

    resp_id    = response[2]
    resp_len   = response[3]
    resp_error = response[4]
    
    # resp_len is the number of bytes after the LENGTH field (error + params + checksum)
    # So number of param bytes = resp_len - 2 (minus error byte and checksum)
    num_params = resp_len - 2
    resp_params = list(response[5 : 5 + num_params])
    resp_chk   = response[5 + num_params]

    # Verify checksum
    calc_body = list(response[2 : 5 + num_params])  # ID through last param
    expected_chk = checksum(calc_body)
    if resp_chk != expected_chk:
        raise ValueError(
            f"Checksum mismatch: got 0x{resp_chk:02X}, "
            f"expected 0x{expected_chk:02X}"
        )

    if resp_error != 0:
        _decode_error(resp_error)

    return {"id": resp_id, "error": resp_error, "params": resp_params}


def _decode_error(error_byte: int):
    """Print human-readable warnings for each error flag."""
    flags = {
        0x01: "Input Voltage Error",
        0x02: "Angle Sensor Error",
        0x04: "Overheat Error",
        0x08: "Over-Current / Electrical Error",
        0x20: "Overload Error",
    }
    msgs = [msg for bit, msg in flags.items() if error_byte & bit]
    if msgs:
        print(f"  ⚠ Servo error flags: {', '.join(msgs)}")


# ─────────────────────────────────────────────────────────────────────────────
# SIGN-MAGNITUDE ENCODING — STS3215 does NOT use two's complement!
# ─────────────────────────────────────────────────────────────────────────────
def to_signed(raw: int, sign_bit: int = 15) -> int:
    """
    Decode a sign-magnitude value from the servo.

    The STS3215 stores negative values by setting a specific bit to 1
    and storing the absolute value in the remaining bits.
    For position/speed the sign bit is bit 15 (of a 16-bit word).
    For load, it's bit 10.

    Example:
        raw = 0x8064  →  bit 15 is set  →  -(0x0064) = -100
    """
    if raw & (1 << sign_bit):
        return -(raw & ~(1 << sign_bit))
    return raw


def from_signed(value: int, sign_bit: int = 15) -> int:
    """
    Encode a signed value into the servo's sign-magnitude format.

    Example:
        from_signed(-100) → 0x8064  (bit 15 set, magnitude 100)
    """
    if value < 0:
        return (-value) | (1 << sign_bit)
    return value


# ─────────────────────────────────────────────────────────────────────────────
# CORE FUNCTIONS: ping / read / write
# ─────────────────────────────────────────────────────────────────────────────
def ping(ser: serial.Serial, servo_id: int) -> int:
    """
    PING — the simplest command.  Asks "are you there?" and the servo
    replies with just an ACK (no data).

    To get the model number, we follow up with a READ of address 3.

    WHY start here?
        If ping works, your wiring, baud rate, and servo ID are all correct.
        It's the fastest way to verify the entire communication chain.

    Returns the model number (777 for STS3215).
    """
    pkt = build_packet(servo_id, INST_PING)
    print(f"  → TX: {pkt.hex(' ')}")
    resp = send_and_receive(ser, pkt, expected_params=0)  # PING doesn't return data
    print(f"  ← RX: ID={resp['id']}, error={resp['error']} (PING OK)")

    # Now READ the model number from address 3
    print(f"  Reading model number from address 3...")
    model = read_register(ser, servo_id, 3, 2)  # Address 3, 2 bytes
    return model


def read_register(ser: serial.Serial, servo_id: int,
                  addr: int, length: int) -> int:
    """
    READ — fetch `length` bytes starting at `addr` from the servo's
    control table.

    HOW IT WORKS:
        We send: [READ instruction] [start_address] [num_bytes]
        Servo replies with the requested bytes as parameters.

    Returns an integer (1-byte or 2-byte value, little-endian).
    """
    pkt = build_packet(servo_id, INST_READ, [addr, length])
    resp = send_and_receive(ser, pkt, expected_params=length)
    if length == 1:
        return resp["params"][0]
    elif length == 2:
        return resp["params"][0] | (resp["params"][1] << 8)
    else:
        return resp["params"]


def write_register(ser: serial.Serial, servo_id: int,
                   addr: int, length: int, value: int):
    """
    WRITE — set `length` bytes starting at `addr` in the servo's
    control table.

    HOW IT WORKS:
        We send: [WRITE instruction] [start_address] [byte0] [byte1]...
        For 2-byte values we split into little-endian (low byte first).

    This is how you move a motor, change its speed, enable torque, etc.
    """
    if length == 1:
        data = [value & 0xFF]
    elif length == 2:
        data = [value & 0xFF, (value >> 8) & 0xFF]
    else:
        raise ValueError("Only 1 or 2 byte writes supported here")

    pkt = build_packet(servo_id, INST_WRITE, [addr] + data)
    resp = send_and_receive(ser, pkt)
    return resp


# ═══════════════════════════════════════════════════════════════════
#  HIGH-LEVEL MOTOR HELPERS
# ═══════════════════════════════════════════════════════════════════

def enable_torque(ser: serial.Serial, servo_id: int):
    """
    Turn on the motor driver.

    WHY do we need this?
        By default the servo is in "free" mode — you can spin it by hand.
        Writing 1 to TORQUE_ENABLE powers the H-bridge so the motor
        actively holds (or moves to) the Goal_Position.

    We also set LOCK=1 so that EPROM registers can't be accidentally
    overwritten while the motor is active.
    """
    write_register(ser, servo_id, *ADDR_TORQUE_ENABLE, 1)
    write_register(ser, servo_id, *ADDR_LOCK, 1)
    print(f"  ✓ Servo {servo_id}: torque ON, EPROM locked")


def disable_torque(ser: serial.Serial, servo_id: int):
    """
    Turn off the motor driver — the servo becomes free-spinning.

    We unlock EPROM first (LOCK=0), then disable torque.
    """
    write_register(ser, servo_id, *ADDR_LOCK, 0)
    write_register(ser, servo_id, *ADDR_TORQUE_ENABLE, 0)
    print(f"  ✓ Servo {servo_id}: torque OFF, EPROM unlocked")


def read_position(ser: serial.Serial, servo_id: int) -> int:
    """
    Read the current shaft position (0–4095 = one full revolution).

    The value uses sign-magnitude encoding (bit 15 is the sign).
    In normal position mode you'll see 0–4095.
    Negative values appear if the motor is configured with an offset.
    """
    raw = read_register(ser, servo_id, *ADDR_PRESENT_POSITION)
    return to_signed(raw, sign_bit=15)


def set_position(ser: serial.Serial, servo_id: int, position: int,
                 speed: int = 0):
    """
    Command the servo to move to `position` (0–4095).

    `speed` sets how fast:
        0   = maximum speed (no speed limit)
        1+  = steps/sec (higher = faster)

    We encode through sign-magnitude in case you need negative positions
    (homing offset scenarios).
    """
    enc_pos = from_signed(position, sign_bit=15)
    write_register(ser, servo_id, *ADDR_GOAL_POSITION, enc_pos)
    if speed > 0:
        enc_spd = from_signed(speed, sign_bit=15)
        write_register(ser, servo_id, *ADDR_GOAL_SPEED, enc_spd)


def read_status(ser: serial.Serial, servo_id: int) -> dict:
    """
    Read a snapshot of the servo's live telemetry.

    Returns a dict with position, speed, load, voltage, and temperature.
    Great for debugging — call this in a loop to watch the servo move.
    """
    pos  = to_signed(read_register(ser, servo_id, *ADDR_PRESENT_POSITION), 15)
    spd  = to_signed(read_register(ser, servo_id, *ADDR_PRESENT_SPEED), 15)
    load = to_signed(read_register(ser, servo_id, *ADDR_PRESENT_LOAD), 10)
    volt = read_register(ser, servo_id, *ADDR_PRESENT_VOLTAGE)
    temp = read_register(ser, servo_id, *ADDR_PRESENT_TEMPERATURE)
    return {
        "position": pos,
        "speed": spd,
        "load": load,
        "voltage_dV": volt,      # in deci-volts (e.g., 74 = 7.4 V)
        "temperature_C": temp,
    }


# ═══════════════════════════════════════════════════════════════════
#  MAIN — Run this script directly to test a single servo
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    SERVO_ID = 1          # ← change this to match your motor's ID

    print("=" * 60)
    print(" STEP 1:  Open serial port")
    print("=" * 60)
    ser = open_port()
    print(f"  Opened {PORT} @ {BAUDRATE} baud\n")

    # ── PING ──────────────────────────────────────────────────────
    print("=" * 60)
    print(" STEP 2:  PING — verify the servo is alive")
    print("=" * 60)
    try:
        model = ping(ser, SERVO_ID)
        #print("model",model)
        print(f"  ✓ Model number = {model}  "
              f"{'(STS3215!)' if model == 777 else '(unexpected)'}\n")
    except TimeoutError as e:
        print(f"  ✗ {e}")
        print("    → Check: power supply, wiring, servo ID, baud rate")
        ser.close()
        exit(1)

    # ── READ STATUS ───────────────────────────────────────────────
    print("=" * 60)
    print(" STEP 3:  READ — current servo status")
    print("=" * 60)
    status = read_status(ser, SERVO_ID)
    for k, v in status.items():
        print(f"  {k:>20s} = {v}")
    print()

    # ── MOVE ──────────────────────────────────────────────────────
    print("=" * 60)
    print(" STEP 4:  WRITE — move the servo")
    print("=" * 60)
    target = 3000          # mid-point of 0-4095 range
    print(f"  Enabling torque on servo {SERVO_ID}...")
    enable_torque(ser, SERVO_ID)
    print(f"  Commanding position → {target} (speed=200)...")
    set_position(ser, SERVO_ID, target, speed=200)

    # Wait and poll position until it arrives
    for _ in range(510):
        pos = read_position(ser, SERVO_ID)
        print(f"    position = {pos}", end="\r")
        print ("just outside the loop")
        if abs(pos - target) < 10:
            print("print loop entered")
            print("this is pos, target", pos, target)
            break
        time.sleep(0.05)
    print(f"\n  ✓ Reached position {read_position(ser, SERVO_ID)}\n")

    # ── CLEANUP ───────────────────────────────────────────────────
    print("=" * 60)
    print(" STEP 5:  Disable torque (servo goes limp)")
    print("=" * 60)
    disable_torque(ser, SERVO_ID)

    ser.close()
    print("\n  Done!  Port closed.")
