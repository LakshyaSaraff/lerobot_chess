#!/usr/bin/env python3
"""
=============================================================================
 STEP 3 — Keyboard Teleoperation
=============================================================================

WHAT IS TELEOPERATION?
    "Teleop" means remote operation — you control the robot in real time.
    Here we map keyboard keys to joint motions so you can move each joint
    of the arm by pressing keys.

HOW IT WORKS:
    1.  We use Python's `curses` library to capture individual key presses
        without waiting for Enter (this is called "raw" or "cbreak" mode).
    2.  Each joint has two keys: one to move it in the + direction,
        one to move it in the – direction.
    3.  Each key press calls `arm.nudge_joint(name, ±step)` which:
        a) Reads the joint's current position
        b) Adds/subtracts `step` ticks
        c) Writes the new Goal_Position

    The loop runs as fast as it can — at 1 Mbps serial, each nudge takes
    about 1-2 ms, so you get very responsive control.

KEY MAP (default):
    ┌────────┬──────────────────┬───────────────┐
    │ Key    │ Joint            │ Direction     │
    ├────────┼──────────────────┼───────────────┤
    │ q / a  │ shoulder_pan     │ + / –         │
    │ w / s  │ shoulder_lift    │ + / –         │
    │ e / d  │ elbow_flex       │ + / –         │
    │ r / f  │ wrist_flex       │ + / –         │
    │ t / g  │ wrist_roll       │ + / –         │
    │ y / h  │ gripper          │ + / – (open/close) │
    ├────────┼──────────────────┼───────────────┤
    │ SPACE  │ all joints       │ go to home    │
    │ p      │ —                │ print status  │
    │ +/-    │ —                │ increase/decrease step size │
    │ ESC    │ —                │ quit          │
    └────────┴──────────────────┴───────────────┘

SAFETY:
    • Torque is enabled when the script starts.
    • All moves are clamped to each joint's [min_pos, max_pos].
    • Pressing ESC disables torque before exiting.
    • Step size starts small (30 ticks ≈ 2.6°) for precise control.

RUNNING:
    cd /home/lakshya/lerobot/src/lerobot/code_self
    python keyboard_teleop.py

    Make sure your terminal window has focus!  curses captures keys from
    the terminal, not from VS Code's editor pane.
=============================================================================
"""

import curses
import time
import sys

from arm_control import RobotArm, DEFAULT_JOINTS

# ─────────────────────────────────────────────────────────────────────────────
# KEY BINDINGS — maps (key_char) → (joint_name, direction)
# ─────────────────────────────────────────────────────────────────────────────

KEY_MAP = {
    # joint_name          +key   -key
    "shoulder_pan":      ("q",   "a"),
    "shoulder_lift":     ("w",   "s"),
    "elbow_flex":        ("e",   "d"),
    "wrist_flex":        ("r",   "f"),
    "wrist_roll":        ("t",   "g"),
    "gripper":           ("y",   "h"),
}

# Invert the map for fast lookup: key_char → (joint_name, sign)
_KEYCHAR_TO_ACTION: dict[str, tuple[str, int]] = {}
for joint, (plus_key, minus_key) in KEY_MAP.items():
    _KEYCHAR_TO_ACTION[plus_key]  = (joint, +1)
    _KEYCHAR_TO_ACTION[minus_key] = (joint, -1)


# ─────────────────────────────────────────────────────────────────────────────
# TELEOP DISPLAY
# ─────────────────────────────────────────────────────────────────────────────

def draw_ui(stdscr, arm: RobotArm, step: int, last_action: str,
            fps: float):
    """
    Draw the live teleoperation UI using curses.

    WHY curses?
        • It lets us redraw specific lines without scrolling garbage.
        • We can capture single key presses without Enter.
        • It handles terminal resize gracefully.
    """
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    # Title
    title = " 🤖  KEYBOARD TELEOPERATION  —  LeRobot Arm "
    stdscr.addstr(0, 0, "═" * min(w - 1, 60))
    stdscr.addstr(1, 0, title[:w - 1], curses.A_BOLD)
    stdscr.addstr(2, 0, "═" * min(w - 1, 60))

    # Joint positions
    row = 4
    stdscr.addstr(row, 0, "Joint Positions:", curses.A_UNDERLINE)
    row += 1

    positions = arm.get_positions()
    for joint_name, (plus_key, minus_key) in KEY_MAP.items():
        pos = positions.get(joint_name, "???")
        # Find the joint config for limits
        jcfg = arm._joint_by_name(joint_name)

        # Draw a mini bar showing where the position is within limits
        bar_width = 20
        if isinstance(pos, int):
            ratio = (pos - jcfg.min_pos) / max(1, jcfg.max_pos - jcfg.min_pos)
            ratio = max(0.0, min(1.0, ratio))
            filled = int(ratio * bar_width)
            bar = "█" * filled + "░" * (bar_width - filled)
        else:
            bar = "?" * bar_width

        line = (f"  [{plus_key.upper()}/{ minus_key.upper()}]  "
                f"{joint_name:<16s}  {pos:>5}  {bar}  "
                f"[{jcfg.min_pos}–{jcfg.max_pos}]")
        stdscr.addstr(row, 0, line[:w - 1])
        row += 1

    # Controls
    row += 1
    stdscr.addstr(row, 0, "Controls:", curses.A_UNDERLINE)
    row += 1
    controls = [
        f"  Step size: {step} ticks  (+ / – to change)",
        "  SPACE  = go to home position",
        "  P      = print full status",
        "  ESC    = quit (disables torque)",
    ]
    for line in controls:
        stdscr.addstr(row, 0, line[:w - 1])
        row += 1

    # Status bar
    row += 1
    stdscr.addstr(row, 0, f"  Last: {last_action:<40s}  FPS: {fps:.0f}")

    stdscr.refresh()


# ─────────────────────────────────────────────────────────────────────────────
# MAIN TELEOP LOOP
# ─────────────────────────────────────────────────────────────────────────────

def teleop_main(stdscr):
    """
    The main teleoperation loop, run inside curses.wrapper().

    FLOW:
        1. Initialize arm (pings all motors, configures them).
        2. Enable torque.
        3. Loop forever:
           a. Read a key press.
           b. If it maps to a joint, nudge that joint.
           c. Redraw the UI with live positions.
        4. On ESC: disable torque, close port, exit.
    """
    # ── curses setup ──────────────────────────────────────────────
    curses.curs_set(0)             # hide cursor
    stdscr.nodelay(True)           # non-blocking getch()
    stdscr.timeout(50)             # refresh at ~20 Hz even if no key

    # ── init arm ──────────────────────────────────────────────────
    # (curses has taken over the terminal, so print() won't work —
    #  we briefly show a "connecting" message)
    stdscr.addstr(0, 0, "Connecting to arm...")
    stdscr.refresh()

    try:
        arm = RobotArm()
    except Exception as e:
        curses.endwin()
        print(f"\n✗ Failed to connect: {e}")
        sys.exit(1)

    arm.enable()

    # ── state ─────────────────────────────────────────────────────
    step = 30                      # ticks per key press (≈ 2.6°)
    last_action = "Ready"
    fps = 0.0
    last_time = time.time()

    try:
        while True:
            # Measure loop rate
            now = time.time()
            dt = now - last_time
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)   # smoothed FPS
            last_time = now

            # Draw the UI
            try:
                draw_ui(stdscr, arm, step, last_action, fps)
            except curses.error:
                pass  # terminal too small — just skip this frame

            # Read key
            key = stdscr.getch()
            if key == -1:
                continue           # no key pressed this frame

            ch = chr(key) if 0 <= key < 256 else ""

            # ── ESC → quit ────────────────────────────────────────
            if key == 27:
                last_action = "Exiting..."
                break

            # ── SPACE → home ──────────────────────────────────────
            if ch == " ":
                arm.go_home(speed=300)
                last_action = "→ HOME"
                continue

            # ── +/- → change step size ────────────────────────────
            if ch == "+" or ch == "=":
                step = min(200, step + 10)
                last_action = f"Step → {step}"
                continue
            if ch == "-" or ch == "_":
                step = max(5, step - 10)
                last_action = f"Step → {step}"
                continue

            # ── P → print full status ─────────────────────────────
            if ch.lower() == "p":
                # We'll just show it in the "last_action" line
                positions = arm.get_positions()
                last_action = "  ".join(
                    f"{n}={p}" for n, p in positions.items()
                )
                continue

            # ── joint keys ────────────────────────────────────────
            if ch.lower() in _KEYCHAR_TO_ACTION:
                joint_name, sign = _KEYCHAR_TO_ACTION[ch.lower()]
                delta = sign * step
                arm.nudge_joint(joint_name, delta)
                direction = "+" if sign > 0 else "–"
                pos = arm.get_joint_position(joint_name)
                last_action = f"{joint_name} {direction}{step} → {pos}"
                continue

    finally:
        # ── cleanup ───────────────────────────────────────────────
        arm.disable()
        arm.close()


# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("Starting keyboard teleoperation...")
    print("(Your terminal will switch to raw mode — press ESC to exit)\n")
    time.sleep(1)
    curses.wrapper(teleop_main)
    print("\nTeleoperation ended.  Bye!")
