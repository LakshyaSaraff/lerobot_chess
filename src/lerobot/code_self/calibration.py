#!/usr/bin/env python3
"""
=============================================================================
 Calibration Tool — Map chess squares to arm joint positions
=============================================================================

This tool lets you teach the arm where each chess square is located by
guiding it to key positions and recording the joint encoder values.

TWO MODES:

  1. FOUR-CORNER MODE (recommended, ~5 min):
     Guide the arm to just 4 corners (a1, a8, h1, h8) plus a safe hover
     height.  All 64 squares are computed via bilinear interpolation.

  2. FULL MODE (~30 min):
     Guide the arm to every single square individually.
     More precise, but tedious.

Both modes also calibrate:
  - SAFE positions (hovering above the board, above all pieces)
  - GRIP positions (at the height where the gripper grabs a piece)
  - GRIPPER open/close positions
  - GRAVEYARD positions (where captured pieces are placed off-board)

Saves everything to chess_calibration.json.

USAGE:
    python calibration.py                  # 4-corner mode (default)
    python calibration.py --mode full      # full per-square mode
    python calibration.py --load           # load & inspect existing calibration
=============================================================================
"""

import json
import os
import sys
import time
from dataclasses import dataclass

from arm_control import RobotArm


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "chess_calibration.json"
)

# Chess files and ranks
FILES = "abcdefgh"
RANKS = "12345678"
ALL_SQUARES = [f + r for r in RANKS for f in FILES]  # a1, b1, ... h8


# ─────────────────────────────────────────────────────────────────────────────
# Interpolation helpers
# ─────────────────────────────────────────────────────────────────────────────

def bilinear_interpolate(corners: dict, file_idx: int, rank_idx: int) -> dict:
    """
    Bilinear interpolation between 4 corner joint positions.

    Given joint positions at corners a1, a8, h1, h8, compute the position
    for any square on the board using bilinear interpolation.

    This works because:
    - The chessboard is a flat, rectangular grid
    - The arm's joint-to-Cartesian mapping is roughly linear over
      the board area (as long as the board is in the arm's comfort zone)

    Args:
        corners:   {"a1": {...}, "a8": {...}, "h1": {...}, "h8": {...}}
                   Each corner maps to {joint_name: encoder_position}
        file_idx:  0 (a-file) to 7 (h-file)
        rank_idx:  0 (rank 1) to 7 (rank 8)

    Returns:
        {joint_name: interpolated_encoder_position}
    """
    # Normalized coordinates: u = file fraction, v = rank fraction
    u = file_idx / 7.0   # 0.0 at a-file, 1.0 at h-file
    v = rank_idx / 7.0   # 0.0 at rank 1,  1.0 at rank 8

    # Bilinear blend: (1-u)(1-v)*a1 + u*(1-v)*h1 + (1-u)*v*a8 + u*v*h8
    a1 = corners["a1"]
    a8 = corners["a8"]
    h1 = corners["h1"]
    h8 = corners["h8"]

    result = {}
    for joint in a1.keys():
        val = (
            (1 - u) * (1 - v) * a1[joint] +
                 u  * (1 - v) * h1[joint] +
            (1 - u) *      v  * a8[joint] +
                 u  *      v  * h8[joint]
        )
        result[joint] = int(round(val))

    return result


# ─────────────────────────────────────────────────────────────────────────────
# Calibration data structure
# ─────────────────────────────────────────────────────────────────────────────

class CalibrationData:
    """
    Holds all calibration data and provides save/load/query methods.

    Data stored:
        squares:       {square_name: {"grip": {joints}, "safe": {joints}}}
        gripper_open:  encoder position for gripper fully open
        gripper_closed: encoder position for gripper closed on a piece
        graveyard:     list of {joints} positions for captured pieces
        safe_height:   {joints} for a global safe hovering position
    """

    def __init__(self):
        self.squares = {}        # "e4" → {"grip": {joints}, "safe": {joints}}
        self.gripper_open = 1800
        self.gripper_closed = 2400
        self.graveyard = []      # list of positions for captured pieces
        self.safe_height = {}    # global safe hover position
        self.mode = "four_corner"

    def save(self, path: str = CALIBRATION_FILE):
        """Save calibration to JSON."""
        data = {
            "mode": self.mode,
            "squares": self.squares,
            "gripper_open": self.gripper_open,
            "gripper_closed": self.gripper_closed,
            "graveyard": self.graveyard,
            "safe_height": self.safe_height,
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"\n  ✓ Calibration saved to {path}")

    @classmethod
    def load(cls, path: str = CALIBRATION_FILE) -> 'CalibrationData':
        """Load calibration from JSON."""
        if not os.path.exists(path):
            raise FileNotFoundError(f"Calibration file not found: {path}")

        with open(path, 'r') as f:
            data = json.load(f)

        cal = cls()
        cal.mode = data.get("mode", "unknown")
        cal.squares = data.get("squares", {})
        cal.gripper_open = data.get("gripper_open", 1800)
        cal.gripper_closed = data.get("gripper_closed", 2400)
        cal.graveyard = data.get("graveyard", [])
        cal.safe_height = data.get("safe_height", {})
        return cal

    def get_square(self, square_name: str) -> dict:
        """
        Get the grip and safe positions for a square.

        Returns: {"grip": {joints}, "safe": {joints}}
        """
        if square_name not in self.squares:
            raise KeyError(
                f"Square '{square_name}' not calibrated.  "
                f"Run calibration first."
            )
        return self.squares[square_name]

    def print_summary(self):
        """Print a summary of the calibration data."""
        print(f"\n  Calibration mode: {self.mode}")
        print(f"  Squares calibrated: {len(self.squares)}")
        print(f"  Gripper open: {self.gripper_open}")
        print(f"  Gripper closed: {self.gripper_closed}")
        print(f"  Graveyard slots: {len(self.graveyard)}")
        print(f"  Safe height: {'set' if self.safe_height else 'NOT SET'}")

        if self.squares:
            # Show a visual grid of which squares are calibrated
            print("\n  Calibrated squares:")
            print("    ┌─" + "──" * 8 + "┐")
            for rank in reversed(RANKS):
                row = f"  {rank} │"
                for file in FILES:
                    sq = file + rank
                    row += " ✓" if sq in self.squares else " ·"
                row += " │"
                print(row)
            print("    └─" + "──" * 8 + "┘")
            print("      a b c d e f g h")


# ─────────────────────────────────────────────────────────────────────────────
# Interactive calibration helpers
# ─────────────────────────────────────────────────────────────────────────────

def _prompt_and_record(arm: RobotArm, message: str) -> dict:
    """
    Prompt the user to guide the arm, then record joint positions.

    The arm is set to limp mode so the user can physically push it.
    """
    arm.disable()
    print(f"\n  >>> {message}")
    input("      Press ENTER when the arm is in position... ")
    positions = arm.get_positions()
    print(f"      Recorded: { {k: v for k, v in positions.items()} }")
    return positions


def calibrate_gripper(arm: RobotArm) -> tuple[int, int]:
    """
    Calibrate the gripper open and closed positions.

    Returns: (gripper_open, gripper_closed)
    """
    print("\n" + "=" * 60)
    print(" GRIPPER CALIBRATION")
    print("=" * 60)

    arm.disable()

    print("\n  Guide the gripper to FULLY OPEN position.")
    input("  Press ENTER when ready... ")
    gripper_open = arm.get_joint_position("gripper")
    print(f"  Gripper OPEN = {gripper_open}")

    print("\n  Now close the gripper around a chess piece (snug, not crushing).")
    input("  Press ENTER when ready... ")
    gripper_closed = arm.get_joint_position("gripper")
    print(f"  Gripper CLOSED = {gripper_closed}")

    return gripper_open, gripper_closed


def calibrate_safe_height(arm: RobotArm) -> dict:
    """
    Calibrate the safe hovering height above the board.

    This position should be high enough to clear ALL pieces (including kings).
    """
    print("\n" + "=" * 60)
    print(" SAFE HEIGHT CALIBRATION")
    print("=" * 60)

    positions = _prompt_and_record(
        arm,
        "Move the arm to a SAFE HOVER position above the CENTER of the board.\n"
        "      This should be HIGH ENOUGH to clear the tallest piece (king ≈ 9.5cm).\n"
        "      The gripper should be pointing straight DOWN."
    )
    return positions


def calibrate_graveyard(arm: RobotArm, num_slots: int = 6) -> list:
    """
    Calibrate positions for captured pieces (off the board).

    Records positions in a line off the board edge.
    """
    print("\n" + "=" * 60)
    print(" GRAVEYARD CALIBRATION")
    print("=" * 60)
    print(f"  We'll record {num_slots} spots for captured pieces.")
    print("  Place them in a row next to the board.\n")

    slots = []
    for i in range(num_slots):
        positions = _prompt_and_record(
            arm,
            f"Guide arm to GRAVEYARD slot {i + 1}/{num_slots} "
            f"(grip height, where a captured piece would be placed)."
        )
        slots.append(positions)

    return slots


# ─────────────────────────────────────────────────────────────────────────────
# Four-corner calibration
# ─────────────────────────────────────────────────────────────────────────────

def calibrate_four_corners(arm: RobotArm) -> CalibrationData:
    """
    Calibrate using just 4 corners + safe height.

    Steps:
        1. Calibrate gripper open/close
        2. Record safe hovering height
        3. Record grip height at 4 corners (a1, a8, h1, h8)
        4. Record safe height at 4 corners
        5. Interpolate all 64 squares
        6. Record graveyard slots
    """
    cal = CalibrationData()
    cal.mode = "four_corner"

    print("\n" + "═" * 60)
    print(" FOUR-CORNER CALIBRATION")
    print("═" * 60)
    print("  This will take about 5 minutes.")
    print("  You'll guide the arm to 4 corners of the board plus a few")
    print("  extra positions.  The arm will be in LIMP mode so you can")
    print("  push it by hand.\n")

    # Step 1: Gripper
    cal.gripper_open, cal.gripper_closed = calibrate_gripper(arm)

    # Step 2: Safe height
    cal.safe_height = calibrate_safe_height(arm)

    # Step 3: Record 4 corners at grip height
    corners_grip = {}
    corners_safe = {}
    corner_squares = ["a1", "h1", "a8", "h8"]

    for sq in corner_squares:
        print(f"\n" + "─" * 40)
        print(f"  CORNER: {sq.upper()}")
        print("─" * 40)

        grip_pos = _prompt_and_record(
            arm,
            f"Move the gripper DOWN to GRIP HEIGHT on square {sq.upper()}.\n"
            f"      (Low enough to grab a piece on this square)"
        )
        corners_grip[sq] = grip_pos

        safe_pos = _prompt_and_record(
            arm,
            f"Now LIFT the arm straight UP above {sq.upper()} to SAFE HEIGHT.\n"
            f"      (High enough to clear all pieces)"
        )
        corners_safe[sq] = safe_pos

    # Step 4: Interpolate all 64 squares
    print("\n  Interpolating 64 squares from 4 corners...")
    for rank_idx, rank in enumerate(RANKS):
        for file_idx, file in enumerate(FILES):
            sq = file + rank
            grip = bilinear_interpolate(corners_grip, file_idx, rank_idx)
            safe = bilinear_interpolate(corners_safe, file_idx, rank_idx)
            cal.squares[sq] = {"grip": grip, "safe": safe}

    print(f"  ✓ {len(cal.squares)} squares calibrated!")

    # Step 5: Graveyard
    cal.graveyard = calibrate_graveyard(arm)

    return cal


# ─────────────────────────────────────────────────────────────────────────────
# Full per-square calibration
# ─────────────────────────────────────────────────────────────────────────────

def calibrate_full(arm: RobotArm) -> CalibrationData:
    """
    Calibrate every single square individually.

    More precise than 4-corner mode but takes ~30 minutes.
    """
    cal = CalibrationData()
    cal.mode = "full"

    print("\n" + "═" * 60)
    print(" FULL CALIBRATION (all 64 squares)")
    print("═" * 60)
    print("  This will take about 30 minutes.")
    print("  You'll guide the arm to each square one by one.\n")

    # Gripper
    cal.gripper_open, cal.gripper_closed = calibrate_gripper(arm)

    # Safe height
    cal.safe_height = calibrate_safe_height(arm)

    # Each square
    total = len(ALL_SQUARES)
    for i, sq in enumerate(ALL_SQUARES):
        print(f"\n" + "─" * 40)
        print(f"  SQUARE {i + 1}/{total}: {sq.upper()}")
        print("─" * 40)

        grip_pos = _prompt_and_record(
            arm,
            f"Move gripper to GRIP HEIGHT on {sq.upper()}."
        )

        safe_pos = _prompt_and_record(
            arm,
            f"LIFT arm to SAFE HEIGHT above {sq.upper()}."
        )

        cal.squares[sq] = {"grip": grip_pos, "safe": safe_pos}

    # Graveyard
    cal.graveyard = calibrate_graveyard(arm)

    return cal


# ─────────────────────────────────────────────────────────────────────────────
# Calibration verification
# ─────────────────────────────────────────────────────────────────────────────

def verify_calibration(arm: RobotArm, cal: CalibrationData):
    """
    Test the calibration by moving the arm to selected squares.

    Lets the user visually verify that the arm goes to the right spots.
    """
    print("\n" + "═" * 60)
    print(" CALIBRATION VERIFICATION")
    print("═" * 60)
    print("  The arm will move to specific squares so you can check accuracy.")
    print("  Type a square name (e.g., 'e4') or 'quit' to exit.\n")

    arm.enable()
    time.sleep(0.5)

    while True:
        sq = input("  Square to test (or 'quit'): ").strip().lower()
        if sq in ('quit', 'q', 'exit'):
            break

        if sq not in cal.squares:
            print(f"  ✗ Unknown square '{sq}'. Use format: e4, a1, h8, etc.")
            continue

        # Move to safe height first
        print(f"  Moving to safe height above {sq.upper()}...")
        safe_pos = cal.squares[sq]["safe"]
        arm.set_all_joints(safe_pos)
        time.sleep(1.5)

        # Descend to grip height
        print(f"  Descending to grip height on {sq.upper()}...")
        grip_pos = cal.squares[sq]["grip"]
        arm.set_all_joints(grip_pos)
        time.sleep(1.5)

        # Back to safe
        print(f"  Returning to safe height...")
        arm.set_all_joints(safe_pos)
        time.sleep(1.0)

        accurate = input("  Was that accurate? (y/n): ").strip().lower()
        if accurate != 'y':
            print("  Consider re-running calibration or using full mode.")

    arm.disable()


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Chess Arm Calibration Tool")
    parser.add_argument("--mode", choices=["four_corner", "full"],
                        default="four_corner",
                        help="Calibration mode (default: four_corner)")
    parser.add_argument("--load", action="store_true",
                        help="Load and inspect existing calibration")
    parser.add_argument("--verify", action="store_true",
                        help="Load calibration and test it")
    parser.add_argument("--output", type=str, default=CALIBRATION_FILE,
                        help="Output file path")
    args = parser.parse_args()

    if args.load:
        print("=" * 60)
        print(" Loading existing calibration")
        print("=" * 60)
        try:
            cal = CalibrationData.load(args.output)
            cal.print_summary()
        except FileNotFoundError as e:
            print(f"  ✗ {e}")
        sys.exit(0)

    # Connect to arm
    print("=" * 60)
    print(" Connecting to arm...")
    print("=" * 60)
    arm = RobotArm()

    if args.verify:
        cal = CalibrationData.load(args.output)
        cal.print_summary()
        verify_calibration(arm, cal)
        arm.close()
        sys.exit(0)

    # Run calibration
    if args.mode == "four_corner":
        cal = calibrate_four_corners(arm)
    else:
        cal = calibrate_full(arm)

    cal.print_summary()
    cal.save(args.output)

    # Offer verification
    print("\n  Would you like to verify the calibration?")
    if input("  (y/n): ").strip().lower() == 'y':
        verify_calibration(arm, cal)

    arm.close()
    print("\n  Calibration complete! 🎉")
