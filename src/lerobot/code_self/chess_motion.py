#!/usr/bin/env python3
"""
=============================================================================
 Chess Motion Planner — Safe piece manipulation for the SO-100 arm
=============================================================================

Translates chess moves (e.g., "e2e4") into safe arm trajectories that
pick up a piece and place it on the target square without hitting
any other pieces on the board.

SAFETY STRATEGY: VERTICAL CLEARANCE
    The arm NEVER moves horizontally at piece height.  Every move follows
    an 8-phase sequence:

        1. Move above source square at SAFE height
        2. Open gripper
        3. Descend straight DOWN to GRIP height (grab the piece)
        4. Close gripper
        5. Lift straight UP to SAFE height
        6. Move horizontally to above the target square (at SAFE height)
        7. Descend straight DOWN to GRIP height (place the piece)
        8. Open gripper, lift back to SAFE height

    Since SAFE height is above ALL pieces (including kings), the arm
    cannot collide with anything during horizontal transit.

SPEED PROFILES:
    - Near the board (descending/ascending): SLOW for precision
    - At safe height (horizontal transit): FASTER for efficiency
    - Gripper actions: brief pause to let the gripper settle

SPECIAL CHESS MOVES:
    - Captures:    remove opponent's piece to graveyard first
    - Castling:    move king, then move rook (two pick-and-place)
    - En passant:  capture the pawn on the passing square
    - Promotion:   handled by placing piece and noting promotion type

USAGE:
    from calibration import CalibrationData
    from chess_motion import ChessMotionPlanner
    from arm_control import RobotArm

    arm = RobotArm()
    cal = CalibrationData.load()
    planner = ChessMotionPlanner(arm, cal)

    planner.execute_chess_move(move, board)  # does the full motion
=============================================================================
"""

import time
import chess
from arm_control import RobotArm
from calibration import CalibrationData


# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────

# Motion timing (seconds)
SLOW_MOVE_DELAY = 0.03     # delay per interpolation step near board
FAST_MOVE_DELAY = 0.02     # delay per interpolation step at safe height
GRIPPER_SETTLE  = 0.4      # pause after gripper open/close
DESCENT_STEPS   = 25       # interpolation steps for vertical motions
TRANSIT_STEPS   = 30       # interpolation steps for horizontal transit
SETTLE_PAUSE    = 0.3      # pause after reaching a position

# Feedback / retry
MAX_PICK_RETRIES = 2       # retries if grip verification fails
GRIP_RETRY_NUDGE = 15      # encoder ticks to nudge DOWN on each retry


# ─────────────────────────────────────────────────────────────────────────────
# Motion Planner
# ─────────────────────────────────────────────────────────────────────────────

class ChessMotionPlanner:
    """
    Translates chess moves into safe arm motions.

    Uses the vertical clearance strategy: always lift above all pieces
    before moving horizontally.
    """

    def __init__(self, arm: RobotArm, calibration: CalibrationData,
                 verifier=None):
        """
        Args:
            arm:          Connected RobotArm instance (torque should be on)
            calibration:  Loaded CalibrationData with all square positions
            verifier:     Optional PickPlaceVerifier for closed-loop feedback.
                          If None, behaviour is purely open-loop (same as before).
        """
        self.arm = arm
        self.cal = calibration
        self.verifier = verifier
        self.graveyard_index = 0   # next available graveyard slot

    # ─────────────────────────────────────────────────────────────────
    # Core motion primitives
    # ─────────────────────────────────────────────────────────────────

    def smooth_move(self, target: dict, steps: int = 20,
                    delay: float = 0.03):
        """
        Smoothly interpolate from current position to target.

        Instead of jumping directly (which causes jerky motion), this
        divides the path into N small steps and commands each one.

        Args:
            target:  {joint_name: position} — target joint positions
            steps:   number of interpolation steps
            delay:   seconds between each step
        """
        current = self.arm.get_positions()

        for i in range(1, steps + 1):
            t = i / steps  # 0.0 → 1.0

            # Linear interpolation for each joint
            interp = {}
            for joint_name in target:
                if joint_name in current:
                    start_val = current[joint_name]
                    end_val = target[joint_name]
                    interp[joint_name] = int(
                        start_val + t * (end_val - start_val)
                    )

            self.arm.set_all_joints(interp)
            time.sleep(delay)

        # Final settle
        time.sleep(SETTLE_PAUSE)

    def open_gripper(self):
        """Open the gripper and wait for it to settle."""
        self.arm.set_joint("gripper", self.cal.gripper_open)
        time.sleep(GRIPPER_SETTLE)

    def close_gripper(self):
        """Close the gripper around a piece and wait for it to settle."""
        self.arm.set_joint("gripper", self.cal.gripper_closed)
        time.sleep(GRIPPER_SETTLE)

    # ─────────────────────────────────────────────────────────────────
    # Pick and Place — the core operation
    # ─────────────────────────────────────────────────────────────────

    def pick_and_place(self, from_square: str, to_square: str):
        """
        Pick up a piece from one square and place it on another.

        SAFE 8-phase motion with optional closed-loop feedback:

            Phase 1: Move to safe height above source
            Phase 2: Open gripper
            Phase 3: Descend to grip height at source
            Phase 4: Close gripper (grab piece)
              └─ VERIFY: gripper load check — retry if no piece sensed
            Phase 5: Ascend to safe height
              └─ VERIFY: camera check — is source square empty?
            Phase 6: Transit horizontally to above target
            Phase 7: Descend to grip height at target
            Phase 8: Open gripper (release piece), ascend
              └─ VERIFY: camera check — is piece on target square?

        If no verifier is attached, all verification steps are skipped
        and behavior is identical to the original open-loop version.
        """
        src = self.cal.get_square(from_square)
        dst = self.cal.get_square(to_square)

        print(f"    Pick: {from_square.upper()} → Place: {to_square.upper()}")

        # ── PICK with grip verification + retry ──────────────────
        grip_ok = False
        for attempt in range(MAX_PICK_RETRIES + 1):
            if attempt > 0:
                print(f"      🔄 Retry {attempt}/{MAX_PICK_RETRIES} — "
                      f"nudging down by {GRIP_RETRY_NUDGE} ticks...")

            # Build grip target, nudging down on retries
            grip_target = dict(src["grip"])  # shallow copy
            if attempt > 0:
                # Nudge wrist_flex DOWN (increase value = lower arm)
                # to compensate for calibration drift
                for joint in ("shoulder_lift", "elbow_flex", "wrist_flex"):
                    if joint in grip_target:
                        grip_target[joint] += GRIP_RETRY_NUDGE * attempt

            # Phase 1: Move to safe height above source square
            print(f"      ↑ Moving above {from_square.upper()} (safe)...")
            self.smooth_move(src["safe"], steps=TRANSIT_STEPS,
                             delay=FAST_MOVE_DELAY)

            # Phase 2: Open gripper
            print(f"      ✋ Opening gripper...")
            self.open_gripper()

            # Phase 3: Descend to grip height (straight down)
            print(f"      ↓ Descending to {from_square.upper()} (grip)...")
            self.smooth_move(grip_target, steps=DESCENT_STEPS,
                             delay=SLOW_MOVE_DELAY)

            # Phase 4: Close gripper (grab the piece)
            print(f"      ✊ Gripping piece...")
            self.close_gripper()

            # ── VERIFY GRIP (load check) ─────────────────────────
            if self.verifier:
                grip_ok = self.verifier.verify_grip()
                if not grip_ok:
                    # Lift back up and retry
                    print(f"      ↑ Lifting (retry)...")
                    self.open_gripper()
                    self.smooth_move(src["safe"], steps=DESCENT_STEPS,
                                     delay=SLOW_MOVE_DELAY)
                    continue  # try again

            grip_ok = True
            break

        if not grip_ok:
            print(f"      ⚠ Failed to grip piece on {from_square.upper()} "
                  f"after {MAX_PICK_RETRIES + 1} attempts!")
            print(f"      Continuing anyway — please check the board.")

        # Phase 5: Ascend to safe height (straight up, holding piece)
        print(f"      ↑ Lifting piece to safe height...")
        self.smooth_move(src["safe"], steps=DESCENT_STEPS,
                         delay=SLOW_MOVE_DELAY)

        # ── VERIFY PICK (camera check: is source square empty?) ──
        if self.verifier:
            if not self.verifier.verify_square_empty(from_square):
                print(f"      ⚠ Source square {from_square.upper()} may "
                      f"still have a piece — continuing anyway.")

        # Phase 6: Transit horizontally to above target (at safe height)
        print(f"      → Moving to above {to_square.upper()} (safe)...")
        self.smooth_move(dst["safe"], steps=TRANSIT_STEPS,
                         delay=FAST_MOVE_DELAY)

        # Phase 7: Descend to grip height at target (straight down)
        print(f"      ↓ Placing piece on {to_square.upper()}...")
        self.smooth_move(dst["grip"], steps=DESCENT_STEPS,
                         delay=SLOW_MOVE_DELAY)

        # Phase 8: Open gripper (release piece)
        print(f"      ✋ Releasing piece...")
        self.open_gripper()

        # Retreat upward
        print(f"      ↑ Retreating to safe height...")
        self.smooth_move(dst["safe"], steps=DESCENT_STEPS,
                         delay=SLOW_MOVE_DELAY)

        # ── VERIFY PLACE (camera check: piece on target?) ────────
        if self.verifier:
            if not self.verifier.verify_square_occupied(to_square):
                print(f"      ⚠ Piece may not be on {to_square.upper()} — "
                      f"please check the board.")

        print(f"    ✓ Move complete: {from_square.upper()} → {to_square.upper()}")

    def move_to_graveyard(self, square: str):
        """
        Pick up a piece from a square and place it in the graveyard.

        Used for capturing opponent's pieces.
        """
        if self.graveyard_index >= len(self.cal.graveyard):
            print("    ⚠ Graveyard is full! Placing piece at last slot.")
            graveyard_pos = self.cal.graveyard[-1]
        else:
            graveyard_pos = self.cal.graveyard[self.graveyard_index]
            self.graveyard_index += 1

        # Pick up the piece
        src = self.cal.get_square(square)

        print(f"    Capturing piece on {square.upper()} → graveyard")

        # Move above source
        self.smooth_move(src["safe"], steps=TRANSIT_STEPS, delay=FAST_MOVE_DELAY)
        self.open_gripper()

        # Descend and grab
        self.smooth_move(src["grip"], steps=DESCENT_STEPS, delay=SLOW_MOVE_DELAY)
        self.close_gripper()

        # Lift
        self.smooth_move(src["safe"], steps=DESCENT_STEPS, delay=SLOW_MOVE_DELAY)

        # Move to graveyard (using safe height first, then graveyard position)
        self.smooth_move(self.cal.safe_height, steps=TRANSIT_STEPS, delay=FAST_MOVE_DELAY)
        self.smooth_move(graveyard_pos, steps=TRANSIT_STEPS, delay=FAST_MOVE_DELAY)

        # Release
        self.open_gripper()

        # Return to safe height
        self.smooth_move(self.cal.safe_height, steps=TRANSIT_STEPS, delay=FAST_MOVE_DELAY)

        print(f"    ✓ Piece captured from {square.upper()}")

    # ─────────────────────────────────────────────────────────────────
    # Chess move execution (handles all special moves)
    # ─────────────────────────────────────────────────────────────────

    def execute_chess_move(self, move: chess.Move, board: chess.Board):
        """
        Execute a chess move with the robot arm.

        Handles all special cases:
            - Normal moves
            - Captures (removes opponent piece first)
            - Castling (moves both king and rook)
            - En passant (captures pawn from its actual square)
            - Promotion (places piece, notes promotion type)

        Args:
            move:   The chess.Move to execute
            board:  The board state BEFORE the move is made
                    (needed to detect captures, castling, etc.)
        """
        from_sq = chess.square_name(move.from_square)
        to_sq = chess.square_name(move.to_square)

        print(f"\n  ♟ Executing: {board.san(move)} ({from_sq} → {to_sq})")

        # ── Handle castling ────────────────────────────────────────
        if board.is_castling(move):
            self._handle_castling(move, board)
            return

        # ── Handle en passant ──────────────────────────────────────
        if board.is_en_passant(move):
            self._handle_en_passant(move, board)
            return

        # ── Handle normal capture ──────────────────────────────────
        if board.is_capture(move):
            print(f"    ⚔ Capture detected!")
            # First remove the captured piece to graveyard
            self.move_to_graveyard(to_sq)

        # ── Move the piece ─────────────────────────────────────────
        self.pick_and_place(from_sq, to_sq)

        # ── Handle promotion ──────────────────────────────────────
        if move.promotion:
            piece_name = chess.piece_name(move.promotion)
            print(f"    👑 Promotion to {piece_name}!")
            print(f"    (Please manually swap the pawn for a {piece_name})")

    def _handle_castling(self, move: chess.Move, board: chess.Board):
        """
        Execute a castling move — requires moving both king and rook.

        Kingside castling (O-O):
            King: e1→g1, Rook: h1→f1  (or e8→g8, h8→f8 for black)
        Queenside castling (O-O-O):
            King: e1→c1, Rook: a1→d1  (or e8→c8, a8→d8 for black)
        """
        from_sq = chess.square_name(move.from_square)
        to_sq = chess.square_name(move.to_square)

        # Determine rook squares based on castling side
        rank = "1" if board.turn == chess.WHITE else "8"

        if chess.square_file(move.to_square) == 6:  # Kingside (g-file)
            rook_from = f"h{rank}"
            rook_to = f"f{rank}"
            print(f"    🏰 Kingside castling!")
        else:  # Queenside (c-file)
            rook_from = f"a{rank}"
            rook_to = f"d{rank}"
            print(f"    🏰 Queenside castling!")

        # Move king first
        print(f"    Moving king: {from_sq.upper()} → {to_sq.upper()}")
        self.pick_and_place(from_sq, to_sq)

        # Then move rook
        print(f"    Moving rook: {rook_from.upper()} → {rook_to.upper()}")
        self.pick_and_place(rook_from, rook_to)

    def _handle_en_passant(self, move: chess.Move, board: chess.Board):
        """
        Execute an en passant capture.

        In en passant, the captured pawn is NOT on the destination square.
        It's on the same file as the destination, but on the starting rank.

        Example: White pawn on e5 captures en passant on d6.
                 The captured pawn is on d5 (not d6).
        """
        from_sq = chess.square_name(move.from_square)
        to_sq = chess.square_name(move.to_square)

        # The captured pawn is at (to_file, from_rank)
        captured_file = chess.square_file(move.to_square)
        captured_rank = chess.square_rank(move.from_square)
        captured_sq = chess.square_name(
            chess.square(captured_file, captured_rank)
        )

        print(f"    🥖 En passant capture!")
        print(f"    Removing pawn from {captured_sq.upper()}")

        # Remove the captured pawn first
        self.move_to_graveyard(captured_sq)

        # Move the capturing pawn
        self.pick_and_place(from_sq, to_sq)

    # ─────────────────────────────────────────────────────────────────
    # Utility methods
    # ─────────────────────────────────────────────────────────────────

    def go_to_safe_height(self):
        """Move the arm to the global safe height position."""
        print("    ↑ Moving to safe height...")
        self.smooth_move(self.cal.safe_height, steps=TRANSIT_STEPS,
                        delay=FAST_MOVE_DELAY)

    def go_home(self):
        """Move arm to its neutral home position."""
        print("    🏠 Going home...")
        self.arm.go_home()

    def test_square(self, square: str):
        """
        Test motion to a single square — hover above, descend, ascend.

        Useful for verifying calibration.
        """
        sq_data = self.cal.get_square(square)

        print(f"  Testing square {square.upper()}...")

        # Go to safe height
        self.smooth_move(self.cal.safe_height, steps=TRANSIT_STEPS,
                        delay=FAST_MOVE_DELAY)

        # Move above the square
        self.smooth_move(sq_data["safe"], steps=TRANSIT_STEPS,
                        delay=FAST_MOVE_DELAY)

        # Descend
        self.open_gripper()
        self.smooth_move(sq_data["grip"], steps=DESCENT_STEPS,
                        delay=SLOW_MOVE_DELAY)

        # Pause to show position
        time.sleep(1.0)

        # Ascend
        self.smooth_move(sq_data["safe"], steps=DESCENT_STEPS,
                        delay=SLOW_MOVE_DELAY)

        print(f"  ✓ Test complete for {square.upper()}")

    def test_move(self, uci_move: str):
        """
        Test a chess move without a real board.

        Args:
            uci_move:  UCI move string like "e2e4"
        """
        board = chess.Board()  # starting position
        move = chess.Move.from_uci(uci_move)

        if move not in board.legal_moves:
            print(f"  ✗ '{uci_move}' is not a legal move from the starting position.")
            return

        print(f"  Testing move: {board.san(move)} ({uci_move})")
        self.execute_chess_move(move, board)
        print(f"  ✓ Move test complete!")


# ═══════════════════════════════════════════════════════════════════
#  MAIN — test the motion planner
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Chess Motion Planner")
    parser.add_argument("--test-square", type=str,
                        help="Test motion to a specific square (e.g., 'e4')")
    parser.add_argument("--test-move", type=str,
                        help="Test a chess move (e.g., 'e2e4')")
    parser.add_argument("--calibration", type=str, default=None,
                        help="Path to calibration file")
    parser.add_argument("--use-camera", action="store_true",
                        help="Enable camera-based visual feedback")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index (default: 0)")
    args = parser.parse_args()

    from calibration import CALIBRATION_FILE

    cal_path = args.calibration or CALIBRATION_FILE
    print("=" * 60)
    print(" Chess Motion Planner Test")
    print("=" * 60)

    # Load calibration
    try:
        cal = CalibrationData.load(cal_path)
        print(f"  ✓ Calibration loaded ({len(cal.squares)} squares)")
    except FileNotFoundError:
        print(f"  ✗ Calibration not found at: {cal_path}")
        print(f"  Run calibration.py first!")
        import sys
        sys.exit(1)

    # Connect to arm
    arm = RobotArm()
    arm.enable()
    time.sleep(0.5)

    # Set up optional visual feedback
    verifier = None
    if args.use_camera:
        from chess_vision import CameraCapture
        from visual_feedback import PickPlaceVerifier
        camera = CameraCapture(device=args.camera)
        verifier = PickPlaceVerifier(arm, camera)

    planner = ChessMotionPlanner(arm, cal, verifier=verifier)

    try:
        if args.test_square:
            planner.test_square(args.test_square)
        elif args.test_move:
            planner.test_move(args.test_move)
        else:
            # Interactive mode
            print("\n  Commands:")
            print("    square <name>  — test a square (e.g., 'square e4')")
            print("    move <uci>     — test a move (e.g., 'move e2e4')")
            print("    home           — go to home position")
            print("    quit           — exit\n")

            while True:
                cmd = input("  > ").strip().lower()
                if cmd.startswith("square "):
                    sq = cmd.split()[1]
                    planner.test_square(sq)
                elif cmd.startswith("move "):
                    mv = cmd.split()[1]
                    planner.test_move(mv)
                elif cmd == "home":
                    planner.go_home()
                elif cmd in ("quit", "q", "exit"):
                    break
                else:
                    print("  Unknown command. Try 'square e4' or 'move e2e4'")

    finally:
        print("\n  Arm is holding position. Press ENTER to release torque...")
        input()
        if verifier:
            verifier.stop_feed()
        arm.disable()
        arm.close()
