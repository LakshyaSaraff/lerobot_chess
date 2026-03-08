#!/usr/bin/env python3
"""
=============================================================================
 Chess Game — Main game loop for the chess-playing robot arm
=============================================================================

Orchestrates all components to play a game of chess:

    Camera → Gemini Vision → Board Detection → Stockfish → Motion Planner

GAME FLOW:
    1. Robot plays as White or Black (configurable).
    2. If robot's turn: Stockfish computes best move, arm executes it.
    3. If human's turn: wait for human to move, press Enter, then camera
       captures the board and Gemini detects the new state.
    4. Repeat until game over.

TWO INPUT MODES FOR HUMAN MOVES:
    1. CAMERA MODE (--input camera): Uses the camera + Gemini Vision API
       to automatically detect the human's move after they press Enter.
    2. MANUAL MODE (--input manual): Human types their move in UCI notation
       (e.g., "e7e5").  No camera needed.  Good for testing.

USAGE:
    # Camera mode, robot plays black, ELO 1500:
    python chess_game.py --color black --elo 1500 --input camera

    # Manual mode, robot plays white:
    python chess_game.py --color white --input manual

    # Test a single move (no game loop):
    python chess_game.py --test-move e2e4

REQUIREMENTS:
    - Arm calibrated (run calibration.py first)
    - Stockfish installed (sudo apt install stockfish)
    - For camera mode: USB camera + GEMINI_API_KEY environment variable
=============================================================================
"""

import sys
import time
import chess

from arm_control import RobotArm
from chess_engine import ChessEngine, detect_human_move
from chess_motion import ChessMotionPlanner
from calibration import CalibrationData, CALIBRATION_FILE


# ─────────────────────────────────────────────────────────────────────────────
# Display helpers
# ─────────────────────────────────────────────────────────────────────────────

def print_board(board: chess.Board, last_move: chess.Move = None):
    """
    Print the chess board with nice formatting.

    Optionally highlights the last move made.
    """
    print()
    print("    a   b   c   d   e   f   g   h")
    print("  ┌───┬───┬───┬───┬───┬───┬───┬───┐")

    for rank in range(7, -1, -1):
        row = f"{rank + 1} │"
        for file in range(8):
            sq = chess.square(file, rank)
            piece = board.piece_at(sq)

            if piece:
                symbol = piece.unicode_symbol()
            else:
                # Checkerboard pattern for empty squares
                symbol = "·" if (rank + file) % 2 == 0 else " "

            # Highlight last move
            if last_move and sq in (last_move.from_square, last_move.to_square):
                row += f"*{symbol}*│"
            else:
                row += f" {symbol} │"

        print(row)
        if rank > 0:
            print("  ├───┼───┼───┼───┼───┼───┼───┼───┤")

    print("  └───┴───┴───┴───┴───┴───┴───┴───┘")
    print("    a   b   c   d   e   f   g   h")
    print()


def print_game_info(board: chess.Board, engine: ChessEngine,
                    move_number: int, robot_color: chess.Color):
    """Print game status information."""
    turn = "White" if board.turn == chess.WHITE else "Black"
    robot = "White" if robot_color == chess.WHITE else "Black"

    print(f"  Move {move_number} | {turn} to play | "
          f"Robot plays {robot}")

    if board.is_check():
        print(f"  ⚠ {turn} is in CHECK!")

    # Show evaluation
    try:
        eval_info = engine.get_evaluation(board, time_limit=0.5)
        print(f"  Evaluation: {eval_info['score_str']}")
    except Exception:
        pass


# ─────────────────────────────────────────────────────────────────────────────
# Human move input
# ─────────────────────────────────────────────────────────────────────────────

def get_human_move_manual(board: chess.Board) -> chess.Move:
    """
    Get the human's move via keyboard input.

    Accepts UCI notation (e.g., "e2e4", "g1f3", "e1g1" for castling).
    Also accepts standard algebraic notation (e.g., "Nf3", "O-O").
    """
    while True:
        raw = input("  Your move (e.g., e7e5 or Nf6): ").strip()

        if raw.lower() in ('quit', 'q', 'exit', 'resign'):
            print("  You resigned!")
            return None

        if raw.lower() in ('draw', 'offer draw'):
            print("  Draw offer noted (but the robot never draws 😈)")
            continue

        # Try UCI notation first
        try:
            move = chess.Move.from_uci(raw)
            if move in board.legal_moves:
                return move
        except (ValueError, chess.InvalidMoveError):
            pass

        # Try standard algebraic notation
        try:
            move = board.parse_san(raw)
            if move in board.legal_moves:
                return move
        except (ValueError, chess.InvalidMoveError, chess.AmbiguousMoveError):
            pass

        print(f"  ✗ '{raw}' is not a valid move. Legal moves:")
        legal = [board.san(m) for m in board.legal_moves]
        # Show in groups of 10
        for i in range(0, len(legal), 10):
            print(f"    {', '.join(legal[i:i+10])}")


def get_human_move_camera(board: chess.Board, camera, detector) -> chess.Move:
    """
    Get the human's move by photographing the board after they play.

    Steps:
        1. Wait for human to press Enter (after making their physical move)
        2. Capture a frame from the camera
        3. Send to Gemini Vision API for board detection
        4. Diff against the previous board state to identify the move
    """
    input("  Make your move on the board, then press ENTER...")

    print("  📷 Capturing board...")
    frame = camera.grab_frame()

    print("  🤖 Detecting board state via Gemini Vision...")
    try:
        new_board = detector.detect_board(frame)
        new_fen_placement = new_board.fen().split(" ")[0]

        print(f"  Detected FEN: {new_fen_placement}")

        # Find which legal move produces this board state
        move = detect_human_move(board, new_fen_placement)
        print(f"  Detected move: {board.san(move)} ({move.uci()})")

        # Confirm with user
        confirm = input("  Is this correct? (y/n): ").strip().lower()
        if confirm != 'y':
            print("  Falling back to manual input...")
            return get_human_move_manual(board)

        return move

    except Exception as e:
        print(f"  ⚠ Vision detection failed: {e}")
        print("  Falling back to manual input...")
        return get_human_move_manual(board)


# ─────────────────────────────────────────────────────────────────────────────
# Main Game Loop
# ─────────────────────────────────────────────────────────────────────────────

class ChessGame:
    """
    Main chess game orchestrator.

    Manages the game loop, input handling, and coordination between
    the vision system, chess engine, and motion planner.
    """

    def __init__(self, robot_color: chess.Color = chess.BLACK,
                 elo: int = 1500, input_mode: str = "manual",
                 camera_device: int = 0, calibration_path: str = None):
        """
        Initialize the chess game.

        Args:
            robot_color:      Which color the robot plays
            elo:              Stockfish difficulty (500-3000)
            input_mode:       "manual" or "camera"
            camera_device:    Camera index for camera mode
            calibration_path: Path to calibration JSON
        """
        self.robot_color = robot_color
        self.input_mode = input_mode
        self.board = chess.Board()
        self.move_number = 1
        self.move_history = []
        self.last_move = None

        # Initialize components
        print("=" * 60)
        print(" ♟  CHESS ROBOT — Initializing")
        print("=" * 60)

        # Chess engine
        print("\n  Setting up chess engine...")
        self.engine = ChessEngine(elo=elo)

        # Calibration + Motion planner
        cal_path = calibration_path or CALIBRATION_FILE
        print(f"\n  Loading calibration from {cal_path}...")
        try:
            self.calibration = CalibrationData.load(cal_path)
            print(f"  ✓ {len(self.calibration.squares)} squares calibrated")
        except FileNotFoundError:
            print(f"  ✗ Calibration file not found: {cal_path}")
            print(f"  Run calibration.py first!")
            sys.exit(1)

        # Robot arm
        print("\n  Connecting to arm...")
        self.arm = RobotArm()
        self.arm.enable()

        # Camera (only if camera mode — must init before verifier)
        self.camera = None
        self.detector = None
        if input_mode == "camera":
            print("\n  Setting up camera and vision...")
            from chess_vision import CameraCapture, BoardDetector
            self.camera = CameraCapture(device=camera_device)
            self.detector = BoardDetector()

        # Visual feedback (uses camera + gripper load sensing)
        self.verifier = None
        if input_mode == "camera":
            from visual_feedback import PickPlaceVerifier
            self.verifier = PickPlaceVerifier(
                self.arm, self.camera
            )

        self.planner = ChessMotionPlanner(
            self.arm, self.calibration, verifier=self.verifier
        )

        robot_side = "White" if robot_color == chess.WHITE else "Black"
        print(f"\n  ✓ All systems ready! Robot plays {robot_side}.")

    def play(self):
        """
        Main game loop.

        Alternates between human and robot turns until the game ends.
        """
        print("\n" + "═" * 60)
        print(" ♟  LET'S PLAY CHESS!")
        print("═" * 60)

        robot_side = "White" if self.robot_color == chess.WHITE else "Black"
        human_side = "Black" if self.robot_color == chess.WHITE else "White"
        print(f"  Robot plays: {robot_side}")
        print(f"  You play:    {human_side}")

        if self.robot_color == chess.WHITE:
            print(f"\n  Robot (White) moves first...")
        else:
            print(f"\n  You (White) move first!")

        print_board(self.board)

        while True:
            # Check for game over
            game_status = self.engine.is_game_over(self.board)
            if game_status["over"]:
                self._handle_game_over(game_status)
                break

            # Print game info
            print("─" * 60)
            print_game_info(self.board, self.engine,
                           self.move_number, self.robot_color)

            if self.board.turn == self.robot_color:
                # Robot's turn
                move = self._robot_turn()
            else:
                # Human's turn
                move = self._human_turn()

            if move is None:
                # Human resigned
                break

            # Make the move on the internal board
            self.move_history.append(self.board.san(move))
            self.board.push(move)
            self.last_move = move

            # Update move counter
            if self.board.turn == chess.WHITE:
                self.move_number += 1

            # Show updated board
            print_board(self.board, self.last_move)

            # Print move history
            self._print_move_history()

    def _robot_turn(self) -> chess.Move:
        """
        Handle the robot's turn: compute best move + execute with arm.
        """
        print(f"\n  🤖 Robot is thinking...")

        # Get best move from Stockfish
        move = self.engine.get_best_move(self.board, time_limit=2.0)
        san = self.board.san(move)
        print(f"  🤖 Robot plays: {san} ({move.uci()})")

        # Execute the move with the arm
        try:
            self.planner.execute_chess_move(move, self.board)
        except Exception as e:
            print(f"\n  ⚠ Arm motion error: {e}")
            print(f"  The move {san} was computed but the arm couldn't "
                  f"execute it.")
            print(f"  Please make the move manually on the board.")
            input("  Press ENTER after making the move manually... ")

        # Optional: full-board verification after robot's move
        if (self.verifier and self.camera and self.detector
                and self.verifier._vision_available()):
            print(f"  📷 Post-move board check...")
            try:
                frame = self.camera.grab_frame()
                detected = self.detector.detect_board(frame)
                expected = self.board.copy()
                expected.push(move)
                expected_fen = expected.fen().split(" ")[0]
                detected_fen = detected.fen().split(" ")[0]
                if expected_fen == detected_fen:
                    print(f"  ✓ Board state matches expected position.")
                else:
                    print(f"  ⚠ Board mismatch detected!")
                    print(f"    Expected: {expected_fen}")
                    print(f"    Detected: {detected_fen}")
                    print(f"    Please verify the board is correct.")
            except Exception as e:
                print(f"  ⚠ Post-move check failed: {e}")

        return move

    def _human_turn(self) -> chess.Move:
        """
        Handle the human's turn: detect their move via camera or keyboard.
        """
        print(f"\n  👤 Your turn!")

        if self.input_mode == "camera" and self.camera and self.detector:
            return get_human_move_camera(
                self.board, self.camera, self.detector
            )
        else:
            return get_human_move_manual(self.board)

    def _handle_game_over(self, status: dict):
        """Display game over information."""
        print("\n" + "═" * 60)
        print(f"  🏁 GAME OVER — {status['reason']}!")
        print(f"  Result: {status['result']}")
        print("═" * 60)

        if status['result'] == "1-0":
            if self.robot_color == chess.WHITE:
                print("  🤖 Robot wins! Better luck next time.")
            else:
                print("  🎉 You win! Congratulations!")
        elif status['result'] == "0-1":
            if self.robot_color == chess.BLACK:
                print("  🤖 Robot wins! Better luck next time.")
            else:
                print("  🎉 You win! Congratulations!")
        else:
            print("  🤝 It's a draw!")

        self._print_move_history()

    def _print_move_history(self):
        """Print the move history in standard PGN-like format."""
        if not self.move_history:
            return

        print("\n  Moves: ", end="")
        for i, san in enumerate(self.move_history):
            if i % 2 == 0:
                print(f"{i // 2 + 1}.", end="")
            print(f"{san} ", end="")
        print()

    def cleanup(self):
        """Clean up all resources."""
        try:
            self.engine.close()
        except Exception:
            pass
        try:
            print("\n  Arm is holding position. Press ENTER to release torque...")
            input()
            if self.verifier:
                self.verifier.stop_feed()
            self.arm.disable()
            self.arm.close()
        except Exception:
            pass
        try:
            if self.camera:
                self.camera.release()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="♟ Chess Robot — Play chess against your LeRobot arm!",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python chess_game.py --color black --elo 1500 --input manual
  python chess_game.py --color white --elo 800 --input camera
  python chess_game.py --test-move e2e4
        """
    )
    parser.add_argument("--color", choices=["white", "black"],
                        default="black",
                        help="Color the ROBOT plays (default: black)")
    parser.add_argument("--elo", type=int, default=1500,
                        help="Stockfish difficulty 500-3000 (default: 1500)")
    parser.add_argument("--input", choices=["manual", "camera"],
                        default="manual", dest="input_mode",
                        help="How human moves are entered (default: manual)")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index (default: 0)")
    parser.add_argument("--calibration", type=str, default=None,
                        help="Path to calibration JSON file")
    parser.add_argument("--test-move", type=str, default=None,
                        help="Test a single move and exit (e.g., 'e2e4')")

    args = parser.parse_args()

    # Map color string to chess.Color
    robot_color = chess.WHITE if args.color == "white" else chess.BLACK

    # Single move test mode
    if args.test_move:
        print("=" * 60)
        print(" Single Move Test")
        print("=" * 60)

        cal_path = args.calibration or CALIBRATION_FILE
        cal = CalibrationData.load(cal_path)
        arm = RobotArm()
        arm.enable()
        planner = ChessMotionPlanner(arm, cal)

        try:
            planner.test_move(args.test_move)
        finally:
            arm.disable()
            arm.close()
        sys.exit(0)

    # Full game mode
    game = ChessGame(
        robot_color=robot_color,
        elo=args.elo,
        input_mode=args.input_mode,
        camera_device=args.camera,
        calibration_path=args.calibration,
    )

    try:
        game.play()
    except KeyboardInterrupt:
        print("\n\n  Game interrupted by user.")
    finally:
        game.cleanup()
        print("\n  Thanks for playing! 🎉")
