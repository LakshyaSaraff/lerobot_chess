#!/usr/bin/env python3
"""
=============================================================================
 Chess Engine — Stockfish wrapper using python-chess
=============================================================================

Provides a clean interface to the Stockfish chess engine.  The engine handles
all the chess logic: legal move validation, best move search, position
evaluation, and special move detection (castling, en passant, promotion).

REQUIREMENTS:
    pip install python-chess
    sudo apt install stockfish

USAGE:
    engine = ChessEngine(elo=1500)
    board  = chess.Board()          # starting position
    move   = engine.get_best_move(board)
    print(f"Engine plays: {move}")  # e.g. e2e4
=============================================================================
"""

import chess
import chess.engine
import shutil
import sys


# ─────────────────────────────────────────────────────────────────────────────
# Find Stockfish binary
# ─────────────────────────────────────────────────────────────────────────────

def _find_stockfish() -> str:
    """
    Locate the Stockfish binary on the system.

    Checks common locations.  If not found, prints install instructions.
    """
    # Check PATH first
    path = shutil.which("stockfish")
    if path:
        return path

    # Common Linux install locations
    common_paths = [
        "/usr/games/stockfish",
        "/usr/local/bin/stockfish",
        "/usr/bin/stockfish",
        "/snap/bin/stockfish",
    ]
    for p in common_paths:
        import os
        if os.path.isfile(p):
            return p

    print("=" * 60)
    print(" ✗ Stockfish not found!")
    print("=" * 60)
    print("  Install it with:")
    print("    sudo apt install stockfish")
    print("  Or download from: https://stockfishchess.org/download/")
    print("=" * 60)
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
# Chess Engine class
# ─────────────────────────────────────────────────────────────────────────────

class ChessEngine:
    """
    Wrapper around Stockfish that provides a simple interface for chess moves.

    Usage:
        engine = ChessEngine(elo=1500)
        board  = chess.Board()
        move   = engine.get_best_move(board)
        engine.close()
    """

    def __init__(self, elo: int = 1500, stockfish_path: str = None):
        """
        Initialize the chess engine.

        Args:
            elo:  Desired playing strength (500-3000).
                  500  = beginner
                  1500 = intermediate (club player)
                  2500 = grandmaster level
                  3000 = maximum (superhuman)
            stockfish_path:  Path to the stockfish binary.  Auto-detected
                             if not provided.
        """
        self.elo = elo
        path = stockfish_path or _find_stockfish()
        print(f"  ♟  Stockfish found: {path}")

        self.engine = chess.engine.SimpleEngine.popen_uci(path)

        # Set skill level based on ELO
        self.set_difficulty(elo)
        print(f"  ♟  Engine ready (ELO ≈ {elo})")

    def set_difficulty(self, elo: int):
        """
        Adjust the engine's playing strength.

        Stockfish's "Skill Level" ranges 0-20.  We map ELO to this scale:
            500  → Skill 0   (plays random-ish moves)
            1500 → Skill 10  (decent club player)
            3000 → Skill 20  (maximum strength)
        """
        self.elo = elo
        # Map ELO (500-3000) to Skill Level (0-20)
        skill = max(0, min(20, int((elo - 500) / 125)))

        self.engine.configure({
            "Skill Level": skill,
            "Threads": 1,        # single thread (we're on CPU)
            "Hash": 64,          # 64 MB hash table (low memory)
        })

    def get_best_move(self, board: chess.Board,
                      time_limit: float = 2.0) -> chess.Move:
        """
        Compute the best move for the current position.

        Args:
            board:       Current board state.
            time_limit:  Maximum thinking time in seconds.

        Returns:
            A chess.Move object.  Use board.san(move) for human-readable
            notation (e.g., "Nf3") or move.uci() for coordinate notation
            (e.g., "g1f3").
        """
        result = self.engine.play(
            board,
            chess.engine.Limit(time=time_limit)
        )
        return result.move

    def get_evaluation(self, board: chess.Board,
                       time_limit: float = 1.0) -> dict:
        """
        Evaluate the current position.

        Returns:
            {
                "score_cp":   centipawn score (positive = white advantage),
                "score_str":  human-readable score string,
                "is_mate":    True if forced checkmate detected,
                "mate_in":    moves to mate (if is_mate),
            }
        """
        info = self.engine.analyse(
            board,
            chess.engine.Limit(time=time_limit)
        )
        score = info["score"].white()

        result = {
            "score_cp": 0,
            "score_str": "0.00",
            "is_mate": False,
            "mate_in": None,
        }

        if score.is_mate():
            mate_in = score.mate()
            result["is_mate"] = True
            result["mate_in"] = mate_in
            result["score_str"] = f"Mate in {abs(mate_in)}"
            result["score_cp"] = 10000 if mate_in > 0 else -10000
        else:
            cp = score.score()
            result["score_cp"] = cp
            result["score_str"] = f"{cp / 100:+.2f}"

        return result

    def is_game_over(self, board: chess.Board) -> dict:
        """
        Check if the game has ended.

        Returns:
            {
                "over":      True if game is finished,
                "result":    "1-0", "0-1", "1/2-1/2", or None,
                "reason":    human-readable reason,
            }
        """
        result = {"over": False, "result": None, "reason": None}

        if board.is_checkmate():
            result["over"] = True
            result["result"] = "1-0" if board.turn == chess.BLACK else "0-1"
            result["reason"] = "Checkmate"
        elif board.is_stalemate():
            result["over"] = True
            result["result"] = "1/2-1/2"
            result["reason"] = "Stalemate"
        elif board.is_insufficient_material():
            result["over"] = True
            result["result"] = "1/2-1/2"
            result["reason"] = "Insufficient material"
        elif board.can_claim_fifty_moves():
            result["over"] = True
            result["result"] = "1/2-1/2"
            result["reason"] = "Fifty-move rule"
        elif board.is_repetition(3):
            result["over"] = True
            result["result"] = "1/2-1/2"
            result["reason"] = "Threefold repetition"

        return result

    def close(self):
        """Shut down the engine process."""
        try:
            self.engine.quit()
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# Helper: detect what move was made by diffing two board states
# ─────────────────────────────────────────────────────────────────────────────

def detect_human_move(old_board: chess.Board,
                      new_fen_piece_placement: str) -> chess.Move:
    """
    Figure out what move the human made by comparing two board states.

    This is used when the camera detects a new board state — we need to
    work out which legal move transforms old_board into the new state.

    Args:
        old_board:  The board state BEFORE the human moved.
        new_fen_piece_placement:  The piece-placement part of the FEN
                                   (e.g., "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR")

    Returns:
        The chess.Move that transforms old_board → new state.

    Raises:
        ValueError if no legal move matches the observed change.
    """
    # Try every legal move and see which one produces the observed position
    for move in old_board.legal_moves:
        test_board = old_board.copy()
        test_board.push(move)
        # Compare only piece placement (first field of FEN)
        test_placement = test_board.fen().split(" ")[0]
        if test_placement == new_fen_piece_placement:
            return move

    raise ValueError(
        f"No legal move transforms the current position into "
        f"'{new_fen_piece_placement}'.  The vision system may have "
        f"misread the board."
    )


# ═══════════════════════════════════════════════════════════════════
#  MAIN — quick test
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 60)
    print(" Chess Engine Test")
    print("=" * 60)

    engine = ChessEngine(elo=1500)
    board = chess.Board()

    print(f"\n  Starting position:\n{board}\n")

    # Play a few moves
    for i in range(6):
        if engine.is_game_over(board)["over"]:
            break

        move = engine.get_best_move(board, time_limit=1.0)
        san = board.san(move)
        side = "White" if board.turn == chess.WHITE else "Black"
        print(f"  {side}: {san} ({move.uci()})")
        board.push(move)

    print(f"\n  Position after moves:\n{board}\n")

    # Evaluate
    evaluation = engine.get_evaluation(board)
    print(f"  Evaluation: {evaluation['score_str']}")

    engine.close()
    print("\n  Done!")
