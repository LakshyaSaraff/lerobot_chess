#!/usr/bin/env python3
"""
=============================================================================
 Chess Vision — Camera capture + Gemini Vision API for board detection
=============================================================================

Uses a top-down camera to photograph the chessboard, then sends the image
to Google's Gemini Vision API (free tier) to identify all pieces and their
positions.

CAMERA SETUP:
    Mount ONE USB webcam directly above the board, looking straight down.
    The entire board (all 8×8 squares) should be visible with some margin.
    A cheap phone tripod, gorillapod, or desk clamp works perfectly.

GEMINI API SETUP:
    1. Go to https://aistudio.google.com/apikey
    2. Create a free API key
    3. Set environment variable:  export GEMINI_API_KEY="your-key-here"
       Or pass it directly to BoardDetector(api_key="...")

FREE TIER LIMITS (more than enough for chess):
    • 5-15 requests/minute
    • 100-1000 requests/day
    • No credit card needed

USAGE:
    camera   = CameraCapture(device=0)
    detector = BoardDetector(api_key="...")

    frame = camera.grab_frame()
    board = detector.detect_board(frame)
    print(board)  # prints the chess board

    camera.release()
=============================================================================
"""

import os
import sys
import time
import base64
import tempfile
import chess

try:
    import cv2
except ImportError:
    print("  ✗ OpenCV not found.  Install with: pip install opencv-python")
    sys.exit(1)

try:
    import google.generativeai as genai
except ImportError:
    print("  ✗ google-generativeai not found.  Install with: pip install google-generativeai")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
# Camera Capture
# ─────────────────────────────────────────────────────────────────────────────

class CameraCapture:
    """
    Simple USB camera wrapper using OpenCV.

    Usage:
        cam = CameraCapture(device=0)  # 0 = first USB camera
        frame = cam.grab_frame()       # returns a numpy array (BGR)
        cam.show_preview()             # pops up a live window
        cam.release()                  # clean up
    """

    def __init__(self, device: int = 0, width: int = 1280, height: int = 720):
        """
        Open a USB camera.

        Args:
            device:  Camera index (0 = first camera, 1 = second, etc.)
            width:   Requested frame width
            height:  Requested frame height
        """
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera device {device}.  "
                "Check that the camera is plugged in and not in use."
            )

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Read actual resolution (camera may not support requested size)
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"  📷 Camera opened: device {device}, {actual_w}×{actual_h}")

    def grab_frame(self):
        """
        Capture a single frame from the camera.

        Returns:
            A numpy array of shape (H, W, 3) in BGR color order.
        """
        ret, frame = self.cap.read()
        if not ret or frame is None:
            raise RuntimeError("Failed to capture frame from camera.")
        return frame

    def save_frame(self, path: str = None) -> str:
        """
        Capture and save a frame to disk.

        Returns the file path.
        """
        frame = self.grab_frame()
        if path is None:
            path = os.path.join(tempfile.gettempdir(), "chess_frame.jpg")
        cv2.imwrite(path, frame)
        return path

    def show_preview(self, window_name: str = "Camera Preview"):
        """
        Show a live camera preview.  Press 'q' to close.
        """
        print("  Showing camera preview. Press 'q' to close...")
        while True:
            frame = self.grab_frame()
            cv2.imshow(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def release(self):
        """Release the camera."""
        if self.cap:
            self.cap.release()
            print("  📷 Camera released.")


# ─────────────────────────────────────────────────────────────────────────────
# Board Detector — Gemini Vision API
# ─────────────────────────────────────────────────────────────────────────────

# The prompt is carefully crafted to get consistent, parseable FEN output.
DETECTION_PROMPT = """You are a chess board analyzer. Look at this top-down photograph of a physical chessboard and identify every piece on every square.

IMPORTANT RULES:
1. The board is photographed from above (top-down view).
2. Identify the orientation: typically White pieces start on ranks 1-2 (bottom of image) and Black pieces start on ranks 7-8 (top of image).
3. For each square, identify if it is empty or contains a piece.
4. Use standard chess notation for pieces:
   - White: K=King, Q=Queen, R=Rook, B=Bishop, N=Knight, P=Pawn
   - Black: k=king, q=queen, r=rook, b=bishop, n=knight, p=pawn
   - Empty squares: count consecutive empty squares as a number (1-8)

OUTPUT FORMAT:
Return ONLY the piece placement part of FEN notation (the first field).
This is 8 ranks separated by slashes, from rank 8 (top) to rank 1 (bottom).

Example starting position:
rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR

Example after 1.e4:
rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR

Return ONLY the FEN piece placement string, nothing else. No explanation, no markdown, no quotes."""


class BoardDetector:
    """
    Detects the chess board state from a camera image using Gemini Vision API.

    Usage:
        detector = BoardDetector(api_key="your-key")
        fen = detector.detect_board(frame)  # returns chess.Board
    """

    def __init__(self, api_key: str = None, model: str = "gemini-2.5-flash"):
        """
        Initialize the Gemini Vision detector.

        Args:
            api_key:  Gemini API key.  If None, reads from GEMINI_API_KEY
                      environment variable.
            model:    Gemini model to use.  "gemini-2.5-flash" is free
                      and handles chess boards well.
        """
        self.api_key = api_key or os.environ.get("GEMINI_API_KEY")
        if not self.api_key:
            print("=" * 60)
            print(" ✗ Gemini API key not found!")
            print("=" * 60)
            print("  Set it with:")
            print("    export GEMINI_API_KEY='your-api-key-here'")
            print("  Get a free key at: https://aistudio.google.com/apikey")
            print("=" * 60)
            sys.exit(1)

        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel(model)
        print(f"  🤖 Gemini Vision ready (model: {model})")

    def detect_board(self, frame, retries: int = 3) -> chess.Board:
        """
        Detect the board state from a camera frame.

        Args:
            frame:    OpenCV frame (numpy array, BGR)
            retries:  Number of retries if the API returns invalid FEN

        Returns:
            A chess.Board object representing the detected position.

        Raises:
            RuntimeError if detection fails after all retries.
        """
        # Convert frame to JPEG bytes for the API
        _, jpeg_bytes = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        image_data = jpeg_bytes.tobytes()

        last_error = None
        for attempt in range(retries):
            try:
                fen_placement = self._call_gemini(image_data)
                board = self._parse_and_validate(fen_placement)
                return board

            except (ValueError, RuntimeError) as e:
                last_error = e
                print(f"  ⚠ Detection attempt {attempt + 1}/{retries} failed: {e}")
                if attempt < retries - 1:
                    time.sleep(1)  # brief pause before retry

        raise RuntimeError(
            f"Board detection failed after {retries} attempts.  "
            f"Last error: {last_error}"
        )

    def detect_board_fen(self, frame, retries: int = 3) -> str:
        """
        Like detect_board() but returns just the FEN piece placement string.
        """
        board = self.detect_board(frame, retries)
        return board.fen().split(" ")[0]

    def _call_gemini(self, image_bytes: bytes) -> str:
        """
        Send the image to Gemini and get the FEN string back.
        """
        import PIL.Image
        import io

        # Convert bytes to PIL Image (what Gemini SDK expects)
        image = PIL.Image.open(io.BytesIO(image_bytes))

        response = self.model.generate_content(
            [DETECTION_PROMPT, image],
            generation_config=genai.types.GenerationConfig(
                temperature=0.1,      # low creativity — we want precision
                max_output_tokens=100, # FEN is always short
            ),
        )

        # Extract the text response
        text = response.text.strip()

        # Clean up — sometimes the model wraps in backticks or adds spaces
        text = text.replace('`', '').replace(' ', '').strip()

        # Sometimes the model returns full FEN (with turn, castling, etc.)
        # We only want the piece placement (first field)
        if ' ' in text:
            text = text.split(' ')[0]

        return text

    def _parse_and_validate(self, fen_placement: str) -> chess.Board:
        """
        Validate the FEN piece placement and return a Board.

        Checks:
            1. Correct format (8 ranks separated by /)
            2. Each rank sums to 8 (pieces + empty squares)
            3. Valid piece characters
            4. Reasonable piece counts (not 5 queens, etc.)
        """
        ranks = fen_placement.split('/')
        if len(ranks) != 8:
            raise ValueError(
                f"Expected 8 ranks in FEN, got {len(ranks)}: '{fen_placement}'"
            )

        valid_pieces = set("KQRBNPkqrbnp")
        piece_counts = {}

        for rank_idx, rank in enumerate(ranks):
            square_count = 0
            for char in rank:
                if char.isdigit():
                    square_count += int(char)
                elif char in valid_pieces:
                    square_count += 1
                    piece_counts[char] = piece_counts.get(char, 0) + 1
                else:
                    raise ValueError(
                        f"Invalid character '{char}' in rank {8 - rank_idx}: "
                        f"'{rank}'"
                    )

            if square_count != 8:
                raise ValueError(
                    f"Rank {8 - rank_idx} has {square_count} squares "
                    f"(expected 8): '{rank}'"
                )

        # Sanity checks on piece counts
        if piece_counts.get('K', 0) != 1:
            raise ValueError(
                f"Expected exactly 1 white king, found "
                f"{piece_counts.get('K', 0)}"
            )
        if piece_counts.get('k', 0) != 1:
            raise ValueError(
                f"Expected exactly 1 black king, found "
                f"{piece_counts.get('k', 0)}"
            )

        # Build a full FEN string (assume white to move, no castling info)
        # The game loop will track the actual turn and castling rights
        full_fen = f"{fen_placement} w - - 0 1"

        try:
            board = chess.Board(full_fen)
        except ValueError as e:
            raise ValueError(f"Invalid FEN: {e}")

        return board


# ═══════════════════════════════════════════════════════════════════
#  MAIN — test camera and/or detection
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Chess Vision System")
    parser.add_argument("--test", action="store_true",
                        help="Test camera only (show preview)")
    parser.add_argument("--detect", action="store_true",
                        help="Capture frame and detect board state")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index (default: 0)")
    parser.add_argument("--image", type=str, default=None,
                        help="Use an image file instead of camera")
    args = parser.parse_args()

    if args.test:
        print("=" * 60)
        print(" Camera Preview Test")
        print("=" * 60)
        cam = CameraCapture(device=args.camera)
        cam.show_preview()
        cam.release()

    elif args.detect:
        print("=" * 60)
        print(" Board Detection Test")
        print("=" * 60)

        detector = BoardDetector()

        if args.image:
            print(f"  Loading image: {args.image}")
            frame = cv2.imread(args.image)
            if frame is None:
                print(f"  ✗ Could not read image: {args.image}")
                sys.exit(1)
        else:
            cam = CameraCapture(device=args.camera)
            print("  Capturing frame...")
            frame = cam.grab_frame()
            cam.release()

        print("  Sending to Gemini Vision API...")
        try:
            board = detector.detect_board(frame)
            fen = board.fen()
            print(f"\n  Detected FEN: {fen}")
            print(f"\n  Board:\n{board}\n")
        except RuntimeError as e:
            print(f"\n  ✗ Detection failed: {e}")

    else:
        parser.print_help()
