#!/usr/bin/env python3
"""
=============================================================================
 Visual Feedback — Camera + gripper-load verification for pick-and-place
=============================================================================

Provides closed-loop feedback for the chess robot's pick-and-place motions.

Instead of blindly trusting the gripper, we verify at two critical moments:

    1. AFTER GRIPPING:  Does the gripper load indicate we're holding a piece?
    2. AFTER LIFTING:   Is the source square now empty?  (camera + Gemini)
    3. AFTER PLACING:   Is there a piece on the target square? (camera + Gemini)

GRIPPER LOAD SENSING:
    The STS3215 servos report their current load.  When the gripper closes
    on a chess piece, the load spikes (resistance from the piece).  When it
    closes on air, the load stays near zero.  We use a threshold to tell
    the difference.

CAMERA / GEMINI VISION:
    Instead of detecting the full board (slow, error-prone), we ask Gemini
    a focused yes/no question about a SINGLE SQUARE:
        "Is there a chess piece on square e2?"
    This is faster, cheaper, and more reliable.

USAGE:
    from visual_feedback import PickPlaceVerifier
    from chess_vision import CameraCapture
    from arm_control import RobotArm

    camera = CameraCapture(device=0)
    arm = RobotArm()
    verifier = PickPlaceVerifier(arm, camera)

    # After closing the gripper:
    has_piece = verifier.verify_grip()

    # After lifting the piece:
    source_empty = verifier.verify_square_empty("e2")

    # After placing the piece:
    piece_placed = verifier.verify_square_occupied("e4")
=============================================================================
"""

import os
import sys
import time
import threading

try:
    import cv2
except ImportError:
    print("  ✗ OpenCV not found. Install with: pip install opencv-python")
    sys.exit(1)

try:
    import google.generativeai as genai
except ImportError:
    print("  ✗ google-generativeai not found. Install with: "
          "pip install google-generativeai")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────

# Gripper load threshold — loads above this (absolute value) mean a piece
# is being gripped.  Loads below mean squeezing air.
# Tune this based on your servo and piece weight.
GRIP_LOAD_THRESHOLD = 40

# How many times to retry a failed pick before giving up.
MAX_PICK_RETRIES = 2

# How many encoder ticks to nudge downward on each grip retry.
# This compensates for slight calibration drift.
GRIP_RETRY_NUDGE = 15

# Pause after gripper settles before reading load (seconds).
LOAD_READ_DELAY = 0.3

# Gemini Vision prompts — focused yes/no questions for speed and reliability.
SQUARE_EMPTY_PROMPT = """Look at this top-down photograph of a physical chessboard.

Focus ONLY on square {square} (file {file}, rank {rank}).

Is square {square} EMPTY (no chess piece on it)?

Rules:
- The board is viewed from above (top-down).
- White pieces are on ranks 1-2 (bottom), Black pieces on ranks 7-8 (top).
- Files go a-h from left to right.
- Answer ONLY "YES" if the square is empty, or "NO" if there is a piece on it.
- Do NOT explain. Just answer YES or NO."""

SQUARE_OCCUPIED_PROMPT = """Look at this top-down photograph of a physical chessboard.

Focus ONLY on square {square} (file {file}, rank {rank}).

Is there a chess piece on square {square}?

Rules:
- The board is viewed from above (top-down).
- White pieces are on ranks 1-2 (bottom), Black pieces on ranks 7-8 (top).
- Files go a-h from left to right.
- Answer ONLY "YES" if there is a piece on the square, or "NO" if it is empty.
- Do NOT explain. Just answer YES or NO."""


# ─────────────────────────────────────────────────────────────────────────────
# LiveFeed — continuous camera preview in a background thread
# ─────────────────────────────────────────────────────────────────────────────

class LiveFeed:
    """
    Runs a continuous camera preview in a background thread.

    The live feed window shows the camera image in real-time with an
    optional status overlay (e.g., "GRIPPING e2", "VERIFYING e4").
    Verification frames can be grabbed from the live feed without
    interrupting the display.
    """

    WINDOW_NAME = "♟ Chess Robot — Live Camera"

    def __init__(self, camera):
        self.camera = camera
        self._running = False
        self._thread = None
        self._status_text = "Initializing..."
        self._latest_frame = None
        self._lock = threading.Lock()

    def start(self):
        """Start the live feed background thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print("  📺 Live camera feed started (press 'q' in the window to close)")

    def stop(self):
        """Stop the live feed and close the window."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            cv2.destroyWindow(self.WINDOW_NAME)
        except Exception:
            pass

    def set_status(self, text: str):
        """Update the overlay status text (e.g., 'Gripping E2')."""
        self._status_text = text

    def grab_frame(self):
        """Get the latest frame from the live feed (thread-safe)."""
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
        # Fallback: grab directly
        return self.camera.grab_frame()

    def _run(self):
        """Background loop: read frames and display them."""
        while self._running:
            try:
                frame = self.camera.grab_frame()
                with self._lock:
                    self._latest_frame = frame.copy()

                # Draw status overlay
                display = frame.copy()
                self._draw_overlay(display)

                cv2.imshow(self.WINDOW_NAME, display)
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    self._running = False
                    break
            except Exception:
                time.sleep(0.1)

    def _draw_overlay(self, frame):
        """Draw status text on the frame."""
        h, w = frame.shape[:2]
        # Semi-transparent bar at bottom
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, h - 40), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        # Status text
        cv2.putText(frame, self._status_text, (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


# ─────────────────────────────────────────────────────────────────────────────
# PickPlaceVerifier
# ─────────────────────────────────────────────────────────────────────────────

class PickPlaceVerifier:
    """
    Verifies pick-and-place actions using gripper load sensing and camera
    vision via the Gemini API.

    When a camera is provided, a live feed window shows the camera in
    real-time throughout the entire pick-and-place operation.

    Usage:
        verifier = PickPlaceVerifier(arm, camera)
        has_piece = verifier.verify_grip()
        is_empty  = verifier.verify_square_empty("e2")
        occupied  = verifier.verify_square_occupied("e4")
    """

    def __init__(self, arm, camera=None, api_key: str = None,
                 model: str = "gemini-2.5-flash",
                 load_threshold: int = GRIP_LOAD_THRESHOLD,
                 save_frames: bool = True):
        """
        Args:
            arm:             RobotArm instance (for reading gripper load)
            camera:          CameraCapture instance (None = skip vision checks)
            api_key:         Gemini API key (reads GEMINI_API_KEY if None)
            model:           Gemini model name
            load_threshold:  Gripper load above this = piece gripped
            save_frames:     If True, save verification frames to disk
        """
        self.arm = arm
        self.camera = camera
        self.load_threshold = load_threshold
        self.save_frames = save_frames
        self.gemini_model = None
        self.live_feed = None

        # Directory for saved verification frames
        self.frames_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "verification_frames"
        )
        if save_frames:
            os.makedirs(self.frames_dir, exist_ok=True)

        # Start live camera feed if camera is available
        if camera is not None:
            self.live_feed = LiveFeed(camera)
            self.live_feed.start()

        # Set up Gemini only if camera is provided
        if camera is not None:
            api_key = api_key or os.environ.get("GEMINI_API_KEY")
            if api_key:
                genai.configure(api_key=api_key)
                self.gemini_model = genai.GenerativeModel(model)
                print(f"  🔍 Visual feedback ready (model: {model})")
            else:
                print("  ⚠ No GEMINI_API_KEY — visual checks disabled, "
                      "gripper load checks still active.")

    def stop_feed(self):
        """Stop the live camera feed."""
        if self.live_feed:
            self.live_feed.stop()

    # ─────────────────────────────────────────────────────────────────
    # Gripper load verification
    # ─────────────────────────────────────────────────────────────────

    def verify_grip(self) -> bool:
        """
        Check if the gripper is actually holding a piece by reading
        the servo load.

        Returns True if the gripper load exceeds the threshold (piece held).
        Returns False if the load is near zero (gripping air).
        """
        if self.live_feed:
            self.live_feed.set_status("CHECKING GRIP...")

        time.sleep(LOAD_READ_DELAY)

        # Read load multiple times and average to reduce noise
        loads = []
        for _ in range(3):
            load = self.arm.get_joint_load("gripper")
            loads.append(abs(load))
            time.sleep(0.05)

        avg_load = sum(loads) / len(loads)
        has_piece = avg_load > self.load_threshold

        if has_piece:
            status = f"GRIP OK (load={avg_load:.0f})"
            print(f"      \u2713 Grip verified (load={avg_load:.0f} > "
                  f"threshold={self.load_threshold})")
        else:
            status = f"NO GRIP (load={avg_load:.0f})"
            print(f"      \u2717 No piece detected in gripper "
                  f"(load={avg_load:.0f} \u2264 threshold={self.load_threshold})")

        if self.live_feed:
            self.live_feed.set_status(status)

        return has_piece

    # ─────────────────────────────────────────────────────────────────
    # Camera / Gemini Vision verification
    # ─────────────────────────────────────────────────────────────────

    def verify_square_empty(self, square: str) -> bool:
        """
        Use the camera + Gemini to check if a square is empty.

        This is called AFTER picking up a piece — the source square
        should now be empty.

        Returns True if the square appears empty, False if a piece
        is still there.  Returns True (optimistic) if vision is
        unavailable.
        """
        if self.live_feed:
            self.live_feed.set_status(f"CHECKING: is {square.upper()} empty?")

        # Save a frame even without Gemini so user can review
        self._save_frame_if_enabled(f"pick_{square}")

        if not self._vision_available():
            return True  # optimistic fallback

        file_letter = square[0]
        rank_number = square[1]

        prompt = SQUARE_EMPTY_PROMPT.format(
            square=square.upper(),
            file=file_letter.upper(),
            rank=rank_number,
        )

        answer = self._ask_gemini(prompt, label=f"pick_{square}")
        is_empty = answer.upper().startswith("YES")

        if is_empty:
            self._set_feed_status(f"{square.upper()} EMPTY \u2713")
            print(f"      \u2713 Square {square.upper()} confirmed empty "
                  f"(camera check)")
        else:
            self._set_feed_status(f"{square.upper()} NOT EMPTY \u2717")
            print(f"      \u2717 Square {square.upper()} still has a piece! "
                  f"(camera check)")

        return is_empty

    def verify_square_occupied(self, square: str) -> bool:
        """
        Use the camera + Gemini to check if a square has a piece on it.

        This is called AFTER placing a piece — the target square
        should now be occupied.

        Returns True if a piece is detected.  Returns True (optimistic)
        if vision is unavailable.
        """
        if self.live_feed:
            self.live_feed.set_status(f"CHECKING: piece on {square.upper()}?")

        # Save a frame even without Gemini so user can review
        self._save_frame_if_enabled(f"place_{square}")

        if not self._vision_available():
            return True  # optimistic fallback

        file_letter = square[0]
        rank_number = square[1]

        prompt = SQUARE_OCCUPIED_PROMPT.format(
            square=square.upper(),
            file=file_letter.upper(),
            rank=rank_number,
        )

        answer = self._ask_gemini(prompt, label=f"place_{square}")
        is_occupied = answer.upper().startswith("YES")

        if is_occupied:
            self._set_feed_status(f"PIECE ON {square.upper()} \u2713")
            print(f"      \u2713 Piece confirmed on {square.upper()} "
                  f"(camera check)")
        else:
            self._set_feed_status(f"NO PIECE ON {square.upper()} \u2717")
            print(f"      \u2717 No piece detected on {square.upper()}! "
                  f"(camera check)")

        return is_occupied

    # ─────────────────────────────────────────────────────────────────
    # Internal helpers
    # ─────────────────────────────────────────────────────────────────

    def _vision_available(self) -> bool:
        """Check if camera and Gemini are available for vision queries."""
        return self.camera is not None and self.gemini_model is not None

    def _set_feed_status(self, text: str):
        """Update the live feed overlay text."""
        if self.live_feed:
            self.live_feed.set_status(text)

    def _save_frame_if_enabled(self, label: str):
        """Save a verification frame to disk (grabs from live feed)."""
        if not self.save_frames or self.camera is None:
            return
        from datetime import datetime
        frame = (self.live_feed.grab_frame() if self.live_feed
                 else self.camera.grab_frame())
        timestamp = datetime.now().strftime("%H%M%S")
        filename = f"{timestamp}_{label}.jpg"
        filepath = os.path.join(self.frames_dir, filename)
        cv2.imwrite(filepath, frame)
        print(f"      \ud83d\udcf8 Frame saved: {filepath}")

    def _ask_gemini(self, prompt: str, label: str = "verify") -> str:
        """
        Grab a frame from the live feed and ask Gemini a focused question.

        Args:
            prompt:  The Gemini prompt to send with the image.
            label:   A short label for logging.

        Returns the raw text response (should be "YES" or "NO").
        """
        import PIL.Image
        import io

        # Grab frame from live feed (or camera directly)
        frame = (self.live_feed.grab_frame() if self.live_feed
                 else self.camera.grab_frame())

        # Convert to JPEG bytes → PIL Image
        _, jpeg_bytes = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
        )
        image = PIL.Image.open(io.BytesIO(jpeg_bytes.tobytes()))

        try:
            response = self.gemini_model.generate_content(
                [prompt, image],
                generation_config=genai.types.GenerationConfig(
                    temperature=0.1,
                    max_output_tokens=10,  # YES/NO is very short
                ),
            )
            text = response.text.strip().upper()
            return text

        except Exception as e:
            print(f"      \u26a0 Gemini query failed: {e}")
            return "YES"  # optimistic fallback on API error


# ═══════════════════════════════════════════════════════════════════
#  MAIN — standalone testing tools
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Visual Feedback — test gripper load and camera checks"
    )
    parser.add_argument("--test-load", action="store_true",
                        help="Test gripper load detection (close gripper "
                             "on a piece vs air)")
    parser.add_argument("--test-vision", action="store_true",
                        help="Test camera square verification")
    parser.add_argument("--square", type=str, default="e2",
                        help="Square to check for --test-vision (default: e2)")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index (default: 0)")
    parser.add_argument("--threshold", type=int, default=GRIP_LOAD_THRESHOLD,
                        help=f"Gripper load threshold (default: "
                             f"{GRIP_LOAD_THRESHOLD})")
    args = parser.parse_args()

    if args.test_load:
        print("=" * 60)
        print(" Gripper Load Test")
        print("=" * 60)
        print("  Close the gripper on a chess piece, then press ENTER.")
        print("  Then close on air and press ENTER again.\n")

        from arm_control import RobotArm
        arm = RobotArm()
        arm.enable()
        time.sleep(0.5)

        verifier = PickPlaceVerifier(arm, camera=None,
                                     load_threshold=args.threshold)

        try:
            # Test with piece
            input("  Close gripper on a PIECE, then press ENTER... ")
            print("  Reading gripper load...")
            verifier.verify_grip()

            # Test without piece
            input("\n  Close gripper on AIR (nothing), then press ENTER... ")
            print("  Reading gripper load...")
            verifier.verify_grip()

        finally:
            print("\n  Arm is holding position. Press ENTER to release torque...")
            input()
            arm.disable()
            arm.close()

    elif args.test_vision:
        print("=" * 60)
        print(" Camera Square Verification Test")
        print("=" * 60)
        print(f"  Testing square: {args.square.upper()}\n")

        from arm_control import RobotArm
        from chess_vision import CameraCapture

        arm = RobotArm()
        camera = CameraCapture(device=args.camera)

        verifier = PickPlaceVerifier(arm, camera,
                                     load_threshold=args.threshold)

        try:
            print(f"\n  Checking if {args.square.upper()} is empty...")
            verifier.verify_square_empty(args.square)

            print(f"\n  Checking if {args.square.upper()} is occupied...")
            verifier.verify_square_occupied(args.square)

        finally:
            print("\n  Arm is holding position. Press ENTER to release torque...")
            input()
            verifier.stop_feed()
            camera.release()
            arm.close()

    else:
        parser.print_help()
