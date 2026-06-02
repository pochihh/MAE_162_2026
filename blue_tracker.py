"""
blue_tracker.py — detects red targets, circles them, and prints the center.
Serves annotated feed at http://<pi-ip>:8081
"""

import subprocess
import threading
import time
import cv2
import numpy as np
from http.server import BaseHTTPRequestHandler, HTTPServer

DEVICE = "/dev/video10"
WIDTH  = 640
HEIGHT = 480
FPS    = 15
PORT   = 8081

FRAME_BYTES = WIDTH * HEIGHT * 3  # ffmpeg outputs bgr24

# Target: RGB(207,19,47) = HSV(351°,91%,81%) → OpenCV H=176, S=232, V=207
# Broad range: wide S/V floors to catch real-world lighting variation
HSV_RED_LOW1  = np.array([159, 131,  99], dtype=np.uint8)
HSV_RED_HIGH1 = np.array([179, 255, 255], dtype=np.uint8)
HSV_RED_LOW2  = np.array([  0, 131,  99], dtype=np.uint8)
HSV_RED_HIGH2 = np.array([ 11, 255, 255], dtype=np.uint8)

MIN_CONTOUR_AREA = 500  # px² — ignore tiny noise blobs

_frame_lock  = threading.Lock()
_latest_jpeg = b""
_latest_center: tuple[int, int] | None = None


def _process_frame(frame: np.ndarray) -> tuple[np.ndarray, tuple[int, int] | None]:
    """Detect blue blobs, draw circles, return annotated frame + largest center."""
    frame = np.ascontiguousarray(frame)
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, HSV_RED_LOW1, HSV_RED_HIGH1),
        cv2.inRange(hsv, HSV_RED_LOW2, HSV_RED_HIGH2),
    )

    # Clean up mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_center = None
    best_area   = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_CONTOUR_AREA:
            continue

        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        cx, cy, radius = int(cx), int(cy), int(radius)

        # Draw circle and crosshair
        cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
        cv2.drawMarker(frame, (cx, cy), (0, 255, 0),
                       cv2.MARKER_CROSS, markerSize=14, thickness=2)
        cv2.putText(frame, f"({cx}, {cy})", (cx + radius + 4, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        if area > best_area:
            best_area   = area
            best_center = (cx, cy)

    if best_center:
        cv2.putText(frame, f"target: {best_center}", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.putText(frame, "no target", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)

    return frame, best_center


def _capture_loop() -> None:
    global _latest_jpeg, _latest_center

    cmd = [
        "ffmpeg", "-loglevel", "quiet",
        "-f", "v4l2", "-input_format", "yuyv422",
        "-video_size", f"{WIDTH}x{HEIGHT}", "-framerate", str(FPS),
        "-i", DEVICE,
        "-f", "rawvideo", "-pix_fmt", "bgr24", "-",
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    print(f"[blue_tracker] opened {DEVICE}  {WIDTH}x{HEIGHT} @ {FPS} fps")

    while True:
        raw = proc.stdout.read(FRAME_BYTES)
        if len(raw) < FRAME_BYTES:
            print("[blue_tracker] ffmpeg stream ended, restarting…")
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            continue

        frame = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3)).copy()
        annotated, center = _process_frame(frame)

        if center is not None and center != _latest_center:
            print(f"[blue_tracker] center: {center}")

        _, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with _frame_lock:
            _latest_jpeg   = buf.tobytes()
            _latest_center = center


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path == "/":
            self._serve_index()
        elif self.path == "/stream":
            self._serve_stream()
        else:
            self.send_error(404)

    def _serve_index(self):
        html = (
            "<html><body style='background:#000;margin:0'>"
            "<img src='/stream' style='width:100%;height:auto'>"
            "</body></html>"
        ).encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def _serve_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    jpeg = _latest_jpeg
                if jpeg:
                    self.wfile.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n"
                        + jpeg + b"\r\n"
                    )
                time.sleep(1.0 / FPS)
        except (BrokenPipeError, ConnectionResetError):
            pass


if __name__ == "__main__":
    t = threading.Thread(target=_capture_loop, daemon=True)
    t.start()

    for _ in range(50):
        with _frame_lock:
            if _latest_jpeg:
                break
        time.sleep(0.1)

    import socket
    ip = socket.gethostbyname(socket.gethostname())
    print(f"[blue_tracker] live feed → http://{ip}:{PORT}")
    print(f"[blue_tracker] tracking red  H:0-10 + H:170-179")
    print("[blue_tracker] Ctrl+C to stop")
    server = HTTPServer(("0.0.0.0", PORT), _Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[blue_tracker] stopped")
