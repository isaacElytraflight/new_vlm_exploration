#!/usr/bin/env python3
"""Display the latest habitat live frame on DISPLAY for noVNC.

cv2.imshow updates the X11 framebuffer with proper damage events; feh's
file-reload path often leaves x11vnc stuck near ~1 FPS even when JPEGs
are written at 15+ FPS.
"""

import os
import signal

import cv2

FRAME = os.environ.get("HABITAT_LIVE_FRAME", "/tmp/habitat_live/frame.jpg")
FPS = float(os.environ.get("HABITAT_VIEW_FPS", "15"))
WINDOW = "habitat-live"

_running = True


def _stop(*_args) -> None:
    global _running
    _running = False


def main() -> None:
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 640, 480)
    delay_ms = max(1, int(1000 / max(FPS, 1.0)))

    while _running:
        frame = cv2.imread(FRAME)
        if frame is not None:
            cv2.imshow(WINDOW, frame)
        key = cv2.waitKey(delay_ms) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
