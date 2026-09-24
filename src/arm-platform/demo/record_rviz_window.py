#!/usr/bin/env python3
"""Record one visible RViz X11 window to MP4 using existing Qt/OpenCV.

No ROS or robot interfaces. Keep the selected window visible during recording.
SIGINT/SIGTERM finishes the MP4 cleanly. Duration is a recording limit only.
"""
import argparse
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QGuiApplication, QImage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window-id', required=True, type=lambda x: int(x, 0))
    parser.add_argument('--output', required=True)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--duration', type=float, default=600.)
    args = parser.parse_args()
    if not 1 <= args.fps <= 30 or not 320 <= args.width <= 2560 or not 0 < args.duration <= 3600:
        parser.error('invalid fps/width/duration')
    path = Path(args.output)
    if path.exists():
        parser.error('output already exists')
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(1)
    app = QGuiApplication(sys.argv[:1])
    screen = app.primaryScreen()
    pix = screen.grabWindow(args.window_id)
    if pix.isNull():
        raise RuntimeError('RViz window capture unavailable')
    width = args.width//2*2
    size = (width, int(round(width*pix.height()/pix.width()/2))*2)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), args.fps, size)
    if not writer.isOpened():
        raise RuntimeError('MP4 encoder unavailable')
    started = time.monotonic()
    frames = 0
    failed = False

    def capture():
        nonlocal frames, failed
        try:
            if time.monotonic()-started >= args.duration:
                app.quit()
                return
            pix = screen.grabWindow(args.window_id)
            if pix.isNull():
                raise RuntimeError('RViz window disappeared')
            img = pix.toImage().convertToFormat(QImage.Format_RGB888)
            ptr = img.bits()
            ptr.setsize(img.byteCount())
            rgb = np.frombuffer(ptr, np.uint8).reshape(img.height(), img.bytesPerLine())[:, :img.width()*3]
            rgb = rgb.reshape(img.height(), img.width(), 3)
            frame = cv2.cvtColor(cv2.resize(rgb, size), cv2.COLOR_RGB2BGR)
            # Wall-clock label makes any recording slowdown visible.
            elapsed = time.monotonic()-started
            cv2.putText(frame, f'REC  {elapsed:06.1f}s', (16, size[1]-42),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (90, 220, 255), 1, cv2.LINE_AA)
            # Qt's coarse timer may fire early; count frames from elapsed time
            # so the nominal MP4 frame rate does not change playback speed.
            due = int(elapsed*args.fps)+1
            while frames < due:
                writer.write(frame)
                frames += 1
        except Exception as error:
            failed = True
            print(f'RECORDING ERROR: {error}', flush=True)
            app.quit()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    timer = QTimer()
    timer.setTimerType(Qt.PreciseTimer)
    timer.timeout.connect(capture)
    timer.start(round(1000/args.fps))
    print(f'RECORDING {path} {size[0]}x{size[1]} at {args.fps} fps', flush=True)
    try:
        app.exec_()
    finally:
        writer.release()
        print(f'SAVED {path}: {frames} frames, {time.monotonic()-started:.2f} wall seconds', flush=True)
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
