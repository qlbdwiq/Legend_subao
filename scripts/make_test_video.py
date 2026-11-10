"""Generate a non-sensitive 720p30 CPU test video; never needs a camera."""
import argparse
from pathlib import Path
import sys

# Use the same dependencies as phase 01, no network access or downloaded assets.
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=int, default=10)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error("--seconds must be from 1 to 300")
    if args.output.exists():
        parser.error("Output already exists; choose a new filename.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*"MJPG"), 30, (1280, 720))
    if not writer.isOpened():
        print("Cannot create MJPEG video. Use an .avi filename and check permissions.", file=sys.stderr)
        return 1
    try:
        for index in range(args.seconds * 30):
            image = np.full((720, 1280, 3), (28, 20, 14), dtype=np.uint8)
            cv2.circle(image, (80 + (index * 7) % 1120, 360), 45, (60, 210, 250), -1)
            cv2.putText(image, f"PHASE 01 TEST  |  frame {index}  |  1280x720 30fps",
                        (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (230, 240, 250), 2)
            writer.write(image)
    finally:
        writer.release()
    print(args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
