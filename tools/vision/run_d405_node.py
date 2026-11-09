#!/usr/bin/env python3
"""Run this checkout's passive ROS adapter with the selected Python environment."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'packages/d405_seam_tracking/src'),
                str(ROOT/'ros2_ws/src/functions/perception/d405_seam_ros')]
from d405_seam_ros.node import main

if __name__ == '__main__':
    main()
