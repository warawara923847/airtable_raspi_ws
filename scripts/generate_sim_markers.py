#!/usr/bin/python3
"""Regenerate the original ArUco dictionary textures used by the MuJoCo scene."""

from pathlib import Path

import cv2


def main():
    """Write reproducible marker images with a physical-size-preserving white border."""
    directory = Path(__file__).resolve().parents[1] / 'src/airtable_sim/mujoco/markers'
    directory.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    for marker_id in (10, 11, 12, 13, 50):
        # 224 px is 0.10 m; 56 px white margin on each side makes a 0.15 m card.
        marker = cv2.aruco.drawMarker(dictionary, marker_id, 224)
        card = cv2.copyMakeBorder(marker, 56, 56, 56, 56, cv2.BORDER_CONSTANT, value=255)
        path = directory / f'marker_{marker_id}.png'
        if not cv2.imwrite(str(path), card):
            raise RuntimeError(f'Cannot write {path}')
        print(path)


if __name__ == '__main__':
    main()
