#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shared helpers for writing PCD files from PointCloud2 messages."""

import math
import struct
from typing import List, Tuple


def extract_valid_points(msg, max_abs: float = 1000.0) -> Tuple[List[Tuple[float, float, float, float]], bool]:
    """Extract valid XYZ(I) points from a PointCloud2 message.

    Returns:
        points: list of (x, y, z, intensity)
        has_intensity: whether the message has an intensity field
    """
    field_names = [field.name for field in msg.fields]
    has_intensity = "intensity" in field_names
    point_step = msg.point_step
    data = msg.data

    valid_points: List[Tuple[float, float, float, float]] = []
    for i in range(0, len(data), point_step):
        if i + 12 > len(data):
            break
        try:
            x = struct.unpack("<f", data[i : i + 4])[0]
            y = struct.unpack("<f", data[i + 4 : i + 8])[0]
            z = struct.unpack("<f", data[i + 8 : i + 12])[0]
            if has_intensity and i + 16 <= len(data):
                intensity = struct.unpack("<f", data[i + 12 : i + 16])[0]
            else:
                intensity = 0.0
        except struct.error:
            continue

        if (
            not math.isnan(x)
            and not math.isnan(y)
            and not math.isnan(z)
            and abs(x) < max_abs
            and abs(y) < max_abs
            and abs(z) < max_abs
        ):
            valid_points.append((x, y, z, intensity))

    return valid_points, has_intensity


def write_pcd_ascii(path: str, points: List[Tuple[float, float, float, float]], has_intensity: bool) -> None:
    """Write PCD file in ASCII format."""
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")

        if has_intensity:
            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
        else:
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")

        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")

        for point in points:
            if has_intensity:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n")
            else:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
