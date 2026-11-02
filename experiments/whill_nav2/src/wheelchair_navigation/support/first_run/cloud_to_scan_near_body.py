#!/usr/bin/env python3
"""Apply the observed left-side self-return mask before the scan is formed."""
import numpy as np


def near_body_ghost_mask(points):
    return ((points[:, 0] >= 0.25) & (points[:, 0] <= 0.42) &
            (points[:, 1] >= 0.285) & (points[:, 1] <= 0.32) &
            (points[:, 2] >= 0.20) & (points[:, 2] <= 0.35))


def main():
    from wheelchair_hardware_bridge import cluster_filter, cloud_to_scan

    original = cluster_filter.small_cluster_mask

    def suppress_near_body(points, center_x=0.):
        return np.logical_or(original(points, center_x), near_body_ghost_mask(points))

    cluster_filter.small_cluster_mask = suppress_near_body
    cloud_to_scan.main()


if __name__ == '__main__':
    main()
