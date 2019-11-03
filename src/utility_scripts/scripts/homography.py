#! /usr/bin/env

import matplotlib.pyplot as plt
from PIL import Image
import pdb
import numpy as np
from operator import itemgetter
import cv2
import argparse
import yaml


def compute_homography(
    image_fname,
    num_points,
    tile_size,
    output_size=(
        100,
        100)):
    f, (ax1, ax2) = plt.subplots(1, 2)
    # pdb.set_trace()
    # note that matplotlib only accepts PIL images natively
    reference_image = Image.open(image_fname)
    ax1.imshow(reference_image)
    # ax2.imshow(np.zeros(output_size).astype(np.uint8))
    # Major ticks every 20, minor ticks every 5
    x_ticks = np.arange(-1 * output_size[0] / 2.0,
                        output_size[0] / 2.0 + 1,
                        output_size[0] / 10.0)
    y_ticks = np.arange(0, output_size[1] + 1, output_size[1] / 10.0)

    ax2.set_xticks(x_ticks)
    ax2.set_yticks(y_ticks)
    ax2.arrow(0, 0, 0, output_size[1] / 10.0, head_width=.02 * output_size[0])

    # And a corresponding grid
    ax2.grid(which='both')
    plt.title(
        "click {} coresponding points in each image, by starting with the left one and alternating".format(num_points))
    # get the clicked points
    f.show()
    points = plt.ginput(num_points * 2, timeout=2000, show_clicks=True)

    # close all the plots
    plt.close()
    evens = [x for x in range(0, 2 * num_points, 2)]
    odds = [x for x in range(1, 2 * num_points, 2)]
    source_pts = np.asarray(itemgetter(*evens)(points))
    dest_pts = np.asarray(itemgetter(*odds)(points)) * tile_size

    dest_pts[:, 0] += output_size[0] / 2.0
    # To account for the reversed convention in an image
    dest_pts[:, 1] = output_size[1] - dest_pts[:, 1]
    pdb.set_trace()
    print("source points {}, dest points {} ".format(source_pts, dest_pts))
    homography, status = cv2.findHomography(source_pts, dest_pts)
    print("homography {} ".format(homography))
    return homography


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--num-points",
        default=4,
        help="More than 4",
        type=int)
    parser.add_argument(
        "--tile-size",
        default=1,
        help="The size in world units of a tile",
        type=float)
    parser.add_argument("--output-size", default=(100, 100), type=tuple)
    parser.add_argument("--output-file", default="homography.yaml", type=str)
    parser.add_argument(
        "--calib-image",
        default="../../../data/calibration_image_pacman.png",
        help="the file path")

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    homography = compute_homography(
        args.calib_image,
        args.num_points,
        args.tile_size,
        args.output_size)
    with open(args.output_file, 'w') as f:
        yaml.dump(homography.tolist(), f)
    warped = cv2.warpPerspective(
        cv2.imread(
            args.calib_image),
        homography,
        args.output_size)
    cv2.imshow(
        "warped",
        cv2.resize(
            warped,
            dsize=(
                args.output_size[0] *
                5,
                args.output_size[1] *
                5)))
    cv2.waitKey(0)
