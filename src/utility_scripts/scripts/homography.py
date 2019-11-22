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
        100),
    num_ticks=20):
    f, (ax1, ax2) = plt.subplots(1, 2)
    # pdb.set_trace()
    # note that matplotlib only accepts PIL images natively
    reference_image = Image.open(image_fname)
    ax1.imshow(reference_image)
    # ax2.imshow(np.zeros(output_size).astype(np.uint8))
    # Major ticks every 20, minor ticks every 5

    x_ticks = np.arange(-1 * output_size[0] / 2.0,
                        output_size[0] / 2.0 + 1,
                        output_size[0] / float(num_ticks))
    y_ticks = np.arange(0, output_size[1] + 1, output_size[1] / float(num_ticks))

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
    dest_pts[:, 1] = (output_size[1] - (dest_pts[:, 1])) 
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
    parser.add_argument("--output-size", default=(100, 100), nargs="+", type=int, help="<x> <y>" )
    parser.add_argument("--output-file", default="homography.yaml", type=str)
    parser.add_argument(
        "--calib-image",
        default="../../../data/calibration_image_pacman.png",
        help="the file path")
    parser.add_argument(
        "--segmentation-image",
        default="../../../data/path_prob.png",
        help="An example segmentation mask to warp")
    parser.add_argument(
        "--just-warp",
        action="store_true")

    return parser.parse_args()

def warp(img, homography, output_size, input_size):
    resized = cv2.resize(img, input_size)
    warped = cv2.warpPerspective(
        resized,
        homography,
        output_size)
    return warped

if __name__ == "__main__":
    args = parse_args()
    if args.just_warp:
        with open(args.output_file, 'r') as f:
            h = yaml.safe_load(f)
        img = cv2.imread(args.calib_image)
        seg = cv2.imread(args.segmentation_image)
        homography = np.asarray(h['homography'])
        warped = warp(seg, homography, tuple(h['output_shape']), (img.shape[1], img.shape[0]))

        cv2.imwrite("warp_input.png", seg)
        cv2.imwrite("warp_output.png", warped)
    else:
        homography = compute_homography(
            args.calib_image,
            args.num_points,
            args.tile_size,
            tuple(args.output_size))
        with open(args.output_file, 'w') as f:
            img = cv2.imread(args.calib_image)
            input_shape = img.shape
            output_dict = {}
            output_dict["homography"] = homography.tolist()
            output_dict["input_shape"] = list(input_shape)
            output_dict["output_shape"] = list(tuple(args.output_size))
            yaml.dump(output_dict, f)

        input_img = cv2.imread(args.calib_image)
        warped = cv2.warpPerspective(
            input_img,
            homography,
            tuple(args.output_size))
        resized_warped = cv2.resize(
                warped,
                dsize=(
                    args.output_size[0] *
                    5,
                    args.output_size[1] *
                    5))
        cv2.imshow("warped", resized_warped)
        cv2.imwrite("warped_calibration.png", resized_warped)
        
        segmentation_image = cv2.imread(args.segmentation_image)
        segmentation_image = cv2.resize(segmentation_image, (input_img.shape[0], input_img.shape[1]))

        warped_mask = cv2.warpPerspective(
            segmentation_image,
            homography,
            tuple(args.output_size))
        resized_warped_mask = cv2.resize(
                warped_mask,
                dsize=(
                    args.output_size[0] *
                    5,
                    args.output_size[1] *
                    5))
        cv2.imshow("mask", resized_warped_mask)
        cv2.imwrite("warped_mask.png", resized_warped_mask)

        cv2.waitKey(0)
