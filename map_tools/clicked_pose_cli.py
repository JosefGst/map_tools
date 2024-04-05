#! /usr/bin/env python3

import argparse


def init_cli(args):
    parser = argparse.ArgumentParser(
        prog="pose",
        description="Subscribe to the clicked_pose topic and save it to a file. \nExample usage: rosrun map_tools clicked_pose.py \nExample usage: rosrun map_tools clicked_pose.py -o my_pose.txt -e ", formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="poses.txt",
        help='Outputs to the specified file path eg."my_pose.txt".',
    )
    parser.add_argument(
        "-e",
        "--euler",
        action="store_true",
        help="Output the orientation in Euler angles [degree].",
    )
    parser.add_argument("-v", "--version", action="version", version="%(prog)s 0.1.0")

    return parser.parse_args(args)