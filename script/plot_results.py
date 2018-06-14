#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy
import argparse
import matplotlib.pyplot as plot


parser = argparse.ArgumentParser()
parser.add_argument(dest="initial_poses", default="",
                    help="The filename that contains the original poses")
parser.add_argument(dest="optimized_poses", default="",
                    help="The filename that contains the optimized poses")
args = parser.parse_args()

poses_original = None
if args.initial_poses != '':
    poses_original = numpy.genfromtxt(args.initial_poses, delimiter=",", usecols = (1, 2))

poses_optimized = None
if args.optimized_poses != '':
    poses_optimized = numpy.genfromtxt(args.optimized_poses, delimiter=",", usecols = (1, 2))

plot.figure()
if poses_original is not None:
    plot.plot(poses_original[:, 0], poses_original[:, 1], '-', label="Original", alpha=0.5, color="green")

if poses_optimized is not None:
    plot.plot(poses_optimized[:, 0], poses_optimized[:, 1], '-', label="Optimized", alpha=0.5, color="blue")

plot.axis('equal')
plot.legend()
plot.show()
