#! /usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import math
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--radius", type=float, default=50.0,
                    help="radius of the circle")
parser.add_argument("--nodes", type=int, default=100,
                    help="the number of nodes")
parser.add_argument("--drift", type=float, default=0.2,
                    help="the coefficient of drift error")
args = parser.parse_args()

radius = args.radius
num_nodes = args.nodes
drift_coeff = args.drift

vertices = []
delta_theta = 2 * math.pi * (1 - drift_coeff) / num_nodes
delta_length = 2 * math.pi * radius / num_nodes

for i in xrange(num_nodes):
    x = radius * math.cos(delta_theta * i + np.random.normal(0, math.pi / 90.0))
    y = radius * math.sin(delta_theta * i + np.random.normal(0, math.pi / 90.0))
    theta = math.pi / 2 + delta_theta * i + np.random.normal(0, math.pi / 90.0)
    vertices.append(["VERTEX_SE2", i, x, y, theta])

constraint = []

for i in xrange(num_nodes - 1):
    (_, i_a, x_a, y_a, theta_a) = vertices[i]
    (_, i_b, x_b, y_b, theta_b) = vertices[i + 1]
    constraint.append(["EDGE_SE2", i_a, i_b,
                       delta_length * math.sin(delta_theta + np.random.normal(0, math.pi / 180.0)),
                       delta_length * math.cos(delta_theta + np.random.normal(0, math.pi / 180.0)),
                       delta_theta + np.random.normal(0, math.pi / 180.0),
                       1.0, 0, 0, 1.0, 0, 1.0])

constraint.append(["EDGE_SE2", num_nodes - 1, 0,
                   delta_length * math.sin(delta_theta + np.random.normal(0, math.pi / 180.0)),
                   delta_length * math.cos(delta_theta + np.random.normal(0, math.pi / 180.0)),
                   delta_theta + np.random.normal(0, math.pi / 180.0),
                   1.0, 0, 0, 1.0, 0, 1.0])

with open("drifted_circle.g2o", "w") as fout:
    writer = csv.writer(fout, delimiter=" ", lineterminator="\n")
    writer.writerows(vertices)

with open("drifted_circle.g2o", "a") as fout:
    writer = csv.writer(fout, delimiter=" ", lineterminator="\n")
    writer.writerows(constraint)
