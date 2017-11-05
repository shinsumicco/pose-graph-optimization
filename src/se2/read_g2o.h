#ifndef SE2_READ_G2O_H
#define SE2_READ_G2O_H

#include <fstream>
#include <string>
#include <iostream>

#include "types.h"

namespace se2 {

void read_vertex(std::ifstream& infile, poses_t& poses) {
    int id;
    pose_2d pose;
    infile >> id >> pose;

    if (poses.count(id) != 0) {
        std::cout << "Duplicate vertex with ID: " << id << std::endl;
        exit(EXIT_FAILURE);
    }

    poses[id] = pose;
}

void read_constraint(std::ifstream& infile, constraints_t& constraints) {
    constraint_3d constraint;
    infile >> constraint;

    constraints.push_back(constraint);
}

void read_g2o_file(const std::string& filename, poses_t& poses, constraints_t& constraints) {
    poses.clear();
    constraints.clear();

    std::ifstream infile(filename.c_str());
    if (!infile) {
        std::cerr << "Invalid file name: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }

    while (infile.good()) {
        std::string data_type;
        infile >> data_type;
        if (data_type == "VERTEX_SE2") {
            read_vertex(infile, poses);
        }
        else if (data_type == "EDGE_SE2") {
            read_constraint(infile, constraints);
        }
        else {
            std::cout << "Unknown data type: " << data_type << std::endl;
            exit(EXIT_FAILURE);
        }
        infile >> std::ws;
    }
}

} // namespace se2

#endif // SE2_READ_G2O_H
