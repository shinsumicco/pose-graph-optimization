#include <iostream>

#include <ceres/ceres.h>
#include <gflags/gflags.h>

#include "problem.h"
#include "read_g2o.h"

DEFINE_string(filename, "", ".g2o file name to read");

int main(int argc, char* argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_filename.empty()) {
        std::cerr << "Error: Use --filename flag to give a file name" << std::endl;
        return EXIT_FAILURE;
    }

    se2::poses_t poses;
    se2::constraints_t constraints;

    se2::read_g2o_file(FLAGS_filename, poses, constraints);

    std::cout << "Number of poses: " << poses.size() << std::endl;
    std::cout << "Number of constraints: " << constraints.size() << std::endl;

    se2::output_poses("poses_original.txt", poses);

    ceres::Problem problem;
    se2::build_problem(constraints, poses, problem);

    se2::solve_problem(problem);

    se2::output_poses("poses_optimized.txt", poses);

    return 0;
}
