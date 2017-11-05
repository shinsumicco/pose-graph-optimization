#ifndef SE2_PROBLEM_H
#define SE2_PROBLEM_H

#include <iostream>
#include <fstream>
#include <string>

#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>

#include "types.h"
#include "error_function.h"

namespace se2 {

void build_problem(const constraints_t& constraints, poses_t& poses, ceres::Problem& problem) {
    ceres::LossFunction* loss_function = nullptr;

    for (const auto& constraint: constraints) {
        auto pose_start_itr = poses.find(constraint.id_start);
        if (pose_start_itr == poses.end()) {
            std::cerr << "Pose ID: " << constraint.id_start << " not found." << std::endl;
            continue;
        }
        auto pose_end_itr = poses.find(constraint.id_end);
        if (pose_end_itr == poses.end()) {
            std::cerr << "Pose ID: " << constraint.id_end << " not found." << std::endl;
            continue;
        }

        // 情報行列をコレスキー分解してマハラノビス距離の計算に用いる
        // (対角行列を想定しているので，情報行列の対角成分の平方根を格納する)
        // (マハラノビス距離の定義を参照)
        std::array<double, 3> decomposed_information;
        for (unsigned int i = 0; i < 3; ++i) {
            decomposed_information.at(i) = sqrt(constraint.information.at(i));
        }

        // AutoDiffCostFunctionでコスト関数とヤコビアンを構築
        // テンプレート引数
        //     残差パラメータ: 3個
        //     第1引数: 1個 (絶対位置A x)
        //     第2引数: 1個 (絶対位置A y)
        //     第3引数: 1個 (絶対位置A theta)
        //     第4引数: 1個 (絶対位置B x)
        //     第5引数: 1個 (絶対位置B y)
        //     第6引数: 1個 (絶対位置B theta)
        //
        // 相対姿勢[x, y, theta]とコレスキー分解した情報行列の対角成分decomposed_informationを設定する
        // (相対姿勢は局所的には信頼できるものとする)
        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<error_function, 3, 1, 1, 1, 1, 1, 1>(
                        new error_function(constraint.x, constraint.y, constraint.theta, decomposed_information));

        // 残差項を設定
        // cost_function構築時に入れた相対姿勢に対して，その相対姿勢の両端のノードの絶対姿勢を入れて絶対姿勢の誤差を計算させる
        problem.AddResidualBlock(cost_function, loss_function,
                                 &(pose_start_itr->second.x),
                                 &(pose_start_itr->second.y),
                                 &(pose_start_itr->second.theta),
                                 &(pose_end_itr->second.x),
                                 &(pose_end_itr->second.y),
                                 &(pose_end_itr->second.theta));
    }

    auto pose_start_itr = poses.begin();
    if (pose_start_itr == poses.end()) {
        std::cerr << "There are no poses" << std::endl;
        exit(EXIT_FAILURE);
    }

    // 経路の始点を最適化時に定数とする
    problem.SetParameterBlockConstant(&(pose_start_itr->second.x));
    problem.SetParameterBlockConstant(&(pose_start_itr->second.y));
    problem.SetParameterBlockConstant(&(pose_start_itr->second.theta));
}

bool solve_problem(ceres::Problem& problem) {
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    return summary.IsSolutionUsable();
}

void output_poses(const std::string& filename, const poses_t& poses) {
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile) {
        std::cerr << "Couldn't open a file: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
    for (const auto& pair : poses) {
        outfile << pair.first << ","
                << pair.second.x << ","
                << pair.second.y << ","
                << pair.second.theta << std::endl;
    }
}

} // namespace se2

#endif // SE2_PROBLEM_H
