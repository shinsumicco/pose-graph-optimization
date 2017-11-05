#ifndef SE2_TYPES_H
#define SE2_TYPES_H

#include <istream>
#include <map>
#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "normalize_angle.h"

namespace se2 {

// 2次元空間中の姿勢を扱う構造体
struct pose_2d {
public:
    // 絶対位置 [x, y]
    double x;
    double y;
    // 絶対ヨー回転 (rad)
    double theta;
};

// g2o形式のファイルからの入力に使うオーバーライド関数
std::istream& operator>>(std::istream& input, pose_2d& pose) {
    input >> pose.x >> pose.y >> pose.theta;
    // 角度の正規化
    pose.theta = normalize_angle(pose.theta);
    return input;
}

// 3次元空間中の姿勢のデータベース (IDの昇順に並べる)
typedef std::map<int, pose_2d, std::less<int>> poses_t;

// グラフの2頂点間の拘束条件(相対姿勢)を扱う構造体
// 頂点id_startから頂点id_endへの相対姿勢を格納する
struct constraint_3d {
public:
    // 相対姿勢の基準となる頂点のID
    int id_start;
    // 相対姿勢の終点となる頂点のID
    int id_end;

    // 頂点id_startから頂点id_endへの相対姿勢
    // (頂点id_endを基準としたベクトルを頂点id_start基準に変換する)
    // 相対位置 [x, y]
    double x;
    double y;
    // 相対ヨー回転 (rad)
    double theta;

    // 情報行列(ガウス誤差を仮定している場合は，相対姿勢の分散共分散行列の逆行列)
    // 簡単化のため対角成分のみを格納する
    // 格納順は[x, y, theta]
    std::array<double, 3> information;
};

// g2o形式のファイルからの入力に使うオーバーライド関数
std::istream& operator>>(std::istream& input, constraint_3d& constraint) {
    input >> constraint.id_start >> constraint.id_end >> constraint.x >> constraint.y >> constraint.theta;
    // 角度の正規化
    constraint.theta = normalize_angle(constraint.theta);
    // 情報行列の入力
    for (int i = 0; i < 3 && input.good(); ++i) {
        for (int j = i; j < 3 && input.good(); ++j) {
            double information_i_j;
            input >> information_i_j;
            // 対角成分のみ格納する
            if (i == j) {
                constraint.information.at(static_cast<unsigned int>(i)) = information_i_j;
            }
        }
    }
    return input;
}

// 拘束条件のデータベース
typedef std::vector<constraint_3d> constraints_t;

} // namespace se2

#endif // SE2_TYPES_H
