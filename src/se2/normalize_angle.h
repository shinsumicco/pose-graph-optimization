#ifndef SE2_NORMALIZE_ANGLE_H
#define SE2_NORMALIZE_ANGLE_H

#include <cmath>

#include "ceres/ceres.h"

namespace se2 {

template <typename T>
inline T normalize_angle(const T& angle_radians) {
    T two_pi(2.0 * M_PI);
    return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

} // namespace se2

#endif // SE2_NORMALIZE_ANGLE_H