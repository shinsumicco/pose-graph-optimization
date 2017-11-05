#ifndef SE2_ERROR_FUNCTION_H
#define SE2_ERROR_FUNCTION_H

#include "types.h"

namespace se2 {

// 誤差関数(残差関数)の関数オブジェクト
class error_function {
public:
    error_function(const double x_ab, const double y_ab, const double theta_ab,
                   std::array<double, 3> decomposed_information)
            : x_ab_(x_ab), y_ab_(y_ab), theta_ab_(theta_ab),
              decomposed_information_(std::move(decomposed_information)) {}

    template<typename T>
    bool operator()(const T* const x_a, const T* const y_a, const T* const theta_a,
                    const T* const x_b, const T* const y_b, const T* const theta_b,
                    T* residuals_ptr) const {
        T residual_x =  ceres::cos(*theta_a) * (*x_b - *x_a) + ceres::sin(*theta_a) * (*y_b - *y_a) - static_cast<T>(x_ab_);
        T residual_y = -ceres::sin(*theta_a) * (*x_b - *x_a) + ceres::cos(*theta_a) * (*y_b - *y_a) - static_cast<T>(y_ab_);
        T residual_theta = normalize_angle((*theta_b - *theta_a) - static_cast<T>(theta_ab_));

        residuals_ptr[0] = residual_x * static_cast<T>(decomposed_information_.at(0));
        residuals_ptr[1] = residual_y * static_cast<T>(decomposed_information_.at(1));
        residuals_ptr[2] = residual_theta * static_cast<T>(decomposed_information_.at(2));

        return true;
    }

private:
    const double x_ab_;
    const double y_ab_;
    const double theta_ab_;
    const std::array<double, 3> decomposed_information_;
};

} // namespace se2

#endif // SE2_ERROR_FUNCTION_H
