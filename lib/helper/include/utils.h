#pragma once
#include <matrix/math.hpp>

namespace helper {
void matrix_copy(const float raw_data[3][3],
                 matrix::SquareMatrix<float, 3> px4_data);
}