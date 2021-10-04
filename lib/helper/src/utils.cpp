#include "utils.h"

void matrix_copy(const float raw_data[3][3],
                 matrix::SquareMatrix<float, 3> px4_data) {
  px4_data(0, 0) = raw_data[0][0];
  px4_data(0, 1) = raw_data[0][1];
  px4_data(0, 2) = raw_data[0][2];
  px4_data(1, 0) = raw_data[1][0];
  px4_data(1, 1) = raw_data[1][1];
  px4_data(1, 2) = raw_data[1][2];
  px4_data(2, 0) = raw_data[2][0];
  px4_data(2, 1) = raw_data[2][1];
  px4_data(2, 2) = raw_data[2][2];
}
