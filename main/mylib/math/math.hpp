#ifndef MYLIB_MATH_HPP
#define MYLIB_MATH_HPP

#include "matrix.hpp"
#include "vector3.hpp"
#include "quaternion.hpp"

namespace mylib {
    const double PI = 3.14159265358979323846;
    const double TWO_PI = 2.0 * PI;
    const double RAD_TO_DEG = 180.0 / PI;
    const double DEG_TO_RAD = PI / 180.0;

    const double RAD_EPS = 1e-5;
    const double VEC_NORM_EPS = 1e-5;
    const double VEC_COS_EPS = 0.999999995; // cos(1e-5 rad)
}

#endif
