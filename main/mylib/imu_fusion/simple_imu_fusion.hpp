#ifndef MYLIB_SIMPLE_IMU_FUSION_HPP
#define MYLIB_SIMPLE_IMU_FUSION_HPP

#include "math.hpp"


namespace mylib {
    class SimpleIMUFusion {
    private:
        static mylib::Vector3<double> GRAVITY_VEC;

        mylib::Quaternion<double> _rot;
        double accel_ratio;
    
    public:
        SimpleIMUFusion(double accel_ratio, mylib::Quaternion<double> init_rot = mylib::Quaternion<double>());
        mylib::Quaternion<double> update(mylib::Vector3<double> accel_vec, mylib::Vector3<double> gyro_vec, double dt);
    };
}

#endif
