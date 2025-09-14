#include "simple_imu_fusion.hpp"


mylib::Vector3<double> mylib::SimpleIMUFusion::GRAVITY_VEC(0.0, -1.0, 0.0);;

mylib::SimpleIMUFusion::SimpleIMUFusion(double accel_ratio, mylib::Quaternion<double> init_rot) {
    this->accel_ratio = accel_ratio;
    this->_rot = init_rot.unit();
}

mylib::Quaternion<double> mylib::SimpleIMUFusion::update(mylib::Vector3<double> accel_vec, mylib::Vector3<double> gyro_vec, double dt) {
    // GYRO ROTATION STEP
    double gyro_vec_norm = gyro_vec.norm();
    mylib::Quaternion<double> gyro_quat = (
        gyro_vec_norm > VEC_NORM_EPS ? // for numerical reason
        mylib::Quaternion<double>(gyro_vec.unit(), gyro_vec_norm * dt) :
        mylib::Quaternion<double>()
    );
    _rot = _rot.quat_mul(gyro_quat);

    // ACCEL ROTATION STEP
    mylib::Vector3<double> orient_grav_dir = _rot.conj().rotate(GRAVITY_VEC);
    mylib::Vector3<double> accel_grav_dir = accel_vec.norm() > VEC_NORM_EPS ? accel_vec.unit() : orient_grav_dir; // for numerical reason

    mylib::Quaternion<double> grav_dir_diff;
    if (orient_grav_dir.dot(accel_grav_dir) < VEC_COS_EPS) { // for numerical reason
        grav_dir_diff = mylib::Quaternion<double>(orient_grav_dir, accel_grav_dir, accel_ratio).conj();
    }
    _rot = _rot.quat_mul(grav_dir_diff);

    _rot = _rot.unit();
    return _rot;
}
