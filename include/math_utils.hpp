//
// Created by Lab611-Y7KP on 24-11-14.
//

#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP
#include <vector>
#include <Eigen/Dense>
#include <iostream>


inline std::vector<double> get_vec_from_trans_matrix(Eigen::Matrix4d mat){
    // input:
    // R00 R01 R02 Tx
    // R10 R11 R12 Ty
    // R20 R21 R22 Tz
    // 0   0   0   1
    // output: x, y, z, yaw(z), pitch(y), roll(x)

    Eigen::Matrix3d rotation_mat = mat.block(0, 0, 3, 3);
    Eigen::Vector3d euler_angles = rotation_mat.eulerAngles(2, 1, 0);  // ZYX
    std::vector<double> vec = {};
    for (int i = 0; i < 3; ++i) {
        vec.push_back(mat(i, 3));
        std::cout << "pos [" << i << "]: " << vec[i] << std::endl;
    }
    for (int i = 0; i < 3; ++i) {
        vec.push_back(euler_angles[i]);
        std::cout << "ang [" << i << "]: " << euler_angles[i] << std::endl;
    }
    return vec;
}


#endif //MATH_UTILS_HPP
