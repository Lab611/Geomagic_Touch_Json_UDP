//
// Created by Lab611-Y7KP on 24-11-14.
//

#ifndef JSON_MSG_BUILDER_HPP
#define JSON_MSG_BUILDER_HPP
#include <nlohmann/json.hpp>
#include <vector>
#include <Eigen/Dense>


// 不同设备拥有不同的标识
typedef enum {
    DEV_UNKNOWN = -1,
    DEV_TOUCH = 1,
    DEV_KEYBOARD = 2,
    DEV_ELITE = 101,
    DEV_NE30 = 102,
} JSON_DEVICE_TYPE;

// 不同命令位
typedef enum {
    CMD_UNKNOWN = -1,
    CMD_MOVE = 0,
    CMD_MOVE_ABS = 1,
    CMD_SET_ABS = 2,
    CMD_RESET = 10,
    CMD_QUIT = 20,
} JSON_CMD_TYPE;

inline nlohmann::json build_json_from_pos_and_rot(JSON_DEVICE_TYPE device,
                                                  JSON_CMD_TYPE cmd = CMD_MOVE,
                                                  std::vector<double> pos_and_rot = {0}) {
    return nlohmann::json{{"device", device}, {"cmd", cmd}, {"data", pos_and_rot}};
}

inline nlohmann::json build_json_from_trans_matrix(JSON_DEVICE_TYPE device,
                                                   JSON_CMD_TYPE cmd = CMD_MOVE,
                                                   Eigen::Matrix4d trans_matrix = Eigen::Matrix4d::Identity()) {
    json trans_json;
    for (int i = 0; i < trans_matrix.rows(); ++i) {
        std::vector<double> row;
        for (int j = 0; j < trans_matrix.cols(); ++j) {
            row.push_back(trans_matrix(i, j));
        }
        trans_json.push_back(row); // 每行是一个 std::vector，放到 JSON 数组中
    }
    return nlohmann::json{{"device", device}, {"cmd", cmd}, {"data_trans", trans_json}};
}


inline std::vector<double> get_vec_from_matrix(
    json msg,
    const Eigen::Matrix4d &rotation_left_offset = Eigen::Matrix4d::Identity(),
    const Eigen::Matrix4d &rotation_right_offset = Eigen::Matrix4d::Identity()) {
    /// return: x, y, z, yaw(z), pitch(y), roll(x)

    // 从 nlohmann::json 反序列化为 Eigen 矩阵
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j_ = 0; j_ < mat.cols(); ++j_) {
            mat(i, j_) = msg["data_trans"][i][j_]; // 将 JSON 中的数据恢复到 Eigen 矩阵中
        }
    }
    std::cout << "before trans mat:\n " << mat << std::endl;
    mat = rotation_left_offset * mat * rotation_right_offset;
    std::cout << "after trans mat:\n " << mat << std::endl;
    Eigen::Matrix3d rotation_mat = mat.block(0, 0, 3, 3);
    Eigen::Vector3d euler_angles = rotation_mat.eulerAngles(2, 1, 0);
    std::vector<double> pos_and_angles = {};
    for (int i = 0; i < 3; ++i) {
        pos_and_angles.push_back(mat(i, 3));
    }
    for (int i = 0; i < 3; ++i) {
        pos_and_angles.push_back(euler_angles[i]);
        std::cout << "pos_and_angles [" << i << "]: " << euler_angles[i] << std::endl;
    }
    // auto roll = pos_and_angles[5];
    // auto pitch = pos_and_angles[4];
    // auto yaw = pos_and_angles[3];
    // Eigen::Matrix3d tH;
    // tH(0, 0) = cos(pitch) * cos(yaw);
    // tH(1, 0) = cos(pitch) * sin(yaw);
    // tH(2, 0) = -sin(pitch);
    // tH(0, 1) = sin(pitch) * cos(yaw) * sin(roll) - cos(roll) * sin(yaw);
    // tH(1, 1) = cos(roll) * cos(yaw) + sin(pitch) * sin(yaw) * sin(roll);
    // tH(2, 1) = cos(pitch) * sin(roll);
    // tH(0, 2) = sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch);
    // tH(1, 2) = cos(roll) * sin(pitch) * sin(yaw) - cos(yaw) * sin(roll);
    // tH(2, 2) = cos(roll) * cos(pitch);
    //
    // std ::cout <<"tH:\n " << tH << std::endl;
    //
    // // 绕 Z 轴旋转矩阵
    // Eigen::Matrix3d R_z;
    // R_z << cos(yaw), -sin(yaw), 0,
    //        sin(yaw), cos(yaw), 0,
    //        0, 0, 1;
    //
    // // 绕 Y 轴旋转矩阵
    // Eigen::Matrix3d R_y;
    // R_y << cos(pitch), 0, sin(pitch),
    //        0, 1, 0,
    //        -sin(pitch), 0, cos(pitch);
    //
    // // 绕 X 轴旋转矩阵
    // Eigen::Matrix3d R_x;
    // R_x << 1, 0, 0,
    //        0, cos(roll), -sin(roll),
    //        0, sin(roll), cos(roll);
    //
    // // 总旋转矩阵 = 绕 Z 轴、Y 轴、X 轴的旋转矩阵的乘积
    // Eigen::Matrix3d R = R_z * R_y * R_x;
    // // 输出旋转矩阵
    // std::cout << "Rotation Matrix (ZYX):\n" << R << std::endl;

    return pos_and_angles;
}

inline nlohmann::json build_json_from_cmd(JSON_DEVICE_TYPE device,
                                          JSON_CMD_TYPE cmd) {
    return nlohmann::json{{"device", device}, {"cmd", cmd}};
}

#endif //JSON_MSG_BUILDER_HPP
