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


inline nlohmann::json build_json_from_cmd(JSON_DEVICE_TYPE device,
                                          JSON_CMD_TYPE cmd) {
    return nlohmann::json{{"device", device}, {"cmd", cmd}};
}

#endif //JSON_MSG_BUILDER_HPP
