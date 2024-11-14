//
// Created by Lab611-Y7KP on 24-11-14.
//

#ifndef JSON_MSG_BUILDER_HPP
#define JSON_MSG_BUILDER_HPP
#include <nlohmann/json.hpp>
#include <vector>

typedef enum {
    DEV_UNKNOWN = -1,
    DEV_TOUCH = 1,
    DEV_KEYBOARD = 2,
    DEV_ELITE = 101,
    DEV_NE30 = 102,
} JSON_DEVICE_TYPE;

typedef enum {
    CMD_UNKNOWN = -1,
    CMD_MOVE = 0,
    CMD_PAUSE = 1,
    CMD_RESET = 2,
    CMD_QUIT = 3,
} JSON_CMD_TYPE;

inline nlohmann::json build_json_from_pos_and_rot(JSON_DEVICE_TYPE device,
                                                  JSON_CMD_TYPE cmd = CMD_MOVE,
                                                  std::vector<double> pos_and_rot={0}) {
    return nlohmann::json{{"device", device}, {"cmd", cmd}, {"data", pos_and_rot}};
}


inline nlohmann::json build_json_from_cmd(JSON_DEVICE_TYPE device,
                                          JSON_CMD_TYPE cmd) {
    return nlohmann::json{{"device", device}, {"cmd", cmd}};
}

#endif //JSON_MSG_BUILDER_HPP
