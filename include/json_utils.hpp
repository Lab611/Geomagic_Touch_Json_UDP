//
// Created by Lab611-Y7KP on 24-11-22.
//

#ifndef JSON_UTILS_HPP
#define JSON_UTILS_HPP
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>


inline nlohmann::json get_json_from_file(const string &file_path) {
    // 打开文件
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Could not open the file: " << file_path << std::endl;
        return 1; // 返回错误码
    }

    // 解析 JSON 文件
    nlohmann::json j;
    try {
        file >> j; // 使用 operator>> 直接读取文件内容到 json 对象
    } catch (const nlohmann::json::parse_error &e) {
        std::cerr << "Parse error: " << e.what() << std::endl;
        return {};
    }
    return j;
}


inline Eigen::Matrix4d get_trans_matrix_from_json(json msg) {
    // 从 nlohmann::json 反序列化为 Eigen 矩阵

    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j_ = 0; j_ < mat.cols(); ++j_) {
            mat(i, j_) = msg["data_trans"][i][j_]; // 将 JSON 中的数据恢复到 Eigen 矩阵中
        }
    }
    return mat;
}


#endif //JSON_UTILS_HPP
