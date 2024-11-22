# Clion 编译的 NE 30 Controller

理论上来说 环境都齐了 需要使用 msvc 做为编译器

msvc 可以通过下载 vs 安装 也可以直接下载对应版本

Clion 的工具链 生成器需要如图所示设置

![img.png](imgs/clion.png)

# 文件结构

## 3rdparty

所有第三方库

1. `eigen-3.4.0` [Eigen 3.4.0](https://eigen.tuxfamily.org/index.php?title=Main_Page) 可能需要自己官网下载重新一次 eigen 库
2. `openhaptics` [手柄 SDK](https://support.3dsystems.com/s/article/OpenHaptics-for-Windows-Developer-Edition-v35?language=en_US)
3. `nlohmann` [nlohmann/json](https://github.com/nlohmann/json)

## include

头文件 这里都用了 hpp 格式 单文件超人

1. `json_msg_builder.hpp` 通信协议
2. `json_udp_client.hpp` UDP 发送消息
3. `haptic_device.hpp` 手柄控制类

## main.cpp

主函数


# 通信协议

```cpp
// ./include/json_msg_builder.hpp

// ...

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
        trans_json.push_back(row);
    }
    return nlohmann::json{{"device", device}, {"cmd", cmd}, {"data_trans", trans_json}};
}


inline nlohmann::json build_json_from_cmd(JSON_DEVICE_TYPE device,
                                          JSON_CMD_TYPE cmd) {
    return nlohmann::json{{"device", device}, {"cmd", cmd}};
}
```
