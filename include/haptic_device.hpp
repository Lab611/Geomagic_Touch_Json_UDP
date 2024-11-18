#include <iostream>
#include <string>
#include <thread>
#include <queue>
#include <chrono>
#include <conio.h>
#include <HD/hd.h>
#include <HL/hl.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduQuaternion.h>
#include <Eigen/Dense>
#include "json_msg_builder.hpp"


using namespace std;


class HapticDevice {
private:
    HHD hHD;
    HDSchedulerHandle hPoseCallback;
    HDSchedulerHandle hForceCallback;
    double pos[6];
    int button1;
    int button2;
    char sendBuf[256];
    bool isRunning;
    bool last_button_1_state = false;
    vector<double> last_vec = vector<double>();
    bool moveAbs = false;


    static HDCallbackCode HDCALLBACK poseCallback(void *pUserData) {
        // 左乘
        const hduMatrix rotation_left_offset = hduMatrix(-1.0, 0.0, 0.0, 0.0,
                                                         0.0, 0.0, -1.0, 0.0,
                                                         0.0, -1.0, 0.0, 0.0,
                                                         0.0, 0.0, 0.0, 1.0);

        // 右乘
        const hduMatrix rotation_right_offset = hduMatrix(1.0, 0.0, 0.0, 0.0,
                                                          0.0, 1.0, 0.0, 0.0,
                                                          0.0, 0.0, 1.0, 0.0,
                                                          0.0, 0.0, 0.0, 1.0);

        auto *device = static_cast<HapticDevice *>(pUserData);
        hduVector3Dd positions;
        hduMatrix transform;
        hduQuaternion orientation;

        hdBeginFrame(hdGetCurrentDevice());

        hdGetDoublev(HD_CURRENT_POSITION, positions);
        hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
        auto apply_offset_transform = rotation_left_offset * transform.getTranspose() * rotation_right_offset;

        auto rotation = hduMatrix();
        apply_offset_transform.getTranspose().getRotationMatrix(rotation);
        orientation = hduQuaternion(rotation);

        // // orientation = hduQuaternion(rotation);
        // for (int i = 0; i < 4; i++) {
        //     for (int j = 0; j < 4; j++) {
        //         cout << apply_offset_transform.get(i, j) <<" ";
        //     }
        //     cout << endl;
        // }

        // device->pos[0] = positions[0];
        // device->pos[1] = positions[1];
        // device->pos[2] = positions[2];
        for (int i = 0; i < 3; i++) {
            device->pos[i] = apply_offset_transform.get(i, 3);
        }


        Eigen::Quaternionf quat(orientation[0], orientation[1], orientation[2], orientation[3]); // 四元数 w, x, y, z
        Eigen::Matrix3f rx = quat.toRotationMatrix();

        // 将四元数转换为欧拉角（单位是弧度）
        Eigen::Vector3f euler_angles = rx.eulerAngles(2, 1, 0);

        device->pos[3] = euler_angles[0];
        device->pos[4] = euler_angles[1];
        device->pos[5] = euler_angles[2];

        // device->pos[3] = orientation[0];
        // device->pos[4] = orientation[1];
        // device->pos[5] = orientation[2];
        // device->pos[6] = orientation[3];

        HDint nbuttons;
        hdGetIntegerv(HD_CURRENT_BUTTONS, &nbuttons);

        device->button1 = nbuttons & HD_DEVICE_BUTTON_1 ? HD_TRUE : HD_FALSE;
        device->button2 = nbuttons & HD_DEVICE_BUTTON_2 ? HD_TRUE : HD_FALSE;

        // force
        const HDdouble kStiffness = 0.035; /* N/mm */
        const HDdouble kGravityWellInfluence = 40; /* mm */

        /* This is the position of the gravity well in cartesian
           (i.e. x,y,z) space. */
        static const hduVector3Dd wellPos = {0, 0, 0};

        // HDErrorInfo error;
        hduVector3Dd force;
        hduVector3Dd positionTwell;

        /* Begin haptics frame.  ( In general, all state-related haptics calls
           should be made within a frame. ) */
        // hdBeginFrame(hdGetCurrentDevice());

        memset(force, 0, sizeof(hduVector3Dd));

        /* >  positionTwell = wellPos-position  <
           Create a vector from the device position towards the gravity
           well's center. */
        hduVecSubtract(positionTwell, wellPos, positions);

        /* If the device position is within some distance of the gravity well's
           center, apply a spring force towards gravity well's center.  The force
           calculation differs from a traditional gravitational body in that the
           closer the device is to the center, the less force the well exerts;
           the device behaves as if a spring were connected between itself and
           the well's center. */
        if (hduVecMagnitude(positionTwell) < kGravityWellInfluence) {
            /* >  F = k * x  <
               F: Force in Newtons (N)
               k: Stiffness of the well (N/mm)
               x: Vector from the device endpoint position to the center
               of the well. */
            hduVecScale(force, positionTwell, kStiffness);
        }

        /* Send the force to the device. */
        hdSetDoublev(HD_CURRENT_FORCE, force);


        hdEndFrame(hdGetCurrentDevice());
        return HD_CALLBACK_CONTINUE;
    }

public:
    HapticDevice() : hHD(HD_INVALID_HANDLE), hPoseCallback(0), hForceCallback(0), button1(0), button2(0),
                     isRunning(false) {
        fill(begin(pos), end(pos), 0.0);
    }

    ~HapticDevice() {
        stop();
    }

    bool initialize() {
        HDErrorInfo error;
        hHD = hdInitDevice(HD_DEFAULT_DEVICE);
        if (HD_DEVICE_ERROR(error = hdGetError())) {
            cerr << "Failed to initialize haptic device" << endl;
            return false;
        }

        hdEnable(HD_FORCE_OUTPUT);
        if (HD_DEVICE_ERROR(error = hdGetError())) {
            cerr << "Failed to enable force output" << endl;
            return false;
        }

        hPoseCallback = hdScheduleAsynchronous(poseCallback, this, HD_DEFAULT_SCHEDULER_PRIORITY);
        hdStartScheduler();
        if (HD_DEVICE_ERROR(error = hdGetError())) {
            cerr << "Failed to start the scheduler" << endl;
            return false;
        }

        isRunning = true;
        return true;
    }

    void setMoveAbs(bool moveAbs_) {
        moveAbs = moveAbs_;
    }

    void stop() {
        if (isRunning) {
            hdStopScheduler();
            if (hPoseCallback) {
                hdUnschedule(hPoseCallback);
            }
            if (hForceCallback) {
                hdUnschedule(hForceCallback);
            }
            hdDisableDevice(hHD);
            isRunning = false;
        }
    }

    nlohmann::json update() {
        if (button1 && !button2) {
            // 只按下了 button 1
            if (last_button_1_state == false) {
                // 第一次按下的话
                std::vector<double> last_vec_raw(std::begin(pos), std::end(pos));
                last_vec.clear();
                last_vec.assign(std::begin(last_vec_raw), std::end(last_vec_raw));
                last_button_1_state = true;
                return build_json_from_cmd(DEV_TOUCH, CMD_UNKNOWN);
            }
            std::vector<double> vec(std::begin(pos), std::end(pos));
            cout << "Button1 pressed! Sending data: " << endl;
            for (auto i = 0; i < 3; i++) {
                // cout << " vec [" << i << "] " << vec[i] << endl;
                const auto a = vec[i];
                vec[i] -= last_vec[i];
                if (!moveAbs) {
                    // 如果绝对位置控制 那么不需要记录上一次的位置
                    last_vec[i] = a;
                }
                // cout << " last_vec [" << i << "] "<< last_vec[i] << endl;
            }

            for (auto i: vec) {
                cout << i << " ";
            }
            cout << endl;
            if (moveAbs) {
                // 绝对位置控制
                return build_json_from_pos_and_rot(DEV_TOUCH, CMD_MOVE_ABS, vec);
            }
            return build_json_from_pos_and_rot(DEV_TOUCH, CMD_MOVE, vec);
        } else if (!button1 && button2) {
            // 只按下了 button 2
            last_button_1_state = false;
            cout << "Button2 pressed! Sending reset signal." << endl;
            return build_json_from_cmd(DEV_TOUCH, CMD_RESET);
        } else if (button1 && button2) {
            // button 1 button 2 同时按下
            cout << "Both buttons pressed! Stopping..." << endl;
            last_button_1_state = false;
            isRunning = false;
            return build_json_from_cmd(DEV_TOUCH, CMD_QUIT);
        }
        last_button_1_state = false;
        // 返回空
        return build_json_from_cmd(DEV_TOUCH, CMD_UNKNOWN);
    }

    bool isRunningStatus() const {
        return isRunning;
    }
};
