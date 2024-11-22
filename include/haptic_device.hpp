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

    Eigen::MatrixXd cur_trans_matrix = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd last_trans_matrix = Eigen::MatrixXd::Identity(4, 4);

    int button1;
    int button2;
    char sendBuf[256];
    bool isRunning;
    bool last_button_1_state = false;
    bool moveAbs = false;


    static HDCallbackCode HDCALLBACK poseCallback(void *pUserData) {
        auto *device = static_cast<HapticDevice *>(pUserData);
        hduVector3Dd positions;
        hduMatrix transform;
        hduQuaternion orientation;

        hdBeginFrame(hdGetCurrentDevice());

        hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
        // 获取旋转矩阵
        auto rotation = hduMatrix();
        transform.getRotationMatrix(rotation);

        // 赋值为 4*4 的 trans matrix
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                device->cur_trans_matrix(i, j) = rotation.get(j, i);
            }
        }
        for (int i = 0; i < 3; i++) {
            device->cur_trans_matrix(i, 3) = (double) transform.get(3, i);
            positions[i] = transform.get(3, i);
        }

        // cout << "\n eigen rotation: \n" << device->cur_trans_matrix;

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
                // 记录上一次的位置
                last_trans_matrix = cur_trans_matrix;
                last_button_1_state = true;
                return build_json_from_cmd(DEV_TOUCH, CMD_UNKNOWN);
            }
            cout << "Button1 pressed! Sending data: " << endl;

            for (auto i = 0; i < 3; i++) {
                const auto cur_pos = last_trans_matrix(i, 3);
                cur_trans_matrix(i, 3) -= last_trans_matrix(i, 3);
                if (!moveAbs) {
                    // 如果绝对位置控制 那么不需要记录上一次的位置
                    last_trans_matrix(i, 3) = cur_pos;
                }
            }
            // cout << "trans matrix: \n" << cur_trans_matrix << endl;
            if (moveAbs) {
                // 绝对位置控制
                return build_json_from_trans_matrix(DEV_TOUCH, CMD_MOVE_ABS, cur_trans_matrix);
            }
            return build_json_from_trans_matrix(DEV_TOUCH, CMD_MOVE, cur_trans_matrix);
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
