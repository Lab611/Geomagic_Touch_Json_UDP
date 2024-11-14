#include <iostream>
#include <string>
#include <thread>
#include <queue>

#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <time.h>

#include <cstdio>
#include <cassert>

#if defined(WIN32)
#include <conio.h>
#else
#include "conio.h"
#endif

#include <HD/hd.h>
#include <HL/hl.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduQuaternion.h>

#pragma region HD
using namespace std;
int button1 = 0;
int button2 = 0;
// HDboolean button1, button2;两个操作臂按钮
double pos[7] = {0, 0, 0};
//一个包含 7 个元素的 double 类型数组，用于表示机械臂的位置和姿态信息。前 3 个元素表示机械臂末端执行器的 x, y, z 坐标，后 4 个元素表示机械臂末端执行器的四元数（用于表示旋转）
double restartFlag = -1;
double read_force[6] = {0, 0, 0, 0, 0, 0};
HDSchedulerHandle hPoseCallback = 0;
HDSchedulerHandle hForceCallback = 0;
double forcesensor[3] = {0, 0, 0};
char sendBuf[200];
char recvBuf[300] = {};
char recvBufForce[100] = {};
double transformarray[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; //一个包含 9 个元素的 double 类型数组，可能表示某种变换矩阵的数据。


HDCallbackCode HDCALLBACK passPoseCallback(void *pUserData) {
    // static int nbuttons;
    HDint nbuttons;
    // hduVector3Dd position;
    hduVector3Dd positions;
    hduMatrix transform;
    hduQuaternion orientation;

    /*定义了一系列局部变量，
    如 nbuttons（按钮状态）、positions（设备当前位置）、transform（设备当前变换矩阵）、
    orientation（设备当前四元数表示的姿态）*/

    hdBeginFrame(hdGetCurrentDevice());

    // if (button1) {
    hdGetDoublev(HD_CURRENT_POSITION, positions);
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
    hduMatrix rotation(transform);

    rotation.getRotationMatrix(rotation);
    orientation = hduQuaternion(rotation);

    const HDdouble kStiffness = 0.035; /* N/mm */
    const HDdouble kGravityWellInfluence = 40; /* mm */

    /* This is the position of the gravity well in cartesian
       (i.e. x,y,z) space. */
    static const hduVector3Dd wellPos = {0, 0, 0};

    HDErrorInfo error;
    hduVector3Dd force;
    hduVector3Dd positionTwell;

    /* Begin haptics frame.  ( In general, all state-related haptics calls
       should be made within a frame. ) */
    // hdBeginFrame(hdGetCurrentDevice());

    /* Get the current position of the device. */
    hdGetDoublev(HD_CURRENT_POSITION, positions);

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


    pos[0] = -positions[0];
    pos[1] = positions[2];
    pos[2] = positions[1];

    pos[3] = orientation[0];
    pos[4] = orientation[1];
    pos[5] = orientation[2];
    pos[6] = orientation[3];



    hdGetIntegerv(HD_CURRENT_BUTTONS, &nbuttons);
    // cout << "nbuttons: " << nbuttons << endl;
    button1 = nbuttons & HD_DEVICE_BUTTON_1 ? HD_TRUE : HD_FALSE;
    button2 = nbuttons & HD_DEVICE_BUTTON_2 ? HD_TRUE : HD_FALSE;

    hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_CONTINUE;
}


#pragma endregion

std::string sendMsg;


int main() {
    cout << "-----------client-----------" << endl;
    // 启动线程池

    /*然后，创建一个 CTcpClientPtr 类型的智能指针 client，并将 listener 作为参数传递给它。CTcpClientPtr 是 HP - Socket 库中的一个辅助类，用于管理 ITcpClient 类型的对象。
    将 listener 传递给 client 的构造函数，意味着 client 将使用 listener 中定义的事件处理方法来处理 TCP 事件*/

    HDErrorInfo error;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);

    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();
        return -1;
    }

    hdEnable(HD_FORCE_OUTPUT);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();
        return -1;
    }

    hPoseCallback = hdScheduleAsynchronous(
        passPoseCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
    printf("Press any key to quit.\n\n");

    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();
        return -1;
    }

    while (true) {
        // cin >> sendMsg;
        /*这段代码包含一个条件判断语句，用于检测键盘输入。如果检测到键盘输入，它会读取输入的键值并与ASCII码27（ESC键）进行比较。
        如果按下的是ESC键，将全局变量SysExit设置为true，表示系统需要退出或停止运行。

        _kbhit()函数用于非阻塞地检测键盘输入。如果有按键被按下，该函数返回非零值；否则返回零。_getch()函数用于获取按下的键值。*/
        if (_kbhit()) {
            if (27 == _getch())
                break;
        }
        sendMsg.clear();
        if (button1 && (!button2)) //检查 button1 是否被按下且 button2 没有被按下
        {
            cout << "button1 pressed!" << endl;
            sprintf_s(sendBuf, "<%-10.5lf%-10.5lf%-10.5lf%-10.5lf%-10.5lf%-10.5lf%-10.5lf>\n",
                      pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]);
            sendMsg = string(sendBuf);
            cout << "\n";
            // sendMsg = "TEST__\n";
            cout << "sendBuf = " << sendBuf << endl;
        }
        if ((!button1) && button2) {
            cout << "button2 pressed!" << endl;
            sendMsg = string("__RESET__\n");
        }
        if (button1 && button2) {
            cout << "finished!,2button are pressed" << endl;

            break;
        }
        Sleep(30);
    }


    while (!_kbhit()) {
        if (!hdWaitForCompletion(hPoseCallback, HD_WAIT_CHECK_STATUS)) {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            _getch();
            break;
        }
    }
    cout << "Client Exit" << endl;

    hdStopScheduler();
    hdUnschedule(hPoseCallback);
    hdUnschedule(hForceCallback);
    hdDisableDevice(hHD);
    return 0;
}
