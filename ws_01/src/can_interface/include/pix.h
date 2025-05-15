#ifndef PIX_H
#define PIX_H

#include <iostream>
#include <chrono>  
#include <thread>  
#include <functional>
#include <system_error>
#include <iostream>
#include <vector>
#include <iomanip> 
#include <cstring>
#include <cerrno>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

using namespace std;

//驱动控制
struct Message_130 {
    bool ACU_ChassisDriverEnCtrl;    //0:disable 1:enable
    uint8_t ACU_ChassisDriverModeCtrl;  //0:speed ctrl mode 1:throttle ctrl mode 2:reserve 3:reserve
    uint8_t ACU_ChassisGearCtrl;   //0:default N 1:D 2:N 3:R
    float ACU_ChassisSpeedCtrl;   //车辆速度控制 0-50 精度 0.01 m/s
    uint16_t ACU_ChassisThrottlePdlTarget; //车辆油门控制 0-100%	精度0.1
    uint8_t ACU_DriveLifeSig;  //车辆速度控制 0-50 精度 0.01 m/s
    uint8_t ACU_CheckSum_130;  //校验sum=byte0 xor byte1 xor...byte6
};

// 制动控制
// CAN ID 0x131 - A2V_BrakeCtrl   制动控制
struct Message_131 {
    bool ACU_ChassisBrakeEn;        //0:disable 1:enable
    bool ACU_ChassisAebCtrl;        //0:disable 1:enable（预留）
    uint16_t ACU_ChassisBrakePdlTarget; //0-100% 精度0.1%
    uint8_t ACU_ChassisEpbCtrl;     //0:default 1:brake 2:release
    uint8_t ACU_BrakeLifeSig;       //循环计数0~15
    uint8_t ACU_CheckSum_131;       //校验sum=byte0 xor byte1 xor...byte6
};

//转向控制
struct Message_132 {
    bool ACU_ChassisSteerEnCtrl;    //0:disable 1:enable
    uint8_t ACU_ChassisSteerModeCtrl; //0:front ackerman 1:same front and back 2:front different back 3:back ackrman 4:front back
    int16_t ACU_ChassisSteerAngleTarget;   //前转向角 -500~500 deg
    int16_t ACU_ChassisSteerAngleRearTarget; //后转向角 -500~500 deg
    uint8_t ACU_ChassisSteerAngleSpeedCtrl ; //转向角速度控制 0-500
    uint8_t ACU_SteerLifeSig;       //循环计数0~15
    uint8_t ACU_CheckSum_132;       //校验sum=byte0 xor byte1 xor...byte6
};

struct Message_133 {
    bool ACU_VehicleLeftLampCtrl; //左转向灯控制 0:off 1:on（字节0 bit2）
    bool ACU_VehicleRightLampCtrl;//右转向灯控制 0:off 1:on（字节0 bit3）
    bool ACU_CheckSumEn;         //校验模式使能 0:enable 1:disable（字节6 bit0）
};


//驱动状态反馈
// CAN ID 0x530 - V2A_DriveStaFb
struct Message_530 {
    bool VCU_ChassisDriverEnSta;    //0:disable 1:enable
    bool VCU_ChassisDiverSlopover;  //0:normal 1:over slop
    uint8_t VCU_ChassisDriverModeSta; //0:speed ctrl mode 1:throttle ctrl mode...
    uint8_t VCU_ChassisGearFb;      //0:no use 1:D 2:N 3:R
    int16_t VCU_ChassisSpeedFb;     //车速反馈 -50~50 m/s 精度0.01（需除以100）
    uint16_t VCU_ChassisThrottlePaldFb; //油门反馈 0~100% 精度0.1%（需除以10）
    int16_t VCU_ChassisAccelerationFb; //加速度 -20~20 m/s² 精度0.01
};



//制动反馈
// CAN ID 0x531 - V2A_BrakeStaFb
struct Message_531 {
    bool VCU_ChassisBrakeEnSta;     //0:disable 1:enable
    bool VCU_VehicleBrakeLampFb;    //0:off 1:on
    uint8_t VCU_ChassisEpbFb;       //0:release 1:brake 2:releasing 3:braking
    uint16_t VCU_ChassisBrakePadlFb; //制动反馈 0~100% 精度0.1%
    bool VCU_AebEnStaFb;            //0:off 1:on
    bool VCU_AebTriggerStaFb;       //0:off 1:aeb trigger
};


//转向反馈
struct Message_532 {
    bool VCU_ChassisSteerEnSta;     //0:disable 1:enable
    bool VCU_ChassisSteerSlopover;  //0:normal 1:over slop
    uint8_t VCU_ChassisSteerWorkMode; //0:machine 1:wire 2:power
    uint8_t VCU_ChassisSteerModeFb; //0:front ackerman 1:same front and back...
    int16_t VCU_ChassisSteerAngleFb; //前转向角 -500~500 deg
    int16_t VCU_ChassisSteerAngleRearFb; //后转向角 -500~500 deg
};


//车辆工作状态反馈
// CAN ID 0x534 - V2A_VehicleWorkStaFb
struct Message_534 {
    uint8_t VCU_DrivingModeFb;      //0:standby 1:self driving 2:remote 3:man
    uint8_t VCU_ChassisPowerStaFb;  //0:init 1:on acc 2:ready 3:off
    uint8_t VCU_ChassisPowerDcSta;  //0:off 1:on 2:standby
    uint8_t VCU_ChassisLowPowerVoltSta; //低压电压 0~25V 精度0.1V
    uint8_t VCU_ChassisEStopStaFb;  //0:no 1:chassis estop...
    bool VCU_CrashFrontSta : 1;     //0:off 1:collide
    bool VCU_CrashRearSta : 1;      //0:off 1:collide
    bool VCU_CrashLeftSta : 1;      //0:off 1:collide
    bool VCU_CrashRightSta : 1;     //0:off 1:collide
    uint8_t VCU_Life;               //循环计数0~15
    uint8_t VCU_CheckSum;           //校验和
};


std::vector<uint8_t> encodeDriveControlMessage(const Message_130& msg);
std::vector<uint8_t> encodeSteerControlMessage(const Message_132& msg);
std::vector<uint8_t> encodeBrakeControlMessage(const Message_131& msg); 
std::vector<uint8_t> encodeVehicleCtrlMessage(const Message_133& msg);
void sendCANFrame(const char* interface, uint32_t can_id, const std::vector<uint8_t>& data);


#endif
