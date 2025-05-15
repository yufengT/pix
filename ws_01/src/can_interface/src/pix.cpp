#include <iostream>
#include "pix.h"
#include <chrono>  
#include <thread>  
using namespace std;



std::vector<uint8_t> encodeDriveControlMessage(const Message_130& msg) {
    std::vector<uint8_t> data(8, 0);
    static uint8_t drive_life = 0;

    // 设置车辆驱动控制使能（第0字节，第0位）
    data[0] = (data[0] & 0xFE) | (msg.ACU_ChassisDriverEnCtrl ? 0x01 : 0x00);

    // 设置驱动模式控制（第0字节，第2-3位）
    data[0] = (data[0] & 0xC3) | ((msg.ACU_ChassisDriverModeCtrl & 0x03) << 2);

    // 设置档位控制（第0字节，第4-8位）
    data[0] = (data[0] & 0x0F) | ((msg.ACU_ChassisGearCtrl & 0x0F) << 4);

    // 设置车辆速度控制（第1-2字节，第8-23位）
    uint16_t speedControl = static_cast<uint16_t>(msg.ACU_ChassisSpeedCtrl * 100.0f); // 转换为整数，单位0.01m/s
    data[2] = (speedControl >> 8) & 0xFF;
    data[1] = speedControl & 0xFF;
    // 设置车辆油门控制（第3-4字节，第24-33位）
    uint16_t throttlePdlTarget = static_cast<uint16_t>(msg.ACU_ChassisThrottlePdlTarget * 10.0f); // 转换为整数，单位0.1%
    data[4] = (throttlePdlTarget >> 8) & 0x03;
    data[3] = throttlePdlTarget & 0xFF;
    // 设置循环计数（第6字节，第48-51位）
    data[6] = drive_life;
    drive_life = (++drive_life > 0x0F) ? (drive_life = 0) : drive_life;
    // 计算校验和（第7字节，第56-63位）
    uint16_t checksum = 0;
    for (size_t i = 0; i < 7; ++i) {
        checksum ^= data[i];
    }
    data[7] = checksum & 0xFF; 

    return data;
}


std::vector<uint8_t> encodeSteerControlMessage(const Message_132& msg) {
    std::vector<uint8_t> data(8, 0);
    static uint8_t steer_life = 0;

// 设置转向使能（第0字节，第0位）
    data[0] = (data[0] & 0xFE) | (msg.ACU_ChassisSteerEnCtrl ? 0x01 : 0x00);

 // 设置转向模式（第0字节，第4-7位）
    data[0] = (data[0] & 0x0F) | ((msg.ACU_ChassisSteerModeCtrl & 0x0F) << 4);

   // 设置前转向控制（第1字节，第8-23位）
    data[1] = msg.ACU_ChassisSteerAngleTarget & 0xFF;
    data[2] = (msg.ACU_ChassisSteerAngleTarget >> 8) & 0xFF;       
  

    // 设置后转向控制（第3字节，第24-39位）
    data[3] = msg.ACU_ChassisSteerAngleRearTarget & 0xFF;
    data[4] = (msg.ACU_ChassisSteerAngleRearTarget >> 8) & 0xFF;
    uint8_t anglespeed = static_cast<uint8_t>(msg.ACU_ChassisSteerAngleSpeedCtrl * 0.5f); 
    data[5] = anglespeed & 0xFF;
    

// 设置生命帧（第6字节，第48-51位）
    data[6] =steer_life;
    steer_life = (++steer_life > 0x0F) ? (steer_life = 0) : steer_life;

    uint16_t checksum = 0;
    for (size_t i = 0; i < 7; ++i) {
        checksum ^= data[i];
    }
    data[7] = checksum & 0xFF;


    return data;
}

std::vector<uint8_t> encodeBrakeControlMessage(const Message_131& msg) {
    std::vector<uint8_t> data(8, 0);
    static uint8_t brake_life = 0;

    // 设置车辆刹车控制使能（第0字节，第0位）
    data[0] = (data[0] & 0xFE) | (msg.ACU_ChassisBrakeEn ? 0x01 : 0x00);
    // 设置AEB使能（第0字节，第4位）
    data[0] = (data[0] & 0xEF) | (msg.ACU_ChassisAebCtrl ? 0x10 : 0x00);

    // 设置车辆刹车控制（第1-2字节，第8-17位）
    uint16_t brakePdlTarget = static_cast<uint16_t>(msg.ACU_ChassisBrakePdlTarget * 10.0f); // 转换为整数，单位0.1%
    data[2] = (brakePdlTarget >> 8) & 0xFF;
    data[1] = brakePdlTarget & 0xFF;

    // 设置驻车控制（第3字节，第24-25位）
    data[3] = (data[3] & 0xFC) | (msg.ACU_ChassisEpbCtrl& 0x03) ;

    // 设置循环计数（第6字节，第48-51位）
    data[6] =brake_life;
    brake_life = (++brake_life > 0x0F) ? (brake_life = 0) : brake_life;

    // 计算校验和（第7字节，第56-63位）
    uint16_t checksum = 0;
    for (size_t i = 0; i < 7; ++i) {
        checksum ^= data[i];
    }
    data[7] = checksum & 0xFF;

    return data;
}


std::vector<uint8_t> encodeVehicleCtrlMessage(const Message_133& msg) {
    std::vector<uint8_t> data(8, 0);
    static int LifeSig = 0;
    // 设置车辆左转向灯
    data[0] = (data[0] & 0xFB) | (msg.ACU_VehicleLeftLampCtrl ? 0x04 : 0x00);

    data[0] = (data[0] & 0xF7) | (msg.ACU_VehicleRightLampCtrl ? 0x08 : 0x00);
    data[6] = (data[6] & 0xFE) | (msg.ACU_CheckSumEn ? 0x01 : 0x00);

    return data;
}


// class CANSender {
//     private:
//         int sockfd;
//         struct sockaddr_can addr;
//         struct ifreq ifr;
    
//         void initCAN(const char* interface) {
//             // 创建socket
//             if ((sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
//                 throw std::runtime_error("Socket creation error");
//             }
//             // 绑定CAN接口
//             strcpy(ifr.ifr_name, interface);
//             ioctl(sockfd, SIOCGIFINDEX, &ifr);
//             addr.can_family = AF_CAN;
//             addr.can_ifindex = ifr.ifr_ifindex;
    
//             if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
//                 close(sockfd);
//                 throw std::runtime_error("Socket bind error");
//             }
//         }

//     public:
//         CANSender(const char* interface = "can0") {
//             initCAN(interface);
//         }
    
//         ~CANSender() {
//             close(sockfd);
//         }
    
//         void sendFrame(uint32_t can_id, const std::vector<uint8_t>& data) {
//             struct can_frame frame;
//             frame.can_id = can_id;
//             frame.can_dlc = data.size();
//             memcpy(frame.data, data.data(), data.size());
    
//             if (write(sockfd, &frame, sizeof(frame)) != sizeof(frame)) {
//                 throw std::runtime_error("CAN send error");
//             }
//         }
//     };
    

    // int main() {
    //     try {
    //         CANSender sender("can0");
            
    //         Message_130 drive_msg{
    //             true,   // 驱动使能
    //             0,      // 速度控制模式
    //             1,      // D档
    //             0.5f,  // 目标速度0.5m/s
    //             0,    // 油門%
    //             0, 0    // 自动填充
    //         };
            
    //         Message_132 steer_msg{
    //             true,   // 转向使能
    //             0,      // 前后相同转向模式
    //             100,    // 前轮转向角00°
    //             100,   // 后轮转向角100°
    //             0, 0    // 自动填充
    //         };
            
    //         while (true) {
    //             auto drive_data = encodeDriveControlMessage(drive_msg);
    //             auto steer_data = encodeSteerControlMessage(steer_msg);
                
    //             sender.sendFrame(0x130, drive_data);
    //             sender.sendFrame(0x132, steer_data);
                
    //             std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //         }
    //     } catch (const std::exception& e) {
    //         std::cerr << "Fatal Error: " << e.what() << std::endl;
    //         return 1;
    //     }
    //     return 0;
    // }


// CAN发送函数封装
void sendCANFrame(const char* interface, uint32_t can_id, const std::vector<uint8_t>& data) {
    static int sockfd = -1;
    static struct sockaddr_can addr;
    static struct ifreq ifr;

    // 初始化（只执行一次）
    if (sockfd == -1) {
        // 创建socket
        if ((sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            throw std::runtime_error("Socket creation error: " + std::string(strerror(errno)));
        }

        // 绑定接口
        strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        
        if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
            close(sockfd);
            throw std::runtime_error("IOCTL error: " + std::string(strerror(errno)));
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sockfd);
            throw std::runtime_error("Bind error: " + std::string(strerror(errno)));
        }
    }

    // 构造并发送CAN帧
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = data.size() > CAN_MAX_DLEN ? CAN_MAX_DLEN : data.size();
    memcpy(frame.data, data.data(), frame.can_dlc);

    if (write(sockfd, &frame, sizeof(frame)) != sizeof(frame)) {
        throw std::runtime_error("CAN send error: " + std::string(strerror(errno)));
    }
}

