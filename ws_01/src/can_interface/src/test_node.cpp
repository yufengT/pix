/*
需求：在终端输出hello
流程：
1.初始化包含头文件
2.初始化ROS2客户端
3.创建节点
4.输出日志
5.释放资源

*/


#include "rclcpp/rclcpp.hpp"
int main (int argc,char **argv){
    //初始化ROS2
    rclcpp::init(argc,argv);
    //创建节点
    auto node = rclcpp::Node ::make_shared("helloworld_node");

    //输出日志
    RCLCPP_INFO(node->get_logger(),"hello world!");

    //释放资源
    rclcpp::shutdown();
    return 0;

} 







