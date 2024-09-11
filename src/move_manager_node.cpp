#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <chrono>
#include "trajectory_planner/msg/move_cmd.hpp"

class MoveManager : public rclcpp::Node
{
    public:
        MoveManager() : Node("MoveManagerNode")
        {
            // Configure QoS to ensure the last message is saved and sent to new subscribers
            rclcpp::QoS qos(rclcpp::KeepLast(1));
            auto _timer_period = std::chrono::milliseconds(1000);
        
            RCLCPP_INFO(this->get_logger(), "Move Manager node started, ready to send signals");
            publisher_ = this->create_publisher<std_msgs::msg::String>("/move_cmd", qos);

            _timer = this->create_wall_timer(_timer_period, std::bind(&MoveManager::key_input, this));
            
        }

        void send_move_cmd(const std::string& cmd)
        {
            auto message = std_msgs::msg::String();
            message.data = cmd;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "%s command signal sent.", cmd.c_str());
        }

        void key_input()
        {
            std::string cmd;
            while(rclcpp::ok()) {
                std::cout << "Enter command [arm | go | takeoff | land | stop | nav | term]: \n"; 
                std::cin >> cmd;
                if(cmd == "term" || cmd == "nav" || cmd == "stop" || cmd == "land" || cmd == "takeoff" || cmd == "go" || cmd == "arm"){
                    send_move_cmd(cmd);

                }else {
                    std::cout << "Enter command [arm | go | takeoff | land | stop | nav | term]: \n"; 
                }
            }
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr _timer;

};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveManager>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
