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
#include <boost/thread.hpp>
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
            _publisher = this->create_publisher<trajectory_planner::msg::MoveCmd>("move_cmd",qos);

            //_timer = this->create_wall_timer(_timer_period, std::bind(&MoveManager::key_input, this));
            boost::thread key_input_t( &MoveManager::key_input, this);
        }

        void send_move_cmd(const std::string& cmd, const geometry_msgs::msg::Pose& pose)
        {
            auto message = trajectory_planner::msg::MoveCmd();
            message.header.stamp = this->now();
            message.command.data = cmd;
            message.pose = pose;
            _publisher->publish(message);
            RCLCPP_INFO(this->get_logger(), "%s command signal sent.", cmd.c_str());
        }

        void key_input()
        {   	
            
            auto sp = geometry_msgs::msg::Pose();
            float  yaw_d;

            while(rclcpp::ok()) {

                //RCLCPP_INFO_ONCE(this->get_logger(), "Enter command [arm | takeoff | go | nav | land | term ]: ");
                std::cout << "Enter command [arm | takeoff | go | nav | land | term ]: \n"; 
                std::cin >> _cmd;

                if(_cmd == "go" || _cmd == "nav") {

                    std::cout << "Enter X coordinate (ENU frame): "; 
                    std::cin >> sp.position.x;
                    std::cout << "Enter Y coordinate (ENU frame): "; 
                    std::cin >> sp.position.y;
                    std::cout << "Enter Z coordinate (ENU frame): "; 
                    std::cin >> sp.position.z;
                    
                    // sp = _T_enu_to_ned*sp;  // No conversione here
                    send_move_cmd(_cmd,sp);
                }
                else if(_cmd == "takeoff") {
                    std::cout << "Enter takeoff altitude (ENU frame): "; 
                    std::cin >> sp.position.z;
                    send_move_cmd(_cmd,sp);
                }
                else if(_cmd == "land") {
                    std::cout << "Landing procedure triggered... \nRemember to kill disarm manually after landed.\n";
                    send_move_cmd(_cmd,sp);
                }
                else if(_cmd == "arm") {
                    send_move_cmd(_cmd,sp);
                }
                else if(_cmd == "term") {
                   send_move_cmd(_cmd,sp);
                }
                else {
                    std::cout << "Unknown command;\n";
                }

            }

            
        }

    private:
        rclcpp::Publisher<trajectory_planner::msg::MoveCmd>::SharedPtr _publisher;
        rclcpp::TimerBase::SharedPtr _timer;
        std::string _cmd;

};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveManager>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
