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
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h"

class MoveManager : public rclcpp::Node
{
    public:
        MoveManager() : Node("MoveManagerNode")
        {
            // Configure QoS to ensure the last message is saved and sent to new subscribers
            rclcpp::QoS qos(rclcpp::KeepLast(1));
            auto _timer_period = std::chrono::milliseconds(1000);
        
            RCLCPP_INFO(this->get_logger(), "Move Manager node started, ready to send signals");
          
            _publisher = this->create_publisher<trajectory_planner::msg::MoveCmd>("move_cmd",qos);  // IMPORTANT: check QoS settings

            _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos_px4 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

            _local_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>( "/mavros/local_position/pose", qos_px4,
            [this](const geometry_msgs::msg::PoseStamped::UniquePtr msg) {
                // Prepare the TransformStamped message
                geometry_msgs::msg::TransformStamped transform_stamped;

                // Set the header
                transform_stamped.header.stamp = this->get_clock()->now();
                transform_stamped.header.frame_id = "fake_odom";  // Set to appropriate frame (ENU)
                transform_stamped.child_frame_id = "base_link";  // Set to appropriate frame

                // Set translation (position)
                transform_stamped.transform.translation.x = msg->pose.position.x;
                transform_stamped.transform.translation.y = msg->pose.position.y;
                transform_stamped.transform.translation.z = msg->pose.position.z;

                // Set rotation (orientation)
                transform_stamped.transform.rotation.x = msg->pose.orientation.x;
                transform_stamped.transform.rotation.y = msg->pose.orientation.y;
                transform_stamped.transform.rotation.z = msg->pose.orientation.z;
                transform_stamped.transform.rotation.w = msg->pose.orientation.w;

                // Broadcast the transform
                _tf_broadcaster->sendTransform(transform_stamped);
                
            });
        

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
                   // send_move_cmd(_cmd,sp);
                }
                else if(_cmd == "takeoff") {
                    std::cout << "Enter takeoff altitude (ENU frame): "; 
                    std::cin >> sp.position.z;
                   // send_move_cmd(_cmd,sp);
                }
                else if(_cmd != "arm" && _cmd != "takeoff" && _cmd != "go" && _cmd != "nav" && _cmd != "land" && _cmd != "term") {
                    RCLCPP_ERROR_ONCE(this->get_logger(), "Unknown command");
                }
                
                send_move_cmd(_cmd,sp);
            }

            
        }


    private:
    
        rclcpp::Publisher<trajectory_planner::msg::MoveCmd>::SharedPtr _publisher;
        // Subscriber for vehicle odometry
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _local_pose_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_publisher;

        // TF broadcaster
        std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;

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
