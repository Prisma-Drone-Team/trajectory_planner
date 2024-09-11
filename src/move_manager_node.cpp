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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
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
            _publisher = this->create_publisher<trajectory_planner::msg::MoveCmd>("move_cmd",qos);

            _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            this->staticTfPub();

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos_px4 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

            _odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/vehicle_odometry", qos_px4,
                [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
                    // Prepare the TransformStamped message
                    geometry_msgs::msg::TransformStamped transform_stamped;

                    // Set the header
                    transform_stamped.header.stamp = this->get_clock()->now();
                    transform_stamped.header.frame_id = "mapNED";  // Set to appropriate frame (ENU)
                    transform_stamped.child_frame_id = "drone_link";  // Set to appropriate frame

                    // Set translation (position)
                    transform_stamped.transform.translation.x = msg->position[0];
                    transform_stamped.transform.translation.y = msg->position[1];
                    transform_stamped.transform.translation.z = msg->position[2];

                    // Set rotation (orientation)
                    transform_stamped.transform.rotation.x = msg->q.data()[0];
                    transform_stamped.transform.rotation.y = msg->q.data()[1];
                    transform_stamped.transform.rotation.z = msg->q.data()[2];
                    transform_stamped.transform.rotation.w = msg->q.data()[3];

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

        void staticTfPub(){
            // CHECK conversion from ENU to NED + move in the launch file static transforms
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = "mapNED";

            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;

            t.transform.rotation.x = 0.7071068;
            t.transform.rotation.y = 0.7071068;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 0.0;

            _static_tf_broadcaster->sendTransform(t);
        }

    private:
    
        rclcpp::Publisher<trajectory_planner::msg::MoveCmd>::SharedPtr _publisher;
        // Subscriber for vehicle odometry
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odometry_sub;

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
