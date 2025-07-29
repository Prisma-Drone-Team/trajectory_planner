#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include "trajectory_planner/msg/move_cmd.hpp"

using namespace std::chrono_literals;

#define SIMULATION 1

class MoveManager : public rclcpp::Node
{
public:
    MoveManager() : Node("MoveManagerNode")
    {
        RCLCPP_INFO(this->get_logger(), "Move Manager node started, ready to send signals");

        this->declare_parameter("do_transform", 0.0);
        _do_transform = this->get_parameter("do_transform").as_double(); // as_bool not working
        RCLCPP_INFO(get_logger(), "do_transform: %f", _do_transform);

        // Configure QoS per ros2
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        rmw_qos_profile_t cmd_qos_profile = rmw_qos_profile_sensor_data;
        cmd_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        cmd_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        cmd_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        auto cmd_qos = rclcpp::QoS(rclcpp::QoSInitialization(cmd_qos_profile.history, 1), cmd_qos_profile);

        _publisher = this->create_publisher<trajectory_planner::msg::MoveCmd>("move_cmd", cmd_qos);

        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        _pdt_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _pdt_tf_listener = std::make_shared<tf2_ros::TransformListener>(*_pdt_tf_buffer);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_px4 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        if (SIMULATION)
        {
            _odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                    "/fmu/out/vehicle_odometry", qos_px4,
                    [this](px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
                        // Store latest odometry message (protect with mutex)
                        boost::mutex::scoped_lock lock(_odom_mutex);
                        _last_odometry_msg = std::move(msg);
                    });
        }

        _pdt_sub = this->create_subscription<std_msgs::msg::String>(
                                                "/seed_pdt_drone/command", 1,
                                                std::bind(&MoveManager::pdt_callback, this, std::placeholders::_1));

        _plan_status_sub =
                this->create_subscription<std_msgs::msg::String>(
                        "/leo/drone/plan_status", 1,
                        [this](const std_msgs::msg::String::UniquePtr msg) {
                            _plan_status = msg->data;
                            if (_plan_status == "FAILED")
                            {
                                std_msgs::msg::String pdt_msg;
                                if (_current_command.substr(0, 5) == "flyto")
                                {
                                    std::vector<std::string> cv = instance2vector(_current_command);
                                    pdt_msg.data = cv[1] + ".unreachable";
                                    _pdt_publisher->publish(pdt_msg);
                                }
                            }
                        });

        // Timer per tf publishing a 100Hz
        _tf_timer = this->create_wall_timer(10ms, std::bind(&MoveManager::timerTfCallback, this));

        // Publisher pdt status
        _pdt_publisher = this->create_publisher<std_msgs::msg::String>("/seed_pdt_drone/status", 1);

        // Avvio thread per comandi PDT in background
        boost::thread pdt_input_t(&MoveManager::pdt_input, this);
    }

    // Funzione per inviare il move command
    void send_move_cmd(const std::string &cmd, const geometry_msgs::msg::Pose &pose)
    {
        auto message = trajectory_planner::msg::MoveCmd();
        message.header.stamp = this->now();
        message.command.data = cmd;
        message.pose = pose;
        _publisher->publish(message);
        RCLCPP_INFO(this->get_logger(), "%s command signal sent. Pose: %f,%f,%f",
                                cmd.c_str(), pose.position.x, pose.position.y, pose.position.z);
    }

    // Timer callback per inviare le trasformazioni tf a 100Hz
    void timerTfCallback()
    {
        // Pubblica il dynamic tf se abbiamo ricevuto odometria
        {
            boost::mutex::scoped_lock lock(_odom_mutex);
            if (_last_odometry_msg)
            {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.stamp = this->get_clock()->now();
                transform_stamped.header.frame_id = "odomNED"; // come nella versione originale
                transform_stamped.child_frame_id = "base_link_FRD";
                transform_stamped.transform.translation.x = _last_odometry_msg->position[0];
                transform_stamped.transform.translation.y = _last_odometry_msg->position[1];
                transform_stamped.transform.translation.z = _last_odometry_msg->position[2];
                transform_stamped.transform.rotation.x = _last_odometry_msg->q.data()[1];
                transform_stamped.transform.rotation.y = _last_odometry_msg->q.data()[2];
                transform_stamped.transform.rotation.z = _last_odometry_msg->q.data()[3];
                transform_stamped.transform.rotation.w = _last_odometry_msg->q.data()[0];
                _tf_broadcaster->sendTransform(transform_stamped);
            }
        }
        // Pubblica le static tf
        staticTfPub();
    }

    // Funzione per le static transforms
    void staticTfPub()
    {
        if(_do_transform){
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "odom";
            t.child_frame_id = "odomNED";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = 0.7071068;
            t.transform.rotation.y = 0.7071068;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 0.0;
            _static_tf_broadcaster->sendTransform(t);

            geometry_msgs::msg::TransformStamped t2;
            t2.header.stamp = this->get_clock()->now();
            t2.header.frame_id = "base_link_FRD";
            t2.child_frame_id = "base_link";
            t2.transform.translation.x = 0.0;
            t2.transform.translation.y = 0.0;
            t2.transform.translation.z = 0.0;
            t2.transform.rotation.x = 1.0;
            t2.transform.rotation.y = 0.0;
            t2.transform.rotation.z = 0.0;
            t2.transform.rotation.w = 0.0;
            _static_tf_broadcaster->sendTransform(t2);

            geometry_msgs::msg::TransformStamped t3;
            t3.header.stamp = this->get_clock()->now();
            t3.header.frame_id = "base_link";
            t3.child_frame_id = "x500_depth_0/OakD-Lite/base_link/StereoOV7251";
            t3.transform.translation.x = 0.15;
            t3.transform.translation.y = 0.03;
            t3.transform.translation.z = -0.202;
            t3.transform.rotation.x = -0.5;
            t3.transform.rotation.y = 0.5;
            t3.transform.rotation.z = -0.5;
            t3.transform.rotation.w = 0.5;
            _static_tf_broadcaster->sendTransform(t3);

            geometry_msgs::msg::TransformStamped t4;
            t4.header.stamp = this->get_clock()->now();
            t4.header.frame_id = "base_link";
            t4.child_frame_id = "x500_depth_0/OakD-Lite/base_link/IMX214";
            t4.transform.translation.x = 0.15;
            t4.transform.translation.y = 0.03;
            t4.transform.translation.z = -0.202;
            t4.transform.rotation.x = -0.5;
            t4.transform.rotation.y = 0.5;
            t4.transform.rotation.z = -0.5;
            t4.transform.rotation.w = 0.5;
            _static_tf_broadcaster->sendTransform(t4);
        }
    }

    bool checkTransform(const std::string &frame_name, geometry_msgs::msg::Pose &pose)
    {
        geometry_msgs::msg::TransformStamped tf_appr;
        try
        {
            tf_appr = _pdt_tf_buffer->lookupTransform("map", frame_name, tf2::TimePointZero);
            pose.position.x = tf_appr.transform.translation.x;
            pose.position.y = tf_appr.transform.translation.y;
            pose.position.z = tf_appr.transform.translation.z;
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not find a transform from map to %s: %s", frame_name.c_str(), ex.what());
            return false;
        }
    }

    std::vector<std::string> instance2vector(std::string schemaInstance)
    {
        bool isAtom = true, isString = false;
        char c;
        std::string app;
        std::vector<std::string> result;
        std::stringstream ss(schemaInstance);
        int count = 0;
        ss >> std::noskipws;
        ss >> c;
        while (!ss.eof())
        {
            if (c == '"' && !isString)
            {
                isString = true;
                app += c;
            }
            else if (c == '"' && isString)
            {
                isString = false;
                app += c;
            }
            else if (isString)
            {
                app += c;
            }
            else if (c == '(' && isAtom)
            {
                isAtom = false;
                result.push_back(app);
                app = "";
            }
            else if (c == '(' || c == '[')
            {
                count++;
                app += c;
            }
            else if ((c == ')' || c == ']') && count != 0)
            {
                count--;
                app += c;
            }
            else if (c != ',' || count != 0)
            {
                app += c;
            }
            else
            {
                result.push_back(app);
                app = "";
            }
            ss >> c;
        }
        if (isAtom)
        {
            if (app.find('\\') != std::string::npos)
            {
                std::stringstream ss2(app);
                std::string substr;
                while (std::getline(ss2, substr, '\\'))
                {
                    result.push_back(substr);
                }
            }
            else
                result.push_back(app);
        }
        else
        {
            app.erase(app.size() - 1);
            result.push_back(app);
        }
        return result;
    }

    void pdt_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        _received_command = msg->data;
    }

    // Thread per read dei comandi PDT
    void pdt_input()
    {
        auto sp = geometry_msgs::msg::Pose();
        sp.position.x = 0.0;
        sp.position.y = 0.0;
        sp.position.z = 0.0;

        std::string cmd_to_send;
        std::vector<std::string> cv;

        while (rclcpp::ok())
        {
            usleep(10000); // 0.01 sec
            if (_current_command != _received_command)
            {
                cv = instance2vector(_received_command);
                if (_plan_status == "RUNNING")
                {
                    cmd_to_send = "stop";
                    send_move_cmd(cmd_to_send, sp);
                }
                while (_plan_status != "STOPPED" && _plan_status != "IDLE" && _plan_status != "FAILED")
                {
                    usleep(100000);
                    RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for STOPPED or IDLE status");
                }
                cv = instance2vector(_received_command);
                RCLCPP_INFO(this->get_logger(), "New command %s", _received_command.c_str());
                if (cv[0] == "flyto")
                {
                    if (checkTransform(cv[1], sp))
                    {
                        _current_command = _received_command;
                        cmd_to_send = "nav";
                        send_move_cmd(cmd_to_send, sp);
                        RCLCPP_INFO(this->get_logger(), "NAV command sent");
                    }
                }
                else if (cv[0] == "takeoff")
                {
                    cmd_to_send = "arm";
                    send_move_cmd(cmd_to_send, sp);
                    cmd_to_send = "takeoff";
                    _current_command = _received_command;
                    sp.position.z = 1.5;
                    send_move_cmd(cmd_to_send, sp);
                    RCLCPP_INFO(this->get_logger(), "TAKEOFF command sent");
                }
                else if (cv[0] == "land")
                {
                    cmd_to_send = "land";
                    _current_command = _received_command;
                    send_move_cmd(cmd_to_send, sp);
                    RCLCPP_INFO(this->get_logger(), "LAND command sent");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid command");
                }
            }
            usleep(100000);
        }
    }

    // Thread per key_input se necessario (simile a pdt_input)
    void key_input()
    {
        auto sp = geometry_msgs::msg::Pose();
        while (rclcpp::ok())
        {
            sp.position.x = 0.0;
            sp.position.y = 0.0;
            sp.position.z = 0.0;
            std::cout << "Enter command [arm | takeoff | go | nav | land | term ]: \n";
            std::cin >> _cmd;
            if (_cmd == "go" || _cmd == "nav")
            {
                std::cout << "Enter X coordinate (ENU frame): ";
                std::cin >> sp.position.x;
                std::cout << "Enter Y coordinate (ENU frame): ";
                std::cin >> sp.position.y;
                std::cout << "Enter Z coordinate (ENU frame): ";
                std::cin >> sp.position.z;
                send_move_cmd(_cmd, sp);
            }
            else if (_cmd == "takeoff")
            {
                std::cout << "Enter takeoff altitude (ENU frame): ";
                std::cin >> sp.position.z;
                send_move_cmd(_cmd, sp);
            }
            else if (_cmd == "arm" || _cmd == "stop" || _cmd == "land" || _cmd == "term")
            {
                send_move_cmd(_cmd, sp);
            }
            else
            {
                RCLCPP_ERROR_ONCE(this->get_logger(), "Unknown command");
            }
            usleep(100000);
        }
    }

private:
    rclcpp::Publisher<trajectory_planner::msg::MoveCmd>::SharedPtr _publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pdt_publisher;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odometry_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _plan_status_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _pdt_sub;

    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;

    rclcpp::TimerBase::SharedPtr _tf_timer;

    std::unique_ptr<tf2_ros::Buffer> _pdt_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _pdt_tf_listener;

    std::string _cmd;
    std::string _received_command = "";
    std::string _current_command = "";
    std::string _plan_status = "";
    double _do_transform;

    // Mutex per proteggere l'ultimo messaggio di odometria
    boost::mutex _odom_mutex;
    px4_msgs::msg::VehicleOdometry::UniquePtr _last_odometry_msg;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
