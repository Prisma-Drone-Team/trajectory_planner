#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <chrono>
#include <cmath>
#include <thread>

using std::placeholders::_1;
using namespace px4_msgs::msg;

// Semplice classe Vector3 per sostituire matrix::Vector3f
class Vector3f {
public:
    float x, y, z;
    
    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    float& operator()(int i) {
        switch(i) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: return x;
        }
    }
    
    Vector3f operator+(const Vector3f& other) const {
        return Vector3f(x + other.x, y + other.y, z + other.z);
    }
    
    Vector3f operator*(float scalar) const {
        return Vector3f(x * scalar, y * scalar, z * scalar);
    }
    
    Vector3f& operator+=(const Vector3f& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    
    bool operator!=(const Vector3f& other) const {
        return (x != other.x || y != other.y || z != other.z);
    }
};

// Semplice classe Quaternion 
class Quaternionf {
public:
    float w, x, y, z;
    
    Quaternionf() : w(1), x(0), y(0), z(0) {}
    Quaternionf(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    float operator()(int i) const {
        switch(i) {
            case 0: return w;
            case 1: return x;
            case 2: return y;
            case 3: return z;
            default: return w;
        }
    }
};

// Funzione per convertire quaternione in yaw
float quaternion_to_yaw(const Quaternionf& q) {
    return std::atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

// Funzione per convertire yaw in quaternione
Quaternionf yaw_to_quaternion(float yaw) {
    float half_yaw = yaw * 0.5f;
    return Quaternionf(std::cos(half_yaw), 0, 0, std::sin(half_yaw));
}

// Funzione per normalizzare angolo tra -PI e PI
float wrap_pi(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("teleop_node")
    {
        // Parametri configurabili
        this->declare_parameter("joy_topic", "/joy");
        this->declare_parameter("odom_topic", "/px4/odometry/out");
        this->declare_parameter("trajectory_setpoint_topic", "/px4/trajectory_setpoint_enu");
        this->declare_parameter("offboard_control_mode_topic", "fmu/in/offboard_control_mode");
        
        // Parametri di controllo
        this->declare_parameter("max_velocity", 2.0);
        this->declare_parameter("max_yaw_rate", 1.0);
        this->declare_parameter("velocity_scale", 1.0);
        this->declare_parameter("yaw_scale", 1.0);
        
        // Mapping assi joystick (configurabile per diversi controller)
        this->declare_parameter("axis_x", 0);      // Roll stick (sinistra/destra)
        this->declare_parameter("axis_y", 1);      // Pitch stick (avanti/indietro)
        this->declare_parameter("axis_z", 3);      // Throttle (su/giù)
        this->declare_parameter("axis_yaw", 2);    // Yaw stick (rotazione)
        
        // Lettura parametri
        joy_topic_ = this->get_parameter("joy_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        trajectory_setpoint_topic_ = this->get_parameter("trajectory_setpoint_topic").as_string();
        offboard_control_mode_topic_ = this->get_parameter("offboard_control_mode_topic").as_string();
        
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
        velocity_scale_ = this->get_parameter("velocity_scale").as_double();
        yaw_scale_ = this->get_parameter("yaw_scale").as_double();
        
        axis_x_ = this->get_parameter("axis_x").as_int();
        axis_y_ = this->get_parameter("axis_y").as_int();
        axis_z_ = this->get_parameter("axis_z").as_int();
        axis_yaw_ = this->get_parameter("axis_yaw").as_int();
        
        // Inizializzazione variabili
        current_position_ = Vector3f(0.0f, 0.0f, 0.0f);
        current_yaw_ = 0.0f;
        target_position_ = Vector3f(0.0f, 0.0f, 0.0f);
        target_velocity_ = Vector3f(0.0f, 0.0f, 0.0f);
        target_yaw_ = 0.0f;
        target_yawspeed_ = 0.0f;
        
        first_odom_received_ = false;
        
        // QoS per PX4
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        
        // Subscribers
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_, 10, std::bind(&TeleopNode::joy_callback, this, _1));
            
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, qos, std::bind(&TeleopNode::odom_callback, this, _1));
        
        // Publishers
        trajectory_setpoint_publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
            trajectory_setpoint_topic_, 10);
            
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
            offboard_control_mode_topic_, qos);
        
        // Timer per il controllo continuo
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&TeleopNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Teleop node initialized");
        RCLCPP_INFO(this->get_logger(), "Joy topic: %s", joy_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Waiting for joystick input and odometry...");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!first_odom_received_) {
            return;
        }
        
        // Verifica che il joystick abbia abbastanza assi
        if (msg->axes.size() <= static_cast<size_t>(std::max({axis_x_, axis_y_, axis_z_, axis_yaw_}))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Joystick doesn't have enough axes");
            return;
        }
        
        // Calcolo velocità target dai comandi joystick
        calculate_target_velocity(msg);
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Aggiorna posizione attuale
        current_position_(0) = msg->pose.pose.position.x;
        current_position_(1) = msg->pose.pose.position.y;
        current_position_(2) = msg->pose.pose.position.z;
        
        // Calcola yaw attuale dal quaternione
        Quaternionf q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
        current_yaw_ = quaternion_to_yaw(q);
        
        // Inizializza target position alla prima odometria
        if (!first_odom_received_) {
            target_position_ = current_position_;
            target_yaw_ = current_yaw_;
            first_odom_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First odometry received. Position: [%.2f, %.2f, %.2f], Yaw: %.2f", 
                current_position_(0), current_position_(1), current_position_(2), current_yaw_);
        }
    }
    
    void calculate_target_velocity(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Converti input joystick in velocità target
        float vx = -msg->axes[axis_yaw_ ] * max_velocity_ * velocity_scale_;
        float vy = msg->axes[axis_z_] * max_velocity_ * velocity_scale_; 
        float vz = msg->axes[ axis_y_ ] * max_velocity_ * velocity_scale_;
        
        target_velocity_(0) = vx;
        target_velocity_(1) = vy;
        target_velocity_(2) = vz;
        
        // Yaw rate
        target_yawspeed_ = msg->axes[axis_x_] * max_yaw_rate_ * yaw_scale_;
        
        // Deadband per evitare drift
        float deadband = 0.05f;
        if (std::abs(target_velocity_(0)) < deadband) target_velocity_(0) = 0.0f;
        if (std::abs(target_velocity_(1)) < deadband) target_velocity_(1) = 0.0f;
        if (std::abs(target_velocity_(2)) < deadband) target_velocity_(2) = 0.0f;
        if (std::abs(target_yawspeed_) < deadband) target_yawspeed_ = 0.0f;
    }
    
    void control_loop()
    {
        if (!first_odom_received_) {
            return;
        }
        
        // Integra velocità per ottenere posizione target
        float dt = 0.05f; // 20Hz
        
        target_position_ += target_velocity_ * dt;
        target_yaw_ += target_yawspeed_ * dt;
        
        // Normalizza yaw
        target_yaw_ = wrap_pi(target_yaw_);
        
        // Pubblica offboard control mode
        publish_offboard_control_mode();
        
        // Pubblica trajectory setpoint
        publish_trajectory_setpoint();
    }
    
    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = true;
        msg.body_rate = false;
        
        offboard_control_mode_publisher_->publish(msg);
    }
    
    void publish_trajectory_setpoint()
    {
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint msg;
        
        // Transform
        geometry_msgs::msg::Transform transform;
        transform.translation.x = target_position_(0);
        transform.translation.y = target_position_(1);
        transform.translation.z = target_position_(2);
        
        // Converti yaw in quaternione
        Quaternionf q_target = yaw_to_quaternion(target_yaw_);
        transform.rotation.w = q_target(0);
        transform.rotation.x = q_target(1);
        transform.rotation.y = q_target(2);
        transform.rotation.z = q_target(3);
        
        // Velocity
        geometry_msgs::msg::Twist velocity;
        velocity.linear.x = target_velocity_(0);
        velocity.linear.y = target_velocity_(1);
        velocity.linear.z = target_velocity_(2);
        velocity.angular.x = 0.0;
        velocity.angular.y = 0.0;
        velocity.angular.z = target_yawspeed_;
        
        // Acceleration (zero per ora)
        geometry_msgs::msg::Twist acceleration;
        acceleration.linear.x = 0.0;
        acceleration.linear.y = 0.0;
        acceleration.linear.z = 0.0;
        acceleration.angular.x = 0.0;
        acceleration.angular.y = 0.0;
        acceleration.angular.z = 0.0;
        
        msg.transforms.push_back(transform);
        msg.velocities.push_back(velocity);
        msg.accelerations.push_back(acceleration);
        msg.time_from_start = rclcpp::Duration::from_seconds(0.0);
        
        trajectory_setpoint_publisher_->publish(msg);
        
        // Debug info
        RCLCPP_DEBUG(this->get_logger(), 
            "Target - Pos: [%.2f, %.2f, %.2f], Vel: [%.2f, %.2f, %.2f], Yaw: %.2f, YawRate: %.2f",
            target_position_(0), target_position_(1), target_position_(2),
            target_velocity_(0), target_velocity_(1), target_velocity_(2),
            target_yaw_, target_yawspeed_);
    }
    
    // Parametri
    std::string joy_topic_;
    std::string odom_topic_;
    std::string trajectory_setpoint_topic_;
    std::string offboard_control_mode_topic_;
    
    double max_velocity_;
    double max_yaw_rate_;
    double velocity_scale_;
    double yaw_scale_;
    
    int axis_x_, axis_y_, axis_z_, axis_yaw_;
    
    // Stato del drone
    Vector3f current_position_;
    float current_yaw_;
    Vector3f target_position_;
    Vector3f target_velocity_;
    float target_yaw_;
    float target_yawspeed_;
    
    bool first_odom_received_;
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting teleop node...");
    RCLCPP_INFO(node->get_logger(), "Controls:");
    RCLCPP_INFO(node->get_logger(), "  Axis 0: X velocity (roll)");
    RCLCPP_INFO(node->get_logger(), "  Axis 1: Y velocity (pitch)");
    RCLCPP_INFO(node->get_logger(), "  Axis 2: Yaw rate");
    RCLCPP_INFO(node->get_logger(), "  Axis 3: Z velocity (throttle)");
    RCLCPP_INFO(node->get_logger(), "Trajectory commands will be published continuously based on joystick input");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
