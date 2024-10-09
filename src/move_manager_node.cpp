#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>
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
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>


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
            _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            this->staticTfPub();

            _pdt_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            _pdt_tf_listener=   std::make_shared<tf2_ros::TransformListener>(*_pdt_tf_buffer);

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos_px4 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

            //_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

            _odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos_px4,
                [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
                    // Prepare the TransformStamped message
                    geometry_msgs::msg::TransformStamped transform_stamped;

                    // Set the header
                    transform_stamped.header.stamp = this->get_clock()->now();
                    transform_stamped.header.frame_id = "odomNED";  // Set to appropriate frame (ENU)
                    transform_stamped.child_frame_id = "base_link_FRD";  // Set to appropriate frame

                    // Set translation (position)
                    transform_stamped.transform.translation.x = msg->position[0];
                    transform_stamped.transform.translation.y = msg->position[1];
                    transform_stamped.transform.translation.z = msg->position[2];

                    // Set rotation (orientation)
                    transform_stamped.transform.rotation.x = msg->q.data()[1];
                    transform_stamped.transform.rotation.y = msg->q.data()[2];
                    transform_stamped.transform.rotation.z = msg->q.data()[3];
                    transform_stamped.transform.rotation.w = msg->q.data()[0];

                    // Broadcast the transform
                    _tf_broadcaster->sendTransform(transform_stamped);
                    
                     // Publish odometry message
                    // nav_msgs::msg::Odometry odom;
                    // odom.header.stamp = this->get_clock()->now();
                    // odom.header.frame_id = "odomNED";  // Parent frame
                    // odom.child_frame_id = "base_link_FRD";  // Child frame
                    
                    // // Set position
                    // odom.pose.pose.position.x = msg->position[0];
                    // odom.pose.pose.position.y = msg->position[1];
                    // odom.pose.pose.position.z = msg->position[2];
                    // odom.pose.pose.orientation.x = msg->q.data()[1];
                    // odom.pose.pose.orientation.y = msg->q.data()[2];
                    // odom.pose.pose.orientation.z = msg->q.data()[3];
                    // odom.pose.pose.orientation.w = msg->q.data()[0];

                    // // Set velocity
                    // odom.twist.twist.linear.x = msg->velocity[0];
                    // odom.twist.twist.linear.y = msg->velocity[1];
                    // odom.twist.twist.linear.z = msg->velocity[2];

                    // // Set angular velocity
                    // odom.twist.twist.angular.x = msg->angular_velocity[0];
                    // odom.twist.twist.angular.y = msg->angular_velocity[1];
                    // odom.twist.twist.angular.z = msg->angular_velocity[2];

                    // odom.pose.covariance[0] = msg->position_variance[0]; 
                    // odom.pose.covariance[7] = msg->position_variance[1]; 
                    // odom.pose.covariance[14] = msg->position_variance[2]; 
                    // odom.pose.covariance[21] = msg->orientation_variance[0];  
                    // odom.pose.covariance[28] = msg->orientation_variance[1]; 
                    // odom.pose.covariance[35] = msg->orientation_variance[2];  

                    // // Set covariance for twist (uncertainty in linear and angular velocities)
                    // odom.twist.covariance[0] = msg->velocity_variance[0];  // x velocity variance
                    // odom.twist.covariance[7] = msg->velocity_variance[1];  // y velocity variance
                    // odom.twist.covariance[14] = msg->velocity_variance[2]; // z velocity variance (unused for 2D)
                    // odom.twist.covariance[21] = 0.0;  // Roll velocity variance (fixed for 2D)
                    // odom.twist.covariance[28] = 0.0;  // Pitch velocity variance (fixed for 2D)
                    // odom.twist.covariance[35] = 0.0;  // Yaw velocity variance

                    //_odom_publisher->publish(odom);
            });
            

            _pdt_sub = this->create_subscription<std_msgs::msg::String>("/seed_inspect_drone/command", qos, std::bind(&MoveManager::pdt_callback, this, std::placeholders::_1));

            _plan_status_sub = this->create_subscription<std_msgs::msg::String>("/leo/drone/plan_status", 1,
                [this](const std_msgs::msg::String::UniquePtr msg) {
                    _plan_status = msg->data;
                });
            
            boost::thread key_input_t( &MoveManager::key_input, this);
            boost::thread pdt_input_t( &MoveManager::pdt_input, this);

        }


        void staticTfPub(){
            // CHECK conversion from ENU to NED + move in the launch file static transforms
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

            geometry_msgs::msg::TransformStamped goal;

            goal.header.stamp = this->get_clock()->now();
            goal.header.frame_id = "odom"; //"map"
            goal.child_frame_id = "goal3";
            goal.transform.translation.x = 5.0;
            goal.transform.translation.y = 5.0;
            goal.transform.translation.z = 1.0;
            goal.transform.rotation.x = 0.7071068;
            goal.transform.rotation.y = 0.7071068;
            goal.transform.rotation.z = 0.0;
            goal.transform.rotation.w = 0.0;

            _static_tf_broadcaster->sendTransform(goal);
        }

        void send_move_cmd(const std::string& cmd, const geometry_msgs::msg::Pose& pose);


    private:

        bool checkTransform(const std::string& frame_name, geometry_msgs::msg::Pose& pose);
        
        void pdt_callback(const std_msgs::msg::String::SharedPtr msg);
        void key_input();
        void pdt_input();
        std::vector<std::string> instance2vector(std::string schemaInstance);
    
        rclcpp::Publisher<trajectory_planner::msg::MoveCmd>::SharedPtr _publisher;
        
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_publisher;

        // Subscriber for vehicle odometry
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odometry_sub;
         
        // Subscriber for planner status
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _plan_status_sub;
        // Subscriber for GCS command
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _pdt_sub;

        // TF broadcaster
        std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;

        rclcpp::TimerBase::SharedPtr _timer;

        std::string _cmd;
        std::string _received_command="";
        std::string _current_command="";

        std::string _plan_status = "";

        std::shared_ptr<tf2_ros::TransformListener> _pdt_tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> _pdt_tf_buffer;
        geometry_msgs::msg::Pose _pdt_tf_goal;

};


bool MoveManager::checkTransform(const std::string& frame_name, geometry_msgs::msg::Pose& pose) {

    geometry_msgs::msg::TransformStamped tf_appr;
    
    try {
        
        tf_appr = _pdt_tf_buffer->lookupTransform( "map", frame_name, tf2::TimePointZero);

        pose.position.x = tf_appr.transform.translation.x;
        pose.position.y = tf_appr.transform.translation.y;
        pose.position.z = tf_appr.transform.translation.z;
        
        return true;
    } catch (const tf2::TransformException & ex) {

        RCLCPP_WARN( this->get_logger(), "Could not find a transform from map to %s: %s", frame_name.c_str(), ex.what());
        return false;
    }

}

std::vector<std::string> MoveManager::instance2vector(std::string schemaInstance){

    bool isAtom=true, isString=false;
    char c;
    std::string app;
    std::vector<std::string> result;
    std::stringstream ss(schemaInstance);
    int count=0;
    ss >> std::noskipws;
    //leggi il primo carattere della stringa
    ss>>c;
    //mentre non sei a fine stringa
    while(!ss.eof())
    {
        //se il carattere è un doppio apice e non sono in una stringa
        if(c=='"' && !isString){
            //allora sono in una stringa
            isString=true;
            //aggiungo l'apice
            app=app+c;
        }
        //se il carattere è un doppio apice e sono in una stringa
        else if(c=='"' && isString){
            //la stringa è finita
            isString=false;
            //aggiungo l'apice
            app=app+c;
            //aggiungila come elemento del funtore
            //result.push_back(app);
        }
        //mentre sono in una stringa
        else if(isString){
            //aggiungi il carattere senza controllarlo
            app=app+c;
        }
        //se sono un atomo ed il carattere letto è una parentesi aperta
        else if(c=='(' && isAtom){
            //non sono più un atomo
            isAtom=false;
            //inserisco il nome come primo elemento del vettore
            result.push_back(app);
            //pulisco la stringa d'appoggio
            app="";
            //salto la parentesi
//            ss>>c;
        }
        else if(c=='(' || c=='['){
            count++;
            app=app+c;
        }
        else if( ( c==')' || c==']' ) && count!=0){
            count--;
            app=app+c;
        }
        //se il carattere letto non è una virgola
        else if(c!=',' || count!=0)
            //aggiungilo alla stringa d'appoggio
            app=app+c;
        //altrimenti (ie. il carattere è una virgola)
        else {
            //inserisci la stringa d'appoggio nel vettore risultato
            result.push_back(app);
            //pulisci la stringa d'appoggio
            app="";
            //ho saltato la virgola
        }
        //leggi il successivo carattere
        ss>>c;
    }
    //se lo schema non ha parametri aggiungi il solo nome (vec[0])
    if(isAtom) {
        //check the \ character and split by it (added 01/12/2020 in seed 4.0)
        if( app.find('\\') != std::string::npos ){
            std::stringstream ss2(app);
            std::string substr;
            //std::cout<<"INSTANCE TO VECTOR: "<<schemaInstance<<std::endl;
            while(std::getline(ss2, substr, '\\')){
                result.push_back(substr);
                //std::cout<<"split: "<<substr<<std::endl;
            }
        }
        else
            result.push_back(app);
    }
    //altrimenti aggiungi l'ultima stringa rimuovendo l'ultima parentesi
    else{
        app.erase(app.size()-1);
        result.push_back(app);
    }
    //ritorna il vettore calcolato
    return result;
}


void MoveManager::pdt_callback(const std_msgs::msg::String::SharedPtr msg){

    _received_command = msg->data;
}


void MoveManager::send_move_cmd(const std::string& cmd, const geometry_msgs::msg::Pose& pose){
    auto message = trajectory_planner::msg::MoveCmd();
    message.header.stamp = this->now();
    message.command.data = cmd;
    message.pose = pose;
    _publisher->publish(message);
    RCLCPP_INFO(this->get_logger(), "%s command signal sent. Pose: %f,%f,%f", cmd.c_str(), pose.position.x, pose.position.y, pose.position.z);
}

void MoveManager::pdt_input(){   	
    
    auto sp = geometry_msgs::msg::Pose();
    // Set the position to zero for safety
    sp.position.x = 0.0;
    sp.position.y = 0.0;
    sp.position.z = 0.0;

    float  yaw_d;
    std::string cmd_to_send;
    std::vector<std::string> cv;

    while(rclcpp::ok()) {

     
        if(_current_command != _received_command) {

            cmd_to_send = "stop";
            send_move_cmd(cmd_to_send,sp);

            while( _plan_status != "STOPPED" && _plan_status != "IDLE"){
                usleep(0.1*16);
                RCLCPP_INFO(this->get_logger(), "Waiting for stop");
            }

            //start action
            
            cv = instance2vector( _received_command);

            RCLCPP_INFO(this->get_logger(),"New command %s", _received_command.c_str() );

            if ( cv[0] == "flyto" ) {
                
                if(checkTransform(cv[1], sp)){
                    _current_command = _received_command;
                    cmd_to_send = "nav";  
                    send_move_cmd(cmd_to_send,sp);
                    RCLCPP_INFO(this->get_logger(), "NAV command sent");
                }           

            }
            else if ( cv[0] == "takeoff" )
            {   
                cmd_to_send = "takeoff";
                _current_command = _received_command;
                sp.position.z = 1.5;
                send_move_cmd(cmd_to_send,sp);
                RCLCPP_INFO(this->get_logger(), "TAKEOFF command sent");
            }
            else if ( cv[0] == "land" )
            {
                cmd_to_send = "land";
                _current_command = _received_command;
                send_move_cmd(cmd_to_send,sp);
                RCLCPP_INFO(this->get_logger(), "LAND command sent");
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Invalid command");
            }
            
        }
        // if( _current_command == _received_command && _plan_status == "REPLAN"){

        //     if ( cv[0] == "flyto" ) {
                
        //         if(checkTransform(cv[1], sp)){
        //             cmd_to_send = "nav";  
        //             send_move_cmd(cmd_to_send,sp);
        //             RCLCPP_INFO(this->get_logger(), "REPLAN: NAV command sent");
        //         }           

        //     }
        // }

    }

    
}

void MoveManager::key_input(){   	
    
    auto sp = geometry_msgs::msg::Pose();
    float  yaw_d;

    while(rclcpp::ok()) {

        // Set the position to zero for safety
        sp.position.x = 0.0;
        sp.position.y = 0.0;
        sp.position.z = 0.0;

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
            
            send_move_cmd(_cmd,sp);
        }
        else if(_cmd == "takeoff") {
            std::cout << "Enter takeoff altitude (ENU frame): "; 
            std::cin >> sp.position.z;
            send_move_cmd(_cmd,sp);
        }
        else if(_cmd == "arm" ) {
            send_move_cmd(_cmd,sp);
        }
        else if(_cmd == "stop" ) {
            send_move_cmd(_cmd,sp);
        }
        else if(_cmd == "land" ) {
            send_move_cmd(_cmd,sp);
        }
        else if(_cmd == "term" ) {
            send_move_cmd(_cmd,sp);
        }
        else if(_cmd != "arm" && _cmd != "takeoff" && _cmd != "go" && _cmd != "nav" && _cmd != "land" && _cmd != "term") {
            RCLCPP_ERROR_ONCE(this->get_logger(), "Unknown command");
        }

    }

    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveManager>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
