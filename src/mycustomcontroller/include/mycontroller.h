#ifndef MY_CUSTOM_CONTROLLER_H
#define MY_CUSTOM_CONTROLLER_H

//Including prrimary packages needed for interfacing correctly with the robot

#include <controller_interface/controller.h>   //contains abstract controller type needed to write our own controller
#include <hardware_interface/joint_command_interface.h>//Provides a way to send our commands to the actuators, aka motors
#include <hardware_interface/joint_state_interface.h> // Provides a way for getting joint states, aka "Feedback" for our controller
#include <pluginlib/class_list_macros.h> //Can be used to provide out controller as a plugin
#include <ros/node_handle.h> //Necessary to create nodes and inter-ROS communication mechanisms
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <string>
#include <thread>
#include <mutex>

namespace mycustomcontroller {

class mycontroller : public controller_interface::Controller<hardware_interface::EffortJointInterface> 
{

public:
    bool init (hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n); //Responsible for initializing our controller
    void update(const ros::Time& time, const ros::Duration& period) override; //Constantly updates the values of the controlled value

     mycontroller() : outer_loop_rate_(40.0), inner_loop_rate_(80.0) {};

     //void starting(const ros::Time& time);
     //void stopping(const ros::Time& time);

private:


    double outer_loop_rate_ , inner_loop_rate_;

    std::mutex mutex_;

    std::vector<hardware_interface::JointHandle> joint_handles_; //Create an array jointhandles, joints handles are objects that represent each joint, used to send effort commands to the joint
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_; //Create an array jointstatehandles, used to receive the states of each joint

//PID parameters for the velocity (inner) control loop//
    double kp_Vel_ , ki_Vel_ , kd_Vel_;

    ros::Timer outer_timer_, inner_timer_;
//PID parameters for the position (outer) control loop//
    double kp_pos_, ki_pos_, kd_pos_;

    double min_effort, max_effort;

    std::vector<double> desired_positions_; //Vector to store the desired values of each position
    std::vector<double> desired_velocities_;

    std::vector<double> prev_errors_vel_; // vector to store the prevoius error values for the Kd element of the PID contorl loop of velocity controller
    std::vector<double> prev_errors_pos_; // vector to store the prevoius error values for the Kd element of the PID contorl loop of position controller

    std::vector<double> integral_terms_vel_; // 
    std::vector<double> integral_terms_pos_; //
    //bool running_;
    ros::Subscriber setpoint_sub_;


    double calculateEffortCommand(const hardware_interface::JointHandle& joint, double desired_velocity, int joint_index); //Inner loop velocity control
    double CalculateDesiredVel(const hardware_interface::JointHandle& joint, double desired_position, int joint_index); //Outer Loop position control
    void setpointCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);   
    void outerLoop(const ros::TimerEvent& event);
    void innerLoop(const ros::TimerEvent& event);


};  



}




#endif