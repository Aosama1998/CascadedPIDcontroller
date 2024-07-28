#include <mycontroller.h>
#include <ros/ros.h>



bool mycustomcontroller::mycontroller::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{
    std::vector<std::string> joint_names;
    if (!n.getParam("joints", joint_names)) 
    { 
        ROS_ERROR("No joints given in the namespace: '%s')", n.getNamespace().c_str());
        return false;
    }

    n.param("kp_vel", kp_Vel_, 1.0);
    n.param("ki_vel", ki_Vel_, 0.0);
    n.param("kd_vel", kd_Vel_, 0.0);

    n.param("kp_pos", kp_pos_, 1.0); 
    n.param("ki_pos", ki_pos_, 0.0);
    n.param("kd_pos", kd_pos_, 0.0);

    for (const auto& joint_name : joint_names) 
    {
        joint_handles_.push_back(hw->getHandle(joint_name));
    }

    prev_errors_vel_.resize(joint_handles_.size(), 0.0);
    integral_terms_vel_.resize(joint_handles_.size(), 0.0);

    ROS_INFO("joint handle size is %ld", joint_handles_.size());

    prev_errors_pos_.resize(joint_handles_.size(), 0.0);
    integral_terms_pos_.resize(joint_handles_.size(), 0.0);

    desired_positions_.resize(joint_handles_.size(), 0.0);
    desired_velocities_.resize(joint_handles_.size(), 0.0);

    setpoint_sub_ = n.subscribe<std_msgs::Float64MultiArray>("Desired_Pos", 10, &mycontroller::setpointCallback, this);

    // Initialize ROS timers for outer and inner loops
    outer_timer_ = n.createTimer(ros::Duration(1.0 / outer_loop_rate_), &mycontroller::outerLoop, this); 
    ROS_INFO("Outer timer initialized properly");
    //ROS_INFO("Outer Loop Rate is %f", outer_loop_rate_);
    inner_timer_ = n.createTimer(ros::Duration(1.0 / inner_loop_rate_), &mycontroller::innerLoop, this); 
    ROS_INFO("Inner timer initialized properly");
    return true; 
}
/*
void mycontroller::starting(const ros::Time& time)
{
    ROS_INFO("Controller starting");
}

void mycontroller::stopping(const ros::Time& time)
{
    ROS_INFO("Controller stopping");
    // Cleanup or stop any ongoing processes if necessary
}
*/

void mycustomcontroller::mycontroller::outerLoop(const ros::TimerEvent&)
{
    ROS_INFO("Entering outerLoop on thread ID: %lu", std::this_thread::get_id());

    auto start_time = std::chrono::high_resolution_clock::now();

    {
         std::lock_guard<std::mutex> lock(mutex_);

        for (size_t i = 0; i < joint_handles_.size(); ++i) 

        {
            desired_velocities_[i] = CalculateDesiredVel(joint_handles_[i], desired_positions_[i], i);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    ROS_INFO("outerLoop duration: %f seconds", elapsed.count());

}

void mycustomcontroller::mycontroller::innerLoop(const ros::TimerEvent&)
{
    ROS_INFO("Entering InnerLoop on thread ID: %lu", std::this_thread::get_id());

    //ROS_INFO("Entering innerLoop repeatedly");

    auto start_time = std::chrono::high_resolution_clock::now();

    std::lock_guard<std::mutex> lock(mutex_);
    {
       
        for (size_t i = 0; i < joint_handles_.size(); ++i) 
        {
            ROS_INFO("Calculating effort command");
            double command_effort = calculateEffortCommand(joint_handles_[i], desired_velocities_[i], i);
            joint_handles_[i].setCommand(command_effort);
             ROS_INFO("Finished Calculating effort command");
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    ROS_INFO("innerLoop duration: %f seconds", elapsed.count());
}

void mycustomcontroller::mycontroller::update(const ros::Time& time, const ros::Duration& period)
{
    // This update function can remain empty or be used to synchronize other tasks
}
 

double mycustomcontroller::mycontroller::CalculateDesiredVel(const hardware_interface::JointHandle& joint, double desired_position, int joint_index)
{
    ROS_INFO("starting to calculate desired velocity right before locking, happening in OuterLoop");

    //std::lock_guard<std::mutex> lock(mutex_);

    double current_position = joint.getPosition();
    double position_error = desired_position - current_position;

    double p_term_pos = kp_pos_ * position_error; 
    integral_terms_pos_[joint_index] += ki_pos_ * position_error;
    double d_term_pos = kd_pos_ * (position_error - prev_errors_pos_[joint_index]);

    double desired_velocity = p_term_pos + integral_terms_pos_[joint_index] + d_term_pos;

    return desired_velocity;

    ROS_INFO("Exiting calculate desiredvel function, velocity equal: %f", desired_velocity);
}

double mycustomcontroller::mycontroller::calculateEffortCommand(const hardware_interface::JointHandle& joint, double desired_velocity, int joint_index)
{
    ROS_INFO("Entering calculate_effort function, in innerLoop");

    //std::lock_guard<std::mutex> lock(mutex_);

    double current_velocity = joint.getVelocity();
    double velocity_error = desired_velocity - current_velocity;

    double p_term_vel = kp_Vel_ * velocity_error;  
    integral_terms_vel_[joint_index] += ki_Vel_ * velocity_error;
    double d_term_vel = kd_Vel_ * (velocity_error - prev_errors_vel_[joint_index]);

    double effort_command = p_term_vel + integral_terms_vel_[joint_index] + d_term_vel;
    prev_errors_vel_[joint_index] = velocity_error;

    return effort_command;

    
}

void mycustomcontroller::mycontroller::setpointCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) 
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t i = 0; i < desired_positions_.size(); ++i) 
    {
        desired_positions_[i] = msg->data[i];
    } 
}

PLUGINLIB_EXPORT_CLASS(mycustomcontroller::mycontroller, controller_interface::ControllerBase)
