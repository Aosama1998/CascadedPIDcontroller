#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "setpoint_publisher_node");
    ros::NodeHandle nh;

    // Publisher for sending setpoints
    ros::Publisher setpoint_pub = nh.advertise<std_msgs::Float64MultiArray>("my_effort_controller/Desired_Pos", 10);

    // Define the setpoint values
    std_msgs::Float64MultiArray setpoints;
    setpoints.data.resize(20); // Adjust the size as per your requirement

    // Sample setpoint values (replace with your logic to generate setpoints)
    for (size_t i = 0; i < setpoints.data.size(); ++i) {
        setpoints.data[i] = 3 * i; // Example: Set some sequential setpoint values
    }

    // Publish setpoints at a fixed rate
    ros::Rate loop_rate(10); // 10 Hz publishing rate
    while (ros::ok()) {
        setpoint_pub.publish(setpoints);

        for (int i =0; i<100000; i++){

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
