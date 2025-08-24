#include "ros/ros.h"
#include "std_msgs/Float64.h"

double current_temperature = 0.0;
double desired_temperature = 25.0; // Target temperature
double kp = 0.5; // Proportional gain

void temperatureCallback(const std_msgs::Float64::ConstPtr& msg) {
    current_temperature = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "temperature_controller");
    ros::NodeHandle nh;

    ros::Subscriber temp_sub = nh.subscribe("current_temperature", 10, temperatureCallback);
    ros::Publisher control_pub = nh.advertise<std_msgs::Float64>("heater_control", 10);

    ros::Rate loop_rate(10); // 10 Hz update rate for controller

    while (ros::ok()) {
        double error = desired_temperature - current_temperature;
        double control_output = kp * error;

        // Clamp control output to reasonable limits (e.g., 0 to 100 for heater power)
        if (control_output < 0.0) control_output = 0.0;
        if (control_output > 100.0) control_output = 100.0;

        std_msgs::Float64 msg;
        msg.data = control_output;
        control_pub.publish(msg);

        ROS_INFO("Control Output: %.2f (Error: %.2f)", control_output, error);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}