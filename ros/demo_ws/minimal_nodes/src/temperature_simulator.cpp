#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "temperature_simulator");
    ros::NodeHandle nh;

    ros::Publisher temp_pub = nh.advertise<std_msgs::Float64>("current_temperature", 10);

    double current_temp = 20.0; // Initial temperature
    double heater_effect = 0.0; // Effect of heater from controller
    double cooling_effect = 0.0; // Effect of cooling from environment
    double desired_temp = 25.0; // Example desired temperature for environment interaction

    ros::Rate loop_rate(1); // 1 Hz update rate

    while (ros::ok()) {
        // Simulate environmental cooling
        cooling_effect = (current_temp - desired_temp) * 0.05; 
        
        // Simulate heater effect (received from controller)
        // In a real system, this would come from a subscriber
        // For this simple simulator, we'll assume a fixed effect for now
        heater_effect = 0.0; // Placeholder: to be updated by a subscriber

        current_temp += heater_effect - cooling_effect;

        std_msgs::Float64 msg;
        msg.data = current_temp;
        temp_pub.publish(msg);

        ROS_INFO("Simulated Temperature: %.2f", current_temp);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}