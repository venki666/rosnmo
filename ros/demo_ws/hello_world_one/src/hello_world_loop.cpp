#include <ros/ros.h>

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "hello_world_loop");

  // Create a NodeHandle object
  ros::NodeHandle nh;

  // Set the loop rate to 10 Hz
  ros::Rate loop_rate(10);

  while (ros::ok()) // Loop while ROS is running
  {

    ROS_INFO("hello_world!"); // Log the message

    ros::spinOnce(); // Process callbacks (e.g., for subscribers if this were a combined node)

    loop_rate.sleep(); // Sleep to maintain the desired loop rate
  }

  return 0;
}