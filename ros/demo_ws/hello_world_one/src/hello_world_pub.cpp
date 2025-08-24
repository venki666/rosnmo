#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "hello_world_publisher");

  // Create a NodeHandle object
  ros::NodeHandle nh;

  // Create a publisher to the "chatter" topic
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64>("Fnumber", 1000);

  // Set the loop rate to 20 Hz
  ros::Rate loop_rate(20);
  std_msgs::Float64 numb ;
  numb.data = 1918.818;
  while (ros::ok()) // Loop while ROS is running
  {

    //std_msgs::String msg;
    //std::stringstream ss;
   // ss << "Hello World " << count;
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str()); // Log the message

    chatter_pub.publish(numb); // Publish the message

    ros::spinOnce(); // Process callbacks (e.g., for subscribers if this were a combined node)

    loop_rate.sleep(); // Sleep to maintain the desired loop rate
    numb.data  += 0.012;
  }

  return 0;
}