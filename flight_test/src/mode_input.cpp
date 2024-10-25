//@Copyright : Dongbin Kim.
//Email : dongbin.kim@unlv.edu
//Date : 2020-05-21
//Title : Unity Input test to ROS for drone in Simulation In the Loop (SITL)
// Arming, Disarming, Takeoff, Land.

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "unity_to_ros");
  ros::NodeHandle n;
  ros::Publisher mode_pub = n.advertise<geometry_msgs::Point>("/HM24/SetMode", 1000);

  ros::Rate loop_rate(50);
  
  double input;
  while (ros::ok())
  {
    geometry_msgs::Point msg;      
    printf("Mode Input \n");
    scanf("%lf",&input);
    if(input == 0){
        break;
    }
    msg.x = input;


    mode_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();    
  }


  return 0;
}