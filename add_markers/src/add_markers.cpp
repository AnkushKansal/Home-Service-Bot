#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


double goals[2][3] = { {-.9, 3.53}, {-.0955, 10.8}};

bool itemPickedUp = false;
bool itemDroppedOff = false;
bool FirstReach = false;
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    
  double robotX = msg->pose.pose.position.x;
  double robotY = msg->pose.pose.position.y;
  // ROS_INFO("Received nav_msgs");

  double distanceToPickup = sqrt(pow(robotX - goals[0][0], 2) + pow(robotY - goals[0][1], 2));
  double distanceToDropoff = sqrt(pow(robotX - goals[1][0], 2) + pow(robotY - goals[1][1], 2));  

  if (distanceToPickup < 0.3) {
    itemPickedUp = true;
    if(!FirstReach)
    	ROS_INFO("Reach the pickup zone!");
  }
  
  if (distanceToDropoff < 0.3) {
    itemDroppedOff = true;
    ROS_INFO("Reach the dropoff zone!");
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber amcl_sub = n.subscribe("amcl_pose", 10, amclCallback);

  visualization_msgs::Marker marker;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "cube";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = goals[0][0];
  marker.pose.position.y = goals[0][1];
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = .5;
  marker.scale.y = .5;
  marker.scale.z = .5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  while (ros::ok())
  {    
    if (!itemPickedUp)
    {
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
    }
    else if (!itemDroppedOff)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      if (!FirstReach){
        FirstReach = true;
        ROS_INFO("Wait here for picking up!");
        ros::Duration(5.0).sleep();
        ROS_INFO("Item Picked up");
      }
    }
    else
    {
      marker.pose.position.x = goals[1][0];;
      marker.pose.position.y = goals[1][1];;
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
    }
    
    ros::spinOnce();
  }
}
