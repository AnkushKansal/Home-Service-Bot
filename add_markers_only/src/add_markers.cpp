#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_only");
  ros::NodeHandle n;
  ros::Rate r(20);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;

  enum State { PICKUP, HIDE, DROP, } state = PICKUP;
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = -.9;
    marker.pose.position.y = 3.53;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = .50;
    marker.scale.y = .50;
    marker.scale.z = .50;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    if (state == PICKUP) {
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
      ROS_INFO("Picking up ... ");
      sleep(5);
      state = HIDE;
    } 
    else if (state == HIDE) {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      ROS_INFO("Item Picked Up..");
      sleep(5);
      state = DROP;
    } 
    else {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = -.0955;
      marker.pose.position.y = 10.8;
      marker_pub.publish(marker);
      ROS_INFO("Dropped Item");
      sleep(5);
    }
  }
  ros::spinOnce();
}