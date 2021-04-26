#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  ROS_INFO("Started to picking Up Objects");
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach

  float goals[2][3] = { {-.9, 3.53,.37}, {-.0955, 10.8,-.57}};

  int num_points = 2;

  for (int i=0; i < num_points; i++){

      goal.target_pose.pose.position.x = goals[i][0];
      goal.target_pose.pose.position.y = goals[i][1]; 
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goals[i][2]); //added this as by default quaternion is zero , bot wont move
      
       // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      // Wait an infinite time for the results
      ac.waitForResult();

      // Check if the robot reached its goal
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        if(i==0)
        	ROS_INFO("Reached to pickUp zone");
        else if(i==1)
          ROS_INFO("Reached to DropOff zone");
      }
      else
        ROS_INFO("Failed to reach goal for some reason");
      ros::Duration(5.0).sleep();
  }

  return 0;
}

