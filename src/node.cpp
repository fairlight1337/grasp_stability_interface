// System
#include <iostream>
#include <cstdlib>

// ROS
#include <ros/ros.h>
#include <grasp_stability_msgs/GraspStability.h>
#include <grasp_stability_msgs/Control.h>

using namespace std;


ros::Publisher pubState;


bool controlCallback(grasp_stability_msgs::Control::Request &req, grasp_stability_msgs::Control::Response &res) {
  if(req.command == grasp_stability_msgs::Control::Request::CTRL_START) {
    cout << "Control command: Start" << endl;
  } else if(req.command == grasp_stability_msgs::Control::Request::CTRL_STOP) {
    cout << "Control command: Stop" << endl;
  }
  
  res.result = grasp_stability_msgs::Control::Response::SUCCESS;
  
  return true;
}


void publishState() {
  grasp_stability_msgs::GraspStability gsMsg;
  gsMsg.measurement_context_id = "left_gripper";
  gsMsg.grasp_quality = 0.75;
  gsMsg.estimation_confidence = 0.93;
  gsMsg.grasp_category = grasp_stability_msgs::GraspStability::GRASP_CAT_MEDIUM;
  
  pubState.publish(gsMsg);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "grasp_stability_estimator");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");
  
  if(ros::ok()) {
    ros::ServiceServer srvControl = nhPrivate.advertiseService("control", controlCallback);
    pubState = nhPrivate.advertise<grasp_stability_msgs::GraspStability>("/grasp_stability_estimator/state", 1);
    
    ros::Rate rtRate(30); // 30 Hz
    
    while(ros::ok()) {
      ros::spinOnce();
      publishState();
      
      rtRate.sleep();
    }
  }
  
  return EXIT_SUCCESS;
}
