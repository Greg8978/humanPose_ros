#include <ostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

#include "posterLib.h"
#include "spark/sparkStruct.h"

#define humanPose_DEBUG 1

using namespace std;

static int getHumanLocation(tf::TransformListener& listener);
static void humanJointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);

//The posters
/*
POSTER_ID posterHumanPose;
POSTER_ID posterHumanJointState;
*/
POSTER_ID posterConfiguration;

SPARK_CONFIGURATION humanConf;

int main(int argc, char** argv){
  //ori_pose humanPose;
/*  if (posterCreate("humanPose", sizeof(ori_pose), &posterHumanPose) != OK) {

    ROS_ERROR("Error: could not create Human Pose poster\n");
    return 1;
  }
  if (posterCreate("humanJointState", sizeof(ori_jointstate), &posterHumanJointState) != OK) {
    ROS_ERROR("Error: could not create Human JointState poster\n");
    return 1;
  }
*/
  humanConf.changed = 1;

  if (posterCreate("human.armature", sizeof(SPARK_CONFIGURATION), &posterConfiguration) != OK) {
    ROS_ERROR("Error: could not create Human JointState poster\n");
    return 1;
  }
  ros::init(argc, argv, "humanPose");
  ros::NodeHandle node;
  tf::TransformListener listener;

  // **************************************************************************
  // Starts listening to the /human/armature/joint_states topic and publish accordingly joints
  // on the humanJointState poster.
  ros::Subscriber sub = node.subscribe("/human/armature/joint_states", 1, humanJointStateCallBack);

  ros::Rate rate(50); //50Hz
  
  while (node.ok()){
    getHumanLocation(listener);
    posterWrite(posterConfiguration, 0, &humanConf, sizeof(humanConf));

    if(humanPose_DEBUG){
      for( int i = 0; i < humanConf.dofNb; i++){
        std::cout << "Poster configuration: dof" << i << " -> " << humanConf.dof[i] << std::endl;
      }
    }


    ros::spinOnce();
    rate.sleep();
  }
  /*
  if (posterDelete(posterPose) != OK) {
    ROS_ERROR("Error: could not delete poster\n");
  }
  if (posterDelete(posterHumanJointState) != OK) {
    ROS_ERROR("Error: could not delete poster\n");
  }
  */

  return 0;
};

static int getHumanLocation(tf::TransformListener& listener){
    tf::StampedTransform transform;
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/map", "/base_link",
                              now, ros::Duration(3.0));
        listener.lookupTransform("/map", "/base_link",
                               now, transform);
        
        humanConf.dof[0] = transform.getOrigin().x();
        humanConf.dof[1] = transform.getOrigin().y();
        humanConf.dof[2] = transform.getOrigin().z()+1.0;
        humanConf.dof[3] = 0.0;
        humanConf.dof[4] = 0.0;
        humanConf.dof[5] = tf::getYaw(transform.getRotation());
        

        if(humanPose_DEBUG){
          printf("%f %f %f %f %f %f\n", transform.getOrigin().x(), transform.getOrigin().y(), 
                transform.getOrigin().z(), humanConf.dof[3], humanConf.dof[4], humanConf.dof[5]);
        }
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
  return TRUE;
}


static void humanJointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg){

  humanConf.dofNb = 50;
  if(msg->position.size() > humanConf.dofNb){
    ROS_ERROR("Nb Dof error: Poster max nbDof %d < Ros nbDof %d", humanConf.dofNb, msg->position.size());
    return;
  }
  humanConf.dof[14] = msg->position[13];        //TorsoX
  humanConf.dof[15] = - msg->position[20];      //Head
  humanConf.dof[18] = 1.39626;                  //Shoulder Right
  humanConf.dof[19] = 0.10472;                  //Shoulder Right
  humanConf.dof[20] = -0.346273;                //Shoulder Right
  humanConf.dof[22] = msg->position[18];        //Right elbow
  humanConf.dof[24] = msg->position[19];        //Right wrist
  humanConf.dof[27] = -1.39626;                 //Left shoulder
  humanConf.dof[28] = - msg->position[14];      //Left shoulder
  humanConf.dof[29] = - 0.174533;               //Left shoulder
  humanConf.dof[31] = msg->position[15];        //Left elbow
  humanConf.dof[33] = msg->position[16];        //Left wrist     
  humanConf.dof[37] = msg->position[24];        //Right hip
  humanConf.dof[39] = msg->position[25];        //Right knee
  humanConf.dof[40] = msg->position[22];        //Right ankle
  humanConf.dof[44] = msg->position[10];        //Left hip
  humanConf.dof[46] = msg->position[11];        //Left knee
  humanConf.dof[47] = msg->position[5];         //Left ankle

  
  
  for(unsigned int i = 0; i < msg->position.size(); i++){
    if(humanPose_DEBUG){
      std::cout << msg->name[i] << "   " << msg->position[i] << std::endl;
    }
  }
  if(humanPose_DEBUG){
    std::cout << std::endl;
  }
}
