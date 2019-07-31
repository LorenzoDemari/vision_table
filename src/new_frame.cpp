#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "new_frame");

  ros::NodeHandle node;

  tf::Vector3 xk(1,0,0);
  tf::Vector3 yk(0,1,0);
  tf::Vector3 zk(0,0,1);

  tf::TransformBroadcaster br;
  tf::TransformListener listener;

  ros::Rate rate(1.0);
  while (node.ok()){
    tf::StampedTransform transformOw;
    try{
      listener.lookupTransform("/kinect2_rgb_optical_frame", "/ar_marker_42", ros::Time(0), transformOw);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf::StampedTransform transformXw;
    try{
      listener.lookupTransform("/kinect2_rgb_optical_frame", "/ar_marker_43", ros::Time(0), transformXw);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf::StampedTransform transformYw;
    try{
      listener.lookupTransform("/kinect2_rgb_optical_frame", "/ar_marker_44", ros::Time(0), transformYw);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    tf::Vector3 Xw(transformXw.getOrigin().x()-transformOw.getOrigin().x(),transformXw.getOrigin().y()-transformOw.getOrigin().y(),transformXw.getOrigin().z()-transformOw.getOrigin().z());
    tf::Vector3 Yw(transformYw.getOrigin().x()-transformOw.getOrigin().x(),transformYw.getOrigin().y()-transformOw.getOrigin().y(),transformYw.getOrigin().z()-transformOw.getOrigin().z());

    

    tf::Transform world;
    world.setOrigin(tf::Vector3(transformOw.getOrigin().x(),transformOw.getOrigin().y(),transformOw.getOrigin().z()));
    world.setRotation(tf::Quaternion(tf::tfAngle (yk, Yw),tf::tfAngle (xk, Xw), 0, 1));

    br.sendTransform(tf::StampedTransform(world, ros::Time::now(), "/kinect2_rgb_optical_frame", "WORLD"));


    rate.sleep();
  }
  return 0;
};
