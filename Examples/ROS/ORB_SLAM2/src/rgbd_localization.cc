/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <unistd.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <boost/bind.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2_ros/transform_listener.h>

// Mavros
#include <mavros_msgs/State.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/timer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"

extern "C" {
#include "swiftnav/coord_system.h"
}

#define TO_RADIANS (M_PI/180.0)
#define TO_DEGREES (180.0/M_PI)

using namespace std;

tf2_ros::Buffer tfBuffer;

tf2::Transform tf_rs_opt_drone;

ros::Publisher pose_publisher; 

bool pose_initiallized    = false;
bool tracker_initiallized = false;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD); // const sensor_msgs::ImuConstPtr& msgIMU

    ORB_SLAM2::System* mpSLAM;
};

cv::Mat initPose;

void state_callback(const mavros_msgs::State::ConstPtr& msg){

	bool is_armed = msg->armed;

	if(is_armed && !pose_initiallized){

		geometry_msgs::TransformStamped transform_stamped_rs, transform_stamped_rs_drone;

		try{

			// Wait 2 secs for the Drone to settle	
			usleep(2000000);

			transform_stamped_rs       = tfBuffer.lookupTransform("rs_camera_optical", "map",   ros::Time(0));
			transform_stamped_rs_drone = tfBuffer.lookupTransform("rs_camera_optical", "drone", ros::Time(0));

			//tf2::Matrix3x3 rotMat(tf2::Quaternion(msgIMU->orientation.x, msgIMU->orientation.y, msgIMU->orientation.z, msgIMU->orientation.w));
			tf2::Matrix3x3 rotMat(tf2::Quaternion(transform_stamped_rs.transform.rotation.x,
			                                    transform_stamped_rs.transform.rotation.y, 
			                                    transform_stamped_rs.transform.rotation.z,
			                                    transform_stamped_rs.transform.rotation.w));  

			initPose.at<float>(0, 0) = rotMat[0][0]; initPose.at<float>(0, 1) = rotMat[0][1]; initPose.at<float>(0, 2) = rotMat[0][2];
			initPose.at<float>(1, 0) = rotMat[1][0]; initPose.at<float>(1, 1) = rotMat[1][1]; initPose.at<float>(1, 2) = rotMat[1][2];
			initPose.at<float>(2, 0) = rotMat[2][0]; initPose.at<float>(2, 1) = rotMat[2][1]; initPose.at<float>(2, 2) = rotMat[2][2];

			initPose.at<float>(0, 3) = transform_stamped_rs.transform.translation.x;
			initPose.at<float>(1, 3) = transform_stamped_rs.transform.translation.y;
			initPose.at<float>(2, 3) = transform_stamped_rs.transform.translation.z;// - transform_stamped_drone.transform.translation.z;

			ROS_INFO("X: %f, Y: %f, Z: %f", initPose.at<float>(0, 3), initPose.at<float>(1, 3), initPose.at<float>(2, 3));

			tf_rs_opt_drone.setOrigin(tf2::Vector3(transform_stamped_rs_drone.transform.translation.x, 
			                 					 transform_stamped_rs_drone.transform.translation.y, 
			                 					 transform_stamped_rs_drone.transform.translation.z));

			tf_rs_opt_drone.setRotation(tf2::Quaternion(transform_stamped_rs_drone.transform.rotation.x,
			                                   transform_stamped_rs_drone.transform.rotation.y, 
			                                   transform_stamped_rs_drone.transform.rotation.z,
			                                   transform_stamped_rs_drone.transform.rotation.w));      

			pose_initiallized = true;

		} catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
		}
	}

	return ;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_NAV");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, false);

    // Assign SLAM system to object
    ImageGrabber igb(&SLAM);

    initPose = cv::Mat::eye(4,4,CV_32F);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol; // , sensor_msgs::Imu

    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub); // imu_sub
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2)); //,_3

    tf2_ros::TransformListener tfListener(tfBuffer);

    //ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback, false);

    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 1, state_callback);

    //message_filters::Subscriber<nav_msgs::Odometry>  odom_sub(nh, "/mavros/local_position/odom", 1);
    //odom_sub.registerCallback(odomCallback);

    // ====== Publishers ======
    pose_publisher  = nh.advertise<geometry_msgs::PoseStamped>("/rgbd_localization/pose", 1);     

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Shutdown Node
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    if(!pose_initiallized) return; 

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat track;
    if(!tracker_initiallized){
      mpSLAM->Reset();
      track = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image, initPose, cv_ptrRGB->header.stamp.toSec());
      tracker_initiallized = true;
    }else{
      track = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    }
    
    //ROS_INFO("Size: %d, %d", track.rows, track.cols);

    if (track.rows == 4 && track.cols == 4) {

        // Broadcast transform
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        // construct a pose message
        geometry_msgs::PoseStamped pose_stamped;

        cv::Mat rot   = track.rowRange(0,3).colRange(0,3).t(); // Rotarion
        cv::Mat trans = -rot*track.rowRange(0,3).col(3);       // Translation
        vector<float> quat = ORB_SLAM2::Converter::toQuaternion(rot);

        tf2::Transform tf_tracked;

        tf_tracked.setOrigin(tf2::Vector3(trans.at<float>(0, 0), trans.at<float>(0, 1), trans.at<float>(0, 2)));
        tf_tracked.setRotation(tf2::Quaternion(quat[0], quat[1], quat[2], quat[3]));
        
        tf_tracked = tf_tracked*tf_rs_opt_drone;

        ros::Time time_now = ros::Time::now();
        transformStamped.header.stamp = time_now;
        transformStamped.header.frame_id = "map";

        transformStamped.child_frame_id  = "drone_visual_odom";

        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = time_now;

        //ROS_INFO("X: %f, Y: %f, Z: %f",trans.at<float>(0, 0), trans.at<float>(0, 1), trans.at<float>(0, 2) );

        tf2::Vector3    estimated_trans = tf_tracked.getOrigin();
        tf2::Quaternion estimated_rot   = tf_tracked.getRotation();
        
        estimated_rot.normalize();

        transformStamped.transform.translation.x = estimated_trans[0];
        transformStamped.transform.translation.y = estimated_trans[1];
        transformStamped.transform.translation.z = estimated_trans[2];

        transformStamped.transform.rotation.x 	 = estimated_rot.x();
        transformStamped.transform.rotation.y 	 = estimated_rot.y();
        transformStamped.transform.rotation.z 	 = estimated_rot.z();
        transformStamped.transform.rotation.w 	 = estimated_rot.w();

        pose_stamped.pose.position.x 	= estimated_trans[0];
        pose_stamped.pose.position.y 	= estimated_trans[1];
        pose_stamped.pose.position.z 	= estimated_trans[2];

        pose_stamped.pose.orientation.x = estimated_rot.x();
        pose_stamped.pose.orientation.y = estimated_rot.y();
        pose_stamped.pose.orientation.z = estimated_rot.z();
        pose_stamped.pose.orientation.w = estimated_rot.w();

        br.sendTransform(transformStamped); 
        pose_publisher.publish(pose_stamped);
    }    
}
