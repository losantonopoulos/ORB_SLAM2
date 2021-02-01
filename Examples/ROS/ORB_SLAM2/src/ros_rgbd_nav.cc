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
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

#include <tf2_ros/transform_listener.h>

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

ros::Publisher mocap_publisher;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD); // const sensor_msgs::ImuConstPtr& msgIMU

    ORB_SLAM2::System* mpSLAM;
};

static void fix_to_ecef(const sensor_msgs::NavSatFix& fix, double ecef[3]) {
  double llh[3] = { fix.latitude * TO_RADIANS,
                    fix.longitude * TO_RADIANS,
                    fix.altitude };
  wgsllh2ecef(llh, ecef);
}

void fix_to_point(const sensor_msgs::NavSatFix& fix,
                  const sensor_msgs::NavSatFix& datum,
                  geometry_msgs::Point* point_ptr) {
  // Convert reference LLH-formatted datum to ECEF format
  double ecef_datum[3];
  fix_to_ecef(datum, ecef_datum);

  // Prepare the appropriate input vector to convert the input latlon
  // to an ECEF triplet.
  double llh[3] = { fix.latitude * TO_RADIANS,
                    fix.longitude * TO_RADIANS,
                    fix.altitude };
  double ecef[3];
  wgsllh2ecef(llh, ecef);

  // ECEF triplet is converted to north-east-down (NED), by combining it
  // with the ECEF-formatted datum point.
  double ned[3];
  wgsecef2ned_d(ecef, ecef_datum, ned);

  // Output data
  point_ptr->x = ned[1];
  point_ptr->y = ned[0];
  point_ptr->z = -ned[2];
}

void point_to_fix(const geometry_msgs::Point& point,
                  const sensor_msgs::NavSatFix& datum,
                  sensor_msgs::NavSatFix* fix_ptr) {
  // Convert reference LLH-formatted datum to ECEF format
  double ecef_datum[3];
  fix_to_ecef(datum, ecef_datum);

  // Prepare NED vector from ENU coordinates, perform conversion in libswiftnav
  // library calls.
  double ned[3] = { point.y, point.x, -point.z };

  double ecef[3];
  wgsned2ecef_d(ned, ecef_datum, ecef);

  double llh_raw[3];
  wgsecef2llh(ecef, llh_raw);

  // Output Fix message. Convert radian latlon output back to degrees.
  fix_ptr->latitude = llh_raw[0] * TO_DEGREES;
  fix_ptr->longitude = llh_raw[1] * TO_DEGREES;
  fix_ptr->altitude = llh_raw[2];
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    sensor_msgs::NavSatFix origin, estimated;

    geometry_msgs::Point curPoint;

    curPoint = msg->pose.pose.position;

    origin.latitude  = -35.3632621;
    origin.longitude = 149.1652374;
    origin.altitude  = 603.0538637324;

    point_to_fix(curPoint, origin, &estimated);

    ROS_INFO("LLH %f, %f, %f", estimated.latitude, estimated.longitude, estimated.altitude );

    return;
}

void gnssCallback(const sensor_msgs::NavSatFixConstPtr& fix_ptr)
{
    sensor_msgs::NavSatFix origin;

    geometry_msgs::Point curPoint;

    origin.latitude  = -35.3632621;
    origin.longitude = 149.1652374;
    origin.altitude  = 603.538637324;

    fix_to_point(*fix_ptr, origin, &curPoint);

    ROS_INFO("ENU %f, %f, %f", curPoint.x, curPoint.y, curPoint.z);
    return;
}

void timerCallback(const ros::TimerEvent&)
{

    ROS_INFO("Timer1");

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("map", "rs", ros::Time(0));

        geometry_msgs::PoseStamped poseStamped;

        poseStamped.header.frame_id = "fcu";
        poseStamped.header.stamp    = ros::Time::now();

        poseStamped.pose.position.x = transformStamped.transform.translation.x;
        poseStamped.pose.position.y = transformStamped.transform.translation.y;
        poseStamped.pose.position.z = transformStamped.transform.translation.z;

        poseStamped.pose.orientation.x = transformStamped.transform.rotation.x;
        poseStamped.pose.orientation.y = transformStamped.transform.rotation.y;
        poseStamped.pose.orientation.z = transformStamped.transform.rotation.z;
        poseStamped.pose.orientation.w = transformStamped.transform.rotation.w;

        mocap_publisher.publish(poseStamped);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }

    ROS_INFO("Timer2");

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

    ros::NodeHandle nh;



    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    //message_filters::Subscriber<sensor_msgs::Imu>   imu_sub(nh, "/sense_gimbal/fake_imu/data", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol; // , sensor_msgs::Imu

    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub); // imu_sub
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2)); //,_3

    mocap_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 1000);

    message_filters::Subscriber<sensor_msgs::NavSatFix>  gps_sub(nh, "/mavros/global_position/global", 1); //"/mavros/global_position/raw/fix"
    gps_sub.registerCallback(gnssCallback);

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback, false);

    //message_filters::Subscriber<nav_msgs::Odometry>  odom_sub(nh, "/mavros/local_position/odom", 1);
    //odom_sub.registerCallback(odomCallback);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Shutdown Node
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)//, const sensor_msgs::ImuConstPtr& msgIMU)
{
    
    cv::Mat initPose = cv::Mat::eye(4,4,CV_32F);

    //tf2::Matrix3x3 rotMat(tf2::Quaternion(msgIMU->orientation.x, msgIMU->orientation.y, msgIMU->orientation.z, msgIMU->orientation.w));
    tf2::Matrix3x3 rotMat(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));  

    initPose.at<float>(0, 0) = rotMat[0][0]; initPose.at<float>(0, 1) = rotMat[0][1]; initPose.at<float>(0, 2) = rotMat[0][2];
    initPose.at<float>(1, 0) = rotMat[1][0]; initPose.at<float>(1, 1) = rotMat[1][1]; initPose.at<float>(1, 2) = rotMat[1][2];
    initPose.at<float>(2, 0) = rotMat[2][0]; initPose.at<float>(2, 1) = rotMat[2][1]; initPose.at<float>(2, 2) = rotMat[2][2];

    initPose.at<float>(0, 3) = 0;
    initPose.at<float>(1, 3) = 1.07;
    initPose.at<float>(2, 3) = 0;

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

    cv::Mat track = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image, initPose, cv_ptrRGB->header.stamp.toSec());

    //ROS_INFO("Size: %d, %d", track.rows, track.cols);

    if (track.rows == 4 && track.cols == 4) {

        // Broadcast transform
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        cv::Mat rot   = track.rowRange(0,3).colRange(0,3).t(); // Rotarion
        cv::Mat trans = -rot*track.rowRange(0,3).col(3);       // Translation
        vector<float> quat = ORB_SLAM2::Converter::toQuaternion(rot);

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id  = "realsense_optical_1";

        //ROS_INFO("X: %f, Y: %f, Z: %f",trans.at<float>(0, 0), trans.at<float>(0, 1), trans.at<float>(0, 2) );

        transformStamped.transform.translation.x = trans.at<float>(0, 0); //trans.at<float>(0, 2);
        transformStamped.transform.translation.y = trans.at<float>(0, 1); //-trans.at<float>(0, 0);
        transformStamped.transform.translation.z = trans.at<float>(0, 2); //-trans.at<float>(0, 1);

        transformStamped.transform.rotation.x = quat[0];
        transformStamped.transform.rotation.y = quat[1];
        transformStamped.transform.rotation.z = quat[2];
        transformStamped.transform.rotation.w = quat[3];

        br.sendTransform(transformStamped); 
    }

    
}


