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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::ImuConstPtr& msgIMU);

    ORB_SLAM2::System* mpSLAM;
};

void orientCallback(const geometry_msgs::QuaternionConstPtr& orient)
{
    ROS_INFO("X: %f", orient->x);

    return;
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
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh,    "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh,  "/camera/depth_registered/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Imu>   imu_sub(nh,    "/sense_gimbal/fake_imu/data", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> sync_pol;

    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub, imu_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2,_3));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::ImuConstPtr& msgIMU)
{
    
    cv::Mat initPose = cv::Mat::eye(4,4,CV_32F);

    tf2::Matrix3x3 rotMat(tf2::Quaternion(msgIMU->orientation.x, msgIMU->orientation.y, msgIMU->orientation.z, msgIMU->orientation.w));  

    initPose.at<float>(0, 0) = rotMat[0][0]; initPose.at<float>(0, 1) = rotMat[0][1]; initPose.at<float>(0, 2) = rotMat[0][2];
    initPose.at<float>(1, 0) = rotMat[1][0]; initPose.at<float>(1, 1) = rotMat[1][1]; initPose.at<float>(1, 2) = rotMat[1][2];
    initPose.at<float>(2, 0) = rotMat[2][0]; initPose.at<float>(2, 1) = rotMat[2][1]; initPose.at<float>(2, 2) = rotMat[2][2];

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

    if (track.rows < 4 || track.cols < 4) return;

    // Broadcast transform
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    cv::Mat rot   = track.rowRange(0,3).colRange(0,3).t(); // Rotarion
    cv::Mat trans = -rot*track.rowRange(0,3).col(3);       // Translation
    vector<float> quat = ORB_SLAM2::Converter::toQuaternion(rot);
    
    /*
    tf2::Quaternion quat(q[0], q[1], q[2], q[3]);

    quat = quat*quat_correction;
    
    */
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id  = "realsense_optical";

    ROS_INFO("X: %f, Y: %f, Z: %f",trans.at<float>(0, 0), trans.at<float>(0, 1), trans.at<float>(0, 2) );

    transformStamped.transform.translation.x = trans.at<float>(0, 0); //trans.at<float>(0, 2);
    transformStamped.transform.translation.y = trans.at<float>(0, 1); //-trans.at<float>(0, 0);
    transformStamped.transform.translation.z = trans.at<float>(0, 2); //-trans.at<float>(0, 1);

    transformStamped.transform.rotation.x = quat[0];
    transformStamped.transform.rotation.y = quat[1];
    transformStamped.transform.rotation.z = quat[2];
    transformStamped.transform.rotation.w = quat[3];

    br.sendTransform(transformStamped); 
}


