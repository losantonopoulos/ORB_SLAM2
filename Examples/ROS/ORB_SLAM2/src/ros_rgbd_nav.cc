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
#include <std_msgs/Float64.h>

#include <cv_bridge/cv_bridge.h>

#include <boost/bind.hpp>

#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <nav_msgs/Odometry.h>

#include <ros/timer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"
#include"../../../include/Nav_config.h"

using namespace std;

// Transform related

tf2_ros::Buffer tfBuffer;
tf2::Transform d435_optical_drone_tf;

// Pose related

ros::Publisher pub_pose; 

// Twist related

// Odometry related

// Setup realted

class PoseEstimation
{
public:
    ros::Time latest_vslam_time;
    ros::Time latest_imu_time;

    bool pose_vslam_updated   = false;
    bool pose_initialized     = false;
    bool velocity_initialized = false;
    bool imu_initialized      = false;

    tf2::Vector3 vslam_velocity_linear;
    tf2::Vector3 vslam_velocity_angular;

    tf2::Vector3 velocity_linear;
    tf2::Vector3 velocity_angular;

    tf2::Transform latest_vslam_tf;
    tf2::Transform latest_tf;

} pose_estimation;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

void imu_callback(const sensor_msgs::ImuConstPtr& msg){

    if(pose_estimation.pose_initialized && pose_estimation.velocity_initialized){

        ros::Time cur_time = msg->header.stamp;

        tf2::Transform imu_initial, imu_final;

        geometry_msgs::Vector3 acc      = msg->linear_acceleration;
        geometry_msgs::Vector3 rot_vel  = msg->angular_velocity;
        geometry_msgs::Quaternion acc_quat = msg->orientation;

        tf2::Quaternion temp_quat;

        tf2::convert(acc_quat, temp_quat);

        // Convert to acctual accelaration data

        tf2::Vector3 acc_linear = tf2::Vector3(acc.x, acc.y, acc.z);

        tf2::Vector3 vel_angular = tf2::Vector3(rot_vel.x, rot_vel.y, rot_vel.z);

        imu_initial.setOrigin(acc_linear);
        imu_initial.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

        imu_final.setOrigin(tf2::Vector3(0.0, 0.0, 9.63));
        imu_final.setRotation(temp_quat);

        imu_final = imu_final.inverse();

        imu_final = imu_initial * imu_final;

        acc_linear = imu_final.getOrigin();

        //ROS_WARN("X: %f, Y: %f, Z: %f", vel_angular[0], vel_angular[1], vel_angular[2]);

        // Intialiaze Inter Translation and Rotation

        tf2::Vector3 inter_translation  = tf2::Vector3(0.0, 0.0, 0.0); 
        tf2::Vector3 inter_rotation     = tf2::Vector3(0.0, 0.0, 0.0); 

        tf2::Vector3 temp_vel_linear  = tf2::Vector3(0.0, 0.0, 0.0); 
        tf2::Vector3 temp_vel_angular = vel_angular; //tf2::Vector3(0.0, 0.0, 0.0); 

        if(pose_estimation.imu_initialized){

            ros::Duration duration_imu = cur_time - pose_estimation.latest_imu_time;
            double duration_imu_sec = duration_imu.toSec();

            if(pose_estimation.pose_vslam_updated){

                ros::Duration duration_vslam = cur_time - pose_estimation.latest_vslam_time;
                double duration_vslam_sec = duration_vslam.toSec();

                pose_estimation.pose_vslam_updated = false;

                inter_translation += pose_estimation.vslam_velocity_linear * duration_vslam_sec;
                //inter_rotation    += temp_vel_angular * duration_sec;
            }

            temp_vel_linear   = acc_linear * duration_imu_sec;

            inter_translation += temp_vel_linear  * duration_imu_sec;
            inter_rotation    += temp_vel_angular * duration_imu_sec;

            pose_estimation.velocity_linear  += temp_vel_linear;
            pose_estimation.velocity_angular =  temp_vel_angular;
        
        }else{

            ros::Duration duration_vslam = cur_time - pose_estimation.latest_vslam_time;
            double duration_vslam_sec = duration_vslam.toSec();

            if(pose_estimation.pose_vslam_updated){

                pose_estimation.pose_vslam_updated = false;

                inter_translation += (pose_estimation.vslam_velocity_linear * duration_vslam_sec);
                //inter_rotation    += temp_vel_angular * duration_sec;
            }

            temp_vel_linear   = acc_linear * duration_vslam_sec;

            inter_translation += temp_vel_linear  * duration_vslam_sec;
            inter_rotation    += temp_vel_angular * duration_vslam_sec;

            pose_estimation.velocity_linear  += temp_vel_linear;
            pose_estimation.velocity_angular =  temp_vel_angular;
        }

        tf2::Transform latest_tf, transition_tf;

        latest_tf.setOrigin(pose_estimation.latest_tf.getOrigin());
        latest_tf.setRotation(pose_estimation.latest_tf.getRotation());

        tf2::Quaternion inter_rotation_quat;
        inter_rotation_quat.setRPY(inter_rotation[0], inter_rotation[1], inter_rotation[2]);

        transition_tf.setOrigin(inter_translation);
        transition_tf.setRotation(inter_rotation_quat);

        latest_tf = latest_tf * transition_tf;

        tf2::Vector3 tf_trans   = latest_tf.getOrigin();
        tf2::Quaternion tf_rot  = latest_tf.getRotation();

        geometry_msgs::PoseWithCovarianceStamped pose_covar_msg;

        pose_covar_msg.header.stamp     = ros::Time::now();
        pose_covar_msg.header.frame_id  = "drone_orbslam";

        pose_covar_msg.pose.pose.position.x = tf_trans[0];
        pose_covar_msg.pose.pose.position.y = tf_trans[1];
        pose_covar_msg.pose.pose.position.z = tf_trans[2];

        pose_covar_msg.pose.pose.orientation.x = tf_rot[0];
        pose_covar_msg.pose.pose.orientation.y = tf_rot[1];
        pose_covar_msg.pose.pose.orientation.z = tf_rot[2];
        pose_covar_msg.pose.pose.orientation.w = tf_rot[3];

        // Apply covariance databy the supplied covariance (can be found in "../../../include/Nav_config.h")

        for(int i=0; i<36; i++){
            pose_covar_msg.pose.covariance[i] = pose_covariance[i];
        }

        // Publish 

        //br.sendTransform(transformStamped); 

        pub_pose.publish(pose_covar_msg);

        pose_estimation.latest_imu_time = cur_time;

        pose_estimation.latest_tf = latest_tf;
        pose_estimation.imu_initialized = true;

    }

    
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
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, false);

    // Assign SLAM system to object
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    // Subscribers

    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 1, imu_callback);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol; 

    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub); // imu_sub
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2)); //,_3

    tf2_ros::TransformListener tfListener(tfBuffer);

    // Publishers
    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/orbslam_pose", 10);

    // Initially, we need to have the transform between the drone and the rs_optical
    geometry_msgs::TransformStamped drone_d435_optical_stamped;

    try {
        drone_d435_optical_stamped = tfBuffer.lookupTransform("d435_optical", "drone", ros::Time(0), ros::Duration(10.0));
    }
    catch(tf2::TransformException &ex){
        
        ROS_ERROR("Could not find transform between 'd435_optical' and 'drone' ");

        return -1;
    }

    // Must export transform from geometry_msgs::TransformStamped

    d435_optical_drone_tf.setOrigin(tf2::Vector3(drone_d435_optical_stamped.transform.translation.x, 
                                                drone_d435_optical_stamped.transform.translation.y, 
                                                drone_d435_optical_stamped.transform.translation.z));

    d435_optical_drone_tf.setRotation(tf2::Quaternion(drone_d435_optical_stamped.transform.rotation.x, 
                                                    drone_d435_optical_stamped.transform.rotation.y, 
                                                    drone_d435_optical_stamped.transform.rotation.z, 
                                                    drone_d435_optical_stamped.transform.rotation.w));

    // Force SLAM to setup the initial pose
    pose_estimation.pose_vslam_updated      = false;
    pose_estimation.pose_initialized        = false;
    pose_estimation.velocity_initialized    = false;
    pose_estimation.imu_initialized         = false;

    // The system stands ready, enable callbacks
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Shutdown Node
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    cv::Mat initPose = cv::Mat::eye(4,4,CV_32F);

    ROS_WARN("odom");

    if(!pose_estimation.pose_initialized){

        // Initially, we need to have the transform between the drone and the rs_optical
        geometry_msgs::TransformStamped map_d435_optical_stamped;

        pose_estimation.pose_vslam_updated      = false;
        pose_estimation.velocity_initialized    = false;
        pose_estimation.imu_initialized         = false;

        // Try to aquire transform
        try {
            map_d435_optical_stamped = tfBuffer.lookupTransform("d435_optical", "map", ros::Time(0), ros::Duration(0.2));
        }
        catch(tf2::TransformException &ex){
            
            ROS_ERROR("Could not find transform between 'd435_optical' and 'map' ");

            return;
        }

        // Setup matrices
        tf2::Matrix3x3 rotMat(tf2::Quaternion(map_d435_optical_stamped.transform.rotation.x, 
                                            map_d435_optical_stamped.transform.rotation.y, 
                                            map_d435_optical_stamped.transform.rotation.z, 
                                            map_d435_optical_stamped.transform.rotation.w));

        initPose.at<float>(0, 0) = rotMat[0][0]; initPose.at<float>(0, 1) = rotMat[0][1]; initPose.at<float>(0, 2) = rotMat[0][2];
        initPose.at<float>(1, 0) = rotMat[1][0]; initPose.at<float>(1, 1) = rotMat[1][1]; initPose.at<float>(1, 2) = rotMat[1][2];
        initPose.at<float>(2, 0) = rotMat[2][0]; initPose.at<float>(2, 1) = rotMat[2][1]; initPose.at<float>(2, 2) = rotMat[2][2];

        initPose.at<float>(0, 3) = map_d435_optical_stamped.transform.translation.x;
        initPose.at<float>(1, 3) = map_d435_optical_stamped.transform.translation.y;
        initPose.at<float>(2, 3) = map_d435_optical_stamped.transform.translation.z;
    }

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

    if(!pose_estimation.pose_initialized){
        
        mpSLAM->Reset();

        track = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image, initPose, cv_ptrRGB->header.stamp.toSec());

    }else{

        track = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    }

    //ROS_INFO("Size: %d, %d", track.rows, track.cols);

    // Check for successful track
    if (track.rows == 4 && track.cols == 4) {

        pose_estimation.pose_initialized = true;

        // Broadcast transform
        static tf2_ros::TransformBroadcaster br;

        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::PoseWithCovarianceStamped pose_covar_msg;

        cv::Mat rot   = track.rowRange(0,3).colRange(0,3).t(); // Rotarion
        cv::Mat trans = -rot*track.rowRange(0,3).col(3);       // Translation
        vector<float> quat = ORB_SLAM2::Converter::toQuaternion(rot);

        ros::Time cur_time = ros::Time::now();
        
        transformStamped.header.stamp   = cur_time;
        pose_covar_msg.header.stamp     = cur_time;

        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id  = "drone_orbslam";

        pose_covar_msg.header.frame_id   = "drone_orbslam";
    
        // Apply the measurement to a transform

        tf2::Transform init_transform, final_transform;

        init_transform.setOrigin(tf2::Vector3(trans.at<float>(0, 0), trans.at<float>(0, 1), trans.at<float>(0, 2))); // (x, y, z)
        init_transform.setRotation(tf2::Quaternion(quat[0], quat[1], quat[2], quat[3])); // (x, y, z, w)

        // Connect the transforms
        final_transform = init_transform * d435_optical_drone_tf;

        // Calculate local velocity (linear + angular)
        if(pose_estimation.velocity_initialized){

            ros::Duration duration = cur_time - pose_estimation.latest_vslam_time;

            tf2::Transform transition_tf = final_transform.inverse() * pose_estimation.latest_vslam_tf;

            transition_tf = transition_tf.inverse();

            tf2::Vector3    velocity_linear = transition_tf.getOrigin();
            tf2::Quaternion velocity_angular_quat = transition_tf.getRotation();

            tf2::Vector3    velocity_angular;

            tf2::Matrix3x3 rot(velocity_angular_quat);
            double roll_t, pitch_t, yaw_t;
            rot.getRPY(roll_t, pitch_t, yaw_t);

            velocity_angular[0]  = roll_t;
            velocity_angular[1]  = pitch_t;
            velocity_angular[2]  = yaw_t;

            velocity_linear  = velocity_linear / duration.toSec();
            velocity_angular = velocity_angular / duration.toSec();
        
            //ROS_WARN("Velocity Linear: X: %f, Y: %f, Z: %f",  velocity_linear[0],  velocity_linear[1],  velocity_linear[2]);
            //ROS_WARN("Velocity Angular: X: %f, Y: %f, Z: %f", velocity_angular[0], velocity_angular[1], velocity_angular[2]);

            // Try estimation

            /*
            velocity_linear  = velocity_linear * duration.toSec();
            velocity_angular = velocity_angular * duration.toSec();

            tf2::Vector3    velocity_linear_est = velocity_linear;
            tf2::Quaternion velocity_angular_quat_est;
            velocity_angular_quat_est.setRPY(velocity_angular[0], velocity_angular[1], velocity_angular[2]);//(double)velocity_angular[0], (double)velocity_angular[1], (double)velocity_angular[2]);//velocity_angular[2], velocity_angular[1], velocity_angular[0]);

            geometry_msgs::TransformStamped trasta;
            tf2::Transform transition_tf_est;

            transition_tf_est.setOrigin(velocity_linear_est);//[0], velocity_linear_est[1], velocity_linear_est[2]);
            transition_tf_est.setRotation(velocity_angular_quat_est);//[0], velocity_angular_quat_est[1], velocity_angular_quat_est[2], velocity_angular_quat_est[3]);

            tf2::Transform tra = latest_odom_drone_tf * transition_tf_est;

            tf2::Vector3 tra_trans   = tra.getOrigin();
            tf2::Quaternion tra_rot  = tra.getRotation();


            trasta.header.stamp   = cur_time;

            trasta.header.frame_id = "odom";
            trasta.child_frame_id  = "trasta";

            trasta.transform.translation.x = tra_trans[0];
            trasta.transform.translation.y = tra_trans[1];
            trasta.transform.translation.z = tra_trans[2];

            trasta.transform.rotation.x = tra_rot[0];
            trasta.transform.rotation.y = tra_rot[1];
            trasta.transform.rotation.z = tra_rot[2];
            trasta.transform.rotation.w = tra_rot[3];


            //br.sendTransform(trasta); 
            */

            pose_estimation.vslam_velocity_linear   = velocity_linear;
            pose_estimation.vslam_velocity_angular  = velocity_angular;

            pose_estimation.velocity_linear     = velocity_linear;
            pose_estimation.velocity_angular    = velocity_angular;

            // Update latest measurements

            pose_estimation.latest_vslam_time   = cur_time;
            pose_estimation.latest_vslam_tf     = final_transform;
            pose_estimation.latest_tf           = final_transform;
            pose_estimation.pose_vslam_updated  = true;
            pose_estimation.imu_initialized     = false;

        }
        
        tf2::Vector3 tf_trans   = final_transform.getOrigin();
        tf2::Quaternion tf_rot  = final_transform.getRotation();

        // Setup Translation

        transformStamped.transform.translation.x = tf_trans[0];
        transformStamped.transform.translation.y = tf_trans[1];
        transformStamped.transform.translation.z = tf_trans[2];

        pose_covar_msg.pose.pose.position.x = tf_trans[0];
        pose_covar_msg.pose.pose.position.y = tf_trans[1];
        pose_covar_msg.pose.pose.position.z = tf_trans[2];

        // Setup Rotation 

        transformStamped.transform.rotation.x = tf_rot[0];
        transformStamped.transform.rotation.y = tf_rot[1];
        transformStamped.transform.rotation.z = tf_rot[2];
        transformStamped.transform.rotation.w = tf_rot[3];

        pose_covar_msg.pose.pose.orientation.x = tf_rot[0];
        pose_covar_msg.pose.pose.orientation.y = tf_rot[1];
        pose_covar_msg.pose.pose.orientation.z = tf_rot[2];
        pose_covar_msg.pose.pose.orientation.w = tf_rot[3];

        // Apply covariance databy the supplied covariance (can be found in "../../../include/Nav_config.h")

        for(int i=0; i<36; i++){
            pose_covar_msg.pose.covariance[i] = pose_covariance[i];
        }

        // Publish 

        //br.sendTransform(transformStamped); 

        pub_pose.publish(pose_covar_msg);

        // Update latest measurements
        pose_estimation.velocity_initialized   = true;
    }

    
}