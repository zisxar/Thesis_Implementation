/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/ImuTypes.h"

#include <pole/LocalizationOnly.h>

using namespace std;

// World transform
Sophus::SE3f Tc0w = Sophus::SE3f();

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

bool localization_only = false;
bool localizing = false;

long unsigned int new_map = -1, curr_map = -2;

bool set_localization_only_handler(pole::LocalizationOnly::Request  &req, pole::LocalizationOnly::Response &res) {
    ROS_INFO("LocalizationOnly= %d", (bool)req.localization_only);
    localization_only = req.localization_only;
    res.localization_only_set_ok = true;
    return true;
}

int main(int argc, char **argv)
{
    // Initializations
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    // World transform
    Eigen::AngleAxisf AngleR(1.57079632679, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf AngleP(0.0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf AngleY(1.57079632679, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf qRPY = AngleR * AngleP * AngleY;
    Eigen::Matrix3f RotRPY = qRPY.matrix();
    Tc0w = Sophus::SE3f(RotRPY, Eigen::Vector3f::Zero());

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,false);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    // Camera initializations and calibration
    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh("~");

    // Get parameter for default value of localization only
    bool default_localization;
    if (nh.getParam("default_localization", default_localization)) {
      ROS_INFO("Default localization_only is: %d", default_localization);
      localization_only = default_localization;
    }
    else {
      ROS_WARN("Failed to get param 'default_localization'");
      localization_only = false;
    }

    ROS_INFO("ORB_SLAM3 STARTED");
    // Service to enable localization only mode
    ros::ServiceServer localization_service = nh.advertiseService("/localization_only", set_localization_only_handler);


    // Message synchronization
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/infra1/image_rect_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/infra2/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

tf::Transform SE3f_to_tfTransform(Sophus::SE3f T_SE3f)
{
    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();

    tf::Matrix3x3 R_tf(
        R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
        R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
        R_mat(2, 0), R_mat(2, 1), R_mat(2, 2)
    );

    tf::Vector3 t_tf(
        t_vec(0),
        t_vec(1),
        t_vec(2)
    );

    return tf::Transform(R_tf, t_tf);
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    Sophus::SE3f track;

    if (localization_only && !localizing) {
        mpSLAM->ActivateLocalizationMode();
        localizing = true;
    }
    if (!localization_only && localizing) {
        mpSLAM->DeactivateLocalizationMode();
        localizing = false;
    }

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        track = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        track = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    new_map = mpSLAM->GetMapIdSystem();
    // ROS_INFO("***** new MAP: %ld prev MAP: %ld *****", new_map, curr_map);
    if (!(new_map == curr_map)) {
        ROS_WARN("***** MAP ID: %ld MAP CT: %d *****", mpSLAM->GetMapIdSystem(), mpSLAM->CountMapsSystem());
        curr_map = mpSLAM->GetMapIdSystem();
    }

    Sophus::SE3f Twc = (track * Tc0w).inverse();

    tf::Transform rs_tf = SE3f_to_tfTransform(Twc);

    tf::Vector3 rs_trans = rs_tf.getOrigin();
    tf::Quaternion rs_quat = rs_tf.getRotation();

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";

    transformStamped.child_frame_id = "realsense_optical";
    transformStamped.transform.translation.x = rs_trans.x();
    transformStamped.transform.translation.y = rs_trans.y();
    transformStamped.transform.translation.z = rs_trans.z();
    transformStamped.transform.rotation.x = rs_quat.x();
    transformStamped.transform.rotation.y = rs_quat.y();
    transformStamped.transform.rotation.z = rs_quat.z();
    transformStamped.transform.rotation.w = rs_quat.w();

    br.sendTransform(transformStamped);
}


