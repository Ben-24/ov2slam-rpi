/**
*    This file is part of OV²SLAM.
*    
*    Copyright (C) 2020 ONERA
*
*    For more information see <https://github.com/ov2slam/ov2slam>
*
*    OV²SLAM is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    OV²SLAM is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with OV²SLAM.  If not, see <https://www.gnu.org/licenses/>.
*
*    Authors: Maxime Ferrera     <maxime.ferrera at gmail dot com> (ONERA, DTIS - IVA),
*             Alexandre Eudes    <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Julien Moras       <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Martial Sanfourche <first.last at onera dot fr>      (ONERA, DTIS - IVA)
*/

#include <iostream>
#include <string>
#include <thread>
// REMOVED: #include <mutex> and #include <queue>
// REASON: The original sync used a manual producer/consumer queue with a ±15ms
// tolerance check in a separate thread (sync_process). When cameras are exactly
// one frame (33ms) apart the queue drains to empty after every throw, causing the
// next arriving pair to also be 33ms apart — it chases itself and never matches.
// REPLACED BY: message_filters::ApproximateTimeSynchronizer, which maintains
// a sliding window across both topics and delivers the closest-in-time pair.
// To revert: restore the includes above, restore subLeftImage/subRightImage,
// sync_process(), img0_buf/img1_buf/img_mutex members, and the subscription +
// sync_thread setup in main(). Also revert CMakeLists.txt message_filters lines.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// REMOVED: #include <image_transport/image_transport.hpp>
// REMOVED: #include <image_transport/subscriber_filter.hpp>
// REPLACED BY: message_filters direct subscribers
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include "ov2slam.hpp"
#include "slam_params.hpp"


class SensorsGrabber {

public:
    SensorsGrabber(SlamManager *slam): pslam_(slam) {
        std::cout << "\nSensors Grabber is created...\n";
    }

    // REMOVED: subLeftImage() and subRightImage() — were the queue push callbacks.
    // REMOVED: img0_buf, img1_buf, img_mutex members.
    // REMOVED: sync_process() — was the drain-and-match thread with ±15ms tolerance.
    // All replaced by stereoCallback() below, called by ApproximateTimeSynchronizer.

    cv::Mat getGrayImageFromMsg(const sensor_msgs::msg::Image &img_msg)
    {
        cv_bridge::CvImageConstPtr ptr;
        try {
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        }
        catch(cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("cv_bridge_logger"), "\n\n\ncv_bridge exeception: %s\n\n\n", e.what());
        }
        return ptr->image;
    }

    // Called by ApproximateTimeSynchronizer with the best-matched left/right pair.
    // Runs directly on the spin() thread — no separate sync thread needed.
    void stereoCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr &img0_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &img1_msg)
    {
        double time0 = rclcpp::Time(img0_msg->header.stamp).seconds();
        cv::Mat image0 = getGrayImageFromMsg(*img0_msg);
        cv::Mat image1 = getGrayImageFromMsg(*img1_msg);
        if (!image0.empty() && !image1.empty()) {
            pslam_->addNewStereoImages(time0, image0, image1);
        }
    }

    // Mono callback — called directly from subscription, no change in behaviour.
    void monoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        double time = rclcpp::Time(img_msg->header.stamp).seconds();
        cv::Mat image0 = getGrayImageFromMsg(*img_msg);
        if (!image0.empty()) {
            pslam_->addNewMonoImage(time, image0);
        }
    }

    SlamManager *pslam_;
};


int main(int argc, char** argv)
{
    // Init the node
    rclcpp::init(argc, argv);

    if(argc < 2)
    {
       std::cout << "\nUsage: rosrun ov2slam ov2slam_node parameters_files/params.yaml\n";
       return 1;
    }

    std::cout << "\nLaunching OV²SLAM...\n\n";

    auto node = rclcpp::Node::make_shared("ov2slam_node");

    // Load the parameters
    std::string parameters_file = argv[1];

    std::cout << "\nLoading parameters file : " << parameters_file << "...\n";

    const cv::FileStorage fsSettings(parameters_file.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
       std::cout << "Failed to open settings file...";
       return 1;
    } else {
        std::cout << "\nParameters file loaded...\n";
    }

    std::shared_ptr<SlamParams> pparams;
    pparams.reset( new SlamParams(fsSettings) );

    // Create the ROS Visualizer
    std::shared_ptr<RosVisualizer> prosviz;
    prosviz.reset( new RosVisualizer(node) );

    // Setting up the SLAM Manager
    SlamManager slam(pparams, prosviz);

    // Start the SLAM thread
    std::thread slamthread(&SlamManager::run, &slam);

    // Create the Bag file reader & callback functions
    SensorsGrabber sb(&slam);

    std::string topic_left  = fsSettings["Camera.topic_left"];
    std::string topic_right = fsSettings["Camera.topic_right"];

    if( pparams->stereo_ )
    {
        // REMOVED: two create_subscription() calls + sync_process thread.
        // REASON: The manual ±15ms tolerance queue never matched frames that are a
        // constant 33ms apart — it drained the queue to empty on every throw, causing
        // the next pair to be equally mismatched. See SensorsGrabber comment above.
        //
        // REPLACED BY: ApproximateTimeSynchronizer with a 10-frame sliding window.
        // It finds the closest-in-time pair regardless of constant timestamp offset.
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

        // Use sensor_data QoS (BEST_EFFORT) to match camera_ros's publisher QoS.
        // The default rmw_qos_profile_default is RELIABLE, which is incompatible
        // with a BEST_EFFORT publisher and silently drops all messages.
        auto sub_left  = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node, topic_left,  rmw_qos_profile_sensor_data);
        auto sub_right = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node, topic_right, rmw_qos_profile_sensor_data);

        auto sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *sub_left, *sub_right);
        sync->registerCallback(&SensorsGrabber::stereoCallback, &sb);

        // ROS Spin — stereoCallback fires here, no separate sync thread needed.
        rclcpp::spin(node);

        // Keep subscribers alive until after spin returns
        (void)sub_left; (void)sub_right; (void)sync;
    }
    else if( pparams->mono_ )
    {
        auto subimg = node->create_subscription<sensor_msgs::msg::Image>(
            topic_left, 2,
            [&sb](const sensor_msgs::msg::Image::ConstSharedPtr &img){ sb.monoCallback(img); });

        rclcpp::spin(node);
        (void)subimg;
    }

    // Request Slam Manager thread to exit
    slam.bexit_required_ = true;

    slamthread.join();

    return 0;
}
