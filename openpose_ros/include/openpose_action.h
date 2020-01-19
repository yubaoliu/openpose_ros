/**
 * yubaoliu89@gmail.com
 * 2020/01/19
 * 1. action server
 * 2. publish result image
 */
#ifndef _OPENPOSE_ACTION_H_
#define _OPENPOSE_ACTION_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <signal.h>
#include <actionlib/server/simple_action_server.h>
#include "openpose_ros_msgs/OpenPoseHumanListAction.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> // Video write

#include <openpose_ros_msgs/BoundingBox.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>

#include <openpose.h>
#include <gflags_options.h>

// OpenPose dependencies
#include <openpose/headers.hpp>

namespace openpose_ros
{
class OpenPoseActionServer
{
protected:
    OpenPose *openpose_;
    cv_bridge::CvImagePtr cv_img_ptr_;
    bool display_output_flag_;
    bool print_keypoints_flag_;
    bool save_original_video_flag_;
    std::string original_video_file_name_;
    bool original_video_writer_initialized_;
    cv::VideoWriter original_video_writer_;

    bool save_openpose_video_flag_;
    std::string openpose_video_file_name_;
    bool openpose_video_writer_initialized_;
    cv::VideoWriter openpose_video_writer_;
    std_msgs::Header image_header_;
    ros::Publisher openpose_human_list_pub_;
    ros::Publisher openpose_result_image_pub_;

    int video_fps_;
    openpose_ros_msgs::OpenPoseHumanList humanList_msg_;
    bool publish_result_image;

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<openpose_ros_msgs::OpenPoseHumanListAction> as_;
    std::string action_name_;
    openpose_ros_msgs::OpenPoseHumanListFeedback feedback_;
    openpose_ros_msgs::OpenPoseHumanListResult result_;

public:
    OpenPoseActionServer(ros::NodeHandle &nh, std::string name, OpenPose &openPose) : as_(nh, name, boost::bind(&OpenPoseActionServer::executeCB, this, _1), false), action_name_(name), nh_(nh), openpose_(&openPose)
    {
        std::string output_topic;
        std::string input_image_transport_type;
        std::string openpose_result_image_topic;

        nh_.param("input_image_transport_type", input_image_transport_type, std::string("raw"));
        nh_.param("output_topic", output_topic, std::string("/openpose_ros/human_list"));
        nh_.param("display_output", display_output_flag_, true);
        nh_.param("print_keypoints", print_keypoints_flag_, false);
        nh_.param("save_original_video", save_original_video_flag_, false);
        nh_.param("save_openpose_video", save_openpose_video_flag_, false);
        nh_.param("original_video_file_name", original_video_file_name_, std::string(""));
        nh_.param("openpose_video_file_name", openpose_video_file_name_, std::string(""));
        nh_.param("video_fps", video_fps_, 10);
        nh_.param("openpose_result_image_topic", openpose_result_image_topic, std::string("/openpose_ros/result_image"));

        publish_result_image = true;
        openpose_human_list_pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>(output_topic, 10);
        openpose_result_image_pub_ = nh_.advertise<sensor_msgs::Image>(openpose_result_image_topic, 10);

        cv_img_ptr_ = nullptr;
        openpose_ = &openPose;

        if (save_original_video_flag_)
        {
            if (original_video_file_name_.empty())
            {
                std::cout << "No original video filename was provided. Not saving original video." << std::endl;
                save_original_video_flag_ = false;
            }
            else
            {
                original_video_writer_initialized_ = false;
            }
        }
        if (save_openpose_video_flag_)
        {
            if (openpose_video_file_name_.empty())
            {
                std::cout << "No openpose video filename was provided. Not saving openpose video." << std::endl;
                save_openpose_video_flag_ = false;
            }
            else
            {
                openpose_video_writer_initialized_ = false;
            }
        }
        std::cout << "Start Action Server" << std::endl;
        // as_.registerPreemptCallback(boost::bind(&OpenPoseActionServer::preemptCB, this));
        as_.start();
    }

    // void preemptCB()
    // {
    //     ROS_INFO("Preempted");
    // }

    void executeCB(const openpose_ros_msgs::OpenPoseHumanListGoalConstPtr &t_goal)
    {
        if (!as_.isActive())
            return;

        processImage(t_goal->image);
        std::cout << "id: " << t_goal->id << std::endl;

        result_.id = t_goal->id;
        result_.humanList = humanList_msg_;

        as_.setSucceeded(result_);
        feedback_.complete = true;
        as_.publishFeedback(feedback_);
    }

    ~OpenPoseActionServer(void)
    {
        std::cout << "----Server shutdown ----" << std::endl;
        as_.shutdown();
    }

    void stop()
    {
        if (save_original_video_flag_)
        {
            original_video_writer_.release();
        }
        if (save_openpose_video_flag_)
        {
            openpose_video_writer_.release();
        }
    }

    void convertImage(const sensor_msgs::Image &msg)
    {
        try
        {
            cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image_header_ = msg.header;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void processImage(const sensor_msgs::Image &msg)
    {
        convertImage(msg);
        std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumToProcess = createDatum();

        bool successfullyEmplaced = openpose_->waitAndEmplace(datumToProcess);

        // Pop frame
        std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumProcessed;
        if (successfullyEmplaced && openpose_->waitAndPop(datumProcessed))
        {
            if (display_output_flag_)
            {
                display(datumProcessed);
            }
            if (publish_result_image)
            {
                publishResultImage(datumProcessed);
            }
            if (print_keypoints_flag_)
            {
                printKeypoints(datumProcessed);
            }
            if (save_original_video_flag_)
            {
                saveOriginalVideo(datumToProcess);
            }
            if (save_openpose_video_flag_)
            {
                saveOpenPoseVideo(datumProcessed);
            }
            humanList_msg_ = getHumanListMsg(datumProcessed);
            publish(humanList_msg_);
        }
        else
        {
            op::log("Processed datum could not be emplaced.", op::Priority::High,
                    __LINE__, __FUNCTION__, __FILE__);
        }
    }

    openpose_ros_msgs::OpenPoseHumanList getHumanListMsg(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumsPtr)
    {
        openpose_ros_msgs::OpenPoseHumanList human_list_msg;
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            const auto &poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            const auto &faceKeypoints = datumsPtr->at(0)->faceKeypoints;
            const auto &leftHandKeypoints = datumsPtr->at(0)->handKeypoints[0];
            const auto &rightHandKeypoints = datumsPtr->at(0)->handKeypoints[1];
            std::vector<op::Rectangle<float>> &face_rectangles = datumsPtr->at(0)->faceRectangles;

            // human_list_msg.header.stamp = ros::Time::now();
            human_list_msg.header.stamp = image_header_.stamp;
            human_list_msg.image_header = image_header_;
            human_list_msg.num_humans = poseKeypoints.getSize(0);

            std::vector<openpose_ros_msgs::OpenPoseHuman> human_list(poseKeypoints.getSize(0));

            for (auto person = 0; person < poseKeypoints.getSize(0); person++)
            {
                openpose_ros_msgs::OpenPoseHuman human;

                double body_min_x = -1;
                double body_max_x = -1;
                double body_min_y = -1;
                double body_max_y = -1;

                int num_body_key_points_with_non_zero_prob = 0;
                for (auto bodyPart = 0; bodyPart < poseKeypoints.getSize(1); bodyPart++)
                {
                    openpose_ros_msgs::PointWithProb body_point_with_prob;
                    body_point_with_prob.x = poseKeypoints[{person, bodyPart, 0}];
                    body_point_with_prob.y = poseKeypoints[{person, bodyPart, 1}];
                    body_point_with_prob.prob = poseKeypoints[{person, bodyPart, 2}];
                    if (body_point_with_prob.prob > 0)
                    {
                        num_body_key_points_with_non_zero_prob++;

                        if (body_min_x == -1 || body_point_with_prob.x < body_min_x)
                        {
                            body_min_x = body_point_with_prob.x;
                        }
                        if (body_point_with_prob.x > body_max_x)
                        {
                            body_max_x = body_point_with_prob.x;
                        }

                        if (body_min_y == -1 || body_point_with_prob.y < body_min_y)
                        {
                            body_min_y = body_point_with_prob.y;
                        }
                        if (body_point_with_prob.y > body_max_y)
                        {
                            body_max_y = body_point_with_prob.y;
                        }
                    }
                    human.body_key_points_with_prob.at(bodyPart) = body_point_with_prob;
                }
                human.num_body_key_points_with_non_zero_prob = num_body_key_points_with_non_zero_prob;
                human.body_bounding_box.x = body_min_x;
                human.body_bounding_box.y = body_min_y;
                human.body_bounding_box.width = body_max_x - body_min_x;
                human.body_bounding_box.height = body_max_y - body_min_y;

                if (FLAGS_face)
                {
                    int num_face_key_points_with_non_zero_prob = 0;

                    for (auto facePart = 0; facePart < faceKeypoints.getSize(1); facePart++)
                    {
                        openpose_ros_msgs::PointWithProb face_point_with_prob;
                        face_point_with_prob.x = faceKeypoints[{person, facePart, 0}];
                        face_point_with_prob.y = faceKeypoints[{person, facePart, 1}];
                        face_point_with_prob.prob = faceKeypoints[{person, facePart, 2}];
                        if (face_point_with_prob.prob > 0)
                        {
                            num_face_key_points_with_non_zero_prob++;
                        }
                        human.face_key_points_with_prob.at(facePart) = face_point_with_prob;
                    }
                    human.num_face_key_points_with_non_zero_prob = num_face_key_points_with_non_zero_prob;

                    openpose_ros_msgs::BoundingBox face_bounding_box;
                    face_bounding_box.x = face_rectangles.at(person).x;
                    face_bounding_box.y = face_rectangles.at(person).y;
                    face_bounding_box.width = face_rectangles.at(person).width;
                    face_bounding_box.height = face_rectangles.at(person).height;
                    human.face_bounding_box = face_bounding_box;
                }

                if (FLAGS_hand)
                {

                    int num_right_hand_key_points_with_non_zero_prob = 0;
                    int num_left_hand_key_points_with_non_zero_prob = 0;

                    for (auto handPart = 0; handPart < rightHandKeypoints.getSize(1); handPart++)
                    {
                        openpose_ros_msgs::PointWithProb right_hand_point_with_prob;
                        openpose_ros_msgs::PointWithProb left_hand_point_with_prob;
                        right_hand_point_with_prob.x = rightHandKeypoints[{person, handPart, 0}];
                        right_hand_point_with_prob.y = rightHandKeypoints[{person, handPart, 1}];
                        right_hand_point_with_prob.prob = rightHandKeypoints[{person, handPart, 2}];
                        if (right_hand_point_with_prob.prob > 0)
                        {
                            num_right_hand_key_points_with_non_zero_prob++;
                        }
                        left_hand_point_with_prob.x = leftHandKeypoints[{person, handPart, 0}];
                        left_hand_point_with_prob.y = leftHandKeypoints[{person, handPart, 1}];
                        left_hand_point_with_prob.prob = leftHandKeypoints[{person, handPart, 2}];
                        if (left_hand_point_with_prob.prob > 0)
                        {
                            num_left_hand_key_points_with_non_zero_prob++;
                        }
                        human.right_hand_key_points_with_prob.at(handPart) = right_hand_point_with_prob;
                        human.left_hand_key_points_with_prob.at(handPart) = left_hand_point_with_prob;
                    }
                    human.num_right_hand_key_points_with_non_zero_prob = num_right_hand_key_points_with_non_zero_prob;
                    human.num_left_hand_key_points_with_non_zero_prob = num_left_hand_key_points_with_non_zero_prob;
                }

                human_list.at(person) = human;
            }

            human_list_msg.human_list = human_list;
            // openpose_human_list_pub_.publish(human_list_msg);
        }
        else
        {
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        }
        return human_list_msg;
    }

    void publish(openpose_ros_msgs::OpenPoseHumanList &t_humanList_msg)
    {
        openpose_human_list_pub_.publish(t_humanList_msg);
    }

    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> createDatum()
    {
        // Close program when empty frame
        if (cv_img_ptr_ == nullptr)
        {
            return nullptr;
        }
        else // if (cv_img_ptr_ == nullptr)
        {
            // Create new datum
            auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
            datumsPtr->emplace_back();
            auto &datumPtr = datumsPtr->at(0);
            datumPtr = std::make_shared<op::Datum>();

            // Fill datum
            datumPtr->cvInputData = cv_img_ptr_->image;

            return datumsPtr;
        }
    }

    cv_bridge::CvImagePtr &getCvImagePtr()
    {
        return cv_img_ptr_;
    }

    void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumsPtr)
    {
        // Example: How to use the pose keypoints
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            op::log("\nKeypoints:");
            // Accesing each element of the keypoints
            const auto &poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            op::log("Person pose keypoints:");
            for (auto person = 0; person < poseKeypoints.getSize(0); person++)
            {
                op::log("Person " + std::to_string(person) + " (x, y, score):");
                for (auto bodyPart = 0; bodyPart < poseKeypoints.getSize(1); bodyPart++)
                {
                    std::string valueToPrint;
                    for (auto xyscore = 0; xyscore < poseKeypoints.getSize(2); xyscore++)
                    {
                        valueToPrint += std::to_string(poseKeypoints[{person, bodyPart, xyscore}]) + " ";
                    }
                    op::log(valueToPrint);
                }
            }
            op::log(" ");
            // Alternative: just getting std::string equivalent
            if (FLAGS_face)
            {
                op::log("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString(), op::Priority::High);
            }
            if (FLAGS_hand)
            {
                op::log("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString(), op::Priority::High);
                op::log("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString(), op::Priority::High);
            }
            // Heatmaps
            const auto &poseHeatMaps = datumsPtr->at(0)->poseHeatMaps;
            if (!poseHeatMaps.empty())
            {
                op::log("Pose heatmaps size: [" + std::to_string(poseHeatMaps.getSize(0)) + ", " + std::to_string(poseHeatMaps.getSize(1)) + ", " + std::to_string(poseHeatMaps.getSize(2)) + "]");
                const auto &faceHeatMaps = datumsPtr->at(0)->faceHeatMaps;
                op::log("Face heatmaps size: [" + std::to_string(faceHeatMaps.getSize(0)) + ", " + std::to_string(faceHeatMaps.getSize(1)) + ", " + std::to_string(faceHeatMaps.getSize(2)) + ", " + std::to_string(faceHeatMaps.getSize(3)) + "]");
                const auto &handHeatMaps = datumsPtr->at(0)->handHeatMaps;
                op::log("Left hand heatmaps size: [" + std::to_string(handHeatMaps[0].getSize(0)) + ", " + std::to_string(handHeatMaps[0].getSize(1)) + ", " + std::to_string(handHeatMaps[0].getSize(2)) + ", " + std::to_string(handHeatMaps[0].getSize(3)) + "]");
                op::log("Right hand heatmaps size: [" + std::to_string(handHeatMaps[1].getSize(0)) + ", " + std::to_string(handHeatMaps[1].getSize(1)) + ", " + std::to_string(handHeatMaps[1].getSize(2)) + ", " + std::to_string(handHeatMaps[1].getSize(3)) + "]");
            }
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
    }

    bool saveOpenPoseVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumsPtr)
    {
        char key = ' ';
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            cv::Mat current_image = datumsPtr->at(0)->cvOutputData;
            if (!current_image.empty())
            {
                if (!openpose_video_writer_initialized_)
                {
                    openpose_video_writer_ = cv::VideoWriter(openpose_video_file_name_, CV_FOURCC('M', 'J', 'P', 'G'), video_fps_, current_image.size());
                    openpose_video_writer_initialized_ = true;
                }
                openpose_video_writer_.write(current_image);
            }
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        return (key == 27);
    }

    bool display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumsPtr)
    {
        // User's displaying/saving/other processing here
        // datum.cvOutputData: rendered frame with pose or heatmaps
        // datum.poseKeypoints: Array<float> with the estimated pose
        char key = ' ';
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            cv::imshow("User worker GUI", datumsPtr->at(0)->cvOutputData);
            // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
            key = (char)cv::waitKey(1);
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        return (key == 27);
    }

    void publishResultImage(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumsPtr)
    {
        // User's displaying/saving/other processing here
        // datum.cvOutputData: rendered frame with pose or heatmaps
        // datum.poseKeypoints: Array<float> with the estimated pose
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            cv::Mat result = datumsPtr->at(0)->cvOutputData;
            sensor_msgs::ImagePtr msg_rgb;
            msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
            openpose_result_image_pub_.publish(msg_rgb);
            // cv::imshow("User worker GUI", datumsPtr->at(0)->cvOutputData);
            // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    }

    bool saveOriginalVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumsPtr)
    {
        char key = ' ';
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            cv::Mat current_image = datumsPtr->at(0)->cvInputData;
            if (!current_image.empty())
            {
                if (!original_video_writer_initialized_)
                {
                    original_video_writer_ = cv::VideoWriter(original_video_file_name_, CV_FOURCC('M', 'J', 'P', 'G'), video_fps_, current_image.size());
                    original_video_writer_initialized_ = true;
                }
                original_video_writer_.write(current_image);
            }
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        return (key == 27);
    }
};

} // namespace openpose_ros

#endif