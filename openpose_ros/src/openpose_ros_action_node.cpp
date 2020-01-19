
/**
 * yubaoliu89@gmail.com
 * 2020/01/19
 * 1. action server
 * 2. publish result image
 */

// C++ std library dependencies
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <thread> // std::this_thread

#include <openpose.h>
// #include <openpose_ros_io.h>
#include <gflags_options.h>

#include "openpose_action.h"

int openPoseROS()
{
    ros::NodeHandle nh("~");
    std::string semantic_action_topic = std::string("/openpose/humanlist");

    // logging_level
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::Profiler::setDefaultX(FLAGS_profile_speed);

    op::log("Starting pose estimation demo.", op::Priority::High);
    const auto timerBegin = std::chrono::high_resolution_clock::now();

    openpose_ros::OpenPose openPose;

    op::log("Starting thread(s)", op::Priority::High);
    openPose.start();

    // OpenPose processing
    // openpose_ros::OpenPoseROSIO openPoseROSIO(openPose);
    openpose_ros::OpenPoseActionServer openPoseROSIO(nh, semantic_action_topic, openPose);

    ros::spin();

    op::log("Stopping thread(s)", op::Priority::High);
    openPose.stop();
    openPoseROSIO.stop();

    // Measuring total time
    const auto now = std::chrono::high_resolution_clock::now();
    const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now - timerBegin).count() * 1e-9;
    const auto message = "Real-time pose estimation demo successfully finished. Total time: " + std::to_string(totalTimeSec) + " seconds.";
    op::log(message, op::Priority::High);

    return 0;
}

int main(int argc, char *argv[])
{
    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Initializing ros
    ros::init(argc, argv, "openpose_ros_node");

    // Running openPoseROS
    return openPoseROS();
}
