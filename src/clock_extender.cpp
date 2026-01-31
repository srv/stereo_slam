/**
 *  @file clock_extender.cpp
 *  @brief Extend the ROS clock by a few seconds after a bagfile has finished.
 *  @package stereo_slam
 *  @project stereo_slam
 *
 *  @author
 *    Bo Miquel Nordfeldt-Fiol (2024–2025)
 *
 *  @license BSD-4-Clause
 *    This file is part of the stereo_slam project and is released under
 *    the BSD 4-Clause License. See the LICENSE file for details.
 */


#include "clock_extender.hpp"


ClockExtender::ClockExtender(): extra_time_{1}, 
                                gap_threshold_{1},
                                watch_interval_{1},
                                extending_{false},
                                clock_received_{false},
                                worker_stop_requested_{false}
{
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Get params.
    nhp.param<double>("extra_time", extra_time_, 1);
    nhp.param<double>("gap_threshold", gap_threshold_, 1);
    nhp.param<double>("watch_interval", watch_interval_, 1);

    // Param debug.
    ROS_WARN_STREAM("[CE:] Get " << extra_time_ << " seconds of extra time");
    ROS_WARN_STREAM("[CE:] Get " << gap_threshold_ << " seconds of gap threshold");
    ROS_WARN_STREAM("[CE:] Get " << watch_interval_ << " seconds of watch interval");

    // Publishers.
    pub_ = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);

    // Subscribers.
    sub_ = nh.subscribe("/clock", 10, &ClockExtender::clockCallback, this);

    // Timers.
    timer_ = nh.createWallTimer(ros::WallDuration(watch_interval_), 
                                &ClockExtender::timerCallback, this);

    last_msg_wall_time_ = std::chrono::steady_clock::now();
}


ClockExtender::~ClockExtender()
{
    // Ask worker to stop and join if running.
    worker_stop_requested_.store(true);
    if (worker_thread_.joinable())
        worker_thread_.join();
}


void ClockExtender::clockCallback(const rosgraph_msgs::ClockConstPtr& msg)
{
    {
        std::scoped_lock lock(clock_mutex_);
        last_clock_time_ = msg->clock;
    }
    last_msg_wall_time_ = std::chrono::steady_clock::now();
    clock_received_.store(true, std::memory_order_release);
}


void ClockExtender::timerCallback(const ros::WallTimerEvent& event)
{
    if (extending_.load(std::memory_order_acquire))
        return;
   
    std::chrono::duration<double> gap = std::chrono::steady_clock::now() - last_msg_wall_time_;

    if (gap.count() > gap_threshold_)
    {
        ROS_INFO_STREAM("[CE:] Detected gap " << gap.count() << "s. Attempting to launch a worker.");

        if (launchWorkerIfNeeded())
            ROS_INFO("[CE:] Worker launched to extend /clock.");
        else
            ROS_INFO("[CE:] Worker was already running (race avoided).");
    }
}


bool ClockExtender::launchWorkerIfNeeded()
{
    bool expected = false;
    if (!extending_.compare_exchange_strong(expected, true))
        return false;

    worker_stop_requested_.store(false);
    worker_thread_ = std::thread(&ClockExtender::extendClockWorker, this);
    return true;
}


void ClockExtender::extendClockWorker()
{

    ros::Time start_time;
    {
        std::scoped_lock lock(clock_mutex_);
        start_time = last_clock_time_;
    }

    if (!clock_received_.load(std::memory_order_acquire))
    {
        ROS_WARN("[CE:] No /clock received, nothing to extend.");
        extending_.store(false);
        ros::requestShutdown();
        return;
    }

    ROS_INFO_STREAM("[CE:] Extending /clock for " << extra_time_ << " seconds ... ");

    const double publish_hz = 50.0;
    const std::chrono::duration<double> dt(1.0 / publish_hz);

    ros::Time t = start_time;
    std::chrono::steady_clock::time_point end_wall = std::chrono::steady_clock::now() + 
                                                     std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(extra_time_));

    while (ros::ok() && std::chrono::steady_clock::now() < end_wall)
    {
        if (worker_stop_requested_.load())
        {
            ROS_WARN("[CE:] Worker stop request, aborting extension.");
            break;
        }

        t += ros::Duration(dt.count());

        rosgraph_msgs::Clock clk_msg;
        clk_msg.clock = t;
        pub_.publish(clk_msg);

        std::this_thread::sleep_for(dt);
    }

    ROS_INFO("[CE:] Clock extension finished.");
    extending_.store(false);
    ros::requestShutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "clock_extender");

    ClockExtender clock_extender;

    ros::spin();

    return 0;
}