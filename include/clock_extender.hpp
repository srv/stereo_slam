/**
 *  @file clock_extender.hpp
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


#ifndef CLOCK_EXTENDER_HPP
#define CLOCK_EXTENDER_HPP

#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>


class ClockExtender
{
    public:

        /** \brief Class constructor.
         */
        ClockExtender();


        /** \brief Class destructor.
         * 
         */
        ~ClockExtender();


    protected:

        /** \brief Clock callback.
         *  \param[in] msg ROS clock message.
         */
        void clockCallback(const rosgraph_msgs::ClockConstPtr& msg);


        /** \brief Callback that is called when the timer is triggered.
         *  \param[in] event ros::VallTimerEvent.
         */
        void timerCallback(const ros::WallTimerEvent& event);


        /** \brief Extend clock using a worker thread.
         */
        void extendClockWorker();


        /** \brief Avoid launching a worker if one is already launched.
         *  @return If it is possible to launch the worker.
        */    
        bool launchWorkerIfNeeded();


    private:
    
        // ROS handles.
        ros::Publisher pub_; //!> Clock publisher.

        ros::Subscriber sub_; //!> Clock subscriber.

        ros::WallTimer timer_; //!> Timer to extend the clock.

        // Params.
        double extra_time_; //!> The additional time you want to extend the clock in seconds.

        double gap_threshold_; //!> Threshold to consider bag finished.

        double watch_interval_; //!> Timer interval.

        // Time.
        ros::Time last_clock_time_; //!> Last ROS time.

        std::chrono::steady_clock::time_point last_msg_wall_time_;

        // Multi-thread.
        std::mutex clock_mutex_; //!> Mutex for last_clock_time_.

        std::thread worker_thread_; //!> The thread that will extend the clock.

        std::atomic<bool> extending_; //!> The clock is extending right now?

        std::atomic<bool> clock_received_; //!> Have I received any messages from the clock?

        std::atomic<bool> worker_stop_requested_;

};


#endif // CLOCK_EXTENDER_HPP