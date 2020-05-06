//
// Created by jbs on 20. 5. 5..
//

#ifndef OBSTACLE_DETECTOR_TRACKED_OBSTACLE_RECT_H
#define OBSTACLE_DETECTOR_TRACKED_OBSTACLE_RECT_H

/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <obstacle_detector/Obstacles.h>
#include "obstacle_detector/utilities/kalman.h"

namespace obstacle_detector
{

    class TrackedObstacleRect {
    public:
        TrackedObstacleRect(const RectangleObstacle& obstacle) : obstacle_(obstacle),
            kf_x_(0, 1, 2), kf_y_(0, 1, 2), kf_th_(0, 1, 2),
            kf_l1_(0, 1, 2),kf_l2_(0, 1, 2)
            {
            fade_counter_ = s_fade_counter_size_;
            initKF();
        }

        void predictState() {
            kf_x_.predictState();
            kf_y_.predictState();
            kf_th_.predictState();
            kf_l1_.predictState();
            kf_l2_.predictState();

            obstacle_.center.x = kf_x_.q_pred(0);
            obstacle_.center.y = kf_y_.q_pred(0);

            obstacle_.velocity.x = kf_x_.q_pred(1);
            obstacle_.velocity.y = kf_y_.q_pred(1);

            obstacle_.theta = kf_th_.q_pred(0);
            obstacle_.l1 = kf_l1_.q_pred(0);
            obstacle_.l2 = kf_l2_.q_pred(0);

            fade_counter_--;
        }

        void correctState(const RectangleObstacle & new_obstacle) {


            double th_pred = kf_th_.q_pred(0);
            double th_allow = M_PI/3;

            kf_x_.y(0) = new_obstacle.center.x;
            kf_y_.y(0) = new_obstacle.center.y;

            double th_obsrv_raw = new_obstacle.theta;

            // In the current implmentation, the theta is limited to [0,pi/2]
            // Thus, we should correct the angle jump for measurement update state
            int n = -5;
            for (; n <= 5 ; n++)
                if (abs(th_pred - (th_obsrv_raw+n*M_PI/2)) < th_allow )
                    break;
            if (abs(th_pred - (th_obsrv_raw+n*M_PI/2)) >= th_allow)
                ROS_WARN("[Tracker] couldn't find proper arranging for angle observation");


            kf_th_.y(0) = th_obsrv_raw+n*M_PI/2;
            if (n%2 == 0 ) {
                kf_l1_.y(0) = new_obstacle.l1;
                kf_l2_.y(0) = new_obstacle.l2;
            }
            else{
                kf_l1_.y(0) = new_obstacle.l2;
                kf_l2_.y(0) = new_obstacle.l1;
            }

            kf_x_.correctState();
            kf_y_.correctState();
            kf_th_.correctState();
            kf_l1_.correctState();
            kf_l2_.correctState();

            obstacle_.center.x = kf_x_.q_est(0);
            obstacle_.center.y = kf_y_.q_est(0);

            obstacle_.velocity.x = kf_x_.q_est(1);
            obstacle_.velocity.y = kf_y_.q_est(1);

            obstacle_.l1 = kf_l1_.q_est(0);
            obstacle_.l2 = kf_l2_.q_est(0);
            obstacle_.theta = kf_th_.q_est(0);

            fade_counter_ = s_fade_counter_size_;
        }

        void updateState() {
            kf_x_.updateState();
            kf_y_.updateState();
            kf_th_.updateState();
            kf_l1_.updateState();
            kf_l2_.updateState();


            obstacle_.center.x = kf_x_.q_est(0);
            obstacle_.center.y = kf_y_.q_est(0);

            obstacle_.velocity.x = kf_x_.q_est(1);
            obstacle_.velocity.y = kf_y_.q_est(1);

            obstacle_.theta = kf_th_.q_est(0);
            obstacle_.l1 = kf_l1_.q_est(0);
            obstacle_.l2 = kf_l2_.q_est(0);

            fade_counter_--;
        }

        static void setSamplingTime(double tp) {
            s_sampling_time_ = tp;
        }

        static void setCounterSize(int size) {
            s_fade_counter_size_ = size;
        }

        static void setCovariances(double process_var, double process_rate_var, double measurement_var) {
            s_process_variance_ = process_var;
            s_process_rate_variance_ = process_rate_var;
            s_measurement_variance_ = measurement_var;
        }

        bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }
        const RectangleObstacle& getObstacle() const { return obstacle_; }
        const KalmanFilter& getKFx() const { return kf_x_; }
        const KalmanFilter& getKFy() const { return kf_y_; }
        const KalmanFilter& getKFth() const { return kf_th_; }
        const KalmanFilter& getKFl1() const { return kf_l1_; }
        const KalmanFilter& getKFl2() const { return kf_l2_; }

    private:
        void initKF() {
            kf_x_.A(0, 1) = s_sampling_time_;
            kf_y_.A(0, 1) = s_sampling_time_;
            kf_th_.A(0, 1) = s_sampling_time_;
            kf_l1_.A(0, 1) = s_sampling_time_;
            kf_l2_.A(0, 1) = s_sampling_time_;

            kf_x_.C(0, 0) = 1.0;
            kf_y_.C(0, 0) = 1.0;
            kf_th_.C(0, 0) = 1.0;
            kf_l1_.C(0, 0) = 1.0;
            kf_l2_.C(0, 0) = 1.0;

            kf_x_.R(0, 0) = s_measurement_variance_;
            kf_y_.R(0, 0) = s_measurement_variance_;
            kf_th_.R(0, 0) = s_measurement_variance_;
            kf_l1_.R(0, 0) = 2*s_measurement_variance_;
            kf_l2_.R(0, 0) = 2*s_measurement_variance_;

            kf_x_.Q(0, 0) = s_process_variance_;
            kf_y_.Q(0, 0) = s_process_variance_;
            kf_th_.Q(0, 0) = s_process_variance_;
            kf_l1_.Q(0, 0) = 1/2*s_process_variance_;
            kf_l2_.Q(0, 0) = 1/2*s_process_variance_;

            kf_x_.Q(1, 1) = s_process_rate_variance_;
            kf_y_.Q(1, 1) = s_process_rate_variance_;
            kf_th_.Q(1, 1) = s_process_rate_variance_;
            kf_l1_.Q(1, 1) = s_process_rate_variance_;
            kf_l2_.Q(1, 1) = s_process_rate_variance_;

            kf_x_.q_pred(0) = obstacle_.center.x;
            kf_y_.q_pred(0) = obstacle_.center.y;
            kf_th_.q_pred(0) = obstacle_.theta;
            kf_l1_.q_pred(0) = obstacle_.l1;
            kf_l2_.q_pred(0) = obstacle_.l2;

            kf_x_.q_pred(1) = obstacle_.velocity.x;
            kf_y_.q_pred(1) = obstacle_.velocity.y;

            kf_x_.q_est(0) = obstacle_.center.x;
            kf_y_.q_est(0) = obstacle_.center.y;
            kf_th_.q_est(0) = obstacle_.theta;
            kf_l1_.q_est(0) = obstacle_.l1;
            kf_l2_.q_est(0) = obstacle_.l2;


            kf_x_.q_est(1) = obstacle_.velocity.x;
            kf_y_.q_est(1) = obstacle_.velocity.y;
        }

        RectangleObstacle obstacle_;

        KalmanFilter kf_x_;
        KalmanFilter kf_y_;
        KalmanFilter kf_th_;
        KalmanFilter kf_l1_;
        KalmanFilter kf_l2_;

        int fade_counter_;

        // Common variables
        static int s_fade_counter_size_;
        static double s_sampling_time_;
        static double s_process_variance_;
        static double s_process_rate_variance_;
        static double s_measurement_variance_;
    };

}



#endif //OBSTACLE_DETECTOR_TRACKED_OBSTACLE_RECT_H
