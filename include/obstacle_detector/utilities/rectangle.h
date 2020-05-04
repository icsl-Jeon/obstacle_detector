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

#include "obstacle_detector/utilities/point.h"
#include "obstacle_detector/utilities/segment.h"
#include "obstacle_detector/utilities/math_utilities.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace obstacle_detector
{

class Rectangle{
public:
        Point center;
        double theta; // R_w_r world to rect
        double l1,l2; // width and height

        double getArea() {return l1*l2;};
        void loadMarkerPoseScale(visualization_msgs::Marker& marker) const{

            marker.pose.position.x = center.x;
            marker.pose.position.y = center.y;
            marker.pose.position.z = 0;

            tf::Quaternion q;
            q.setRPY(0,0,theta);
            q.normalize();

            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            marker.scale.x = l1;
            marker.scale.y = l2;
            marker.scale.z = 0.4; // set as parameter ?

        }

        std::vector<Point> getCorners() const{

            std::vector<Point> corners;
            corners.push_back(center +  transformPoint(Point(l1/2,l2/2),0,0,theta));
            corners.push_back(center +  transformPoint(Point(l1/2,-l2/2),0,0,theta));
            corners.push_back(center +  transformPoint(Point(-l1/2,l2/2),0,0,theta));
            corners.push_back(center +  transformPoint(Point(-l1/2,-l2/2),0,0,theta));
            return corners;
        }



        double distTo(const Rectangle& rect){
            double minDistVertex = 1e+6;

            std::vector<Point> corners = getCorners();
            std::vector<Point> corners2 = rect.getCorners();

            for (auto i = corners.begin() ; i < corners.end();i++)
                for (auto j = corners2.begin() ; j < corners2.end();j++)
                    if (minDistVertex > (*i-*j).length())
                        minDistVertex = (*i-*j).length();

            return minDistVertex;
        }


    };

    Rectangle sum(const Rectangle& rect1,const Rectangle& rect2){
        double thetaNew = (rect1.theta + rect2.theta)/2;
        std::vector<Point> corner1 = rect1.getCorners();
        std::vector<Point> corner2 = rect2.getCorners();

        // projection to the new theta cooridnate
        for (Point& it : corner1 )
            it = transformPoint(it,0,0,-thetaNew);
        for (Point& it : corner2 )
            it = transformPoint(it,0,0,-thetaNew);

        double xmax = -1e+6,ymax = -1e+6,xmin = 1e+6,ymin = 1e+6;

        for (uint i = 0 ; i < 4 ; i++){
            if (max(corner1[i].x , corner2[i].x ) > xmax)
                xmax = max(corner1[i].x , corner2[i].x );
            if (min(corner1[i].x , corner2[i].x ) < xmin)
                xmin = min(corner1[i].x , corner2[i].x );
            if (max(corner1[i].y , corner2[i].y ) > ymax)
                ymax = max(corner1[i].y , corner2[i].y );
            if (min(corner1[i].y , corner2[i].y ) < ymin)
                ymin = min(corner1[i].y , corner2[i].y );
        }

        Rectangle newRect;
        newRect.theta = thetaNew;
        newRect.l1 = xmax - xmin;
        newRect.l2 = ymax - ymin;
        newRect.center = transformPoint(Point((xmax+xmin)/2,(ymax+ymin)/2),0,0,thetaNew);
        return newRect;
    }
} // namespace obstacle_detector
