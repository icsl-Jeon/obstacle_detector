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

#include "obstacle_detector/obstacle_extractor.h"
#include "obstacle_detector/utilities/figure_fitting.h"
#include "obstacle_detector/utilities/math_utilities.h"

using namespace std;
using namespace obstacle_detector;

ObstacleExtractor::ObstacleExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  params_srv_ = nh_local_.advertiseService("params", &ObstacleExtractor::updateParams, this);
  initialize();
}

ObstacleExtractor::~ObstacleExtractor() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("use_scan");
  nh_local_.deleteParam("use_pcl");

  nh_local_.deleteParam("use_split_and_merge");
  nh_local_.deleteParam("circles_from_visibles");
  nh_local_.deleteParam("discard_converted_segments");
  nh_local_.deleteParam("transform_coordinates");

  nh_local_.deleteParam("min_group_points");

  nh_local_.deleteParam("max_group_distance");
  nh_local_.deleteParam("distance_proportion");
  nh_local_.deleteParam("max_split_distance");
  nh_local_.deleteParam("max_merge_separation");
  nh_local_.deleteParam("max_merge_spread");
  nh_local_.deleteParam("max_circle_radius");
  nh_local_.deleteParam("radius_enlargement");

  nh_local_.deleteParam("min_x_limit");
  nh_local_.deleteParam("max_x_limit");
  nh_local_.deleteParam("min_y_limit");
  nh_local_.deleteParam("max_y_limit");

  nh_local_.deleteParam("frame_id");
}

bool ObstacleExtractor::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("use_scan", p_use_scan_, false);
  nh_local_.param<bool>("use_pcl", p_use_pcl_, true);

  nh_local_.param<bool>("use_split_and_merge", p_use_split_and_merge_, true);
  nh_local_.param<bool>("circles_from_visibles", p_circles_from_visibles_, true);
  nh_local_.param<bool>("discard_converted_segments", p_discard_converted_segments_, true);
  nh_local_.param<bool>("transform_coordinates", p_transform_coordinates_, true);

  nh_local_.param<int>("min_group_points", p_min_group_points_, 5);

  nh_local_.param<double>("max_group_distance", p_max_group_distance_, 0.1);
  nh_local_.param<double>("distance_proportion", p_distance_proportion_, 0.00628);
  nh_local_.param<double>("max_split_distance", p_max_split_distance_, 0.2);
  nh_local_.param<double>("max_merge_separation", p_max_merge_separation_, 0.2);
  nh_local_.param<double>("max_merge_spread", p_max_merge_spread_, 0.2);
  nh_local_.param<double>("max_circle_radius", p_max_circle_radius_, 0.6);
  nh_local_.param<double>("radius_enlargement", p_radius_enlargement_, 0.25);

  nh_local_.param<double>("min_x_limit", p_min_x_limit_, -10.0);
  nh_local_.param<double>("max_x_limit", p_max_x_limit_,  10.0);
  nh_local_.param<double>("min_y_limit", p_min_y_limit_, -10.0);
  nh_local_.param<double>("max_y_limit", p_max_y_limit_,  10.0);

  nh_local_.param<string>("frame_id", p_frame_id_, "map");

  nh_local_.param<double>("max_box_edge_length",p_max_box_edge,8.0);
  nh_local_.param<double>("max_box_area",p_max_box_area,30.0);
  nh_local_.param<double>("min_box_area",p_min_box_area,3.0);
  nh_local_.param<double>("max_box_edge_ratio",p_max_box_wh_ratio,10.0);
  nh_local_.param<double>("max_merge_theta",p_max_merge_theta_diff,M_PI/20);
  nh_local_.param<double>("max_merge_rect_dist",p_max_merge_rect_dist,0.3);


    if (p_active_ != prev_active) {
    if (p_active_) {
      if (p_use_scan_)
        scan_sub_ = nh_.subscribe("scan", 10, &ObstacleExtractor::scanCallback, this);
      else if (p_use_pcl_)
        pcl_sub_ = nh_.subscribe("pcl", 10, &ObstacleExtractor::pclCallback, this);

      obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("raw_obstacles", 10);
      rect_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lshape_boxes",10);
    }
    else {
      // Send empty message
      obstacle_detector::ObstaclesPtr obstacles_msg(new obstacle_detector::Obstacles);
      obstacles_msg->header.frame_id = p_frame_id_;
      obstacles_msg->header.stamp = ros::Time::now();
      obstacles_pub_.publish(obstacles_msg);

      scan_sub_.shutdown();
      pcl_sub_.shutdown();
      obstacles_pub_.shutdown();
    }
  }

  return true;
}

void ObstacleExtractor::scanCallback(const sensor_msgs::LaserScan::ConstPtr scan_msg) {
  base_frame_id_ = scan_msg->header.frame_id;
  stamp_ = scan_msg->header.stamp;

  double phi = scan_msg->angle_min;

  for (const float r : scan_msg->ranges) {
    if (r >= scan_msg->range_min && r <= scan_msg->range_max)
      input_points_.push_back(Point::fromPoolarCoords(r, phi));

    phi += scan_msg->angle_increment;
  }

  processPoints();
}

void ObstacleExtractor::pclCallback(const sensor_msgs::PointCloud::ConstPtr pcl_msg) {
  base_frame_id_ = pcl_msg->header.frame_id;
  stamp_ = pcl_msg->header.stamp;

  for (const geometry_msgs::Point32& point : pcl_msg->points)
    input_points_.push_back(Point(point.x, point.y));

  processPoints();
}

void ObstacleExtractor::processPoints() {
  segments_.clear();
  circles_.clear();
  circles_fit_.clear(); // JBS
    rectangles_.clear(); // JBS

  groupPoints();  // Grouping points simultaneously detects segments
  mergeSegments();
  mergeRects(); //

//  cout << "---------------" << endl;

//  detectCircles();
//  mergeCircles();

  publishObstacles();

  input_points_.clear();
}
// Raw point cluster -> generating segment
void ObstacleExtractor::groupPoints() {
  static double sin_dp = sin(2.0 * p_distance_proportion_);

  PointSet point_set;
  point_set.begin = input_points_.begin();
  point_set.end = input_points_.begin();
  point_set.num_points = 1;
  point_set.is_visible = true;

  for (PointIterator point = input_points_.begin()++; point != input_points_.end(); ++point) {
    double range = (*point).length();
    double distance = (*point - *point_set.end).length();

    if (distance < p_max_group_distance_ + range * p_distance_proportion_) {
      point_set.end = point;
      point_set.num_points++;
    }
    else {
      double prev_range = (*point_set.end).length();

      // Heron's equation
      double p = (range + prev_range + distance) / 2.0;
      double S = sqrt(p * (p - range) * (p - prev_range) * (p - distance));
      double sin_d = 2.0 * S / (range * prev_range); // Sine of angle between beams

      // TODO: This condition can be fulfilled if the point are on the opposite sides
      // of the scanner (angle = 180 deg). Needs another check.
      if (abs(sin_d) < sin_dp && range < prev_range)
        point_set.is_visible = false;

      detectSegments(point_set);
      detectLRect(point_set);

      // Begin new point set
      point_set.begin = point;
      point_set.end = point;
      point_set.num_points = 1;
      point_set.is_visible = (abs(sin_d) > sin_dp || range < prev_range);
    }
  }

  detectSegments(point_set); // Check the last point set too
  detectLRect(point_set);

}

void ObstacleExtractor::detectLRect(const PointSet& point_set) {

    if (point_set.num_points < p_min_group_points_)
        return;

    uint nAngleStep = 10;
    double angleSeg = M_PI/2/nAngleStep;

    uint nAngleBest = 0;
    double bestScore = 0;
    Rectangle bestRect;

    vector<double> C1(point_set.num_points),C2(point_set.num_points); // xy projected set

    for(uint n = 0; n<nAngleStep; n++){
        double theta = angleSeg*n;
        uint pntIdx = 0;
        double c1min = 1e+6,c1max = -1e+6,c2min = 1e+6,c2max = -1e+6;

        // extract min and max for each axis
        for (PointIterator point = point_set.begin; point != point_set.end; ++point){
            if (pntIdx < point_set.num_points) {
                getTransformedXY(*point, theta, C1[pntIdx], C2[pntIdx]);

                if (c1min > C1[pntIdx])
                    c1min = C1[pntIdx];
                if (c1max < C1[pntIdx])
                    c1max = C1[pntIdx];
                if (c2min > C2[pntIdx])
                    c2min = C2[pntIdx];
                if (c2max < C2[pntIdx])
                    c2max = C2[pntIdx];

                pntIdx++;
            }
        }

        // examine criteria
        double score = 0 ;
        double deps = 0.3;
        for (uint m = 0 ; m < point_set.num_points ; m++){
            double d1 = min(C1[m] - c1min, c1max - C1[m]);
            double d2 = min(C2[m] - c2min, c2max - C2[m]);
            score += 1/max(min(d1,d2),deps);
        }
//        cout << score << ", ";


        if (score > bestScore){

            double centerX = (c1min + c1max)/2;
            double centerY = (c2min + c2max)/2;

            bestRect.center.x = cos(theta) * centerX - sin(theta) * centerY;
            bestRect.center.y = sin(theta) * centerX + cos(theta) * centerY;

            bestRect.l1 = c1max - c1min;
            bestRect.l2 = c2max - c2min;

            bestRect.theta = theta;

            bestScore = score;
        }
    }
//    cout << "best: " << bestScore << " with theta "<<bestRect.theta<< endl;
    if (max(bestRect.l1,bestRect.l2)<p_max_box_edge and bestRect.getArea() < p_max_box_area and
            bestRect.getArea() > p_min_box_area and
            max(bestRect.l1,bestRect.l2)/min(bestRect.l1,bestRect.l2) < p_max_box_wh_ratio )
        rectangles_.push_back(bestRect);

}



void ObstacleExtractor::detectSegments(const PointSet& point_set) {
  if (point_set.num_points < p_min_group_points_)
    return;

  Segment segment(*point_set.begin, *point_set.end);  // Use Iterative End Point Fit

  if (p_use_split_and_merge_)
    segment = fitSegment(point_set);

  PointIterator set_divider;
  double max_distance = 0.0;
  double distance     = 0.0;

  int split_index = 0; // Natural index of splitting point (counting from 1)
  int point_index = 0; // Natural index of current point in the set

  // Seek the point of division
  for (PointIterator point = point_set.begin; point != point_set.end; ++point) {
    ++point_index;

    if ((distance = segment.distanceTo(*point)) >= max_distance) {
      double r = (*point).length();

      if (distance > p_max_split_distance_ + r * p_distance_proportion_) {
        max_distance = distance;
        set_divider = point;
        split_index = point_index;
      }
    }
  }

  // Split the set only if the sub-groups are not 'small'
  if (max_distance > 0.0 && split_index > p_min_group_points_ && split_index < point_set.num_points - p_min_group_points_) {
    set_divider = input_points_.insert(set_divider, *set_divider);  // Clone the dividing point for each group

    PointSet subset1, subset2;
    subset1.begin = point_set.begin;
    subset1.end = set_divider;
    subset1.num_points = split_index;
    subset1.is_visible = point_set.is_visible;

    subset2.begin = ++set_divider;
    subset2.end = point_set.end;
    subset2.num_points = point_set.num_points - split_index;
    subset2.is_visible = point_set.is_visible;

    detectSegments(subset1);
    detectSegments(subset2);
  } else {  // Add the segment
    if (!p_use_split_and_merge_)
      segment = fitSegment(point_set);

    segments_.push_back(segment);
  }
}

//void ObstacleExtractor::detectCirclesJBS(PointSet pointSet) {
//    if (point_set.num_points < p_min_group_points_)
//        return;
//
//    Circle mec;
//    int n = pointSet.num_points;
//
//
//
//
//    circles_fit_.push_back(Circle);
//
//}


void ObstacleExtractor::mergeSegments() {
  for (auto i = segments_.begin(); i != segments_.end(); ++i) {
    for (auto j = i; j != segments_.end(); ++j) {
      Segment merged_segment;

      if (compareSegments(*i, *j, merged_segment)) {
        auto temp_itr = segments_.insert(i, merged_segment);
        segments_.erase(i);
        segments_.erase(j);
        i = --temp_itr; // Check the new segment against others
        break;
      }
    }
  }
}

void ObstacleExtractor::mergeRects() {
    uint n = 0;
//    for (auto i = rectangles_.begin() ; i != rectangles_.end() ; ++i,n++)
//                printf("detected rect [%f,%f] (%d / %d) \n",
//                        i->center.x,i->center.y,n,rectangles_.size());
    if (rectangles_.size() > 1)
      for (auto i = rectangles_.begin() ; i != rectangles_.end() ; ++i)
        for(auto j = i ; j != rectangles_.end() ; ++j){
            Rectangle mergedRect;

            if (i!=j) {

                double angleDiff = abs(i->theta - j->theta);
                double minVertDist = i->distTo(*j);
//                printf("dist btw [%f,%f] and [%f,%f] =  %f\n",
//                        i->center.x,i->center.y,j->center.x,j->center.y,minVertDist);

                if ((angleDiff < p_max_merge_theta_diff or (M_PI / 2 - angleDiff) < p_max_merge_theta_diff) and
                    minVertDist < p_max_merge_rect_dist) {
                    mergedRect = sum(*i, *j);
                    auto temp_itr = rectangles_.insert(i, mergedRect);
                    rectangles_.erase(i);
                    rectangles_.erase(j);
                    i = --temp_itr;
                    break;
                }
            }
        }

}


bool ObstacleExtractor::compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment) {
  if (&s1 == &s2)
    return false;

  // Segments must be provided counter-clockwise
  if (s1.first_point.cross(s2.first_point) < 0.0)
    return compareSegments(s2, s1, merged_segment);

  if (checkSegmentsProximity(s1, s2)) {
    vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

    Segment segment = fitSegment(point_sets);

    if (checkSegmentsCollinearity(segment, s1, s2)) {
      merged_segment = segment;
      return true;
    }
  }

  return false;
}

bool ObstacleExtractor::checkSegmentsProximity(const Segment& s1, const Segment& s2) {
  return (s1.trueDistanceTo(s2.first_point) < p_max_merge_separation_ ||
          s1.trueDistanceTo(s2.last_point)  < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.first_point) < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.last_point)  < p_max_merge_separation_);
}

bool ObstacleExtractor::checkSegmentsCollinearity(const Segment& segment, const Segment& s1, const Segment& s2) {
  return (segment.distanceTo(s1.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s1.last_point)  < p_max_merge_spread_ &&
          segment.distanceTo(s2.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s2.last_point)  < p_max_merge_spread_);
}

void ObstacleExtractor::detectCircles() {
  for (auto segment = segments_.begin(); segment != segments_.end(); ++segment) {
    if (p_circles_from_visibles_) {
      bool segment_is_visible = true;
      for (const PointSet& ps : segment->point_sets) {
        if (!ps.is_visible) {
          segment_is_visible = false;
          break;
        }
      }
      if (!segment_is_visible)
        continue;
    }

    Circle circle(*segment);
    circle.radius += p_radius_enlargement_;

    if (circle.radius < p_max_circle_radius_) {
      circles_.push_back(circle);

      if (p_discard_converted_segments_) {
        segment = segments_.erase(segment);
        --segment;
      }
    }
  }
}

void ObstacleExtractor::mergeCircles() {
  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
    for (auto j = i; j != circles_.end(); ++j) {
      Circle merged_circle;

      if (compareCircles(*i, *j, merged_circle)) {
        auto temp_itr = circles_.insert(i, merged_circle);
        circles_.erase(i);
        circles_.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool ObstacleExtractor::compareCircles(const Circle& c1, const Circle& c2, Circle& merged_circle) {
  if (&c1 == &c2)
    return false;

  // If circle c1 is fully inside c2 - merge and leave as c2
  if (c2.radius - c1.radius >= (c2.center - c1.center).length()) {
    merged_circle = c2;
    return true;
  }

  // If circle c2 is fully inside c1 - merge and leave as c1
  if (c1.radius - c2.radius >= (c2.center - c1.center).length()) {
    merged_circle = c1;
    return true;
  }

  // If circles intersect and are 'small' - merge
  if (c1.radius + c2.radius >= (c2.center - c1.center).length()) {
    Point center = c1.center + (c2.center - c1.center) * c1.radius / (c1.radius + c2.radius);
    double radius = (c1.center - center).length() + c1.radius;

    Circle circle(center, radius);
    circle.radius += max(c1.radius, c2.radius);

    if (circle.radius < p_max_circle_radius_) {
      circle.point_sets.insert(circle.point_sets.end(), c1.point_sets.begin(), c1.point_sets.end());
      circle.point_sets.insert(circle.point_sets.end(), c2.point_sets.begin(), c2.point_sets.end());
      merged_circle = circle;
      return true;
    }
  }

  return false;
}

void ObstacleExtractor::publishObstacles() {



  obstacle_detector::ObstaclesPtr obstacles_msg(new obstacle_detector::Obstacles);
  obstacles_msg->header.stamp = stamp_;

  // for publishing box
  visualization_msgs::MarkerArray detectedBoxMsg;
  visualization_msgs::Marker boxBase;
  boxBase.header.frame_id = base_frame_id_;
  boxBase.color.r = 1.0;
  boxBase.color.g = 0.0;
  boxBase.color.b = 0.0;
  boxBase.color.a = 0.4;
  boxBase.header.stamp = stamp_;
  boxBase.type = visualization_msgs::Marker::CUBE;

  boxBase.action = 3;
  boxBase.id = 0;
  boxBase.ns = "LRects";
  detectedBoxMsg.markers.push_back(boxBase);

  if (p_transform_coordinates_) {
    tf::StampedTransform transform;

    try {
      tf_listener_.waitForTransform(p_frame_id_, base_frame_id_, stamp_, ros::Duration(0.1));
      tf_listener_.lookupTransform(p_frame_id_, base_frame_id_, stamp_, transform);
    }
    catch (tf::TransformException& ex) {
      ROS_INFO_STREAM(ex.what());
      return;
    }

    for (Segment& s : segments_) {
      s.first_point = transformPoint(s.first_point, transform);
      s.last_point = transformPoint(s.last_point, transform);
    }

    for (Circle& c : circles_)
      c.center = transformPoint(c.center, transform);

    obstacles_msg->header.frame_id = p_frame_id_;
  }
  else
    obstacles_msg->header.frame_id = base_frame_id_;


  for (const Segment& s : segments_) {
    SegmentObstacle segment;

    segment.first_point.x = s.first_point.x;
    segment.first_point.y = s.first_point.y;
    segment.last_point.x = s.last_point.x;
    segment.last_point.y = s.last_point.y;

    obstacles_msg->segments.push_back(segment);
  }

  for (const Circle& c : circles_) {
    if (c.center.x > p_min_x_limit_ && c.center.x < p_max_x_limit_ &&
        c.center.y > p_min_y_limit_ && c.center.y < p_max_y_limit_) {
        CircleObstacle circle;

        circle.center.x = c.center.x;
        circle.center.y = c.center.y;
        circle.velocity.x = 0.0;
        circle.velocity.y = 0.0;
        circle.radius = c.radius;
        circle.true_radius = c.radius - p_radius_enlargement_;

        obstacles_msg->circles.push_back(circle);
    }
  }

  // Detected box
  printf ("number of rects = %d \n",rectangles_.size());
  for (const Rectangle& c: rectangles_){
    if (c.center.x > p_min_x_limit_ && c.center.x < p_max_x_limit_ &&
        c.center.y > p_min_y_limit_ && c.center.y < p_max_y_limit_){
        c.loadMarkerPoseScale(boxBase);
        boxBase.action = 0;
        boxBase.id ++;
        boxBase.header.stamp = ros::Time::now();
        detectedBoxMsg.markers.push_back(boxBase);

    }
  }

  obstacles_pub_.publish(obstacles_msg);
  rect_pub_.publish(detectedBoxMsg);



}
