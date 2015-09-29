/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Kei Okada nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/lk_demo.cpp
/**
 * This is a demo of Lukas-Kanade optical flow lkdemo(),
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "std_srvs/Empty.h"
#include "opencv_apps/FlowArrayStamped.h"

namespace lk_flow {

class LKFlowNodelet : public nodelet::Nodelet
{
	ros::NodeHandle nh_, pnh_;

	image_transport::Subscriber img_sub_;
	ros::Publisher flows_pub_;
	ros::ServiceServer reset_points_service_;
	boost::shared_ptr<image_transport::ImageTransport> it_;

	int min_features_count_;
	int max_features_count_;
	int sub_pix_win_size_;
	int win_size_;

	std::vector<cv::Point2f> prev_points_;
	cv::Mat gray_, prev_gray_;

	int flows_subscriber_count_;

	int subscriberCount();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void subscribe();
	void unsubscribe();
	void flows_connectCb(const ros::SingleSubscriberPublisher& ssp);
	void flows_disconnectCb(const ros::SingleSubscriberPublisher&);

public:
	virtual void onInit();
};

int LKFlowNodelet::subscriberCount()
{
	return flows_subscriber_count_;
}

void LKFlowNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try {
		std::vector<cv::Point2f> found_points;
		std::vector<cv::Point2f> valid_points;

		// Convert the image into something opencv can handle.
		cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;

		// Messages
		opencv_apps::FlowArrayStamped flows_msg;
		flows_msg.header = msg->header;

		// Do the work
		cv::cvtColor( frame, gray_, cv::COLOR_BGR2GRAY );

		cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03); // TODO: make configurable
		cv::Size sub_pix_win_size(sub_pix_win_size_, sub_pix_win_size_);
		cv::Size win_size(win_size_, win_size_);

		if (prev_points_.size() < min_features_count_) {
			cv::goodFeaturesToTrack(gray_, prev_points_, max_features_count_, 0.01, 10, cv::Mat(), 3, 0, 0.04); // TODO: make configurable
			cv::cornerSubPix(gray_, prev_points_, sub_pix_win_size, cv::Size(-1,-1), termcrit);
		}

		if( !prev_points_.empty() ) {
			if(prev_gray_.empty()) {
				gray_.copyTo(prev_gray_);
			}

			std::vector<uchar> status;
			std::vector<float> err;

			cv::calcOpticalFlowPyrLK(prev_gray_, gray_, prev_points_, found_points, status, err, win_size, 3, termcrit, 0, 0.001);

			for(size_t i = 0; i < found_points.size(); i++ ) {
				if( !status[i] ) {
					continue;
				}
				opencv_apps::Flow flow_msg;
				opencv_apps::Point2D point_msg;
				opencv_apps::Point2D velocity_msg;
				point_msg.x = found_points[i].x;
				point_msg.y = found_points[i].y;
				velocity_msg.x = found_points[i].x - prev_points_[i].x;
				velocity_msg.y = found_points[i].y - prev_points_[i].y;
				flow_msg.point = point_msg;
				flow_msg.velocity = velocity_msg;
				flows_msg.flow.push_back(flow_msg);
				valid_points.push_back(found_points[i]);
			}

			flows_pub_.publish(flows_msg);
		}

		prev_points_ = valid_points;
		cv::swap(prev_gray_, gray_);

    } catch (cv::Exception &e) {
	    NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
}

void LKFlowNodelet::subscribe()
{
	NODELET_DEBUG("Subscribing to image topic.");
	img_sub_ = it_->subscribe("image", 10, &LKFlowNodelet::imageCallback, this);
}

void LKFlowNodelet::unsubscribe()
{
	NODELET_DEBUG("Unsubscribing from image topic.");
	img_sub_.shutdown();
}
void LKFlowNodelet::flows_connectCb(const ros::SingleSubscriberPublisher& ssp)
{
	flows_subscriber_count_++;
	if (subscriberCount() == 1) {
		subscribe();
	}
}

void LKFlowNodelet::flows_disconnectCb(const ros::SingleSubscriberPublisher&)
{
	flows_subscriber_count_--;
	if (subscriberCount() == 0) {
		unsubscribe();
	}
}

void LKFlowNodelet::onInit()
{
	nh_ = getNodeHandle();
	pnh_ = getPrivateNodeHandle();

	pnh_.param("min_features_count", min_features_count_, 100);
	pnh_.param("max_features_count", max_features_count_, 500);
	pnh_.param("sub_pix_win_size", sub_pix_win_size_, 10);
	pnh_.param("win_size", win_size_, 31);

	it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));

	flows_subscriber_count_ = 0;

	ros::SubscriberStatusCallback flows_connect_cb    = boost::bind(&LKFlowNodelet::flows_connectCb, this, _1);
	ros::SubscriberStatusCallback flows_disconnect_cb = boost::bind(&LKFlowNodelet::flows_disconnectCb, this, _1);

	flows_pub_ = nh_.advertise<opencv_apps::FlowArrayStamped>("flows", 1, flows_connect_cb, flows_disconnect_cb);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lk_flow::LKFlowNodelet, nodelet::Nodelet);
