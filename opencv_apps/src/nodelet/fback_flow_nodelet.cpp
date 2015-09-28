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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/fback.cpp
/*
 * This program demonstrates dense optical flow algorithm by Gunnar Farneback
 * Mainly the function: calcOpticalFlowFarneback()
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/video/tracking.hpp>

#include "opencv_apps/FlowArrayStamped.h"

namespace fback_flow {

class FBackFlowNodelet : public nodelet::Nodelet
{
private:
	image_transport::Subscriber img_sub_;
	ros::Publisher flows_pub_;

	boost::shared_ptr<image_transport::ImageTransport> it_;
	ros::NodeHandle nh_, pnh_;

	int flows_subscriber_count_;

	cv::Mat prevgray_, gray_, flow_;

	double pyr_scale_;
	int levels_;
	int winsize_;
	int iterations_;
	int poly_n_;
	double poly_sigma_;

	int subscriberCount();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void subscribe();
	void unsubscribe();
	void flows_connectCb(const ros::SingleSubscriberPublisher& ssp);
	void flows_disconnectCb(const ros::SingleSubscriberPublisher&);

public:
	virtual void onInit();
};

int FBackFlowNodelet::subscriberCount()
{
	return flows_subscriber_count_;
}

void FBackFlowNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	// Work on the image.
	try {
		// Convert the image into something opencv can handle.
		cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;

		// Messages
		opencv_apps::FlowArrayStamped flows_msg;
		flows_msg.header = msg->header;

		// Do the work
		cv::cvtColor( frame, gray_, cv::COLOR_BGR2GRAY );

		if( prevgray_.data && subscriberCount() > 0) {
			cv::calcOpticalFlowFarneback(prevgray_, gray_, flow_
			                             , pyr_scale_
			                             , levels_
			                             , winsize_
			                             , iterations_
			                             , poly_n_
			                             , poly_sigma_
			                             , 0 // flags
				);

			int step = 1;
			for(int y = 0; y < flow_.rows; y += step) {
				for(int x = 0; x < flow_.cols; x += step) {
					const cv::Point2f& fxy = flow_.at<cv::Point2f>(y, x);
					if (flows_subscriber_count_ > 0) {
						opencv_apps::Flow flow_msg;
						opencv_apps::Point2D point_msg;
						opencv_apps::Point2D velocity_msg;
						point_msg.x = x;
						point_msg.y = y;
						velocity_msg.x = fxy.x;
						velocity_msg.y = fxy.y;
						flow_msg.point = point_msg;
						flow_msg.velocity = velocity_msg;
						flows_msg.flow.push_back(flow_msg);
					}
				}
			}
		}

		std::swap(prevgray_, gray_);

		if (flows_subscriber_count_ > 0 && flows_msg.flow.size() > 0) {
			// Publish the flows.
			flows_pub_.publish(flows_msg);
		}
	} catch (cv::Exception &e) {
		NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
	}
}

void FBackFlowNodelet::subscribe()
{
	NODELET_DEBUG("Subscribing to image topic.");
	img_sub_ = it_->subscribe("image", 10, &FBackFlowNodelet::imageCallback, this);
}

void FBackFlowNodelet::unsubscribe()
{
	NODELET_DEBUG("Unsubscribing from image topic.");
	img_sub_.shutdown();
}

void FBackFlowNodelet::flows_connectCb(const ros::SingleSubscriberPublisher& ssp)
{
	flows_subscriber_count_++;
	if (subscriberCount() == 1) {
		subscribe();
	}
}

void FBackFlowNodelet::flows_disconnectCb(const ros::SingleSubscriberPublisher&)
{
	flows_subscriber_count_--;
	if (subscriberCount() == 0) {
		unsubscribe();
	}
}

void FBackFlowNodelet::onInit()
{
	nh_ = getNodeHandle();
	pnh_ = getPrivateNodeHandle();

	pnh_.param("pyr_scale", pyr_scale_, 0.5);
	pnh_.param("levels", levels_, 3);
	pnh_.param("winsize", winsize_, 15);
	pnh_.param("iterations", iterations_, 3);
	pnh_.param("poly_n", poly_n_, 5);
	pnh_.param("poly_sigma", poly_sigma_, 1.2);
	
	it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));

	flows_subscriber_count_ = 0;

	ros::SubscriberStatusCallback flows_connect_cb    = boost::bind(&FBackFlowNodelet::flows_connectCb, this, _1);
	ros::SubscriberStatusCallback flows_disconnect_cb = boost::bind(&FBackFlowNodelet::flows_disconnectCb, this, _1);

	flows_pub_ = nh_.advertise<opencv_apps::FlowArrayStamped>("flows", 1, flows_connect_cb, flows_disconnect_cb);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fback_flow::FBackFlowNodelet, nodelet::Nodelet);
