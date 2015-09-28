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

/*
 * This program visualizes optical flow fields.
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "opencv_apps/FlowArrayStamped.h"

namespace flow_drawer {

class FlowDrawerNodelet : public nodelet::Nodelet
{
	boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, opencv_apps::FlowArrayStamped> > sync_;
	boost::shared_ptr<image_transport::ImageTransport> it_;

	image_transport::SubscriberFilter img_sub_;
	message_filters::Subscriber<opencv_apps::FlowArrayStamped> flows_sub_;

	image_transport::Publisher img_pub_;

	void imageFlowCallback(const sensor_msgs::ImageConstPtr& img_msg, const opencv_apps::FlowArrayStampedConstPtr& flows_msg);

public:
	virtual void onInit();
};

void FlowDrawerNodelet::imageFlowCallback(const sensor_msgs::ImageConstPtr& img_msg, const opencv_apps::FlowArrayStampedConstPtr& flows_msg)
{
	bool is_dense = img_msg->width * img_msg->height == flows_msg->flow.size();

	try {
		cv::Mat frame = cv_bridge::toCvCopy(img_msg, img_msg->encoding)->image;
		cv::Scalar color = cv::Scalar(0, 255, 0);

		if (is_dense) {
			int step = 16;
			for(int y = 0; y < img_msg->height; y += step) {
				for(int x = 0; x < img_msg->width; x += step) {
					const opencv_apps::Flow& f = flows_msg->flow[x + y * img_msg->width];
					const float fx = f.point.x;
					const float fy = f.point.y;
					const float dx = f.velocity.x;
					const float dy = f.velocity.y;
					cv::line(frame, cv::Point(fx, fy), cv::Point(cvRound(fx+dx), cvRound(fy+dy)), color);
					cv::circle(frame, cv::Point(fx,fy), 2, color, -1);
				}
			}
		} else {
			for(int i = 0; i < flows_msg->flow.size(); ++i) {
				const opencv_apps::Flow& f = flows_msg->flow[i];
				const float fx = f.point.x;
				const float fy = f.point.y;
				const float dx = f.velocity.x;
				const float dy = f.velocity.y;
				cv::line(frame, cv::Point(fx, fy), cv::Point(cvRound(fx+dx), cvRound(fy+dy)), color);
				cv::circle(frame, cv::Point(fx,fy), 2, color, -1);
			}
		}

		sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(img_msg->header, img_msg->encoding, frame).toImageMsg();
		img_pub_.publish(out_img);
	} catch (cv::Exception &e) {
		NODELET_ERROR("Processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
	}
}

void FlowDrawerNodelet::onInit()
{
	it_.reset(new image_transport::ImageTransport(getNodeHandle()));

    img_pub_ = it_->advertise("image_flows", 1);

    img_sub_.subscribe(*it_, "image", 1);
    flows_sub_.subscribe(getNodeHandle(), "flows", 1);

    sync_.reset(new message_filters::TimeSynchronizer<sensor_msgs::Image, opencv_apps::FlowArrayStamped>(img_sub_, flows_sub_, 20));
    sync_->registerCallback( boost::bind( &FlowDrawerNodelet::imageFlowCallback, this, _1, _2 ) );
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(flow_drawer::FlowDrawerNodelet, nodelet::Nodelet);
