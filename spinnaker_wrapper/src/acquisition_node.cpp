//ros
#include <ros/ros.h>
#include <ros/package.h>

//opencv
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// #include <sensor_msgs/image_encodings.h>

//spinnaker
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <spinnaker_wrapper/spinnaker_wrapper.hpp>

//other
#include <iostream>
#include <sstream>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

image_transport::Publisher img_pub_;
cv::Mat image_;

void callback(const ros::TimerEvent& event)
{
	if(image_.data)
    {	
    	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
		img_pub_.publish(img_msg);

	}
}

int main(int argc, char** argv)
{
	// Set up ROS
	ros::init(argc, argv, "stream_example_node");
	ros::NodeHandle nh;

	ROS_INFO("stream_example_node running...");

	// Image publisher
	image_transport::ImageTransport it(nh);
	img_pub_ = it.advertise("spinnaker/image", 1);

	// Timer for image publisher (10 Hz)
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), callback);

	//create object
	ROS_Spinnaker spinnaker;

	/********* Old Code *********/
	// if(spinnaker.start())
	// {
	// 	ROS_ERROR("Error! Could not initialize any cameras!");
	// 	return 1;
	// }
	
	// spinnaker.acquire_images();

	/********* New Code *********/
	spinnaker.load_cameras();
	spinnaker.init();
	spinnaker.set_enum_value("AcquisitionMode", "Continuous");
	spinnaker.set_buffer_size(1);
	spinnaker.begin_acquisition();

	// ros::Rate loop_rate(10);
	while(ros::ok())
	{
		//grab frame
		image_ = spinnaker.grab_frame();
		
		// //display frame
		// cv::namedWindow("current Image", CV_WINDOW_NORMAL);
		// cv::imshow("current Image", image);

		// char keyPress = cv::waitKey(1);
		// if(keyPress == 'q')
		// {
		//     ROS_INFO("Exiting main loop...");
		//     cv::destroyWindow("current Image");
		//     break;
		// }

		//publish image (instead use timer to set rate)
		// sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		// img_pub.publish(img_msg);

		//spin
		ros::spinOnce();
		// loop_rate.sleep();
	}

	spinnaker.end_acquisition();
	spinnaker.deinit();

	return 0;
}
