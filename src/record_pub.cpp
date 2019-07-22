#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

std::vector<cv::Mat> video_to_img(std::string video_path)
{
	cv::VideoCapture cap(video_path.c_str());
	bool video_loaded = false;
	std::vector<cv::Mat> images;
	std::cout<<"start loading video."<<std::endl;
	while (!video_loaded) 
	{
		cv::Mat frame;
		cap >> frame;
		if (frame.empty())
		{
      break;
		}
		images.push_back(frame);
		//cv::imshow("Frame", frame);
		//cv::waitKey(1);
  }
	std::cout<<"video loaded. "<<images.size()<<" images in total."<<std::endl;
	cap.release();
	//cv::destroyWindow("Frame");
	video_loaded = true;
	return images;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "record_pub");
  ros::NodeHandle nh("~");

	//paths
	std::string video_path;
	std::string img_path;
	std::string gaze_path;
	std::string fixation_path;
	std::stringstream img_name;

	nh.param("video_path", video_path, std::string(""));
	video_path += "/world.mp4";

	nh.param("image_path", img_path, std::string(""));

	nh.param("gaze_path", gaze_path, std::string(""));
	gaze_path += "/gaze_positions.csv";

	nh.param("fixation_path", fixation_path, std::string(""));
	fixation_path += "/fixations.csv";
	//std::cout<<gaze_path.c_str()<<std::endl;

	//publishers
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_img = it.advertise("/pupil_capture/world", 5);
	ros::Publisher pub_gaze = nh.advertise<geometry_msgs::PointStamped>("/pupil_capture/gaze_std", 5);
	ros::Publisher pub_fixation = nh.advertise<geometry_msgs::PointStamped>("/pupil_capture/fixation", 5);
	
  //msgs
	geometry_msgs::PointStamped msg_gaze_std, msg_fixation;	
  sensor_msgs::ImagePtr msg_img;

	//read img from video
	std::vector<cv::Mat> images = video_to_img(video_path);

	//load gaze data
	
	
	//load fixation data
	

  ros::Rate loop_rate(50);
	int index = 0;
  while (nh.ok()) 
	{
		//std::cout<<"iter"<<index<<std::endl;
		if(index < images.size())
		{
			msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", images[index]).toImageMsg();
			msg_img->header.stamp = ros::Time::now();
		}
		else
		{	
			std::cout<<"all images published."<<std::endl;
			break;
		}
    pub_img.publish(msg_img);
		//pub_gaze.publish(msg_gaze_std);
		//pub_fixation.publish(msg_fixation);
		
		index += 1;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
