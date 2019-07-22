#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <fstream>
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
}

std::vector<cv::Point2f> format_pupil_labs_gaze(std::string gaze_path)
{
	std::vector<cv::Point2f> gaze;
	std::ifstream ifs (gaze_path.c_str());
	std::string line;
	std::vector<std::vector<std::string> > values, new_values;
	while(std::getline(ifs, line))
	{
	  std::string line_value;
	  std::vector<std::string> line_values;
	  std::stringstream ss(line);
	  while(std::getline(ss, line_value, ','))
	  {
	    line_values.push_back(line_value);
	  }
	  values.emplace_back(line_values);
	}
  for(int i=1; i<values.size(); i++)
	{
		if(i<values.size()-1)
		{
			if( std::stod(values[i][1])!= std::stod(values[i+1][1]))
			{
				new_values.push_back(values[i]);
			}
		}
		if(i==values.size()-1)
		{
			new_values.push_back(values[i]);
		}
	}
	ifs.close();
	
	for(int j=0;j<new_values.size();j++)
	{
		gaze.push_back( cv::Point2f(std::stof(new_values[j][3]), std::stof(new_values[j][4])) );
		//std::cout<<"x: "<<new_values[j][3]<<" y: "<<new_values[j][4]<<std::endl;
	}
	return gaze;
}

std::vector<cv::Point2f> format_pupil_labs_fixation(std::string fixation_path)
{
	std::vector<cv::Point2f> fixation;
	std::ifstream ifs (fixation_path.c_str());
	std::string line;
	std::vector<std::vector<std::string> > values, new_values;
	while(std::getline(ifs, line))
	{
	  std::string line_value;
	  std::vector<std::string> line_values;
	  std::stringstream ss(line);
	  while(std::getline(ss, line_value, ','))
	  {
	    line_values.push_back(line_value);
	  }
	  values.emplace_back(line_values);
	}
	int index = 1;
  for(int i=0; i<std::stoi(values[values.size()-1][4]); i++)
	{
		if(i>=std::stod(values[index][3]) && i<=std::stod(values[index][4]))
		{
			fixation.push_back(cv::Point2f(std::stod(values[index][5]), std::stod(values[index][6])) );
		}
		else
		{
			if(i>std::stod(values[index][4]))
			{
				index += 1;
			}
			fixation.push_back(cv::Point2f(0.0, 0.0));
		}	
	}
	ifs.close();
	return fixation;
}

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
	std::cout<<gaze_path.c_str()<<std::endl;

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
	std::vector<cv::Point2f> gaze_pos = format_pupil_labs_gaze(gaze_path.c_str());
	
	//load fixation data
	std::vector<cv::Point2f> fixation_pos = format_pupil_labs_fixation(fixation_path);

  ros::Rate loop_rate(50);
	int index = 0;
	int length = std::min({images.size(), gaze_pos.size()});
  while (nh.ok()) 
	{
		//std::cout<<"iter"<<index<<std::endl;
		if(index < length)
		{
			if(index < fixation_pos.size())
			{
				if(fixation_pos[index].x != 0.0)
				{
					msg_fixation.point.x = fixation_pos[index].x;
					msg_fixation.point.y = fixation_pos[index].y;
					msg_fixation.header.stamp = ros::Time::now();
					pub_fixation.publish(msg_fixation);
				}
			}
	
			msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", images[index]).toImageMsg();
			msg_img->header.stamp = ros::Time::now();

			msg_gaze_std.point.x = gaze_pos[index].x;
			msg_gaze_std.point.y = gaze_pos[index].y;
			msg_gaze_std.header.stamp = ros::Time::now();
		}
		else
		{	
			std::cout<<"all messages published."<<std::endl;
			break;
		}
    pub_img.publish(msg_img);
		pub_gaze.publish(msg_gaze_std);

		index += 1;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
