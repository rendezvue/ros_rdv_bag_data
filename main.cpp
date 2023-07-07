#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/date_time/gregorian/greg_date.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ros_rdv_bag_data");
	ros::NodeHandle n;

	//파라미터를 이용하여 경로 정보 얻기
	std::string str_path_src, str_path_dst ;
	n.getParam("/ros_rdv_bag_data/path_src", str_path_src) ;
	n.getParam("/ros_rdv_bag_data/path_dst", str_path_dst) ;

	printf("src = %s\n", str_path_src.c_str() ) ;
	printf("dst = %s\n", str_path_dst.c_str() ) ;

	//bag파일 읽기
	rosbag::Bag bag;
	bag.open(str_path_src, rosbag::bagmode::Read);  // BagMode is Read by default
	printf("read : %s\n", str_path_src.c_str()) ;
	
	//bag파일의 토픽리스트 얻기
	rosbag::View view(bag);
	std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
	std::vector<std::string> topic_list;
	
	BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos) 
	{
	  topic_list.push_back(info->topic);
	}

	//토픽 리스트 출력
	int size_topic_list = topic_list.size() ;
	printf("\n\nChecked %d topics from %s file ----- \n", size_topic_list, str_path_src.c_str()) ;
	for( int i=0 ; i<size_topic_list ; i++ )
	{
		printf("Topic : %s\n", topic_list[i].c_str()) ;
	}
	printf("-----\n") ;
	
	

    std::string rgb_cam_image = "arena_camera_node/image_raw";
	std::string nir_cam_image = "/arena_camera_node_2/image_raw";
	std::string stereo_cam_left_image = "/zed2i/zed_node/left/image_rect_color";
	std::string stereo_cam_right_image = "/zed2i/zed_node/right/image_rect_color";

    std::vector<std::string> topics;
    topics.push_back(rgb_cam_image);
    topics.push_back(nir_cam_image);
    topics.push_back(stereo_cam_left_image);
    topics.push_back(stereo_cam_right_image);

	int count = 0 ;
	for(rosbag::MessageInstance const m: rosbag::View(bag))
	{
		if (m.getTopic() == rgb_cam_image) 
		{
		#if 1
			sensor_msgs::Image::ConstPtr rgb_image_ptr = m.instantiate<sensor_msgs::Image>() ;
				
			cv_bridge::CvImagePtr cv_ptr;
			cv::Mat image ;
			try 
			{		
				cv_ptr = cv_bridge::toCvCopy(rgb_image_ptr, sensor_msgs::image_encodings::RGB8);
				cv_ptr->image.copyTo(image) ;

				if( !image.empty() )
				{
					//png이미지 저장
					std::string str_write_path = str_path_dst + "/rgb_" + std::to_string(count) + ".png" ;
					cv::imwrite(str_write_path, image) ;
				}
			}
			catch ( cv_bridge::Exception& e ) 
			{
				ROS_ERROR("error! rgb_cam_image cv_bridge exception: %s", e.what() );
			}
		#endif
		}

		if (m.getTopic() == nir_cam_image) 
		{
			sensor_msgs::Image::ConstPtr rgb_image_ptr = m.instantiate<sensor_msgs::Image>() ;
				
			cv_bridge::CvImagePtr cv_ptr;
			cv::Mat image ;
			try 
			{		
				cv_ptr = cv_bridge::toCvCopy(rgb_image_ptr, sensor_msgs::image_encodings::MONO8);
				if( cv_ptr )
				{
					cv_ptr->image.copyTo(image) ;

					if( !image.empty() )
					{
						//png이미지 저장
						std::string str_write_path = str_path_dst + "/nir_" + std::to_string(count) + ".png" ;
						cv::imwrite(str_write_path, image) ;
					}
				}
			}
			catch ( cv_bridge::Exception& e ) 
			{
				ROS_ERROR("error! rgb_cam_image cv_bridge exception: %s", e.what() );
			}
		}

		if (m.getTopic() == stereo_cam_left_image) 
		{
			sensor_msgs::Image::ConstPtr rgb_image_ptr = m.instantiate<sensor_msgs::Image>() ;
				
			cv_bridge::CvImagePtr cv_ptr;
			cv::Mat image ;
			try 
			{		
				cv_ptr = cv_bridge::toCvCopy(rgb_image_ptr, sensor_msgs::image_encodings::RGB8);
				cv_ptr->image.copyTo(image) ;

				if( !image.empty() )
				{
					//png이미지 저장
					std::string str_write_path = str_path_dst + "/left_" + std::to_string(count) + ".png" ;
					cv::imwrite(str_write_path, image) ;
				}
			}
			catch ( cv_bridge::Exception& e ) 
			{
				ROS_ERROR("error! left cv_bridge exception: %s", e.what() );
			}
		}

		if (m.getTopic() == stereo_cam_right_image) 
		{
			sensor_msgs::Image::ConstPtr rgb_image_ptr = m.instantiate<sensor_msgs::Image>() ;
				
			cv_bridge::CvImagePtr cv_ptr;
			cv::Mat image ;
			try 
			{		
				cv_ptr = cv_bridge::toCvCopy(rgb_image_ptr, sensor_msgs::image_encodings::RGB8);
				cv_ptr->image.copyTo(image) ;

				if( !image.empty() )
				{
					//png이미지 저장
					std::string str_write_path = str_path_dst + "/right_" + std::to_string(count) + ".png" ;
					cv::imwrite(str_write_path, image) ;
				}
			}
			catch ( cv_bridge::Exception& e ) 
			{
				ROS_ERROR("error! right cv_bridge exception: %s", e.what() );
			}
		}

		count++ ;

	}

	printf("EoF\n") ;

	bag.close();
	ros::shutdown();
	
	return 0 ;
}
