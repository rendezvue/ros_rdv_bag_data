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

void Image_File_Write(sensor_msgs::Image::ConstPtr image_ptr, std::string str_name)
{
	//printf("image_ptr encoding = %s\n", image_ptr->encoding.c_str()) ;
										
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat image ;
	try 
	{		
		std::string str_encoding = image_ptr->encoding;
		
		//if( str_encoding == "bgr8" || str_encoding == "rgb8" )
		
		cv_ptr = cv_bridge::toCvCopy(image_ptr, str_encoding);
		cv_ptr->image.copyTo(image) ;

		if( !image.empty() )
		{
			cv::imwrite(str_name, image) ;
		}
	}
	catch ( cv_bridge::Exception& e ) 
	{
		ROS_ERROR("error! cv_bridge exception: %s - %s", e.what(), str_name.c_str() );
	}
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ros_rdv_bag_data");
	ros::NodeHandle n;

	//파라미터를 이용하여 경로 정보 얻기
	std::string str_path_src, str_path_dst ;
	n.getParam("/ros_rdv_bag_data/path_src", str_path_src) ;
	n.getParam("/ros_rdv_bag_data/path_dst", str_path_dst) ;
	int hz = -1 ;
	n.getParam("/ros_rdv_bag_data/hz", hz) ;
	double duration = -1 ;
	if( hz > 0 )
	{
		duration = 1.0 / (double)hz ;
	}

	printf("\n\nParam -----\n") ;
	printf("src = %s\n", str_path_src.c_str() ) ;
	printf("dst = %s\n", str_path_dst.c_str() ) ;
	printf("hz = %d\n", hz ) ;
	printf(" - diration = %f\n", duration ) ;
	printf("-----\n") ;
		
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

	//bag파일의 timestamp정보
	ros::Time btime = view.getBeginTime();
    ros::Time etime = view.getEndTime();

	std::string str_btime = std::to_string(btime.sec) + "." + std::to_string(btime.nsec) ;
	std::string str_etime = std::to_string(etime.sec) + "." + std::to_string(etime.nsec) ;
	double dbl_btime = std::stod(str_btime) ;
	double dbl_etime = std::stod(str_etime) ;
	
	printf("\n\nTime Stamp -----\n") ;
	printf("start time : %s(%f)\n", str_btime.c_str(), dbl_btime) ;
	printf("end time : %s(%f)\n", str_etime.c_str(), dbl_etime) ;
	printf("-----\n") ;
	
	//ros::Time stamp = ros::Time::now();
	//std::stringstream ss;
	//ss << stamp.sec << "." << stamp.nsec;
	//std::cout << ss.str() << std::endl;


	//기본 파일 정보 
	int ifind = str_path_src.rfind('/') ;
	std::string str_base_info ;
	if( ifind >= 0 )
	{
		int len = str_path_src.size() - ifind ;
		str_base_info = str_path_src.substr(ifind+1, len) ;
	}
	
	printf("bag file name : %s\n", str_base_info.c_str()) ;

	if( !str_base_info.empty() )
	{
		std::string str_only_file_name = boost::filesystem::change_extension(str_base_info, "").string();
		printf("base info : %s\n", str_only_file_name.c_str()) ;

		//저장하고자 하는 토픽 리스트
		std::string rgb_cam_image = "/arena_camera_node/image_raw";
		std::string nir_cam_image = "/arena_camera_node_2/image_raw";
		std::string stereo_cam_left_image = "/zed2i/zed_node/left/image_rect_color";
		std::string stereo_cam_right_image = "/zed2i/zed_node/right/image_rect_color"; 
		
		std::vector<std::pair<std::string, double>> vec_topic;
		vec_topic.push_back(std::make_pair(rgb_cam_image, dbl_btime)) ;
		vec_topic.push_back(std::make_pair(nir_cam_image, dbl_btime)) ;
		vec_topic.push_back(std::make_pair(stereo_cam_left_image, dbl_btime)) ;
		vec_topic.push_back(std::make_pair(stereo_cam_right_image, dbl_btime)) ;

		for(rosbag::MessageInstance const m: rosbag::View(bag))
		{
			//현재 ros timestamp 읽기
			ros::Time mytime = m.getTime() ;
			std::string str_mytime = std::to_string(mytime.sec) + "." + std::to_string(mytime.nsec) ;
			double dbl_mytime = std::stod(str_mytime) ;
			
			std::string str_topic_name = m.getTopic();			
			std::vector<std::pair<std::string, double>>::iterator it = find_if(vec_topic.begin(), vec_topic.end(), [&str_topic_name](const std::pair<std::string, double>& elem){ return elem.first == str_topic_name; });
			if (it != vec_topic.end())
			{
				//내가 저장했던 시간과 현재 시간 간격을 체크
				double dbl_save_time_duration = dbl_mytime - it->second ;

				//그 간격이 사용자가 설정한 hz의 duration보다 크다면 저장한다. 
				if( dbl_save_time_duration >= duration )
				{
					//시간 간격을 체크하기 위한 저장 시간 저장
					it->second = dbl_mytime ;

					std::string str_sensor_name ;
					std::size_t found = str_topic_name.find("arena_camera_node/");
					if (found!=std::string::npos)	str_sensor_name = "rgb" ;
					found = str_topic_name.find("arena_camera_node_2/");
					if (found!=std::string::npos)	str_sensor_name = "nir" ;
					found = str_topic_name.find("zed_node/left/");
					if (found!=std::string::npos)	str_sensor_name = "left" ;
					found = str_topic_name.find("zed_node/right/");
					if (found!=std::string::npos)	str_sensor_name = "right" ;

					std::string str_write_path = str_path_dst + "/" + str_only_file_name + "_" + str_sensor_name + "_" + std::to_string(dbl_mytime) + ".png" ;

					sensor_msgs::Image::ConstPtr image_ptr = m.instantiate<sensor_msgs::Image>() ;
					
					Image_File_Write(image_ptr, str_write_path);
				}

			}
		}

		printf("EoF\n") ;
	}

	bag.close();
	ros::shutdown();
	
	return 0 ;
}
