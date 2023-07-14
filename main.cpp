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
#include <boost/locale.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

typedef struct FileInfo_
{
	std::string str_device_name ;
	std::string str_date ;
	std::string str_time ;
} FileInfo ;

std::vector<std::string> Split(const std::string& s, char seperator)
{
	std::vector<std::string> output;

	std::string::size_type prev_pos = 0, pos = 0;

	while ((pos = s.find(seperator, pos)) != std::string::npos)
	{
		std::string substring(s.substr(prev_pos, pos - prev_pos));

		if( !substring.empty() )	output.push_back(substring);

		prev_pos = ++pos;
	}

	int last_len = pos - prev_pos ;
	if( last_len > 0 )	output.push_back(s.substr(prev_pos, pos - prev_pos)); // Last word

	return output;
}


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

	//parsing해야 하는 bag파일 리스트 수집
	std::vector<std::string> vec_str_path_src ;
	
	//src가 폴더인지 bag파일인지 확인
	if(boost::filesystem::exists(str_path_src))
	{
		if( boost::filesystem::is_directory(str_path_src) )	//폴더라면 내부에 bag파일을 모두 수집한다.
		{
			for (auto const & entry : boost::filesystem::recursive_directory_iterator(str_path_src))
	        {
	            if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ".bag")
	            {
	            	std::string str_bag_path = entry.path().string() ;
					vec_str_path_src.push_back(str_bag_path ) ;
	            }
	        }
		}
		else
		{
			vec_str_path_src.push_back(str_path_src) ;
		}
	}
	    
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

	unsigned int data_length_by_sec = 0 ;
	if( !str_base_info.empty() )
	{
		std::string str_only_file_name = boost::filesystem::change_extension(str_base_info, "").string();
		printf("base info : %s\n", str_only_file_name.c_str()) ;

		//총 몇초짜리지??
		data_length_by_sec = (unsigned int)(dbl_etime - dbl_btime) ;
		printf(" - Total %d sec.\n", data_length_by_sec) ;	
		
		//파일명에서 정보 확인
		FileInfo fileinfo ;
		std::vector<std::string> vec_str_base_info = Split(str_only_file_name, '_') ;
		int size_str_base_info = vec_str_base_info.size() ;
		if( size_str_base_info > 0 )
		{
			if( size_str_base_info > 0)	fileinfo.str_device_name = vec_str_base_info[0] ;
			if( size_str_base_info > 1)	fileinfo.str_date = vec_str_base_info[1] ;
			if( size_str_base_info > 2)	fileinfo.str_time = vec_str_base_info[2] ;
					
			for( int i=0 ; i<size_str_base_info ; i++ )
			{
				printf(" - [%d] %s\n", i, vec_str_base_info[i].c_str()) ;	
			}

			printf("\n\n") ;	
		}		

		//저장하고자 하는 토픽 리스트
		std::string rgb_cam_image = "/arena_camera_node/image_raw";
		std::string nir_cam_image = "/arena_camera_node_2/image_raw";
		std::string stereo_cam_left_image = "/zed2i/zed_node/left/image_rect_color";
		std::string stereo_cam_right_image = "/zed2i/zed_node/right/image_rect_color"; 

		unsigned int count_rgb_cam_image = 0;
		unsigned int count_nir_cam_image = 0;
		unsigned int count_stereo_cam_left_image = 0;
		unsigned int count_stereo_cam_right_image = 0; 
		
		std::vector<std::pair<std::string, double>> vec_topic;
		vec_topic.push_back(std::make_pair(rgb_cam_image, dbl_btime)) ;
		vec_topic.push_back(std::make_pair(nir_cam_image, dbl_btime)) ;
		vec_topic.push_back(std::make_pair(stereo_cam_left_image, dbl_btime)) ;
		vec_topic.push_back(std::make_pair(stereo_cam_right_image, dbl_btime)) ;

		std::vector<unsigned int> vec_topic_count;
		vec_topic_count.resize(vec_topic.size()) ;

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
				std::string str_sensor_name ;
				std::size_t found = str_topic_name.find("arena_camera_node/");
				if (found!=std::string::npos)
				{
					str_sensor_name = "rgb" ;
					count_rgb_cam_image++ ;
				}
				
				found = str_topic_name.find("arena_camera_node_2/");
				if (found!=std::string::npos)
				{
					str_sensor_name = "nir" ;
					count_nir_cam_image++ ;
				}

				found = str_topic_name.find("zed_node/left/");
				if (found!=std::string::npos)
				{
					str_sensor_name = "left" ;
					count_stereo_cam_left_image++ ;
				}

				found = str_topic_name.find("zed_node/right/");
				if (found!=std::string::npos)
				{
					str_sensor_name = "right" ;
					count_stereo_cam_right_image++ ;
				}
				
				//내가 저장했던 시간과 현재 시간 간격을 체크
				double dbl_save_time_duration = dbl_mytime - it->second ;

				//그 간격이 사용자가 설정한 hz의 duration보다 크다면 저장한다. 
				if( dbl_save_time_duration >= duration )
				{
					//시간 간격을 체크하기 위한 저장 시간 저장
					it->second = dbl_mytime ;
	
					std::string str_write_path = str_path_dst + "/" + str_only_file_name + "_" + str_sensor_name + "_" + std::to_string(dbl_mytime) + ".png" ;

					sensor_msgs::Image::ConstPtr image_ptr = m.instantiate<sensor_msgs::Image>() ;
					
					Image_File_Write(image_ptr, str_write_path);
				}

			}
		}

		//csv파일 저장
		std::string str_haead = std::string("DAQ Number, 장소, 날짜, 시간, 센서 이름, 수집 수량, 학습유효 수량");
		std::string str_rgb_info = fileinfo.str_device_name + "," + "" + "," + fileinfo.str_date + "," + fileinfo.str_time + "," + "rgb" + "," + std::to_string(count_rgb_cam_image) + "," + std::to_string(data_length_by_sec) ;
		std::string str_nir_info = fileinfo.str_device_name + "," + "" + "," + fileinfo.str_date + "," + fileinfo.str_time + "," + "nir" + "," + std::to_string(count_rgb_cam_image) + "," + std::to_string(data_length_by_sec) ;
		std::string str_left_info = fileinfo.str_device_name + "," + "" + "," + fileinfo.str_date + "," + fileinfo.str_time + "," + "stereo left" + "," + std::to_string(count_rgb_cam_image) + "," + std::to_string(data_length_by_sec) ;
		std::string str_right_info = fileinfo.str_device_name + "," + "" + "," + fileinfo.str_date + "," + fileinfo.str_time + "," + "stereo right" + "," + std::to_string(count_rgb_cam_image) + "," + std::to_string(data_length_by_sec) ;
		
			
		std::ofstream outfile;
		std::string str_csv_write_path = str_path_dst + "/" + str_only_file_name + ".csv" ;
		outfile.open (str_csv_write_path);
		outfile << boost::locale::conv::between(str_haead, "EUC-KR", "UTF-8") << std::endl ;
		outfile << boost::locale::conv::between(str_rgb_info, "EUC-KR", "UTF-8") << std::endl ;
		outfile << boost::locale::conv::between(str_nir_info, "EUC-KR", "UTF-8") << std::endl ;
		outfile << boost::locale::conv::between(str_left_info, "EUC-KR", "UTF-8") << std::endl ;
		outfile << boost::locale::conv::between(str_right_info, "EUC-KR", "UTF-8") << std::endl ;
		outfile.close();

		printf("Write CSV File = %s\n\n\n", str_csv_write_path.c_str()) ;

		printf("EoF\n") ;
		printf("\n\nComplete -----\n") ;
		printf("hm... Ctrl + C\n\n\n\n") ;
	}


	bag.close();
	ros::shutdown();
	
	return 0 ;
}
