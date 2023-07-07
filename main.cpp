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

#include <opencv2/opencv.hpp>


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ros_rdv_bag_data");
	ros::NodeHandle n;

	std::string str_path_src, str_path_dst ;
	
	n.getParam("/ros_rdv_bag_data/path_src", str_path_src) ;
	n.getParam("/ros_rdv_bag_data/path_dst", str_path_dst) ;

	printf("src = %s\n", str_path_src.c_str() ) ;
	printf("dst = %s\n", str_path_dst.c_str() ) ;

	rosbag::Bag bag;
	bag.open(str_path_src);  // BagMode is Read by default

	int count = 0 ;
	for(rosbag::MessageInstance const m: rosbag::View(bag))
	{
		printf("data count = %d\n", count++) ;
	}

	printf("EoF\n") ;

	ros::shutdown();
	
	return 0 ;
}
