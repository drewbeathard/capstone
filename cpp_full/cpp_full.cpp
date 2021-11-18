

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

    
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char **argv)
{
    std::cout << "WHATS HAPPENING " << std::endl;
    rosbag::Bag bag;
    bag.open("/mnt/1tb/combined_10min_bag/combined_10_min.bag", rosbag::bagmode::Read);



    std::vector<std::string> topics;
    topics.push_back(std::string("/H01/odometry"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));





    foreach(rosbag::MessageInstance const m, view)
    {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = m.instantiate<pcl::PointXYZ>();

        // std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        // if (i != NULL)
        //     std::cout << i->data << std::endl;

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        // m.instantiate<pcl::PointXYZ>();

        sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(cloud, pcl_cloud);

        std::cout << " HELOOOOO " << std::endl;

        
    }





    bag.close();

    return 0;
}



// #include <rosbag/bag.h>
// #include <rosbag/view.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/String.h>

// // #include <boost/foreach.hpp>
// // #define foreach BOOST_FOREACH

// rosbag::Bag bag;
// bag.open("/mnt/1tb/combined_10min_bag/combined_10_min.bag", rosbag::bagmode::Read);

// std::vector<std::string> topics;
// topics.push_back(std::string("/H01/odometry"));

// rosbag::View view(bag, rosbag::TopicQuery(topics));

// // foreach(rosbag::MessageInstance const m, view)
// // {
// //     std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
// //     if (s != NULL)
// //         std::cout << s->data << std::endl;

// //     std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
// //     if (i != NULL)
// //         std::cout << i->data << std::endl;
// // }

// bag.close();