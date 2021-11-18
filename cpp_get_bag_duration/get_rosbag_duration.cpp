#include <rosbag/bag.h>
#include <rosbag/view.h>

int main(int argc, char **argv)
{
    rosbag::Bag bag;
    bag.open("/mnt/1tb/combined_10min_bag/combined_10_min.bag", rosbag::bagmode::Read);

    rosbag::View view(bag);

    ros::Time bag_begin_time = view.getBeginTime();
    ros::Time bag_end_time = view.getEndTime();

    std::cout << "ROS bag time: " << (bag_end_time-bag_begin_time).toSec() << "(s)" << std::endl;

    bag.close();

    return 0;
}