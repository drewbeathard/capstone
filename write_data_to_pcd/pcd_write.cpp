#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>



int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    //fill in the cloud data
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (auto& point: cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

    for (const auto& point: cloud)
        std::cerr << "  " << point.x << " " << point.y << " " << point.z << std::endl;


    

    // cloud viewer part
    std::cout << "About to do cloud viewer" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    pcl::visualization::CloudViewer viewer ("simple cloud viewer");

    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {

    }

    return (0);
}