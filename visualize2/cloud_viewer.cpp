#include <pcl/visualization/cloud_viewer.h>

void cloud_viewer()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    pcl::visualization::CloudViewer viewer ("simple cloud viewer");

    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {

    }
}