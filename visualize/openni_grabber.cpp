#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

class SimpleOpenNIViewer
{
    public:
        SimpleOpenNIViewer() : viewer ("PCL_OpenNI_Viewer"){}
        void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
        {
            if (!viewer.wasStopped())
                viewer.showCloud(cloud);
        }

        pcl::visualization::CloudViewer viewer;

        void run()
        {
            pcl::Grabber* interface = new pcl::OpenNIGrabber();
            boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
            interface -> registerCallback(f);
            interface -> start();
            while(!viewer.wasStopped())
            {
                boost::this_thread::sleep(boost::posix_time::seconds(1));
            }
            interface -> stop();
        }
};

int main()
{
    SimpleOpenNIViewer v;
    v.run();
    return 0;
}