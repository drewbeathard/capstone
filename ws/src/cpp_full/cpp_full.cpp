

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h> 
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <pcl/common/io.h> // for concatenateFields
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <unistd.h> // for delay on visualization



struct Odometry
{ 
    float x_trans;
    float y_trans;
    float z_trans;
    float x_orientation;
    float y_orientation;
    float z_orientation;
    float w_orientation;
} previousOdom, currentOdom;
// previous Odom is the last odometry reading that we will compare with the current one. If they are different enough,
// we will add the new reading to the cloud


//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char **argv)
{
    rosbag::Bag bag;
    bag.open(argv[1], rosbag::bagmode::Read); // dirctory location of bag is argument for program

    //bag.open("/mnt/1tb/combined_10min_bag/combined_10_min.bag", rosbag::bagmode::Read);


    // push to vector names of point cloud and odometry topics 
    std::vector<std::string> topics;
    topics.push_back(std::string("/H01/horiz/os_cloud_node/points"));
    topics.push_back(std::string("/H01/odometry"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    bool have_odom = false; // bool set to true when you have an odom message -> wait for very next point cloud message
    int counter = 0; // counter iterates on each loop of messages, choose how often to read those messages
    int bigCounter = 0;
    int cloudCounter = 0;
    int big_step = 100; // use this to keyframe values, we will only look at every x messages
    int small_step = 1; // set to small step when you get odometry, so that you recieve the very next pointcloud
    int iterator = big_step; // when counter reaches iterator, look at messages

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_pointcloud (new pcl::PointCloud<pcl::PointXYZ> ()); // global pointcloud holds each new pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr concat_pointcloud (new pcl::PointCloud<pcl::PointXYZ> ()); // concat pointcloud is large final pointcloud
   

    foreach(rosbag::MessageInstance const m, view) // loop through each message in ros bag
    {

        counter++;

        if (counter == iterator)
        {
            counter = 0;

            sensor_msgs::PointCloud2::ConstPtr input_pointcloud = m.instantiate<sensor_msgs::PointCloud2>();
            // instantiate pointcloud ROS message 

            nav_msgs::Odometry::ConstPtr input_odom = m.instantiate<nav_msgs::Odometry>();
            // instantiate odometry ROS message

            if (input_odom != NULL) // if we get odometry message
            {
                iterator = small_step;
                // set global variable params so we can reference this with pointcloud 
                //std::cout << "we have recieved input odom" << std::endl;
                currentOdom.x_trans = input_odom -> pose.pose.position.x;
                currentOdom.y_trans = input_odom -> pose.pose.position.y;
                currentOdom.z_trans = input_odom -> pose.pose.position.z;
                currentOdom.x_orientation = input_odom -> pose.pose.orientation.x;
                currentOdom.y_orientation = input_odom -> pose.pose.orientation.y;
                currentOdom.z_orientation = input_odom -> pose.pose.orientation.z;
                currentOdom.w_orientation = input_odom -> pose.pose.orientation.w;


                if (previousOdom.x_trans == NULL) // if this odom message is our very first reading, use it regardless
                {
                    have_odom = true;
                    previousOdom = currentOdom; 
                }
                // if it is different (enough) from last odom message, set have_odom to true so that we accept next pointcloud message
                else // compare current odom with previous odom
                {
                    float xyz_dist = sqrt(pow(previousOdom.x_trans-currentOdom.x_trans, 2) + pow(previousOdom.y_trans - currentOdom.y_trans, 2) + pow(previousOdom.z_trans - currentOdom.z_trans, 2));
                    // xyz distance is distance robot has moved 

                    // if robot has rotated on z axis or moved, set have_odom to true
                    if (std::abs(previousOdom.z_orientation - currentOdom.z_orientation) > .01 || xyz_dist > .2)
                    {
                        // if previous odom is sufficiently different form current odom, set have odom to true
                        have_odom = true;
                        // std::cout << "Setting have odom to true" << std::endl;
                        previousOdom = currentOdom;
                    }

                }

            }

            if (input_pointcloud != NULL) // we have an input pointcloud
            {
                if (have_odom) // if we have a unique odom reading to go with this pointcloud 
                {
                    iterator = big_step; // set iterator back to big step
                    // PERFORM TRANSFORM
                    //std::cout << "we have recieved input PC. preparing for concat / processing" << std::endl;
                    have_odom = false; // reset odom bool so we don't accept each pointcloud

                    pcl::fromROSMsg(*input_pointcloud, *global_pointcloud); // convert pointcloud ROS message to PointXYZ (pointcloud datatype)

                    // begin transformation using tf
                    tf::StampedTransform tform_msg; // msg is for x y z transformation
                    tform_msg.setOrigin(tf::Vector3(currentOdom.x_trans, currentOdom.y_trans, currentOdom.z_trans));
                    tf::Quaternion q(currentOdom.x_orientation, currentOdom.y_orientation, currentOdom.z_orientation, currentOdom.w_orientation); // quaternion
                    tform_msg.setRotation(q);

                    // Executing the transformation
                    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ()); // init pointCloud for new transformed data
                    Eigen::Matrix4f tform_mat; // declare transform matrix
                    pcl_ros::transformAsMatrix(tform_msg, tform_mat); 
                    pcl::transformPointCloud(*global_pointcloud, *transformed_cloud, tform_mat); // transform global pointcloud using tform_mat matrix (Matrix4f)



                    *concat_pointcloud += *transformed_cloud;
                    // std::cout << "transformed pointcloud width and height: " << transformed_cloud -> width << " " << transformed_cloud -> height << std::endl;
                    // std::cout << "concat pointcloud width and height: " << concat_pointcloud -> width << " " << concat_pointcloud -> height << std::endl;

                    
                
                    cloudCounter++;
                    std::cout << "Cloud counter: " << cloudCounter << std::endl << std::endl << std::endl;
                }
            }
            
        
            
        }
    }

    // number of clouds that went into final cloud    
    //std::cout << "Cloud counter: " << cloudCounter << std::endl << std::endl << std::endl;

    // we now have a (huge) pointcloud
    // run voxelgrid filter to reduce number of points 
    //Then, a pcl::VoxelGrid filter is created with a leaf size of 1cm, the input data is passed, and the output is computed and stored in cloud_filtered.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    std::cout << "PointCloud before VoxelGrid: " <<concat_pointcloud->width * concat_pointcloud->height << std::endl;
    // create filter objects
    pcl::VoxelGrid<pcl::PointXYZ>sor;
	sor.setInputCloud(concat_pointcloud);//Set the input point cloud
    float leafSize = .16;
	sor.setLeafSize(leafSize,leafSize,leafSize);//size of voxels
	sor.filter(*cloud_filtered);//put the filtered results cloud_filtered

    std::cout << "PointCloud after VoxelGrid: " <<cloud_filtered->width * cloud_filtered->height << std::endl;
    

    // statistical outlier removal
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliered(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloud_filtered);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (1.0);
    sor2.filter (*cloud_outliered);

    std::cout << "PointCloud after outliering: " <<cloud_outliered->width * cloud_outliered->height << std::endl;




    // Visualization
    pcl::visualization::PCLVisualizer viewer ("outlier and filter");

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> concat_cloud_color_handler (concat_pointcloud, 30, 230, 20); // Red
    // viewer.addPointCloud (concat_pointcloud, concat_cloud_color_handler, "concat_pointcloud");

    // // Define R,G,B colors for the point cloud
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_cloud_color_handler (cloud_filtered, 255, 255, 255);
    // // We add the point cloud to the viewer and pass the color handler
    // viewer.addPointCloud (cloud_filtered, filtered_cloud_color_handler, "cloud_filtered");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outliered_cloud_color_handler (cloud_outliered, 255,255,255); // white
    viewer.addPointCloud (cloud_outliered, outliered_cloud_color_handler, "cloud_outliered");
    // choose which cloud you want to visualize


    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "concat_pointcloud");
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_outliered");
    viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }


    bag.close();
    return 0;
}

