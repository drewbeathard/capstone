

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>
    
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


// matrix transforms
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/io.h> // for concatenateFields

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>



//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char **argv)
{
    rosbag::Bag bag;
    bag.open("/mnt/1tb/combined_10min_bag/combined_10_min.bag", rosbag::bagmode::Read);



    std::vector<std::string> topics;
    topics.push_back(std::string("/H01/horiz/os_cloud_node/points"));
    topics.push_back(std::string("/H01/odometry"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    bool have_pointcloud = false;
    bool have_odom = false;
    int counter = 0;
    int bigCounter = 0;
    int cloudCounter = 0;
    int big_step = 1000; // use this to keyframe values 
    int small_step = 1;
    int iterator = big_step;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_pointcloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr concat_pointcloud (new pcl::PointCloud<pcl::PointXYZ> ());
   


    foreach(rosbag::MessageInstance const m, view)
    {

        if (counter == iterator) // typically it will be big_step, set to 1 if we have a pointcloud so that we can get the very next odometry reading to match it
        {
            counter = 0;

            sensor_msgs::PointCloud2::ConstPtr input_pointcloud = m.instantiate<sensor_msgs::PointCloud2>();
            // instantiate pointcloud ROS message 

            nav_msgs::Odometry::ConstPtr input_odom = m.instantiate<nav_msgs::Odometry>();
            // instantiate odometry ROS message


            if (input_pointcloud != NULL) // if we get a pointcloud 
            {
                if (have_pointcloud == false)
                {
                    pcl::fromROSMsg(*input_pointcloud, *global_pointcloud);
                    have_pointcloud = true;
                    iterator = small_step; //(1)
                    std_msgs::Header hp = input_pointcloud->header;
                    std::cout << hp << std::endl;

                }    
            } 

            if (input_odom != NULL) // if we get odom, THIS IS WHERE WE SHOULD DO MATRIX TRANSFORM! 
            {
                if (have_pointcloud == true) // if we have a recent matching pointcloud ready to use
                {
                    std_msgs::Header ho = input_odom->header;
                    std::cout<<ho<<std::endl; //all at once
                    have_pointcloud = false;
                    iterator = big_step; // change iterator back -> skip a number of messages 



                    // we now have global pointcloud (global_pointcloud) and last odometry message (input_odom)
                    // conduct matrix transform on this 
                    
                   
                    //Eigen::Affine3f transform = Eigen::Affine3f::Identity();

                    // Define a translation and rotation

                    //msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w)

                    // get all 7 transform values from message
                    float x_trans = input_odom -> pose.pose.position.x;
                    float y_trans = input_odom -> pose.pose.position.y;
                    float z_trans = input_odom -> pose.pose.position.z;
                    float x_orientation = input_odom -> pose.pose.orientation.x;
                    float y_orientation = input_odom -> pose.pose.orientation.y;
                    float z_orientation = input_odom -> pose.pose.orientation.z;
                    float w_orientation = input_odom -> pose.pose.orientation.w;

                    std::cout <<"INPUT ODOM" << std::endl;
                    std::cout << "x_trans: " << x_trans  << std::endl;
                    std::cout << "y_trans: " << y_trans  << std::endl;
                    std::cout << "z_trans: " << z_trans  << std::endl;
                    std::cout << "x_orientation: " << x_orientation  << std::endl;
                    std::cout << "y_orientation: " << y_orientation  << std::endl;
                    std::cout << "z_orientation: " << z_orientation  << std::endl;
                    std::cout << "w_orientation: " << w_orientation  << std::endl;

                    // begin transformation using tf
                    tf::StampedTransform tform_msg; // msg is for x y z transformation
                    tform_msg.setOrigin(tf::Vector3(x_trans, y_trans, z_trans));
                    tf::Quaternion q(x_orientation, y_orientation, z_orientation, w_orientation); // quaternion
                    // q.setRPY(x_orientation, y_orientation, z_orientation); // setRPY only takes 3 args, how do i add w?
                    tform_msg.setRotation(q);

                    // // Executing the transformation
                    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ()); // init pointCloud for new transformed data
                    Eigen::Matrix4f tform_mat; // declare transform matrix
                    pcl_ros::transformAsMatrix(tform_msg, tform_mat); 
                    pcl::transformPointCloud(*global_pointcloud, *transformed_cloud, tform_mat); // transform global pointcloud using tform_mat matrix (Matrix4f)



                    *concat_pointcloud += *transformed_cloud;
                    std::cout << "transformed pointcloud width and height: " << transformed_cloud -> width << " " << transformed_cloud -> height << std::endl;
                    std::cout << "concat pointcloud width and height: " << concat_pointcloud -> width << " " << concat_pointcloud -> height << std::endl;

                    // transform.translation() << x_trans, y_trans, z_trans;
                    // transform.rotate (Eigen::AngleAxisf (theta_trans, Eigen::Vector3f::UnitZ()));

                    // // // Print the transformation
                    // // printf ("\nMethod #2: using an Affine3f\n");
                    // //std::cout << transform.matrix() << std::endl;

                    
                    

                    // *concat_pointcloud += *transformed_cloud;
                    // std::cout << "global pointcloud width and height: " << global_pointcloud -> width << " " << global_pointcloud -> height << std::endl;

                    // std::cout << "concat pointcloud width and height: " << concat_pointcloud -> width << " " << concat_pointcloud -> height << std::endl;

                    cloudCounter++;
                    std::cout << "Cloud counter: " << cloudCounter << std::endl << std::endl << std::endl;

                }
            } 

        }

        counter++;
        bigCounter++;
    }

    // // print out concat_pointcloud information:
    // std::cout << "global pointcloud width and height: " << global_pointcloud -> width << " " << global_pointcloud -> height << std::endl;

    // std::cout << "Big counter: " << bigCounter << std::endl;
    // std::cout << "Cloud counter: " << cloudCounter << std::endl;

    // std::cout << concat_pointcloud -> width << " " << concat_pointcloud -> height << std::endl;

    // Visualization
    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
        "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (concat_pointcloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (concat_pointcloud, source_cloud_color_handler, "concat_pointcloud");

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (pc1, 230, 20, 20); // Red
    // viewer.addPointCloud (pc1, transformed_cloud_color_handler, "pc1");

    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "concat_pointcloud");
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pc1");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
        time.sleep(300);
        // add a delay here 
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