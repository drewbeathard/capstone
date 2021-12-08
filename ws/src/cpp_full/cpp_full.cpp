

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
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


// matrix transforms
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

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
        sensor_msgs::PointCloud2::ConstPtr input_pointcloud = m.instantiate<sensor_msgs::PointCloud2>();
        // instantiate pointcloud ROS message 

        nav_msgs::Odometry::ConstPtr input_odom = m.instantiate<nav_msgs::Odometry>();
        // instantiate odometry ROS message

        if (input_odom != NULL) // if we get odometry message
        {
            // set global variable params so we can reference this in pointcloud part
            std::cout << "we have recieved input odom" << std::endl;
            currentOdom.x_trans = input_odom -> pose.pose.position.x;
            currentOdom.y_trans = input_odom -> pose.pose.position.y;
            currentOdom.z_trans = input_odom -> pose.pose.position.z;
            currentOdom.x_orientation = input_odom -> pose.pose.orientation.x;
            currentOdom.y_orientation = input_odom -> pose.pose.orientation.y;
            currentOdom.z_orientation = input_odom -> pose.pose.orientation.z;
            currentOdom.w_orientation = input_odom -> pose.pose.orientation.w;
            // if it is different (enough) from last odom message, set have_odom to true so that we accept next pointcloud message

            if (previousOdom.x_trans == NULL) // if this odom message is our very first reading, use it regardless
            {
                have_odom = true;
                previousOdom = currentOdom; 
            }
            else // compare current odom with previous odom
            {
                if (std::abs(previousOdom.z_orientation - currentOdom.z_orientation) > .002)
                {
                    // if previous odom is sufficiently different form current odom, set have odom to true
                    have_odom = true;
                    std::cout << "Setting have odom to true" << std::endl;
                    previousOdom = currentOdom;
                }

            }

        }

        if (input_pointcloud != NULL) // we have an input pointcloud
        {
            if (have_odom) // if we have a unique odom reading to go with this pointcloud 
            {
                // PERFORM TRANSFORM
                std::cout << "we have recieved input PC. preparing for concat / processing" << std::endl;
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
                std::cout << "transformed pointcloud width and height: " << transformed_cloud -> width << " " << transformed_cloud -> height << std::endl;
                std::cout << "concat pointcloud width and height: " << concat_pointcloud -> width << " " << concat_pointcloud -> height << std::endl;

                
            
                cloudCounter++;
                std::cout << "Cloud counter: " << cloudCounter << std::endl << std::endl << std::endl;
            }
        }
        
        // if (cloudCounter == 300)
        // {
        //     break;
        // }
    }

        cloudCounter++;
        std::cout << "Cloud counter: " << cloudCounter << std::endl << std::endl << std::endl;





    // we now have a (huge) pointcloud
    // run voxelgrid filter to reduce number of points 
    //Then, a pcl::VoxelGrid filter is created with a leaf size of 1cm, the input data is passed, and the output is computed and stored in cloud_filtered.

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());


    std::cout << "PointCloud before Filtering: " <<concat_pointcloud->width * concat_pointcloud->height << std::endl;

    // create filter objects
    pcl::VoxelGrid<pcl::PointXYZ>sor;
	sor.setInputCloud(concat_pointcloud);//Set the input point cloud
	sor.setLeafSize(0.1f,0.1f,0.1f);//When the filter is provided voxel edge of 10cm
	sor.filter(*cloud_filtered);//put the filtered results cloud_filtered

    std::cout << "PointCloud after Filtering: " <<cloud_filtered->width * cloud_filtered->height << std::endl;
    

   

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliered(new pcl::PointCloud<pcl::PointXYZ>());

    // time for statistical outlier removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloud_filtered);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (1.0);
    sor2.filter (*cloud_outliered);

    std::cout << "PointCloud after outliering: " <<cloud_outliered->width * cloud_outliered->height << std::endl;




    // Visualization
    printf(  "\nPoint cloud colors :  white  = filtered pointcloud\n"
        "                        red  = outliered point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("outlier and filter");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_cloud_color_handler (cloud_filtered, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (cloud_filtered, filtered_cloud_color_handler, "cloud_filtered");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outliered_cloud_color_handler (cloud_outliered, 230, 20, 20); // Red
    viewer.addPointCloud (cloud_outliered, outliered_cloud_color_handler, "cloud_outliered");

    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_outliered");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }



    // // Visualization
    // // printf(  "\nPoint cloud colors :  white  = original point cloud\n"
    // //     "                        red  = transformed point cloud\n");
    // pcl::visualization::PCLVisualizer viewer ("cloud filtered");

    // // Define R,G,B colors for the point cloud
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud_filtered, 230, 20, 20);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (cloud_outliered, 255, 255, 255);
    // // We add the point cloud to the viewer and pass the color handler
    // viewer.addPointCloud (cloud_filtered, source_cloud_color_handler, "cloud_filtered");
    // viewer.addPointCloud (cloud_outliered, source_cloud_color_handler, "cloud_outliered");

    // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (pc1, 230, 20, 20); // Red
    // // viewer.addPointCloud (pc1, transformed_cloud_color_handler, "pc1");

    // viewer.addCoordinateSystem (1.0, "cloud_filtered", 0);
    // viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered");
    // // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pc1");
    // //viewer.setPosition(800, 400); // Setting visualiser window position

    // while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    //     viewer.spinOnce (100);
    //     boost::this_thread::sleep (boost::posix_time::microseconds (100000));

    // }




    bag.close();
    return 0;
}


    //     if (counter == iterator) // typically it will be big_step, set to 1 if we have a pointcloud so that we can get the very next odometry reading to match it
    //     {
    //         counter = 0;

    //         sensor_msgs::PointCloud2::ConstPtr input_pointcloud = m.instantiate<sensor_msgs::PointCloud2>();
    //         // instantiate pointcloud ROS message 

    //         nav_msgs::Odometry::ConstPtr input_odom = m.instantiate<nav_msgs::Odometry>();
    //         // instantiate odometry ROS message


    //         if (input_pointcloud != NULL) // if we get a pointcloud 
    //         {
    //             if (have_pointcloud == false)
    //             {
    //                 pcl::fromROSMsg(*input_pointcloud, *global_pointcloud);
    //                 have_pointcloud = true;
    //                 iterator = small_step; //(1)
    //                 std_msgs::Header hp = input_pointcloud->header;
    //                 std::cout << hp << std::endl;
                    

    //             }    
    //         } 

    //         if (input_odom != NULL) // if we get odom, THIS IS WHERE WE SHOULD DO MATRIX TRANSFORM! 
    //         {
    //             if (have_pointcloud == true) // if we have a recent matching pointcloud ready to use
    //             {
    //                 std_msgs::Header ho = input_odom->header;
    //                 std::cout<<ho<<std::endl; //all at once
    //                 have_pointcloud = false;
    //                 iterator = big_step; // change iterator back -> skip a number of messages 



    //                 // we now have global pointcloud (global_pointcloud) and last odometry message (input_odom)
    //                 // conduct matrix transform on this 
                    
                   
    //                 //Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    //                 // Define a translation and rotation

    //                 //msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w)

    //                 // get all 7 transform values from message
    //                 float x_trans = input_odom -> pose.pose.position.x;
    //                 float y_trans = input_odom -> pose.pose.position.y;
    //                 float z_trans = input_odom -> pose.pose.position.z;
    //                 float x_orientation = input_odom -> pose.pose.orientation.x;
    //                 float y_orientation = input_odom -> pose.pose.orientation.y;
    //                 float z_orientation = input_odom -> pose.pose.orientation.z;
    //                 float w_orientation = input_odom -> pose.pose.orientation.w;

    //                 std::cout <<"INPUT ODOM" << std::endl;
    //                 std::cout << "x_trans: " << x_trans  << std::endl;
    //                 std::cout << "y_trans: " << y_trans  << std::endl;
    //                 std::cout << "z_trans: " << z_trans  << std::endl;
    //                 std::cout << "x_orientation: " << x_orientation  << std::endl;
    //                 std::cout << "y_orientation: " << y_orientation  << std::endl;
    //                 std::cout << "z_orientation: " << z_orientation  << std::endl;
    //                 std::cout << "w_orientation: " << w_orientation  << std::endl;

    //                 // begin transformation using tf
    //                 tf::StampedTransform tform_msg; // msg is for x y z transformation
    //                 tform_msg.setOrigin(tf::Vector3(x_trans, y_trans, z_trans));
    //                 tf::Quaternion q(x_orientation, y_orientation, z_orientation, w_orientation); // quaternion
    //                 // q.setRPY(x_orientation, y_orientation, z_orientation); // setRPY only takes 3 args, how do i add w?
    //                 tform_msg.setRotation(q);

    //                 // // Executing the transformation
    //                 pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ()); // init pointCloud for new transformed data
    //                 Eigen::Matrix4f tform_mat; // declare transform matrix
    //                 pcl_ros::transformAsMatrix(tform_msg, tform_mat); 
    //                 pcl::transformPointCloud(*global_pointcloud, *transformed_cloud, tform_mat); // transform global pointcloud using tform_mat matrix (Matrix4f)



    //                 *concat_pointcloud += *transformed_cloud;
    //                 std::cout << "transformed pointcloud width and height: " << transformed_cloud -> width << " " << transformed_cloud -> height << std::endl;
    //                 std::cout << "concat pointcloud width and height: " << concat_pointcloud -> width << " " << concat_pointcloud -> height << std::endl;

    //                 // transform.translation() << x_trans, y_trans, z_trans;
    //                 // transform.rotate (Eigen::AngleAxisf (theta_trans, Eigen::Vector3f::UnitZ()));

    //                 // // // Print the transformation
    //                 // // printf ("\nMethod #2: using an Affine3f\n");
    //                 // //std::cout << transform.matrix() << std::endl;

                    
                    

    //                 // *concat_pointcloud += *transformed_cloud;
    //                 // std::cout << "global pointcloud width and height: " << global_pointcloud -> width << " " << global_pointcloud -> height << std::endl;

    //                 // std::cout << "concat pointcloud width and height: " << concat_pointcloud -> width << " " << concat_pointcloud -> height << std::endl;

    //                 cloudCounter++;
    //                 std::cout << "Cloud counter: " << cloudCounter << std::endl << std::endl << std::endl;

    //             }
    //         } 

    //     }

    //     counter++;
    //     bigCounter++;
    // }

    // // print out concat_pointcloud information:
    // std::cout << "global pointcloud width and height: " << global_pointcloud -> width << " " << global_pointcloud -> height << std::endl;

    // std::cout << "Big counter: " << bigCounter << std::endl;
    // std::cout << "Cloud counter: " << cloudCounter << std::endl;

    // std::cout << concat_pointcloud -> width << " " << concat_pointcloud -> height << std::endl;

    // Visualization
//     printf(  "\nPoint cloud colors :  white  = original point cloud\n"
//         "                        red  = transformed point cloud\n");
//     pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

//     // Define R,G,B colors for the point cloud
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (concat_pointcloud, 255, 255, 255);
//     // We add the point cloud to the viewer and pass the color handler
//     viewer.addPointCloud (concat_pointcloud, source_cloud_color_handler, "concat_pointcloud");

//     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (pc1, 230, 20, 20); // Red
//     // viewer.addPointCloud (pc1, transformed_cloud_color_handler, "pc1");

//     viewer.addCoordinateSystem (1.0, "cloud", 0);
//     viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "concat_pointcloud");
//     // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pc1");
//     //viewer.setPosition(800, 400); // Setting visualiser window position

//     while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
//         viewer.spinOnce ();
//         time.sleep(300);
//         // add a delay here 
//     }




//     bag.close();

//     return 0;

// }


