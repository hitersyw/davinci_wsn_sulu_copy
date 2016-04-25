#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
#include <tf/LinearMath/Matrix3x3.h>


#include <Eigen/Eigen> // these to use some of the features of Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>

geometry_msgs::Point g_point;
bool g_flag = false;

void myCallBack(const geometry_msgs::Point& centroid)
{
    g_point.x = centroid.x;
    g_point.y = centroid.y;
    g_point.z = centroid.z;
    g_flag = true;
}


//here are a couple of utility fncs useful for converting transform data types
Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3d e;
    // treat the Eigen::Affine as a 4x4 matrix:
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i]; //copy the origin from tf to Eigen
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j]; //and copy 3x3 rotation matrix
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "cub_maker"); // this will be the node name;
    ros::NodeHandle nh;

    ROS_INFO("hello, world");
    ros::Rate timer(4); //timer to run at 4 Hz

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "cube_centroid_marker", 0 );
    ros::Subscriber vis_sub = nh.subscribe("/blue_centroid", 1, myCallBack);

    visualization_msgs::Marker marker;  // instantiate a marker object
    geometry_msgs::Point point;  // points will be used to specify where the markers go
    marker.header.frame_id = "/world"; //base_link"; // select the reference frame
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    // use SPHERE if you only want a single marker
    marker.type = visualization_msgs::Marker::SPHERE_LIST; //SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    // if just using a single marker, specify the coordinates here, like this:

    //marker.pose.position.x = 0.4;  
    //marker.pose.position.y = -0.4;
    //marker.pose.position.z = 0;
    //ROS_INFO("x,y,z = %f %f, %f",marker.pose.position.x,marker.pose.position.y, marker.pose.position.z);    
    // otherwise, for a list of markers, put their coordinates in the "points" array, as below
    
    //whether a single marker or list of markers, need to specify marker properties
    // these will all be the same for SPHERE_LIST
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    Eigen::Affine3d affine_left_camera_wrt_world_from_tf;
    Eigen::Vector3d original_centroid;
    Eigen::Vector3d transformed_centroid;

    tf::StampedTransform tf_sensor_frame_to_world_frame; //need objects of this type to hold tf's
    tf::TransformListener tfListener; //create a TransformListener to listen for tf's and assemble them

    //the tf listener needs to "warm up", in that it needs to collect a complete set of transforms
    // to deduce arbitrary connectivities; use try/catch to let it fail until success
    bool tferr = true;
    ROS_INFO("waiting for tf between left_camera_link and world...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame.
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("world", "left_camera_optical_frame", ros::Time(0), tf_sensor_frame_to_world_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll

    ros::Rate sleep_timer(1.0); //a timer for desired rate, e.g. 1Hz


    while(ros::ok())
    {
        tfListener.lookupTransform("world", "left_camera_optical_frame", ros::Time(0), tf_sensor_frame_to_world_frame);
        affine_left_camera_wrt_world_from_tf = transformTFToEigen(tf_sensor_frame_to_world_frame);


        marker.points.clear();
        if(g_flag == false)
        {
            ros::spinOnce();
        }

        original_centroid << g_point.x, g_point.y, g_point.z;
        transformed_centroid = affine_left_camera_wrt_world_from_tf * original_centroid;
        point.x = transformed_centroid(0);
        point.y = transformed_centroid(1);
        point.z = transformed_centroid(2);



        ROS_INFO("x = %f, y = %f, z = %f",point.x,point.y,point.z);
        marker.points.push_back(point);
        vis_pub.publish( marker );
        timer.sleep();
        ros::spinOnce();
    }
//    //as an example, let's build a wall of red spheres:
//    double dist = 0.5; // robot distance from the wall, along x-axis in pelvis frame
//    double corner_y = -0.5; // y-coordinate of right edge of the wall
//    double corner_z = 0.0; // z-coordinate of lower-right corner of wall, as seen by Atlas
//    point.x = dist;
//    point.y = corner_y;
//
// /    point.z = corner_z;
//
//    for (int j = 0; j < 10; j++) {
//
//        point.y = corner_y;
//        point.x = dist;
//        marker.points.push_back(point);
//        vis_pub.publish( marker );
//        timer.sleep();
//        for (int k = 0; k < 10; k++) {
//            point.y += 0.1;
//            marker.points.push_back(point);
//            ROS_INFO("z,y = %f, %f",point.z,point.y);
//            vis_pub.publish( marker );
//             timer.sleep();
//        }
//        point.z += 0.1; // next row up...
//    }
//
//
//    for (int k=0; k<10; k++) {
//        vis_pub.publish( marker );
//        timer.sleep();
//        ROS_INFO("publishing...");
//    // }
    return 0;
}





