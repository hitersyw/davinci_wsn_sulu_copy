// This is the interactive marker
// Su Lu, based on the example_interactive_marker of Wyatt Newman
#include <ros/ros.h>
#include <iostream>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>

void processFeedback (
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	ROS_INFO_STREAM(feedback->marker_name << "is now at"
			<< feedback->pose.position.x << "," << feedback->pose.position.y
			<< "," << feedback->pose.position.z);
	ROS_INFO_STREAM("reference frame is :" << feedback->header.frame_id);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "psm_interactive_marker"); // the node name is "psm_interactive_marker"
	// create an interactive marker for our server
    interactive_markers::InteractiveMarkerServer server("psm_interactive_marker");

    // create an interactive marker for our server
	visualization_msgs::InteractiveMarker psm_int_marker;
    psm_int_marker.header.frame_id = "base_link";
    psm_int_marker.header.stamp=ros::Time::now();
    psm_int_marker.name = "psm_interactive_marker";
    psm_int_marker.description = "6-DOF Control";
  
    geometry_msgs::Point temp_point;
    // create an arrow marker; do this 3 times to create a triad (frame)
    visualization_msgs::Marker arrow_marker_x; //this one for the x axis
    arrow_marker_x.type = visualization_msgs::Marker::ARROW; //ROS example was a CUBE; changed to ARROW
    // specify/push-in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_x.points.push_back(temp_point);
    // Specify and push in the end point for the arrow 
    temp_point.x = 0.2; // arrow is this long in x direction
    temp_point.y = 0.0;
    temp_point.z = 0.0;
    arrow_marker_x.points.push_back(temp_point);

    // make the arrow very thin
    arrow_marker_x.scale.x = 0.01;
    arrow_marker_x.scale.y = 0.01;
    arrow_marker_x.scale.z = 0.01;

    arrow_marker_x.color.r = 1.0; // red, for the x axis
    arrow_marker_x.color.g = 0.0;
    arrow_marker_x.color.b = 0.0;
    arrow_marker_x.color.a = 1.0;

    // do this again for the y axis:
    visualization_msgs::Marker arrow_marker_y;
    arrow_marker_y.type = visualization_msgs::Marker::ARROW; 
    // Push in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_y.points.push_back(temp_point);
    // Push in the end point for the arrow 
    temp_point.x = 0.0;
    temp_point.y = 0.2; // points in the y direction
    temp_point.z = 0.0;
    arrow_marker_y.points.push_back(temp_point);

    arrow_marker_y.scale.x = 0.01;
    arrow_marker_y.scale.y = 0.01;
    arrow_marker_y.scale.z = 0.01;

    arrow_marker_y.color.r = 0.0;
    arrow_marker_y.color.g = 1.0; // color it green, for y axis
    arrow_marker_y.color.b = 0.0;
    arrow_marker_y.color.a = 1.0;

    // now the z axis
    visualization_msgs::Marker arrow_marker_z;
    arrow_marker_z.type = visualization_msgs::Marker::ARROW; //CUBE;
    // Push in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_z.points.push_back(temp_point);
   // Push in the end point for the arrow 
    temp_point.x = 0.0;
    temp_point.y = 0.0;
    temp_point.z = 0.2;
    arrow_marker_z.points.push_back(temp_point);

    arrow_marker_z.scale.x = 0.01;
    arrow_marker_z.scale.y = 0.01;
    arrow_marker_z.scale.z = 0.01;

    arrow_marker_z.color.r = 0.0;
    arrow_marker_z.color.g = 0.0;
    arrow_marker_z.color.b = 1.0;
    arrow_marker_z.color.a = 1.0;
  
    // create a control that contains the markers
    visualization_msgs::InteractiveMarkerControl psm_IM_constrol;
    psm_IM_constrol.always_visible = true;
    //IM_control.markers.push_back(sphere_marker);
    
    psm_IM_constrol.markers.push_back(arrow_marker_x);
    psm_IM_constrol.markers.push_back(arrow_marker_y);
    psm_IM_constrol.markers.push_back(arrow_marker_z);
    
    // add the control to the interactive marker
    psm_int_marker.controls.push_back(psm_IM_constrol);
  
    // create a control that will move the marker
    // this control does not contain any markers,
    // which will cause RViz to insert three arrows
    visualization_msgs::InteractiveMarkerControl translate_control_x;
    translate_control_x.name = "move_x";
    translate_control_x.always_visible = true;
    translate_control_x.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    translate_control_x.orientation.x = 1; //point this in the x direction
    translate_control_x.orientation.y = 0;
    translate_control_x.orientation.z = 0;
    translate_control_x.orientation.w = 1;

    visualization_msgs::InteractiveMarkerControl translate_control_z;
    translate_control_z.name = "move_z";
    translate_control_z.always_visible = true;
    translate_control_z.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    translate_control_z.orientation.x = 0; //point this in the z direction
    translate_control_z.orientation.y = 1;
    translate_control_z.orientation.z = 0;
    translate_control_z.orientation.w = 1;

    visualization_msgs::InteractiveMarkerControl translate_control_y;
    translate_control_y.name = "move_y";
    translate_control_y.always_visible = true;
    translate_control_y.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    translate_control_y.orientation.x = 0; //point this in the y direction
    translate_control_y.orientation.y = 0;
    translate_control_y.orientation.z = 1;
    translate_control_y.orientation.w = 1;

    // add x-rotation control 
    visualization_msgs::InteractiveMarkerControl rotx_control;
    rotx_control.name = "rot_x";
    rotx_control.always_visible = true;
    rotx_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    rotx_control.orientation.x = 1;
    rotx_control.orientation.y = 0;
    rotx_control.orientation.z = 0;
    rotx_control.orientation.w = 1;

    // add z-rotation control
    visualization_msgs::InteractiveMarkerControl rotz_control;
    rotz_control.name = "rot_z";
    rotz_control.always_visible = true;
    rotz_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    rotz_control.orientation.x = 0;
    rotz_control.orientation.y = 1;
    rotz_control.orientation.z = 0;
    rotz_control.orientation.w = 1;

    // add y-rotation control
    visualization_msgs::InteractiveMarkerControl roty_control;
    roty_control.name = "rot_y";
    roty_control.always_visible = true;
    roty_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    roty_control.orientation.x = 0;
    roty_control.orientation.y = 0;
    roty_control.orientation.z = 1;
    roty_control.orientation.w = 1;

    // add the control to the interactive marker
    psm_int_marker.controls.push_back(translate_control_x);    
    psm_int_marker.controls.push_back(translate_control_y);    
    psm_int_marker.controls.push_back(translate_control_z);
    psm_int_marker.controls.push_back(rotx_control);
    psm_int_marker.controls.push_back(rotz_control);
    psm_int_marker.controls.push_back(roty_control);
  
    /** Scale Down: this makes all of the arrows/disks for the user controls smaller than the default size */
    psm_int_marker.scale = 0.2;
    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(psm_int_marker, &processFeedback);
  
    // 'commit' changes and send to all clients
    server.applyChanges();
  
    // start the ROS main loop
    ros::spin();
  }
