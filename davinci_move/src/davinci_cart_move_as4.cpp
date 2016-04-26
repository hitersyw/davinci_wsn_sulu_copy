//cart_move_as.cpp
// action server accepts desired Cartesian poses (grippers 1 and 2), gripper_angs, and arrival_time (move duration)
// does IK on destination poses to get JS poses;
// stuffs a trajectory message
// specifies this as a goal to joint-space interpolator action server to execute
#include<ros/ros.h>
#include <davinci_kinematics/davinci_joint_publisher.h>
#include <davinci_kinematics/davinci_kinematics.h>
#include <davinci_traj_streamer/davinci_traj_streamer.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../baxter_traj_streamer/action/traj.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (traj) and appended name (Action)
// If you write a new client of the server in this package, you will need to include baxter_traj_streamer in your package.xml,
// and include the header file below
#include<davinci_traj_streamer/trajAction.h>
#include<cwru_action/cart_moveAction.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <sensor_msgs/JointState.h>

using namespace std;

double g_gripper_ang1=0.0;
double g_gripper_ang2=0.0;
double g_arrival_time = 0.0;

Vectorq7x1 g_q_vec;  //global, filled by callback
bool g_got_callback=false;

void jsCallback(const sensor_msgs::JointState& jointState) 
{ 
        g_q_vec(0) = jointState.position[12]; 
        g_q_vec(1) = jointState.position[1];
        g_q_vec(2) = jointState.position[0];
        g_q_vec(3) = jointState.position[7];          //rotation about tool shaft:     
        g_q_vec(4) = jointState.position[10];        // wrist bend:
        g_q_vec(5) = jointState.position[11];       // q_vec[5] rotates both jaws together
        g_q_vec(6) = 0.0;  // decide what to do with this--2nd jaw
    g_got_callback = true;
} 

Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3d e;
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i];
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j];
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}

geometry_msgs::Pose transformEigenAffine3fToPose(Eigen::Affine3f e) {
    Eigen::Vector3f Oe;
    Eigen::Matrix3f Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaternionf q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
  Eigen::Affine3d affine;

    Eigen::Vector3d Oe;

    Oe(0)= pose.position.x;
    Oe(1)= pose.position.y;
    Oe(2)= pose.position.z;
    affine.translation() = Oe;

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;   
 return affine;
}

class CartMoveActionServer {
private:
	ros::NodeHandle nh_;
    //here is our action server to accept cartesian goals:
        actionlib::SimpleActionServer<cwru_action::cart_moveAction> cart_move_as_;

    actionlib::SimpleActionClient<davinci_traj_streamer::trajAction> js_action_client_; //("trajActionServer", true);

    //messages to receive cartesian goals / return results:
    cwru_action::cart_moveGoal cart_goal_;
    cwru_action::cart_moveResult cart_result_;

    //messages to send goals/get results from joint-space interpolator action server:
    davinci_traj_streamer::trajGoal js_goal_; // goal message, received from client
    davinci_traj_streamer::trajResult js_result_; // put results here, to be sent back to the client when done w/ goal
    //davinci_traj_streamer::trajFeedback feedback_; // not used in this example; 
    //callback fnc for joint-space action server to return result to this node:
    void js_doneCb_(const actionlib::SimpleClientGoalState& state,
        const davinci_traj_streamer::trajResultConstPtr& result);

    Eigen::Affine3d des_gripper_affine1_,des_gripper_affine2_;
    Eigen::Affine3d des_gripper1_affine_wrt_lcamera_,des_gripper2_affine_wrt_lcamera_;
    Eigen::Affine3d gripper1_affine_last_commanded_pose_,gripper2_affine_last_commanded_pose_;
    double gripper_ang1_,last_gripper_ang1_;
    double gripper_ang2_,last_gripper_ang2_; 
    double arrival_time_; 
    Eigen::Affine3d affine_lcamera_to_psm_one_,affine_lcamera_to_psm_two_,affine_lcamera_to_world_;

    // Action Server Interface
    void executeCB(const actionlib::SimpleActionServer<cwru_action::cart_moveAction>::GoalConstPtr& goal);
    Davinci_fwd_solver davinci_fwd_solver_; //instantiate a forward-kinematics solver    
    Davinci_IK_solver ik_solver_;
public:
    //constructor:
    CartMoveActionServer(ros::NodeHandle &nh); //define the body of the constructor outside of class definition

    ~CartMoveActionServer(void) {
    }
    void set_lcam2world(Eigen::Affine3d xf) { affine_lcamera_to_world_=xf; };
    void set_lcam2psm1(Eigen::Affine3d xf) { affine_lcamera_to_psm_one_=xf; };
    void set_lcam2psm2(Eigen::Affine3d xf) { affine_lcamera_to_psm_two_=xf; };
    void set_init_pose_gripper1(Eigen::Affine3d init_pose) {gripper1_affine_last_commanded_pose_ = init_pose;  };
    void set_init_pose_gripper2(Eigen::Affine3d init_pose) {gripper2_affine_last_commanded_pose_ = init_pose;  };    
};

CartMoveActionServer::CartMoveActionServer(ros::NodeHandle &nh):nh_(nh),
cart_move_as_(nh, "cartMoveActionServer", boost::bind(&CartMoveActionServer::executeCB, this, _1), false),
js_action_client_("trajActionServer", true)
{
    ROS_INFO("starting action server: cartMoveActionServer ");
    cart_move_as_.start(); //start the server running

   // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = js_action_client_.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        int max_tries = 0;
        while (!server_exists) {
           server_exists = js_action_client_.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
           // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
           ros::spinOnce();
           ros::Duration(0.1).sleep();
           ROS_INFO("retrying...");
           max_tries++;
           if (max_tries>100)
               break;
        }

    if (!server_exists) {                               
        ROS_WARN("could not connect to server; will keep trying indefinitely");
        // bail out; optionally, could print a warning message and retry
    }
    server_exists = js_action_client_.waitForServer(); //wait forever 


    ROS_INFO("connected to joint-space interpolator action server"); // if here, then we connected to the server;
 
    // add these 11/12/15
    ROS_INFO("getting transforms from camera to PSMs");
    tf::TransformListener tfListener;
    tf::StampedTransform tfResult_one,tfResult_two, tfResult_zero;    
    Eigen::Affine3d affine_lcamera_to_psm_one,affine_lcamera_to_psm_two,affine_gripper_wrt_base, affine_lcamera_to_world;
   bool tferr=true;
    ROS_INFO("waiting for tf between base and right_hand...");
    while (tferr) {
        tferr=false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame.                                           
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                tfListener.lookupTransform("left_camera_optical_frame","one_psm_base_link",  ros::Time(0), tfResult_one);
                tfListener.lookupTransform("left_camera_optical_frame","two_psm_base_link",  ros::Time(0), tfResult_two);
                tfListener.lookupTransform("left_camera_optical_frame","world",  ros::Time(0), tfResult_zero);
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");
    //affine_lcamera_to_psm_one is the position/orientation of psm1 base frame w/rt left camera link frame
    // need to extend this to camera optical frame
    affine_lcamera_to_psm_one_ = transformTFToEigen(tfResult_one);
    affine_lcamera_to_psm_two_ = transformTFToEigen(tfResult_two); 
    affine_lcamera_to_world_ = transformTFToEigen(tfResult_zero); 
    ROS_INFO("transform from left camera to psm one:");
    cout<<affine_lcamera_to_psm_one_.linear()<<endl;
    cout<<affine_lcamera_to_psm_one_.translation().transpose()<<endl;
    ROS_INFO("transform from left camera to psm two:");  

    cout<<affine_lcamera_to_psm_two_.linear()<<endl;
    cout<<affine_lcamera_to_psm_two_.translation().transpose()<<endl; 
    ROS_INFO("transform from left camera to world:");
    cout<<affine_lcamera_to_world_.linear()<<endl;
    cout<<affine_lcamera_to_world_.translation().transpose()<<endl; 

}

void CartMoveActionServer::executeCB(const actionlib::SimpleActionServer<cwru_action::cart_moveAction>::GoalConstPtr& goal) {

    ROS_INFO("in executeCB of CartMoveActionServer");
   cart_result_.err_code=0;
   cart_move_as_.isActive();
   //unpack the necessary info:
   gripper_ang1_ = goal->gripper_jaw_angle1;
   gripper_ang2_ = goal->gripper_jaw_angle2;
   arrival_time_ = goal->move_time;
   // interpret the desired gripper poses:
   geometry_msgs::PoseStamped des_pose_gripper1 = goal->des_pose_gripper1;
   geometry_msgs::PoseStamped des_pose_gripper2 = goal->des_pose_gripper2;
   // convert the above to affine objects:
   des_gripper1_affine_wrt_lcamera_ = transformPoseToEigenAffine3d(des_pose_gripper1.pose);
   cout<<"gripper1 desired pose;  "<<endl;
   cout<<des_gripper1_affine_wrt_lcamera_.linear()<<endl;
   cout<<"origin: "<<des_gripper1_affine_wrt_lcamera_.translation().transpose()<<endl;

   des_gripper2_affine_wrt_lcamera_ = transformPoseToEigenAffine3d(des_pose_gripper2.pose);
   cout<<"gripper2 desired pose;  "<<endl;
   cout<<des_gripper2_affine_wrt_lcamera_.linear()<<endl;
   cout<<"origin: "<<des_gripper2_affine_wrt_lcamera_.translation().transpose()<<endl;

   //do IK to convert these to joint angles:
    //Eigen::VectorXd q_vec1,q_vec2;
    Vectorq7x1 q_vec1,q_vec2, q_vec1_jstate;
    q_vec1.resize(7);
    q_vec2.resize(7);

    
    trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
    des_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    des_trajectory.joint_names.clear(); //could put joint names in...but I assume a fixed order and fixed size, so this is unnecessary
    // if using wsn's trajectory streamer action server
    des_trajectory.header.stamp = ros::Time::now();

    trajectory_msgs::JointTrajectoryPoint trajectory_point; //,trajectory_point2;         
    trajectory_point.positions.resize(14);
    
    ROS_INFO("\n");
    ROS_INFO("stored previous command to gripper one: ");
    cout<<gripper1_affine_last_commanded_pose_.linear()<<endl;
    cout<<"origin: "<<gripper1_affine_last_commanded_pose_.translation().transpose()<<endl;
   

    // first, reiterate previous command:
    // this could be easier, if saved previous joint-space trajectory point...
    des_gripper_affine1_ = affine_lcamera_to_psm_one_.inverse()*gripper1_affine_last_commanded_pose_; //previous pose
    ik_solver_.ik_solve(des_gripper_affine1_); //convert desired pose into equiv joint displacements
    q_vec1 = ik_solver_.get_soln(); 
    q_vec1(6) = last_gripper_ang1_; // include desired gripper opening angle
    cout<<"q_vec1 of stored pose: "<<endl;
    for (int i=0;i<6;i++) {
        cout<<q_vec1[i]<<", ";
    }
    cout<<endl;
    
    des_gripper_affine2_ = affine_lcamera_to_psm_two_.inverse()*gripper2_affine_last_commanded_pose_; //previous pose
    ik_solver_.ik_solve(des_gripper_affine2_); //convert desired pose into equiv joint displacements
    q_vec2 = ik_solver_.get_soln(); 

    q_vec2(6) = last_gripper_ang2_; // include desired gripper opening angle
      
     for (int i=0;i<7;i++) {
            trajectory_point.positions[i] = q_vec1(i);
            trajectory_point.positions[i+7] = q_vec2(i);  
        }
    cout<<"start traj pt: "<<endl;
    for (int i=0;i<14;i++) {
        cout<<trajectory_point.positions[i]<<", ";
    }
    cout<<endl;
      trajectory_point.time_from_start = ros::Duration(0.0); // start time set to 0
    // PUSH IN THE START POINT:
      des_trajectory.points.push_back(trajectory_point);            

    // compute and append the goal point, in joint space trajectory:
    des_gripper_affine1_ = affine_lcamera_to_psm_one_.inverse()*des_gripper1_affine_wrt_lcamera_;


    ik_solver_.ik_solve(des_gripper_affine1_); //convert desired pose into equiv joint displacements
    ROS_INFO("desired gripper one location in base frame: ");
    cout<<"gripper1 desired pose;  "<<endl;
    cout<<des_gripper_affine1_.linear()<<endl;
    cout<<"origin: "<<des_gripper_affine1_.translation().transpose()<<endl;
    q_vec1 = ik_solver_.get_soln(); 
    q_vec1(6) = gripper_ang1_; // include desired gripper opening angle
    cout<<"q_vec1 of goal pose: "<<endl;
    for (int i=0;i<6;i++) {
        cout<<q_vec1[i]<<", ";
    }
    cout<<endl;

    des_gripper_affine2_ = affine_lcamera_to_psm_two_.inverse()*des_gripper2_affine_wrt_lcamera_;
    ROS_INFO("desired gripper two location in base frame: ");
    cout<<"gripper2 desired pose;  "<<endl;
    cout<<des_gripper_affine2_.linear()<<endl;
    cout<<"origin: "<<des_gripper_affine2_.translation().transpose()<<endl;

    ik_solver_.ik_solve(des_gripper_affine2_); //convert desired pose into equiv joint displacements
    q_vec2 = ik_solver_.get_soln();  

    q_vec2(6) = gripper_ang2_;
     cout<<"q_vec2 of goal pose: "<<endl;
    for (int i=0;i<6;i++) {
        cout<<q_vec2[i]<<", ";
    }
    cout<<endl;

        for (int i=0;i<7;i++) {
            trajectory_point.positions[i] = q_vec1(i);
            trajectory_point.positions[i+7] = q_vec2(i);  
        }
      trajectory_point.time_from_start = ros::Duration(arrival_time_);

   cout<<"goal traj pt: "<<endl;
    for (int i=0;i<14;i++) {
        cout<<trajectory_point.positions[i]<<", ";
    }
    cout<<endl;
      des_trajectory.points.push_back(trajectory_point);
    

   Davinci_fwd_solver davinci_fwd_solver;
   Eigen::Affine3d affine_gripper_wrt_psm1, affine_gripper_wrt_psm1_FK;
   Eigen::Vector3d tip_from_FK_of_IK;
   ROS_INFO("FK of IK soln: ");
   affine_gripper_wrt_psm1 = davinci_fwd_solver.fwd_kin_solve(q_vec1);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_gripper_wrt_psm1.linear()<<endl;
   tip_from_FK_of_IK = affine_gripper_wrt_psm1.translation();
   cout<<"origin: ";
   cout<<tip_from_FK_of_IK.transpose()<<endl;   
   cout<<"origin in left camera frame: ";
   cout<<affine_lcamera_to_psm_one_*tip_from_FK_of_IK<<endl;   


    js_goal_.trajectory = des_trajectory;

    // Need boost::bind to pass in the 'this' pointer
  // see example: http://library.isr.ist.utl.pt/docs/roswiki/actionlib_tutorials%282f%29Tutorials%282f%29Writing%2820%29a%2820%29Callback%2820%29Based%2820%29Simple%2820%29Action%2820%29Client.html
  //  ac.sendGoal(goal,
  //              boost::bind(&MyNode::doneCb, this, _1, _2),
  //              Client::SimpleActiveCallback(),
  //              Client::SimpleFeedbackCallback());

    Eigen::Vector3d tip_from_FK;
   ROS_INFO("gripper tip frame from FK: ");   
   q_vec1_jstate = g_q_vec;
   affine_gripper_wrt_psm1_FK = davinci_fwd_solver.fwd_kin_solve(q_vec1_jstate);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_gripper_wrt_psm1_FK.linear()<<endl;
   tip_from_FK = affine_gripper_wrt_psm1_FK.translation();
   cout<<"origin from FK: "<<tip_from_FK.transpose()<<endl; 
   cout<<endl;
   cout<<"origin from FK in left camera frame: ";
   cout<<(affine_lcamera_to_world_*tip_from_FK).transpose()<<endl;   

//tf::StampedTransform tfResult_one,tfResult_two;  
     tf::TransformListener tfListener;  
    tf::StampedTransform tf_init_gripper1,tf_init_gripper2; 
    // get these transform values to CartMoveActionServer
    //Eigen::Affine3d affine_lcamera_to_psm_one,affine_lcamera_to_psm_two,affine_gripper_wrt_base;
    // need to get these poses from goal message
    Eigen::Affine3d init_gripper_affine1,init_gripper_affine2;
   bool tferr=true;

    ROS_INFO("waiting for tf between base and right_hand...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                //tfListener.lookupTransform("left_camera_optical_frame","one_psm_base_link",  ros::Time(0), tfResult_one);
                //tfListener.lookupTransform("left_camera_optical_frame","two_psm_base_link",  ros::Time(0), tfResult_two);
                tfListener.lookupTransform("left_camera_optical_frame","one_tool_tip_link",  ros::Time(0), tf_init_gripper1);                
                tfListener.lookupTransform("left_camera_optical_frame","two_tool_tip_link",  ros::Time(0), tf_init_gripper2);                
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");

    init_gripper_affine1 =  transformTFToEigen(tf_init_gripper1);  
    init_gripper_affine2 =  transformTFToEigen(tf_init_gripper2);     

    ROS_INFO("current pose, gripper 1:");
    cout<<init_gripper_affine1.linear()<<endl;
    cout<<init_gripper_affine1.translation().transpose()<<endl;     
    
    ROS_INFO("current pose, gripper 2:");
    cout<<init_gripper_affine2.linear()<<endl;
    cout<<init_gripper_affine2.translation().transpose()<<endl;   

    js_action_client_.sendGoal(js_goal_, boost::bind(&CartMoveActionServer::js_doneCb_,this,_1,_2)); // we could also name additional callback functions here, if desired
    //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //alt--more callback funcs possible
    
    /*
   //Eigen::Vector3d tip_from_FK;
   ROS_INFO("gripper tip frame from FK: ");   
   q_vec2_jstate = g_q_vec;
   affine_gripper_wrt_psm2 = davinci_fwd_solver.fwd_kin_solve(q_vec2_jstate);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_gripper_wrt_psm2.linear()<<endl;
   tip_from_FK = affine_gripper_wrt_psm2.translation();
   cout<<"origin from FK: "<<tip_from_FK.transpose()<<endl;
   cout<<endl;
   cout<<"origin from FK in left camera frame: ";
   cout<<affine_lcamera_to_psm_two_*tip_from_FK<<endl;   
*/



    double t_timeout=arrival_time_+5.0; //wait 2 sec longer than expected duration of move
    
    bool finished_before_timeout = js_action_client_.waitForResult(ros::Duration(t_timeout));
    //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!finished_before_timeout) {
        ROS_WARN("joint-space interpolation result is overdue ");
    } else {
        ROS_INFO("finished before timeout");
    }

    ROS_INFO("completed callback" );
    cart_move_as_.setSucceeded(cart_result_); // tell the client that we were successful acting on the request, and return the "result" message 
    
    //let's remember the last pose commanded, so we can use it as start pose for next move
    gripper1_affine_last_commanded_pose_ = des_gripper1_affine_wrt_lcamera_; //origin from FK in left camera frame no TF;
    gripper2_affine_last_commanded_pose_ = des_gripper2_affine_wrt_lcamera_;    
    //and the jaw opening angles:
    last_gripper_ang1_=gripper_ang1_;
    last_gripper_ang2_=gripper_ang2_;


   //tf::StampedTransform tfResult_one,tfResult_two;  
    // tf::TransformListener tfListener;  
    tf::StampedTransform tf_init_gripper1_base,tf_init_gripper2_base,tf_init_gripper1_world,tf_world_wrt_left_optical_frame; 
    // get these transform values to CartMoveActionServer
    //Eigen::Affine3d affine_lcamera_to_psm_one,affine_lcamera_to_psm_two,affine_gripper_wrt_base;
    // need to get these poses from goal message
    Eigen::Affine3d init_gripper_affine1_base,init_gripper_affine2_base,init_gripper_affine1_world,affine_world_to_lcamera;
    tferr=true;

    ROS_INFO("waiting for tf between base and right_hand...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                //tfListener.lookupTransform("left_camera_optical_frame","one_psm_base_link",  ros::Time(0), tfResult_one);
                //tfListener.lookupTransform("left_camera_optical_frame","two_psm_base_link",  ros::Time(0), tfResult_two);
                tfListener.lookupTransform("left_camera_optical_frame","one_tool_tip_link",  ros::Time(0), tf_init_gripper1);                
                tfListener.lookupTransform("left_camera_optical_frame","two_tool_tip_link",  ros::Time(0), tf_init_gripper2);   
                 tfListener.lookupTransform("one_psm_base_link","one_tool_tip_link",  ros::Time(0), tf_init_gripper1_base); 
                 tfListener.lookupTransform("one_psm_base_link","one_tool_tip_link",  ros::Time(0), tf_init_gripper1_base); 
                  tfListener.lookupTransform("world","one_tool_tip_link",  ros::Time(0), tf_init_gripper1_world); 
                  tfListener.lookupTransform("left_camera_optical_frame","world",  ros::Time(0), tf_world_wrt_left_optical_frame);   
                tfListener.lookupTransform("two_psm_base_link","two_tool_tip_link",  ros::Time(0), tf_init_gripper2_base);                      
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");

     ROS_INFO("transform world to left_camera_optical_frame:");
    affine_world_to_lcamera = transformTFToEigen(tf_world_wrt_left_optical_frame);

    init_gripper_affine1 =  transformTFToEigen(tf_init_gripper1);  
    init_gripper_affine2 =  transformTFToEigen(tf_init_gripper2);     

    ROS_INFO("current pose, gripper 1 tf wrt left optical camera frame:");
    cout<<init_gripper_affine1.linear()<<endl;
    cout<<init_gripper_affine1.translation().transpose()<<endl;     
    
    ROS_INFO("current pose, gripper 2:");
    cout<<init_gripper_affine2.linear()<<endl;
    cout<<init_gripper_affine2.translation().transpose()<<endl;    

       init_gripper_affine1_base =  transformTFToEigen(tf_init_gripper1_base);  
    init_gripper_affine2_base =  transformTFToEigen(tf_init_gripper2_base);     

    ROS_INFO("current pose, gripper 1 wrt base:");
    cout<<init_gripper_affine1_base.linear()<<endl;
    cout<<init_gripper_affine1_base.translation().transpose()<<endl;     
    
    ros::NodeHandle nb; 
  ros::Subscriber joint_state_subscriber1= nb.subscribe("davinci/joint_states",1,jsCallback);    
  
  ROS_INFO("waiting to receive joint states");
  while (!g_got_callback)   {
      ros::spinOnce();
  }

     //Eigen::Vector3d tip_from_FK;
   ROS_INFO("gripper tip frame from FK: ");   
   q_vec1_jstate = g_q_vec;
           cout<<"using q_vec = "<<q_vec1_jstate.transpose()<<endl;
   affine_gripper_wrt_psm1_FK = davinci_fwd_solver.fwd_kin_solve(q_vec1_jstate);
   cout<<"affine linear (R): "<<endl;
   cout<<affine_gripper_wrt_psm1_FK.linear()<<endl;
   tip_from_FK = affine_gripper_wrt_psm1_FK.translation();
   cout<<"origin from FK: "<<tip_from_FK.transpose()<<endl; 
   cout<<endl;

    init_gripper_affine1_world =  transformTFToEigen(tf_init_gripper1_world);     

    ROS_INFO("current pose, gripper 1 wrt world:");
    cout<<init_gripper_affine1_world.linear()<<endl;
    cout<<init_gripper_affine1_world.translation().transpose()<<endl;     

    Eigen::Affine3d gripper_left_optical_frame;
    gripper_left_optical_frame = affine_world_to_lcamera*init_gripper_affine1_world;


    ROS_INFO("current pose, gripper 1 wrt left_camera_optical_frame:");
    cout<<gripper_left_optical_frame.linear()<<endl;
    cout<<gripper_left_optical_frame.translation().transpose()<<endl;     



    ROS_INFO("current pose, gripper 1 wrt world:");
    cout<<init_gripper_affine1_world.linear()<<endl;
    cout<<init_gripper_affine1_world.translation().transpose()<<endl;     
    ROS_INFO("current pose, gripper 2 wrt base:");
    cout<<init_gripper_affine2_base.linear()<<endl;
    cout<<init_gripper_affine2_base.translation().transpose()<<endl;    

    des_gripper_affine1_ = affine_lcamera_to_psm_one_.inverse()* init_gripper_affine1; //previous pose
    ik_solver_.ik_solve(des_gripper_affine1_); //convert desired pose into equiv joint displacements
    q_vec1 = ik_solver_.get_soln(); 
    q_vec1(6) = last_gripper_ang1_; // include desired gripper opening angle
    cout<<"q_vec1 of current pose: "<<endl;
    for (int i=0;i<6;i++) {
        cout<<q_vec1[i]<<", ";
    }
    cout<<endl;

    des_gripper_affine2_ = affine_lcamera_to_psm_two_.inverse()* init_gripper_affine2; //previous pose
    ik_solver_.ik_solve(des_gripper_affine2_); //convert desired pose into equiv joint displacements
    q_vec2 = ik_solver_.get_soln(); 
    q_vec2(6) = last_gripper_ang2_; // include desired gripper opening angle
    cout<<"q_vec2 of current pose: "<<endl;
    for (int i=0;i<6;i++) {
        cout<<q_vec2[i]<<", ";
    }
    cout<<endl;

     ROS_INFO("Done!!! ");
}

void CartMoveActionServer::js_doneCb_(const actionlib::SimpleClientGoalState& state,
        const davinci_traj_streamer::trajResultConstPtr& result) {
  ROS_INFO("done-callback pinged by joint-space interpolator action server done");
}


//this is how the joint-space interpolator action server communicates back to this node
void doneCb(const actionlib::SimpleClientGoalState& state,
        const davinci_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d", result->return_val, result->traj_id);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "cart_move_as"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
     ros::NodeHandle n; // need this to establish communications with our new node 
      //create a Subscriber object and have it subscribe to the topic "topic1" 
      // the function "myCallback" will wake up whenever a new message is published to topic1 
      // the real work is done inside the callback function 

     Vectorq7x1 q_vec_test;
      
  ros::Subscriber joint_state_subscriber= n.subscribe("davinci/joint_states",1,jsCallback);    
  
  ROS_INFO("waiting to receive joint states");
  while (!g_got_callback)   {
      ros::spinOnce();
  }
 
    Eigen::Affine3d init_gripper_affine1,init_gripper_affine2;
   ROS_INFO("getting transforms from camera to PSMs");
    tf::TransformListener tfListener;
    tf::StampedTransform tfResult_one,tfResult_two,tfResult_zero;    
    tf::StampedTransform tf_init_gripper1,tf_init_gripper2, tf_gripper1_wrt_psm_one; 
    // get these transform values to CartMoveActionServer
    Eigen::Affine3d affine_lcamera_to_psm_one,affine_lcamera_to_psm_two,affine_gripper_wrt_base,affine_lcamera_to_world,affine_gripper_wrt_base_msg;
    // need to get these poses from goal message
    Eigen::Affine3d des_gripper1_affine_wrt_lcamera,des_gripper2_affine_wrt_lcamera;
   bool tferr=true;
    ROS_INFO("waiting for tf between base and right_hand...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                tfListener.lookupTransform("left_camera_optical_frame","one_psm_base_link",  ros::Time(0), tfResult_one);
                tfListener.lookupTransform("left_camera_optical_frame","two_psm_base_link",  ros::Time(0), tfResult_two);
                tfListener.lookupTransform("left_camera_optical_frame","one_tool_tip_link",  ros::Time(0), tf_init_gripper1);
                tfListener.lookupTransform("one_psm_base_link","one_tool_tip_link",  ros::Time(0), tf_gripper1_wrt_psm_one);               
                tfListener.lookupTransform("left_camera_optical_frame","two_tool_tip_link",  ros::Time(0), tf_init_gripper2);           
                tfListener.lookupTransform("left_camera_optical_frame","world",  ros::Time(0), tfResult_zero);          
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");
    //affine_lcamera_to_psm_one is the position/orientation of psm1 base frame w/rt left camera link frame
    // need to extend this to camera optical frame
    affine_lcamera_to_world = transformTFToEigen(tfResult_zero);
    affine_lcamera_to_psm_one = transformTFToEigen(tfResult_one);
    affine_lcamera_to_psm_two = transformTFToEigen(tfResult_two); 
    affine_gripper_wrt_base = transformTFToEigen(tf_gripper1_wrt_psm_one);
    init_gripper_affine1 =  transformTFToEigen(tf_init_gripper1);  
    init_gripper_affine2 =  transformTFToEigen(tf_init_gripper2);     
    ROS_INFO("transform from left camera to world:");
    cout<<affine_lcamera_to_world.linear()<<endl;
    cout<<affine_lcamera_to_world.translation().transpose()<<endl; 
    ROS_INFO("transform from left camera to psm one:");
    cout<<affine_lcamera_to_psm_one.linear()<<endl;
    cout<<affine_lcamera_to_psm_one.translation().transpose()<<endl;
    ROS_INFO("transform from left camera to psm two:");
    cout<<affine_lcamera_to_psm_two.linear()<<endl;
    cout<<affine_lcamera_to_psm_two.translation().transpose()<<endl; 
    

    ROS_INFO("transform from left camera to tool tip one:");
    cout<<init_gripper_affine1.linear()<<endl;
    cout<<init_gripper_affine1.translation().transpose()<<endl;

     ROS_INFO("transform from left camera to tool tip two:");
    cout<<init_gripper_affine2.linear()<<endl;
    cout<<init_gripper_affine2.translation().transpose()<<endl;

    ROS_INFO("current pose, gripper 1:");
    cout<<init_gripper_affine1.linear()<<endl;
    cout<<init_gripper_affine1.translation().transpose()<<endl;     
  
    ROS_INFO("current pose, gripper 2:");
    cout<<init_gripper_affine2.linear()<<endl;
    cout<<init_gripper_affine2.translation().transpose()<<endl;   

    ROS_INFO("current pose, gripper 1 wrt one_psm_base:");
    cout<<affine_gripper_wrt_base.linear()<<endl;
    cout<<affine_gripper_wrt_base.translation().transpose()<<endl;        

    //ros::spin();

    ROS_INFO("instantiating a cartesian-move action server: ");
    CartMoveActionServer cartMoveActionServer(nh);
     ROS_INFO("Done!!!!!2 ");
    //inform cartMoveActionServer of camera frame transforms:
    cartMoveActionServer.set_lcam2world(affine_lcamera_to_world);
    cartMoveActionServer.set_lcam2psm1(affine_lcamera_to_psm_one);
    cartMoveActionServer.set_lcam2psm2(affine_lcamera_to_psm_two);
    cartMoveActionServer.set_init_pose_gripper1(init_gripper_affine1);
    cartMoveActionServer.set_init_pose_gripper2(init_gripper_affine2);    
    
    while(ros::ok()) {
      ros::spinOnce();
    }
}


