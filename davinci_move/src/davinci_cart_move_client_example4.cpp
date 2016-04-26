// cart_move_client_example: 
// wsn, Sept, 2015...example for test/debug cart_move_as action server
//move the left gripper


#include <ros/ros.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <math.h>
#include <cwru_msgs/VecOfDoubles.h>
#include <vector>


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cwru_action/cart_moveAction.h>
//#include <cwru_action/trajAction.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

#include <davinci_kinematics/davinci_kinematics.h>
#include <davinci_kinematics/davinci_joint_publisher.h>
#include <davinci_traj_streamer/davinci_traj_streamer.h>

// this to subscribe to joint states:
#include <sensor_msgs/JointState.h>
//stuff to command joint values:
//#include <baxter_core_msgs/JointCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


using namespace std;

vector <double> vec_of_centroid; 
/*
void myCallbackCentroid(const cwru_msgs::VecOfDoubles& message_holder) {
    // check for data on topic "velocity" 
    //ROS_INFO("received centroid value is: %f", message_holder.dbl_vec);
    //vector <double> vec_of_doubles = message_holder.dbl_vec; // post the received data in a global var for access by 
    vec_of_centroid = message_holder.dbl_vec;
    //main prog. 
}
*/
void myCallbackCentroid(const geometry_msgs::Point positionPoint) {
    // check for data on topic "velocity" 
    //ROS_INFO("received centroid value is: %f", message_holder.dbl_vec);
    //vector <double> vec_of_doubles = message_holder.dbl_vec; // post the received data in a global var for access by 
    double xx;
    double yy;
    double zz;
    xx = positionPoint.x;
    yy = positionPoint.y;
    zz = positionPoint.z;
    vec_of_centroid.push_back(xx);
    vec_of_centroid.push_back(yy);
    vec_of_centroid.push_back(zz);
    //main prog. 
}

/*
geometry_msgs::PoseStamped get_rt_tool_pose_stamped(void) { 
  geometry_msgs::PoseStamped tool_pose_stamped_;
  return tool_pose_stamped_;
}
*/
Eigen::Affine3d get_rt_tool_pose_stamped(void) { 
  Eigen::Affine3d tool_pose_stamped_;
  return tool_pose_stamped_;
}
/*
geometry_msgs::PoseStamped get_rt_tool_pose_stamped(void) { 
  geometry_msgs::PoseStamped tool_pose_stamped_;
  return tool_pose_stamped_;
}
*/
//utility fnc to convert an Eigen::Affine3d object into an equivalent geometry_msgs::Pose object
geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e) {
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaterniond q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

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


// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const cwru_action::cart_moveResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    //ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
}

//gripper 1 is robot's "right" arm; gripper 2 is left
cwru_action::cart_moveGoal pack_goal(Eigen::Affine3d affine_des_gripper1,
				     Eigen::Affine3d affine_des_gripper2,
				     double jaw1_angle,double jaw2_angle,double move_time) {
        cwru_action::cart_moveGoal goal;

	//pack the goal object:
   	goal.gripper_jaw_angle1 = jaw1_angle;
  	goal.gripper_jaw_angle2 = jaw2_angle;
	goal.move_time = move_time;

	//nuisance...must re-expressed desired poses from Affine to PoseStamped to populate action goal
   	geometry_msgs::PoseStamped des_pose_gripper1;
          des_pose_gripper1.pose = transformEigenAffine3dToPose(affine_des_gripper1);
	  des_pose_gripper1.header.stamp = ros::Time::now();
	geometry_msgs::PoseStamped des_pose_gripper2;
	  des_pose_gripper2.pose = transformEigenAffine3dToPose(affine_des_gripper2);
	  des_pose_gripper2.header.stamp = ros::Time::now();

	goal.des_pose_gripper1 = des_pose_gripper1;
	goal.des_pose_gripper2 = des_pose_gripper2;
 
	return goal;
}


int main(int argc, char** argv) {
        ros::init(argc, argv, "cart_move_client_node"); // name this node 
        ros::NodeHandle nh; //standard ros node handle      

    ros::Subscriber my_subscriber_object = nh.subscribe("blue_centroid", 1, myCallbackCentroid);


    Eigen::Affine3d affine_lcamera_to_psm_one,affine_lcamera_to_psm_two,affine_gripper_wrt_base;




    tf::StampedTransform tf_sensor_to_base_frame; //transform sensor frame to davinci base frame
    tf::StampedTransform tfResult_one,tfResult_two,tf_camera_to_left_optical,tf_camera_to_world;   
    tf::TransformListener tf_listener;          //start a transform listener


    Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    Davinci_IK_solver ik_solver;
  

   Eigen::Affine3d affine_one_psm_wrt_world, affine_left_camera_wrt_world; 
   tf::StampedTransform tfResult_psm_one_to_world,tfResult_left_camera_to_world;
    bool tferr = true;
    //------Subscribe to Tranform Listener-------------
    ROS_INFO("waiting for tf between one_psm_base_link and camera frame...");
    while (tferr)
    {
        tferr = false;
        try
        {
            tf_listener.lookupTransform("one_psm_base_link","camera", 
                                        ros::Time(0), 
                                        tf_sensor_to_base_frame);

            tf_listener.lookupTransform("left_camera_optical_frame","one_psm_base_link",  ros::Time(0), tfResult_one);
            tf_listener.lookupTransform("left_camera_optical_frame","two_psm_base_link",  ros::Time(0), tfResult_two);

            tf_listener.lookupTransform("left_camera_optical_frame","camera",  ros::Time(0), tf_camera_to_left_optical);
            tf_listener.lookupTransform("world","camera",  ros::Time(0), tf_camera_to_world);
            tf_listener.lookupTransform("world","one_psm_base_link",  ros::Time(0), tfResult_psm_one_to_world);
            tf_listener.lookupTransform("world","left_camera_optical_frame",  ros::Time(0), tfResult_left_camera_to_world);
        }
        catch (tf::TransformException &exception)
        {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }

 //tf-listener found a complete chain from kinect pc frame to Torso;
    ROS_INFO("tf is good"); 
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3d affine_camera_wrt_base_from_tf, affine_camera_wrt_left_optical,affine_camera_wrt_world;  
     // we'll get the individual tf's from the transform listener
    // this one has already been set--complete transform from sensor frame to base
    affine_camera_wrt_world = transformTFToEigen(tf_camera_to_world);
    affine_one_psm_wrt_world = transformTFToEigen(tfResult_psm_one_to_world);
    affine_left_camera_wrt_world = transformTFToEigen(tfResult_left_camera_to_world);
    // here's what we get from transform listener in single request from camera frame to base
    ROS_INFO("affine of base wrt to world: ");
    ROS_INFO("orientation: ");
    cout<<affine_one_psm_wrt_world.linear()<<endl;
    cout<<"origin: "<<affine_one_psm_wrt_world.translation().transpose()<<endl;

    ROS_INFO("affine of left optical frame wrt to world: ");
    ROS_INFO("orientation: ");
    cout<<affine_left_camera_wrt_world.linear()<<endl;
    cout<<"origin: "<<affine_left_camera_wrt_world.translation().transpose()<<endl;

    affine_camera_wrt_base_from_tf = transformTFToEigen(tf_sensor_to_base_frame);
    affine_camera_wrt_left_optical = transformTFToEigen(tf_camera_to_left_optical);
    // here's what we get from transform listener in single request from camera frame to base
    ROS_INFO("affine of camera_frame w/rt base, per tf: ");
    ROS_INFO("orientation: ");
    cout<<affine_camera_wrt_base_from_tf.linear()<<endl;
    cout<<"origin: "<<affine_camera_wrt_base_from_tf.translation().transpose()<<endl;
    

    ROS_INFO("affine of camera_frame w/rt left_optical, per tf: ");
    ROS_INFO("orientation: ");
    cout<<affine_camera_wrt_left_optical.linear()<<endl;
    cout<<"origin: "<<affine_camera_wrt_left_optical.translation().transpose()<<endl;


    ROS_INFO("affine of camera_frame w/rt world, per tf: ");
    ROS_INFO("orientation: ");
    cout<<affine_camera_wrt_world.linear()<<endl;
    cout<<"origin: "<<affine_camera_wrt_world.translation().transpose()<<endl;
    int nvals = vec_of_centroid.size(); //ask the vector how long it is


    Eigen::Vector3d p_wrt_camera;

//p_wrt_camera << 0,0,0;

    for (int i=0;i<nvals;i++) {
        ROS_INFO("%f",vec_of_centroid[i]); //print out all the values
    // note: with corresponding publisher, this vector gets longer each publication
    }
    ROS_INFO("\n");
    for (int i=0;i<nvals;i++) {
        p_wrt_camera[i]=vec_of_centroid[i];
        ROS_INFO("%f",p_wrt_camera[i]); //print out all the values
    // note: with corresponding publisher, this vector gets longer each publication
    }
    //let's say we have a point, "p", as detected in the sensor frame;
    // arbitrarily, initialize this to [1;2;3]
    //p_wrt_camera = vec_of_centroid;
    ROS_INFO("\n");
    cout<<"blue pixel point w/rt sensor: "<<p_wrt_camera.transpose()<<endl;
    // here's how to convert this point to the world frame:
    Eigen::Vector3d p_wrt_world,p_wrt_psm_one,p_wrt_left_optical_frame;
    p_wrt_world = affine_camera_wrt_world*p_wrt_camera;
     ROS_INFO("\n");
    cout<<"blue pixel point w/rt world: "<<p_wrt_world.transpose()<<endl;
    p_wrt_psm_one = affine_camera_wrt_base_from_tf * p_wrt_camera;
    ROS_INFO("\n");
    cout<<"blue pixel point w/rt base: "<<p_wrt_psm_one.transpose()<<endl;
    p_wrt_left_optical_frame = affine_camera_wrt_left_optical * p_wrt_camera;
    ROS_INFO("\n");
    cout<<"blue pixel point w/rt left_camera_optical_frame: "<<p_wrt_left_optical_frame.transpose()<<endl;

    //we can transform in the opposite direction with the transform inverse:
    Eigen::Vector3d p_back_in_camera_frame;
    p_back_in_camera_frame = affine_camera_wrt_world.inverse()*p_wrt_world;
     ROS_INFO("\n");
    cout<<" p back in sensor frame: "<<p_back_in_camera_frame.transpose()<<endl;
    

    affine_lcamera_to_psm_one = transformTFToEigen(tfResult_one);
    affine_lcamera_to_psm_two = transformTFToEigen(tfResult_two); 
    ROS_INFO("transform from left camera to psm one:");
    cout<<affine_lcamera_to_psm_one.linear()<<endl;
    cout<<affine_lcamera_to_psm_one.translation().transpose()<<endl;
    ROS_INFO("transform from left camera to psm two:");
    cout<<affine_lcamera_to_psm_two.linear()<<endl;
    cout<<affine_lcamera_to_psm_two.translation().transpose()<<endl; 

    Eigen::Vector3d p_wrt_lcamera, p_wrt_lcamera2;
    p_wrt_lcamera = affine_lcamera_to_psm_one* p_wrt_psm_one;
    p_wrt_lcamera2 = affine_camera_wrt_left_optical* p_wrt_camera;
    cout<<"blue pixel point w/rt left camera frame before adjustment: "<<p_wrt_lcamera.transpose()<<endl;
    ROS_INFO("\n");
    cout<<"blue pixel point w/rt left camera frame before adjustment2: "<< p_wrt_lcamera2.transpose()<<endl;
    p_wrt_lcamera[2] = p_wrt_lcamera[2];
    p_wrt_lcamera2[2] = p_wrt_lcamera2[2];
     ROS_INFO("\n");
    cout<<"blue pixel point w/rt left camera frame after adjustment: "<<p_wrt_lcamera.transpose()<<endl;
  
	Eigen::Affine3d affine_des_gripper1,affine_des_gripper2;
        double gripper1_jaw_angle,gripper2_jaw_angle,move_time;
	Eigen::Vector3d tip_origin1,tip_origin2;
	Eigen::Matrix3d R_gripper1,R_gripper2;
	Eigen::Vector3d z_vec1,z_vec2,x_vec1,x_vec2,y_vec1,y_vec2;

  //affine_des_gripper2.translation()=p_wrt_lcamera;      
  /*
    z_vec1<<1,0,0;
  x_vec1<<0,0,-1;
  y_vec1 = z_vec1.cross(x_vec1);
  R_gripper1.col(0)=x_vec1;
  R_gripper1.col(1)=y_vec1;        
  R_gripper1.col(2)=z_vec1;
  affine_des_gripper1.linear()=R_gripper1;
  */
        // hard-code desired gripper poses
        // gripper 1 is DaVinci's right arm (from robot's viewpoint)
    
  /*  tip_origin1<<0,0,0;
	z_vec1<<1,0,0;
	x_vec1<<0,0,-1;
	y_vec1 = z_vec1.cross(x_vec1);
 	R_gripper1.col(0)=x_vec1;
 	R_gripper1.col(1)=y_vec1;        
 	R_gripper1.col(2)=z_vec1;


	affine_des_gripper1.linear()=R_gripper1;
        //p_wrt_lcamera[1] = 0.038;
     //p_wrt_lcamera[0] = -0.038;
	affine_des_gripper1.translation()=p_wrt_lcamera;*/
Eigen::Vector3d cube_wrt_world;
cube_wrt_world << -0.00594228, -0.00145275,  0.703299;
 tip_origin1 = affine_left_camera_wrt_world.inverse() * cube_wrt_world;

  tip_origin1  << -0.00984301, 0.00125127,   0.196701;
  ROS_INFO("\n");
    //cout<<" p cube in left optical frame: "<< tip_origin1.transpose()<<endl;
    z_vec1<<0.974336,-0.216532,0.0615097;
    x_vec1<<0.0551814,0.0351571,-0.997857;
    y_vec1 = z_vec1.cross(x_vec1);
    R_gripper1.col(0)=x_vec1;
    R_gripper1.col(1)=y_vec1;        
    R_gripper1.col(2)=z_vec1;
    affine_des_gripper1.linear()=R_gripper1;
        //p_wrt_lcamera[1] = 0.038;
     //p_wrt_lcamera[0] = -0.038;
    p_wrt_lcamera[2] = p_wrt_lcamera[2] - 0.1;
    affine_des_gripper1.translation()=p_wrt_lcamera;
// affine_des_gripper1.translation()=tip_origin1;
/*
 0.0133402   0.133221   0.990997
-0.0339994   0.990572  -0.132706
 -0.999333 -0.0319229  0.0177439
    /   tip_origin1<<  -0.0274186, 0.0209048,  0.190676;
    z_vec1<<0.990997,-0.132706,0.0177439;
    x_vec1<<0.0133402,-0.0339994,-0.999333;


     0.0313775   0.217676   0.975516
-0.0464126   0.975262  -0.216127
 -0.998429 -0.0384948  0.0407042
-0.0284446 0.0186364  0.191883

 0.0551814   0.218231   0.974336
-0.0351571   0.975642  -0.216532
 -0.997857 -0.0223063  0.0615097
-0.0289714 0.0178309  0.192823

*/
  /*  tip_origin1<<  -0.0289714, 0.0178309,  0.192823;
    z_vec1<<0.974336,-0.216532,0.0615097;
    x_vec1<<0.0551814,0.0351571,-0.997857;
    y_vec1 = z_vec1.cross(x_vec1);
    R_gripper1.col(0)=x_vec1;
    R_gripper1.col(1)=y_vec1;        
    R_gripper1.col(2)=z_vec1;
    affine_des_gripper1.linear()=R_gripper1;
        //p_wrt_lcamera[1] = 0.038;
     //p_wrt_lcamera[0] = -0.038;
    affine_des_gripper1.translation()=tip_origin1;*/
/*0.00408784  0.0658249   0.997823
-0.0205252   0.997626 -0.0657279
 -0.999781 -0.0202118  0.0054292
-0.0269578 0.0228099  0.189967
*/  Eigen::Vector3d p_cube_wrt_world;
p_cube_wrt_world = affine_left_camera_wrt_world*tip_origin1;
 ROS_INFO("\n");
    cout<<" p cube in world frame: "<<p_cube_wrt_world.transpose()<<endl;
    
    	gripper1_jaw_angle=0.7;

        // and specify the "left" gripper pose:


    
	tip_origin2<<0.05,0,0.15;
    /*
	z_vec2<<-1,0,0;
	x_vec2<<0,0,-1;
	y_vec2 = z_vec2.cross(x_vec2);
 	R_gripper2.col(0)=x_vec2;
 	R_gripper2.col(1)=y_vec2;        
 	R_gripper2.col(2)=z_vec2;
	affine_des_gripper2.linear()=R_gripper2;
    //affine_des_gripper2.translation()=p_wrt_lcamera2;*/

    affine_des_gripper2.translation()=tip_origin2;
	gripper2_jaw_angle=0.0;
	move_time=5.0;

          
        // here is a "goal" object compatible with the server, as defined in cwru_action_server/cart_move
        cwru_action::cart_moveGoal goal, goal1, goal2, goal3; 

	goal = pack_goal(affine_des_gripper1,affine_des_gripper2,gripper1_jaw_angle,gripper2_jaw_angle,move_time);


    double move_time1=2.0;
    tip_origin1[0] = -0.00604301;
    affine_des_gripper1.translation()=tip_origin1;
    goal1  = pack_goal(affine_des_gripper1,affine_des_gripper2,gripper1_jaw_angle,gripper2_jaw_angle,move_time1+move_time);

    double move_time2=2.0;
    gripper1_jaw_angle=0.3;
	goal2  = pack_goal(affine_des_gripper1,affine_des_gripper2,gripper1_jaw_angle,gripper2_jaw_angle,move_time2+move_time1+move_time);

    double move_time3=2.5;
    tip_origin1[2] = 0.17;
	affine_des_gripper1.translation()=tip_origin1;
	goal3  = pack_goal(affine_des_gripper1,affine_des_gripper2,gripper1_jaw_angle,gripper2_jaw_angle,move_time3+move_time2+move_time1+move_time);


      /*
	//pack the goal object:
   	goal.gripper_jaw_angle1 = gripper1_jaw_angle;
  	goal.gripper_jaw_angle2 = gripper2_jaw_angle;
	goal.move_time = move_time;

	//nuisance...must re-expressed desired poses from Affine to PoseStamped to populate action goal
   	geometry_msgs::PoseStamped des_pose_gripper1;
          des_pose_gripper1.pose = transformEigenAffine3dToPose(affine_des_gripper1);
	  des_pose_gripper1.header.stamp = ros::Time::now();
	geometry_msgs::PoseStamped des_pose_gripper2;
	  des_pose_gripper2.pose = transformEigenAffine3dToPose(affine_des_gripper2);
	  des_pose_gripper2.header.stamp = ros::Time::now();

	goal.des_pose_gripper1 = des_pose_gripper1;
	goal.des_pose_gripper2 = des_pose_gripper2;
       */

        // use the name of our server, which is: cartMoveActionServer (named in cart_move_as.cpp)
        actionlib::SimpleActionClient<cwru_action::cart_moveAction> action_client("cartMoveActionServer", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = false;
        int max_tries = 0;
        while (!server_exists) {
           server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
           // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
           ros::spinOnce();
           ros::Duration(0.1).sleep();
           ROS_INFO("retrying...");
           max_tries++;
           if (max_tries>100)
               break;
        }


        if (!server_exists) {
            ROS_WARN("could not connect to server; quitting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        //server_exists = action_client.waitForServer(); //wait forever 
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        // stuff a goal message:
        //goal.traj_id = g_count; // this merely sequentially numbers the goals sent

        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(move_time+2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 0;
        }
        else {
            ROS_INFO("finished before timeout");
        }



		  ros::Duration(1).sleep(); // sleep for half a second// Handle USB events
		  ros::spinOnce();                   // Handle ROS events
          
		// new goal
		action_client.sendGoal(goal1,&doneCb);

        finished_before_timeout = action_client.waitForResult(ros::Duration(move_time+move_time1+2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result new ");
            return 0;
        }
        else {
            ROS_INFO("finished before timeout new");
        }

        		  ros::Duration(1).sleep(); // sleep for half a second// Handle USB events
		  ros::spinOnce();   

		action_client.sendGoal(goal2,&doneCb);

		 finished_before_timeout = action_client.waitForResult(ros::Duration(move_time+move_time1+move_time2+2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result new2 ");
            return 0;
        }
        else {
            ROS_INFO("finished before timeout new2");
        }

          ros::Duration(1).sleep(); // sleep for half a second// Handle USB events
          ros::spinOnce();   

        action_client.sendGoal(goal3,&doneCb);

         finished_before_timeout = action_client.waitForResult(ros::Duration(move_time+move_time1+move_time2+move_time3+2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result new2 ");
            return 0;
        }
        else {
            ROS_INFO("finished before timeout new2");
        }
          	
    return 0;
}

