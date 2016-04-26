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

void myCallbackCentroid(const cwru_msgs::VecOfDoubles& message_holder) {
    // check for data on topic "velocity" 
    //ROS_INFO("received centroid value is: %f", message_holder.dbl_vec);
    //vector <double> vec_of_doubles = message_holder.dbl_vec; // post the received data in a global var for access by 
    vec_of_centroid = message_holder.dbl_vec;
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


    Eigen::Affine3d affine_gripper_wrt_base;




    tf::StampedTransform tf_sensor_to_base_frame; //transform sensor frame to davinci base frame
    tf::StampedTransform tfResult_one,tfResult_two;   
    tf::TransformListener tf_listener;          //start a transform listener


    Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    Davinci_IK_solver ik_solver;
  

    bool tferr = true;
    //------Subscribe to Tranform Listener-------------
    ROS_INFO("waiting for tf between two_psm_base_link and camera frame...");
    while (tferr)
    {
        tferr = false;
        try
        {
            tf_listener.lookupTransform("two_psm_base_link","camera", 
                                        ros::Time(0), 
                                        tf_sensor_to_base_frame);
        
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
    Eigen::Affine3d affine_camera_wrt_base_from_tf;  
     // we'll get the individual tf's from the transform listener
    // this one has already been set--complete transform from sensor frame to base
    affine_camera_wrt_base_from_tf = transformTFToEigen(tf_sensor_to_base_frame);
  
    // here's what we get from transform listener in single request from camera frame to base
    ROS_INFO("affine of camera_frame w/rt base, per tf: ");
    ROS_INFO("orientation: ");
    cout<<affine_camera_wrt_base_from_tf.linear()<<endl;
    cout<<"origin: "<<affine_camera_wrt_base_from_tf.translation().transpose()<<endl;



    int nvals = vec_of_centroid.size(); //ask the vector how long it is

    for (int i=0;i<nvals;i++) {
        ROS_INFO("%f",vec_of_centroid[i]); //print out all the values
    // note: with corresponding publisher, this vector gets longer each publication
    }
    ROS_INFO("\n");
    Eigen::Vector3d p_wrt_camera;
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
    Eigen::Vector3d p_wrt_world;
    p_wrt_world = affine_camera_wrt_base_from_tf*p_wrt_camera;
     ROS_INFO("\n");
    cout<<"blue pixel point w/rt world: "<<p_wrt_world.transpose()<<endl;
    
    //we can transform in the opposite direction with the transform inverse:
    Eigen::Vector3d p_back_in_camera_frame;
    p_back_in_camera_frame = affine_camera_wrt_base_from_tf.inverse()*p_wrt_world;
     ROS_INFO("\n");
    cout<<" p back in sensor frame: "<<p_back_in_camera_frame.transpose()<<endl;


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
  
    tip_origin1<<0,0,0;
	z_vec1<<0,0,1;
	x_vec1<<1,0,0;
	y_vec1 = z_vec1.cross(x_vec1);
 	R_gripper1.col(0)=x_vec1;
 	R_gripper1.col(1)=y_vec1;        
 	R_gripper1.col(2)=z_vec1;
	affine_des_gripper1.linear()=R_gripper1;
	affine_des_gripper1.translation()=tip_origin1;
	gripper1_jaw_angle=0.3;

        // and specify the "left" gripper pose:
    /*
	//tip_origin2<<0,0,0.155;
	z_vec2<<1,0,0;
	x_vec2<<0,-1,0;
	y_vec2 = z_vec2.cross(x_vec2);
 	R_gripper2.col(0)=x_vec2;
 	R_gripper2.col(1)=y_vec2;        
 	R_gripper2.col(2)=z_vec2;
	affine_des_gripper2.linear()=R_gripper2;*/
    //p_wrt_lcamera[1] = 0.038;
     //p_wrt_lcamera[0] = -0.038;
	affine_des_gripper2.translation()=p_wrt_world;
	gripper2_jaw_angle=0.1;
	move_time=3.0;

          
        // here is a "goal" object compatible with the server, as defined in cwru_action_server/cart_move
        cwru_action::cart_moveGoal goal; 

	goal = pack_goal(affine_des_gripper1,affine_des_gripper2,gripper1_jaw_angle,gripper2_jaw_angle,move_time);

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

    return 0;
}

