#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream> 
#include <math.h>
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "tf2_ros/tranform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"


std_srvs::Trigger begin_comp;

// Global vector declarations
std::vector<osrf_gear::Order::ConstPtr> orders;
std::vector<std::string> prod_types;
std::vector<std::vector<osrf_gear::StorageUnit>> prod_bins;
std::vector<osrf_gear::LogicalCameraImage> bin_cams;
std::vector<osrf_gear::LogicalCameraImage> qual_cams;
std::vector<osrf_gear::LogicalCameraImage> agv_cams;
geometry_msgs::PoseStamped part_pose, goal_pose;
// current state of joints of robot
sensor_msgs::JointState joint_states;

// Declare the transformation buffer to maintain a list of transformations
tf2_ros::Buffer tfBuffer;
// Instantiate a listener that listens to the tf and tf_static topics and to update the buffer.
tf2_ros::TransformListener tfListener(tfBuffer);

// Instantiate variables for use with the kinematic system.
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6]
trajectory_msgs::JointTrajectory desired;


void ordersCallback(const osrf_gear::Order::ConstPtr& msg) {
	// Recieve order and assign to msg variable
	ROS_INFO("Order received: [%s]", msg->specific_order.c_str());
	// Push recieved order onto back of vector
	orders.push_back(msg);
}

void binCamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, int bin){
    bin_cams.at(bin-1) = *msg;
}

void avgCamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, int agv){
    agv_cams.at(agv-1) = *msg;
}

void qualCamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, int bin){
    qual_cams.at(bin-1) = *msg;
}

void getProducts(const osrf_gear::Order& order)
{
  // Iterate through all shipments in order and all products in shipment
  for (const auto& shipment : order.shipments){
    for (const auto& product : shipment.products){
      // Add product type to global vector
      prod_types.push_back(product.type);

      // Gets material location of product
      osrf_gear::GetMaterialLocations get_material_location;
      get_mat_loc.request.material_type=product.type;
      mat_loc_client.call(get_material_location);
      
      // Adds bins where product can be found to global vector
      prod_bins.push_back(get_material_location.response.storage_units);

      // Displays info for product using Pose type
      geometry_msgs::Pose prod_pose = get_product_pose(&product.type, &get_material_location.response.storage_units.front())
      ROS_INFO("PRODUCT: [%s] BIN : [%s] POSE : [%s]", product.type, get_material_location.response.storage_units.unit_id, prod_pose);
    }
  }
}


geometry_msgs::TransformStamped get_transform(const std:String& end_frame, const std:String& start_frame){ 
    geometry_msgs::TransformStamped tfStamped
    try {
        tfStamped = tfBuffer.lookupTransform(*end_frame, *start_frame, ros::Time(0.0), ros::Duration(1.0));
        ROS_DEBUG("Transform to [%s] from [%s}]", tfStamped.header.frame_id.c_str(). tfStamped.child_frame_id.c_str());
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
    // tf2_ross::Buffer.lookupTransform("to_frame", "from_frame", "how_recent", "how_long_to_wait_for_transform");
}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_msg){
  joint_states = *joint_msg;
}



int main(int argc, char **argv) {


    // Initialize important variables and processes
    ros::init(argc, argv, "ariac_view");
    ros::NodeHandle n;
    std_srvs::SetBool my_bool_var;
    ros::Rate loop_rate(0.5);

    // Subscribe to orders messages
    orders.clear();
    ros::Subscriber orders_sub = n.subscribe("/ariac/orders", 1000, ordersCallback);

    // Subscribe to agv cameras
    auto agv1_sub = nh.subscribe("/ariac/agv1/state", 1000, agv1_callback);
    auto agv2_sub = nh.subscribe("/ariac/agv2/state", 1000, agv2_callback);

    // Create arrays to hold subscribers
    std::array<ros::Subscriber, 6> bin_cam_subs;
    std::array<ros::Subscriber, 2> agv_cam_subs;
    std::array<ros::Subscriber, 2> qual_cam_subs;

    ROS_INFO("Subscribing to logic cameras...");

    // Generate bin camera subscribers
    for (int i = 0; i < sizeof(bin_cam_subs); i++){
        const std::string topic = "/ariac/logical_camera_bin" + std::to_string(i+1);
        bin_cam_subs[i] = nh.subscribe<osrf_gear::LogicalCameraImage>(topic, 1000, binCamCallback);
    }

    ROS_INFO("Subscribing to agv and  quality cameras");

    // Generate subscribers for agv and quality camera
    for (int i = 0; i <= 1; i++) {
        const std::string agv_topic = "/ariac/logical_camera_agv" + std::to_string(i);
        const std::string quality_topic = "/ariac/quality_control_sensor_" + std::to_string(i);

        agv_cam_subs[i] = nh.subscribe<osrf_gear::LogicalCameraImage>(agv_topic, 1000, avgCamCallback);
        qual_cam_subs[i] = nh.subscribe<osrf_gear::LogicalCameraImage>(quality_topic, 1000, qualCamCallback);
    }
    //joint_state subscriber
    ros::Subscriber joint_states_subscriber = n.subscribe("/ariac/arm1/joint_states", 10, jointStatesCallback);
    // Where is the end effector given the joint angles.
    // joint_states.position[0] is the linear_arm_actuator_joint
    q_pose[0] = joint_states.position[1];
    q_pose[1] = joint_states.position[2];
    q_pose[2] = joint_states.position[3];
    q_pose[3] = joint_states.position[4];
    q_pose[4] = joint_states.position[5];
    q_pose[5] = joint_states.position[6];
    
    ur_kinematics::forward((float *)&q_pose, (double *)&T_pose);
    
    // Desired pose of the end effector wrt the base_link.
    T_des[0][3] = desired.pose.position.x;
    T_des[1][3] = desired.pose.position.y;
    T_des[2][3] = desired.pose.position.z + 0.3; // above part
    T_des[3][3] = 1.0;
    // The orientation of the end effector so that the end effector is down.
    T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
    T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
    T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
    T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;
    
    int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
    
    
    
    	// Declare a variable for generating and publishing a trajectory.
	trajectory_msgs::JointTrajectory joint_trajectory;
	// Fill out the joint trajectory header.
	// Each joint trajectory should have an non-monotonically increasing sequence number.
	joint_trajectory.header.seq = count++;
	joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
	joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.
	5 of 9
	© Copyright 2019-2023, Gregory S. Lee, Ph.D.
	Modern Robot Programming ECSE 373/473
	// Set the names of the joints being used. All must be present.
	joint_trajectory.joint_names.clear();
	joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
	joint_trajectory.joint_names.push_back("shoulder_pan_joint");
	joint_trajectory.joint_names.push_back("shoulder_lift_joint");
	joint_trajectory.joint_names.push_back("elbow_joint");
	joint_trajectory.joint_names.push_back("wrist_1_joint");
	joint_trajectory.joint_names.push_back("wrist_2_joint");
	joint_trajectory.joint_names.push_back("wrist_3_joint");
	// Set a start and end point.
	joint_trajectory.points.resize(2);
	// Set the start point to the current position of the joints from joint_states.
	joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
				joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}
	// When to start (immediately upon receipt).
	joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
	// Must select which of the num_sols solutions to use. Just start with the first.
	int q_des_indx = 0;
	// Set the end point for the movement
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
	// Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse
	kinematics solution.
	joint_trajectory.points[1].positions[0] = joint_states.position[1];
	// The actuators are commanded in an odd order, enter the joint positions in the correct
	positions
	for (int indy = 0; indy < 6; indy++) {
	joint_trajectory.points[1].positions[indy + 1] = q_sols[q_sols_indx][indy];
	}
	// How long to take for the movement.
	joint_trajectory.points[1].time_from_start = ros::Duration
    
    // Create a subscriber for receiving the state of the joints of the robot.
    ros::Subscriber joint_states_h = n.subscribe("ariac/arm1/joint_states", 10, jointCB);
    



    // Initiate the competition client
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    begin_client.waitForExistence();
    my_bool_var.request.data = true;


    // Service success message variable
	std_srvs::Trigger begin_comp;

	// Service call success variable
	int service_call_succeeded;
	service_call_succeeded = begin_client.call(begin_comp);
	
	// Outputs for service or service call failure and success
	if(!service_call_succeeded){
		ROS_ERROR("Competition service call failed! Goodness Gracious!!");
	}
	
	if(!begin_comp.response.success) {
		ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
	} 
	else {
		ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
	}
	
	// Wait for existence of PoseIK service
	 if (!ros::service::waitForService("PoseIK", 10)) {
	ROS_WARN("Inverse Kinematic service not found");
	}
	
	// Wait for existence of material_locations service
	if (!ros::service::waitForService("/ariac/material_locations", ros::Duration(10))) {
    	ROS_ERROR("Material location service is not available after 30s.");
	}
	
    	ROS_INFO("Beginning of ROS loop");
    	
    	ros::AsyncSpinner spinner(1); // Use 1 thread
	spinner.start(); // A spinner makes calling ros::spin() unnecessary.
	
	while(ros::ok()) {
		
		// Clear variables to be used
		prod_types.clear();
		prod_bins.clear();
		
		// Publish current joints states every 10 seconds
		ROS_INFO_STREAM_THROTTLE(10, joint_states);
		

		// Iterate while there are orders left to process
		while (order.size() > 0){
			// Get first order in queue
			order=orders.at(0);
			
			getProducts(&order);

			// Display the type of the first object in the order and its location
			ROS_INFO("The first product is of type : %s,  and is located in the bin : %s", prod_types.front(),prod_bins.front().front().unit_id);
		    
		
		
			// Copy pose from the logical camera.
			part_pose.pose = get_product_pose(&product_type_vector.at(0), &product_bin_vector.at(0).at(0));
			tf2::doTransform(&part_pose, &goal_pose, &transformStamped);

			// Rearrange data to more useful formula "logical_camera1_frame" and "arm1_base_frame"
			std::String bin = prod_bins.at(0).at(0).unit_id;
			char n = current_bin[3];
			std::String cam_frame = "logical_camera";
			cam_frame.append(n);
			cam_frame.append("_frame");
			std::String arm_fr = "arm1_base_frame";
			geometry_msgs::TransformStamped log_transform = get_transform(&arm_frame,&cam_frame)
			

			// Add height to the goal pose.
			goal_pose.pose.position.z += 0.10; // 10 cm above the part
			// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
			goal_pose.pose.orientation.w = 0.707;
			goal_pose.pose.orientation.x = 0.0;
			goal_pose.pose.orientation.y = 0.707;
			goal_pose.pose.orientation.z = 0.0;

			// Ask arm to move
			tf2::doTransform(&part_pose, &goal_pose, &log_transform);
			 
			loop_rate.sleep();
			++count;
			
			// Remove Processed order from queue
			orders.erase(orders.begin());
		}

	}

	return 0;

	
}
