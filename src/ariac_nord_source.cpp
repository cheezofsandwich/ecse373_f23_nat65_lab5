#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream> 
#include <math.h>
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/StorageUnit.h"
#include "ik_service/PoseIK.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include <unistd.h>


std_srvs::Trigger begin_comp;
std::vector<osrf_gear::Order> orders;
osrf_gear::LogicalCameraImage::ConstPtr camera_images[10];
sensor_msgs::JointState joint_states;
std::string camera_topics[] = {
	"/ariac/logical_camera_bin1",
	"/ariac/logical_camera_bin2",
	"/ariac/logical_camera_bin3",
	"/ariac/logical_camera_bin4",
	"/ariac/logical_camera_bin5",
	"/ariac/logical_camera_bin6",
	"/ariac/logical_camera_agv1",
	"/ariac/logical_camera_agv2",
	"/ariac/quality_control_sensor_1",
	"/ariac/quality_control_sensor_2"
	};
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];
int count = 0;
trajectory_msgs::JointTrajectory desired;


void ordersCallback(const osrf_gear::Order msg) {

	// Push recieved order onto back of vector
	orders.push_back(msg);
	// Publish to feed that order was recieved
	ROS_INFO("Received: %s", msg.order_id.c_str());
}

void cameraCallback(int index, const osrf_gear::LogicalCameraImage::ConstPtr& image) {
	camera_images[index] = image;
}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_msg){
	joint_states = *joint_msg;
}


geometry_msgs::TransformStamped get_transform(const std::string frame){ 
	geometry_msgs::TransformStamped transform;
	
	try {	
		transform = tfBuffer.lookupTransform("arm1_base_link", frame, ros::Time(0.0), ros::Duration(1.0));
		ROS_DEBUG("Transform to [%s] from [%s]", transform.header.frame_id.c_str(), transform.child_frame_id.c_str());	
		
	} catch (tf2::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}

	return transform;
}



trajectory_msgs::JointTrajectory prepareJT() {
	
	trajectory_msgs::JointTrajectory joint_trajectory;

	int count = 0;
	joint_trajectory.header.seq = count++;
	joint_trajectory.header.stamp = ros::Time::now();
	joint_trajectory.header.frame_id = "/world";

	// Set the names of the joints being used.

	joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
	joint_trajectory.joint_names.push_back("shoulder_pan_joint");
	joint_trajectory.joint_names.push_back("shoulder_lift_joint");
	joint_trajectory.joint_names.push_back("elbow_joint");
	joint_trajectory.joint_names.push_back("wrist_1_joint");
	joint_trajectory.joint_names.push_back("wrist_2_joint");
	joint_trajectory.joint_names.push_back("wrist_3_joint");

	// Set beginning and end locations
	joint_trajectory.points.resize(2);

	// Set start to be current position
	joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	
	for (int i = 0; i < joint_trajectory.joint_names.size(); i++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (joint_trajectory.joint_names[i] == joint_states.name[indz]) {
				joint_trajectory.points[0].positions[i] = joint_states.position[indz];
				break;
			}
		}
	}

	joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

	// Set final point
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

	// Set linear arm state separately
	joint_trajectory.points[1].positions[0] = joint_states.position[1];
	joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
    
	return joint_trajectory;	
}
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_as;
void callActionServer(trajectory_msgs::JointTrajectory joint_trajectory) {
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_as;

	// Create location to store running action server
	control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
	
	joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
	joint_trajectory_as.action_goal.header.seq = count++;
	joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
	joint_trajectory_as.action_goal.header.frame_id = "/world";
	joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
	joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);

	// Communicate action server results
	actionlib::SimpleClientGoalState state = trajectory_as->sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(20.0), ros::Duration(20.0));
	ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
}

void moveArm(osrf_gear::Model model, std::string source_frame, double z_incr) {	
	ros::ServiceClient ik_client;

	// Get pose from cam
	geometry_msgs::TransformStamped transform = get_transform(source_frame);

	geometry_msgs::PoseStamped part_pose; 
	geometry_msgs::PoseStamped goal_pose;

	part_pose.pose = model.pose;
	tf2::doTransform(part_pose, goal_pose, transform);

	goal_pose.pose.position.z += z_incr;

	geometry_msgs::Point position = goal_pose.pose.position;
	geometry_msgs::Quaternion orientation = goal_pose.pose.orientation;

	ROS_WARN("Position: x=%f, y=%f, z=%f (relative to arm)", position.x, position.y, position.z);
	ROS_WARN("Orientation: w= %f, x=%f, y=%f, z=%f (relative to arm)", orientation.w, orientation.x, orientation.y, orientation.z);


	ik_service::PoseIK ik_pose; 
	ik_pose.request.part_pose = goal_pose.pose;	
	ik_client.call(ik_pose);

	int num_sols = ik_pose.response.num_sols;

	ROS_INFO("Number of poses returned [%i]", num_sols);

	if (num_sols == 0) {
		return;
	}

	trajectory_msgs::JointTrajectory joint_trajectory = prepareJT();	

	ROS_INFO_STREAM("Angle: " << ik_pose.response.joint_solutions[0]);

	// Enter joint positions
	for (int i = 0; i < 6; i++) {
	joint_trajectory.points[1].positions[i+1] = 			 
		ik_pose.response.joint_solutions[0].joint_angles[i];
	}
	ROS_INFO_STREAM("Angle: " << joint_trajectory.points[1]);

	joint_trajectory.points[1].time_from_start = ros::Duration(5.0);

	callActionServer(joint_trajectory);
	ros::Duration(joint_trajectory.points[1].time_from_start).sleep();
}

void moveBase(double base_pos) {	
	trajectory_msgs::JointTrajectory joint_trajectory = prepareJT();
	
	for (int i = 0; i < joint_trajectory.joint_names.size(); i++) {
        for (int j = 0; j < joint_states.name.size(); j++) {
            if (joint_trajectory.joint_names[i] == joint_states.name[j]) {
                joint_trajectory.points[0].positions[i] = joint_states.position[j];
                joint_trajectory.points[1].positions[i] = joint_states.position[j];
                break;
            }
        }
    }
    
    joint_trajectory.points[1].positions[0] = base_pos;
    joint_trajectory.points[1].time_from_start = ros::Duration(5.0);
    callActionServer(joint_trajectory);
}


void processOrder(ros::NodeHandle &n) {
	ros::ServiceClient mat_loc_client;
	mat_loc_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
	// If orders list is empty, exit
	if (orders.size() == 0) {
		return;
	}
	// Set current order to first in queue
	osrf_gear::Order order = orders.at(0);
	
	// Iterate through all shipments and all products in each shipment
	for(osrf_gear::Shipment shipment: order.shipments) {
		for(osrf_gear::Product product:shipment.products) {
			// Get the name of the current bin
			
			// Default is none
			std::string bin = "NONE";
			// Call material locater service
			osrf_gear::GetMaterialLocations locater;
			// Get material type to be located
			locater.request.material_type = product.type;
			// Call using location client
			mat_loc_client.call(locater);
			
			// Iterate through storage units returned by location service and client
			for (osrf_gear::StorageUnit stor_unit : locater.response.storage_units){				
				bin = stor_unit.unit_id;
			}
			
			// if the bin doesn't exist for the product, move to the next product
			if (bin == "NONE") {
				continue;
			}
			
			// iterate through all cameras to scan for product
			for(int i = 0; i < 10; i++) {
				// if product is not found in bin, continue to next camera
				if(camera_topics[i].find(bin) == std::string::npos) {
					continue;
				}
				
				
				for (osrf_gear::Model model : camera_images[i]->models) {
					// if the product type exists in the model type (i.e. it is a match), return information about product
					if (strstr(product.type.c_str(), model.type.c_str())) {
						geometry_msgs::Point position = model.pose.position;
						
						ROS_WARN("Model type: %s", product.type.c_str());
						ROS_WARN("Bin: %s", bin.c_str());
						ROS_WARN("Position: x=%f, y=%f, z=%f (relative to camera)", position.x, position.y, position.z); 
						
						std::string sourceFrame = "logical_camera_"+bin+"_frame";				
						
						double base_pos = 0; // base of arm
						
						// Begin moving arm to retrieve product
						
						if (bin == "bin4") {
							base_pos = .60 ;
						}
						else if (bin == "bin5") {
							base_pos = 1.15;
						} 
						else if (bin == "bin6") {
							base_pos = 1.91;
						}
						
						ROS_INFO("Moving base to %s", bin.c_str());			
						moveBase(base_pos);		
						
						double h_adj = .12;

						moveArm(model, sourceFrame, h_adj); // move arm
						moveArm(model, sourceFrame, h_adj * -1); // lower arm
						moveArm(model, sourceFrame, h_adj); // raise arm
					
						// Loop can be broken since product was retrieved
						break;					
					}
				}	
			}
			
		}
	}

	orders.erase(orders.begin());
}



int main(int argc, char **argv) {


	// Initialize important variables and processes
	ros::init(argc, argv, "ariac_entry");
	usleep(1000000*10);
	ros::NodeHandle n;
	
	ros::Rate loop_rate(5);

	// Define client for getting material locations
	

	// Subscribe to orders messages
	orders.clear();
	ros::Subscriber orders_sub = n.subscribe("/ariac/orders", 1000, ordersCallback);

	ROS_INFO("Subscribing to cameras");

	// Generate subscribers for agv and quality camera
	

	// Generate subscriber to joint states client
	ros::Subscriber joint_states_subscriber = n.subscribe("/ariac/arm1/joint_states", 10, jointStatesCallback);




	// Wait for existence of PoseIK service
	if (!ros::service::waitForService("PoseIK", 10)) {
	ROS_WARN("Inverse Kinematic service not found");
	}

	// Wait for existence of material_locations service
	if (!ros::service::waitForService("/ariac/material_locations", ros::Duration(10))) {
	ROS_ERROR("Material location service is not available after 30s.");
	}

	// Start competition
	std_srvs::SetBool my_bool_var;
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

	ros::AsyncSpinner spinner(1); // Use 1 thread
	spinner.start(); // A spinner makes calling ros::spin() unnecessary.
	ROS_INFO("Beginning of ROS loop");
	while(ros::ok()) {
		// Process first order in queue
		processOrder(n);
		loop_rate.sleep();
		

	}

	return 0;

	
}
