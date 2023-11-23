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

std_srvs::Trigger begin_comp;

// Global vector declarations
std::vector<osrf_gear::Order::ConstPtr> orders;
std::vector<std::string> prod_types;
std::vector<std::vector<osrf_gear::StorageUnit>> prod_bins;
std::vector<osrf_gear::LogicalCameraImage> bin_cams;
std::vector<osrf_gear::LogicalCameraImage> qual_cams;
std::vector<osrf_gear::LogicalCameraImage> agv_cams;
geometry_msgs::PoseStamped part_pose, goal_pose;

// Declare the transformation buffer to maintain a list of transformations
tf2_ros::Buffer tfBuffer;
// Instantiate a listener that listens to the tf and tf_static topics and to update the buffer.
tf2_ros::TransformListener tfListener(tfBuffer);


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
	
    ROS_INFO("Beginning of ROS loop");
	
	while(ros::ok()) {
		
		// Clear variables to be used
		prod_types.clear();
		prod_bins.clear();

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
