#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream> 
#include <math.h>
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

std_srvs::Trigger begin_comp;

// Global vector for orders
std::vector<osrf_gear::Order::ConstPtr> orders;

// List of 10 camera topics
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

// Array of logical camera images
osrf_gear::LogicalCameraImage::ConstPtr log_cam_imgs[10];
int img_rec[10];


void ordersCallback(const osrf_gear::Order::ConstPtr& msg) {
	// Recieve order and assign to msg variable
	ROS_INFO("Order received: [%s]", msg->specific_order.c_str());
	// Push recieved order onto back of vector
	orders.push_back(msg);
	
	
	// Obtain and print type of first object in order
	std::string object_type = msg->shipments[0].products[0].type;
	ROS_INFO("Type of first product: [%s]", object_type.c_str());
	
	
	// Invoke material_location to find where product type may be found
	material_locations_srv.request.material_type = object_type;
	int successful_call = material_locations_client.call(material_locations_srv);
	if(!successful_call) {
		ROS_ERROR("material_locations call failed!");
		return;
	}
	// Use desired unit to locate desired bin
	std::string bin;
	std::string unit = material_locations_srv.response.storage_units[0].unit;
	// Determine whether unit is on belt or in bin, then assign location to variable
	if(unit_id == "belt") {
		bin = material_locations_srv.response.storage_units.back().unit_id;
	} else {
		bin = unit_id;
	}
	// Return location
	ROS_INFO("The desired product is in bin [%s]", bin.c_str());
	
	
	// Use logical cameras to locate parts
	for(int i = 0; i<10; i++){
		// if bin value exists in camera_topics array
		if(std::find(std::begin(camera_topics), std::end(camera_topics), camera_topics[i]) != std::end(camera_topics)){
			for(osrf_gear::Model model : logical_camera_images[i]->models) {
				if(model.type == part_type) {
				
				
				
				}
				
			}
		
		
		
		// Break loop after all actions performed
		break
		}
		
	}
	
	
	
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "ariac_view");


	ros::NodeHandle n;
	std_srvs::SetBool my_bool_var;
	
	my_bool_var.request.data = true;
	
	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
	
	
	// Subscribe to orders messages
	orders.clear();
	ros::Subscriber orders_sub = n.subscribe("/ariac/orders", 1000, ordersCallback);
	
	
	
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
	
	
	ros::Rate loop_rate(0.5);
	while(ros::ok()) {
		loop_rate.sleep();
	}

	return 0;

	
}

