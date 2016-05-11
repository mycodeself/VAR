#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
//#include <gazebo/ModelState.h>
//#include <gazebo>
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"


ros::Subscriber subModelState;
ros::ServiceClient client;
gazebo_msgs::SetModelState setModelState;
gazebo_msgs::ModelState modelState;

void bucle()
{
	ros::Rate rate(10); // Especifica el tiempo de bucle en Hertzios. Ahora está en ciclo por segundo, pero normalmente usaremos un valor de 10 (un ciclo cada 100ms).
	while (ros::ok()) { // Bucle que estaremos ejecutando hasta que paremos este nodo o el roscore pare.
		//geometry_msgs::Twist msg; // Este mensaje está compuesto por dos componentes: linear y angular. Permite especificar dichas velocidades
					  //  Cada componente tiene tres posibles valores: x, y, z, para cada componente de la velocidad. En el caso de
					  // robots que reciben velocidad linear y angular, debemos especificar la x linear y la z angular.
		//msg.linear.x = forwardVel;
		//msg.angular.z = rotateVel;
		//commandPub.publish(msg);		
		ros::spinOnce(); // Se procesarán todas las llamadas pendientes, es decir, llamará a callBack
		rate.sleep(); // Espera a que finalice el ciclo
	}
}

void ModelStateCallback(const gazebo_msgs::ModelState::ConstPtr& msg)
{
	ROS_INFO_STREAM("Posicion Actual: x [" << msg->pose.position.x << "], y[" << msg->pose.position.y << "], z ["<< msg->pose.position.z << "]"); 
	if(modelState.model_name == "")
	{
		modelState.model_name 		= msg->model_name;
		modelState.reference_frame 	= msg->reference_frame;
	}
	char cmd_buff[50];
	std::cin.getline(cmd_buff, 50);
	/*
	int iKeyValue = GetKey();
	switch(iKeyValue)
	{
		geometry_msgs::Pose nextPose  	= msg->pose;
		geometry_msgs::Twist nextTwist 	= msg->twist;

		case VK_LEFT:
			nextTwist.angular.y -= 2;
		break;
		case VK_RIGHT:
			nextTwist.angular.y += 2;
		break;
		case VK_KEY_W:
			nextPose.position.z -= 0.5;
		break;
		case VK_KEY_S:
			nextPose.position.z += 0.5;
		break;
		case VK_KEY_A:
			nextPose.position.x += 0.5;
		break;
		case VK_KEY_D:
			nextPose.position.x += 0.5;
		break;
*/
	geometry_msgs::Pose nextPose  	= msg->pose;
	geometry_msgs::Twist nextTwist 	= msg->twist;
	switch(cmd_buff[0])
	{

		case 'w':case 'W':
			nextPose.position.z -= 0.5;
		break;
		case 's':case 'S':
			nextPose.position.z += 0.5;
		break;
		case 'a':case 'A':
			nextPose.position.x += 0.5;
		break;
		case 'd':case 'D':
			nextPose.position.x += 0.5;
		break;
		case 'k':case 'K':		// Rotar iz
			nextTwist.angular.y -= 2;
		break;
		case 'l':case 'L':		// Rotar de
			nextTwist.angular.y += 2;
		break;
	}

	modelState.pose 	= nextPose;
	modelState.twist 	= nextTwist;

	setModelState.request.model_state = modelState;
	
	client.call(setModelState);

	ROS_INFO_STREAM("Posicion siguiente: 	x [" << nextPose.position.x << "], y[" << nextPose.position.y << "], z ["<< nextPose.position.z << "]"); 
	ROS_INFO_STREAM("Orientación siguiente: x [" << nextTwist.linear.x << "], y[" << nextTwist.linear.y << "], z ["<< nextTwist.linear.z << "]"); 
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_turtlebot");
	ros::NodeHandle nh;

	subModelState 				= nh.subscribe("mobile_base", 1, ModelStateCallback);
	client 						= nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/SetModelState");
  	modelState.model_name 		= "";
  	modelState.reference_frame 	= "";
	bucle();
}