#include "ros/ros.h"
//#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
//#include <gazebo/ModelState.h>
//#include <gazebo>
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
//#include "gazebo_msgs/GetModelState.h"
#include <stdio.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>


ros::Subscriber subModelState;
ros::ServiceClient clientSetModelState;
//ros::ServiceClient clientGetModelState;
gazebo_msgs::ModelState modelState;
//gazebo_msgs::GetModelState getModelState;


int getKey();
///int getch() ;
void bucle();
void moveTurtlebot();

/*void getKey()
{
	int key = getch();
	//ROS_INFO_STREAM("Tecla pulsada: " << (char)key);
}*/

int getKey() 
{
  struct termios oldt,
                 newt;
  int            ch;
  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~( ICANON | ECHO );
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
  return ch;
}
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

		/*getModelState.request.model_name = "mobile_base";		
		clientGetModelState.call(getModelState);

		if(getModelState.response.success)
		{
			//ROS_INFO_STREAM("Posicion actual: x [" << getModelState.response.pose.position.x << "]");
			
		}*/
		moveTurtlebot();
		//ros::spinOnce(); // Se procesarán todas las llamadas pendientes, es decir, llamará a callBack
		rate.sleep(); // Espera a que finalice el ciclo
	}
}

void moveTurtlebot()
{
	int key = getKey();
	gazebo_msgs::SetModelState setModelState;
	setModelState.request.model_state.model_name = "mobile_base";
	setModelState.request.model_state.reference_frame = "world";

	switch(key)
	{

		case 'w':case 'W':
			setModelState.request.model_state.twist.linear.x = 0.1;
		break;
		case 's':case 'S':
			setModelState.request.model_state.twist.linear.x = -0.1;
		break;
		case 'a':case 'A':
			setModelState.request.model_state.twist.linear.y = -0.1;
		break;
		case 'd':case 'D':
			setModelState.request.model_state.twist.linear.y = 0.1;
		break;
		case 'k':case 'K':		// Rotar iz
			setModelState.request.model_state.twist.angular.z = -0.1;
		break;
		case 'l':case 'L':		// Rotar de
			setModelState.request.model_state.twist.angular.z = 0.1;
		break;
	}
	
	clientSetModelState.call(setModelState);
	ROS_INFO_STREAM("Twist actual" << setModelState.request.model_state.twist);
}


void ModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	/*int id_mb = msg->name.size()-1;
	ROS_INFO_STREAM("Posicion Actual: x [" << msg->pose[id_mb].position.x << "], y[" << msg->pose[id_mb].position.y << "], z ["<< msg->pose[id_mb].position.z << "]"); 
	if(modelState.model_name == "")
	{
		//modelState.model_name 		= msg->name[0];
		//modelState.reference_frame 	= msg->reference_frame;
	}
	char cmd_buff[50];
	std::cin.getline(cmd_buff, 50);*/
	/*
	int iKeyValue = GetKey();
	switch(iKeyValue)
	{
		geometry_msgs::Pose nextPose  	= msg->pose[0];
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
	/*geometry_msgs::Pose nextPose  	= msg->pose[0];
	geometry_msgs::Twist nextTwist 	= msg->twist[0];
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
	
	clientSetModelState.call(setModelState);

	ROS_INFO_STREAM("Posicion siguiente: 	x [" << nextPose.position.x << "], y[" << nextPose.position.y << "], z ["<< nextPose.position.z << "]"); 
	ROS_INFO_STREAM("Orientación siguiente: x [" << nextTwist.linear.x << "], y[" << nextTwist.linear.y << "], z ["<< nextTwist.linear.z << "]"); */
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_turtlebot");
	ros::NodeHandle nh;

	//subModelState 				= nh.subscribe("/gazebo/model_states", 10, ModelStateCallback);
	//clientGetModelState			= nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	clientSetModelState 		= nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  	modelState.model_name 		= "";
  	modelState.reference_frame 	= "";
	bucle();
}