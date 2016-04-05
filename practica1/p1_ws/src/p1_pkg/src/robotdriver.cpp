#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h> 
#define DEBUG 0 // macro que nos permite mostrar datos de debug
#define VELOCITY 0.5 // velocidad de 0.5m/s
#define TURBO_VELOCITY 0.9 // velocidad turbo 0.9m/s solo durante 5 segundos al observar un robot
#define DISTANCE_OBS_AVOID 2 // margen de distancia cuando detectamos un obstaculo
#define PI 3.1415926535897 // número pi para los cálculos con ángulos y demás
class RobotDriver
{
    private:
        ros::NodeHandle nh_;                        // interfaz para crear los suscritores/publicadores
        ros::Publisher cmd_vel_pub_;                // publicador para mandar comandos al tópico velocity
        ros::Subscriber laser_sub_;                 // suscriptor para obtener los mensajes del laser
        ros::Subscriber imu_sub_;                   // suscriptor para obtener los mensajes del Imu
        ros::Subscriber odom_sub_;                  // suscriptor para obtener los datos de la odometría
        ros::Subscriber go_sub_;					// suscriptor para obtener la salida de la carrera
        geometry_msgs::Twist twist_msg_;            // mensaje que será enviado al publicador que contiene los comandos de velocidad
        nav_msgs::Odometry odom_msg_;               // mensaje que contiene los datos de la odometría recibidos
        sensor_msgs::Imu imu_msg_rcvd_;             // mensaje que contiene los datos del Imu recibidos
        sensor_msgs::LaserScan laser_msg_rcvd_;     // mensaje que contiene los datos del Laser recibidos
        bool go_;
        double goal_x_;
        double goal_y_;
        void Navigate()
        {
            bool turn_left = false, turn_right = false;
            // Quaternion cons los datos recibidos por el IMU
            tf::Quaternion quat(imu_msg_rcvd_.orientation.x, 
                imu_msg_rcvd_.orientation.y, 
                imu_msg_rcvd_.orientation.z, 
                imu_msg_rcvd_.orientation.w);  
            // Creamos una matrix 3x3 a partir del quaternion para obtener de manera sencilla
            // el roll, pitch y yaw.
            tf::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            double rotation = 0.0, velocity = VELOCITY; // rotación a aplicar 
            mat.getRPY(roll, pitch, yaw); // obtenemos roll, pitch, yaw
            double laser_theta = laser_msg_rcvd_.angle_min;
            for(int i=0;i<laser_msg_rcvd_.ranges.size();i++){
                if(laser_msg_rcvd_.ranges[i] < DISTANCE_OBS_AVOID) // algo esta cercano
                {
                    if(laser_theta >= ((-PI)/2.0) && laser_theta <= 0.0)
                    {
                        // algo ha sido detectado en el cuadrante derecho del laser
#if DEBUG
                        ROS_INFO_STREAM("Object detected in right quadrant");
#endif
                        if(!turn_left)
                        {
                            // rotamos a la izquierda
                            rotation += 1.0;
                            turn_left = true;
                        }
                    }else if(laser_theta >= 0.0 && laser_theta <= (PI/2.0))
                    {
                        // algo ha sido detectado en el cuadrante izquierdo del laser
#if DEBUG                        
                        ROS_INFO_STREAM("Object detected in left quadrant");
#endif                        
                        if(!turn_right)
                        {
                            rotation += -1.0;
                            turn_right = true;
                        }
                    }
                    if(laser_theta >= -PI/2.0 && laser_theta < PI/2.0 && laser_msg_rcvd_.ranges[i] < 1.5)
                    {
#if DEBUG
                        ROS_INFO_STREAM("Object detected at front");
#endif
                        velocity = velocity/2;
                        // algo ha sido detectado en el frente, con un cono de 20 grados
                    }
                }
                laser_theta += laser_msg_rcvd_.angle_increment;
            }            
            twist_msg_.linear.x = velocity;
            twist_msg_.angular.z = rotation - yaw;  
            cmd_vel_pub_.publish(twist_msg_); // mensaje twist para mover el robot
        }
    public:
        //ROS node initialization
        RobotDriver(ros::NodeHandle &nh) {
            nh_ = nh;
            // subscriber para la odometría
            odom_sub_ = nh_.subscribe("/robot6/odom", 1, &RobotDriver::OdomCallback, this);
            // publisher para comandos de velocidad
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot6/commands/velocity", 1);
            // subscriber para información del IMU
            imu_sub_ = nh_.subscribe("/robot6/sensors/imu_data", 1, &RobotDriver::IMUCallback, this);
            // subscriber para el laser scan
            laser_sub_ = nh_.subscribe("/robot6/scan", 1, &RobotDriver::LaserCallback, this);
            // velocidad por defecto en el eje X
            twist_msg_.linear.x = 0;
            // rotación por defecto en el eje z
            twist_msg_.angular.z = 0;
            go_ = false;
            go_sub_ = nh.subscribe("/control_tower/race_state", 1, &RobotDriver::GoCallback, this);

            goal_x_ = odom_msg_.pose.pose.position.x;
            goal_y_ = odom_msg_.pose.pose.position.y;
        }
        // Callback para el IMU
        void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
        {
            imu_msg_rcvd_ = *msg;
#if DEBUG
            ROS_INFO("Quaternion from IMU: (%f, %f, %f, %f)", 
                msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
#endif
        }
        // Calback para el Laser
        void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
            laser_msg_rcvd_ = *msg;
#if DEBUG
            ROS_INFO_STREAM("AngleMin: " << msg->angle_min); // Mínimo valor angular del láser
            ROS_INFO_STREAM("AngleMax: " << msg->angle_max); // Máximo valor angular del láser
            ROS_INFO_STREAM("AngleIncrement: " << msg->angle_increment); // Incremento angular entre dos beams
            ROS_INFO_STREAM("RangeMin: " << msg->range_min); // Mínimo valor que devuelve el láser
            ROS_INFO_STREAM("RangeMax: " << msg->range_max); // Máximo valor que devuelve el láser. Valores por debajo y por encima de estos rangos no deben ser tenidos en cuenta.
            for (int i=0; i< msg->ranges.size(); i++) {
                ROS_INFO_STREAM("Values[" << i << "]:" << msg->ranges[i]); // Acceso a los valores de rango
            }
#endif
        }
        // Callaback para la odometría
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            odom_msg_ = *msg;
        }

        void GoCallback(const std_msgs::String::ConstPtr& msg){
        	if(msg->data == "GO"){
        		go_ = true;
        	}
        }
        // Bucle principal
        void Init()
        {
            ros::Rate rate(10.0);
            while(ros::ok())
            {
            	if(go_) // se navegara cuando se de la salida
                	Navigate();
                ros::spinOnce();
                rate.sleep();
            }
        }

};
int main(int argc, char** argv) {
    //init the ROS node
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    RobotDriver driver(nh);
    driver.Init();
    return 0;
}