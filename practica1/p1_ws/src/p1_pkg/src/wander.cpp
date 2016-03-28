#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> 
#include <ctime> 

class Wander {
	protected:
	ros::Publisher commandPub; 
	ros::Subscriber laserSub; 

	double forwardVel;
	double rotateVel;
	double closestRange;

	public:
	Wander(ros::NodeHandle& nh) { //: rotateStartTime(ros::Time::now()), rotateDuration(0.f) {
		forwardVel=1.0; // 1m/s es una velocidad bastante alta
		rotateVel=0.0;
		closestRange=0.0;
		srand(time(NULL));
		// Este método nos permite indicar al sistema que vamos a publicar mensajes de cmd_vel
		// El valor de 1 indica que si acumulamos varios mensajes, solo el último será enviado.
		// El método devuelve el Publisher que recibirá los mensajes.
		commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		// Suscribe el método commandCallback al tópico base_scan (el láser proporcionado por Stage)
		// El método commandCallback será llamado cada vez que el emisor (stage) publique datos 
		laserSub = nh.subscribe("base_scan", 1, &Wander::commandCallback, this);
	};

	// Procesa los datos de láser
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		ROS_INFO_STREAM("AngleMin: " << msg->angle_min); // Mínimo valor angular del láser
		ROS_INFO_STREAM("AngleMax: " << msg->angle_max); // Máximo valor angular del láser
		ROS_INFO_STREAM("AngleIncrement: " << msg->angle_increment); // Incremento angular entre dos beams
		ROS_INFO_STREAM("RangeMin: " << msg->range_min); // Mínimo valor que devuelve el láser
		ROS_INFO_STREAM("RangeMax: " << msg->range_max); // Máximo valor que devuelve el láser. Valores por debajo y por encima de estos rangos no deben ser tenidos en cuenta.
		int totalValues = ceil((msg->angle_max-msg->angle_min)/msg->angle_increment); // Total de valores que devuelve el láser
		for (int i=0; i< totalValues; i++) {
			ROS_INFO_STREAM("Values[" << i << "]:" << msg->ranges[i]); // Acceso a los valores de rango
		}

		// TODO: a partir de los datos del láser se tiene que modificar las variables forwardVel y rotateVel para hacer que el robot no choque.
	};

	// Bucle principal
	void bucle() {
		ros::Rate rate(1); // Especifica el tiempo de bucle en Hertzios. Ahora está en ciclo por segundo, pero normalmente usaremos un valor de 10 (un ciclo cada 100ms).
		while (ros::ok()) { // Bucle que estaremos ejecutando hasta que paremos este nodo o el roscore pare.
			geometry_msgs::Twist msg; // Este mensaje está compuesto por dos componentes: linear y angular. Permite especificar dichas velocidades
						  //  Cada componente tiene tres posibles valores: x, y, z, para cada componente de la velocidad. En el caso de
						  // robots que reciben velocidad linear y angular, debemos especificar la x linear y la z angular.
			msg.linear.x = forwardVel;
			msg.angular.z = rotateVel;
			commandPub.publish(msg);		
			ros::spinOnce(); // Se procesarán todas las llamadas pendientes, es decir, llamará a callBack
			rate.sleep(); // Espera a que finalice el ciclo
		}
	};
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "wander"); // Inicializa un nuevo nodo llamado wander
	ros::NodeHandle nh;
	Wander wand(nh); // Crea un objeto de esta clase y lo asocia con roscore
	wand.bucle(); // Ejecuta el bucle
	return 0;
};

