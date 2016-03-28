#include <ros/ros.h>
#include "panorama.cpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    Panorama panorama;
    panorama.Init();
    return 0;
}