//#include "stdafx.h"
#include "../include/SimpleOpenNIViewer.h"
#include "ros/ros.h"

int main(int argc,char **argv)
{ 
	ros::init(argc, argv, "TrainMain");
	SimpleOpenNIViewer v;
	v.SetInputDirectory("E:\\frame_saver_output\\");
	v.run ();
	return 0;
}
