//#include "stdafx.h"
#include "../include/SimpleOpenNIViewer.h"

int main()
{ 
	SimpleOpenNIViewer v;
	v.SetOutputDirectory("E:\\frame_saver_output\\");
	v.run ();
	return 0;
}
