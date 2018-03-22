#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../include/System.h"


#include "MsgSync/MsgSynchronizer.h"

#include "../../src/IMU/imudata.h"
#include "../../src/IMU/configparam.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

using namespace std;

int main(int argc,char **argv)
{
  ros::init(argc,argv,"Stereo");
  ros::start();

}
