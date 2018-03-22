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

#define DEF_320x240 0

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"Stereo_mapping");
    ros::start();

    if ( 2 != argc )
    {
        std::cerr << std::endl << "Usage: ./robot path_to_settings" << std::endl;
        return -1;
    }
    cv::FileStorage top_setting( argv[1], cv::FileStorage::READ );
        if ( ! top_setting.isOpened( ) )
        {
            std::cerr << "Failed to open settings file: " << argv[1] << std::endl;
            return -1;
        }

    #if DEF_320x240
        // ORB_SLAM2 settings
        std::string str_orb_setting_320( top_setting["orb_setting_320"] );
        cv::FileStorage orb_setting_320( str_orb_setting_320, cv::FileStorage::READ );
        if ( ! orb_setting_320.isOpened( ) )
        {
            std::cerr << "Failed to open settings file: " << str_orb_setting_320 << std::endl;
            return -1;
        }

        // ORB Vocabulary Path
        std::string str_orb_vocabulary( orb_setting_320["orb_vocabulary"] );
    #else
        // ORB_SLAM2 settings
        std::string str_orb_setting_640( top_setting["orb_setting_640"] );
        cv::FileStorage orb_setting_640( str_orb_setting_640, cv::FileStorage::READ );
        if ( ! orb_setting_640.isOpened( ) )
        {
            std::cerr << "Failed to open settings file: " << str_orb_setting_640 << std::endl;
            return -1;
        }
        // ORB Vocabulary Path
        std::string str_orb_vocabulary( orb_setting_640["orb_vocabulary"] );
    #endif

    // Sensor settings
    std::string str_sensor_setting( top_setting["sensor_setting"] );
    cv::FileStorage sensor_setting( str_sensor_setting, cv::FileStorage::READ );
    if ( ! sensor_setting.isOpened( ) )
    {
        std::cerr << "Failed to open settings file: " << str_sensor_setting << std::endl;
        return -1;
    }

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
    ORB_SLAM2::ConfigParam config(argv[2]);

    double imageMsgDelaySec = config.GetImageDelayToIMU();
    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    ros::NodeHandle nh;
    ros::Subscriber imagesub;
    ros::Subscriber imusub;
    if(ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
         imagesub = nh.subscribe(config._imageTopic, /*200*/ 2, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
         imusub = nh.subscribe(config._imuTopic, 200, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);
    }
    sensor_msgs::ImageConstPtr imageMsg;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;


}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

}
