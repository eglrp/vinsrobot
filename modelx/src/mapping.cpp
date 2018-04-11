/*****************************
 * VIO used in mapping and locate.
 * Stereo-VIO is more complicated and unstable than mono-VIO,but it's effect is better.
*****************************/
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
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

static volatile int keepRunning = 1;
void sig_handler( int sig )
{
    if ( sig == SIGINT )
    {
        keepRunning = 0;
    }
}

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

    signal( SIGINT, sig_handler );

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
    ros::Subscriber imagesub0;
    ros::Subscriber imagesub1;
    ros::Subscriber imusub;
    if(ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
         imagesub0 = nh.subscribe(config._imageLeftTopic, /*200*/ 2, &ORBVIO::MsgSynchronizer::imageleftCallback, &msgsync);
         imagesub1 = nh.subscribe(config._imageRightTopic, /*200*/ 2, &ORBVIO::MsgSynchronizer::imagerightCallback, &msgsync);
         imusub = nh.subscribe(config._imuTopic, 200, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);
    }
    sensor_msgs::ImageConstPtr imageLeftMsg;
    sensor_msgs::ImageConstPtr imageRightMsg;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();

    ros::Rate r(1000);

    if(!ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
        ROS_WARN("Run not-realtime");

        std::string bagfile = config._bagfile;
        rosbag::Bag bag;
        bag.open(bagfile,rosbag::bagmode::Read);

        std::vector<std::string> topics;
        std::string imutopic = config._imuTopic;
        std::string imagelefttopic = config._imageLeftTopic;
        std::string imagerighttopic = config._imageRightTopic;
        topics.push_back(imagelefttopic);
        topics.push_back(imagerighttopic);
        topics.push_back(imutopic);

        rosbag::View view(bag, rosbag::TopicQuery(topics));
        //while(ros::ok())
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>();
            if(simu!=NULL)
                msgsync.imuCallback(simu);
            sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
            if(simage!=NULL)
                msgsync.imageleftCallback(simage);
            if(simage!=NULL)
                msgsync.imagerightCallback(simage);
            bool bdata = msgsync.getRecentMsgs(imageLeftMsg,vimuMsg);

            if(bdata)
            {
                std::vector<ORB_SLAM2::IMUData> vimuData;
                //ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
                for(unsigned int i=0;i<vimuMsg.size();i++)
                {
                    sensor_msgs::ImuConstPtr imuMsg = vimuMsg[i];
                    double ax = imuMsg->linear_acceleration.x;
                    double ay = imuMsg->linear_acceleration.y;
                    double az = imuMsg->linear_acceleration.z;
                    if(bAccMultiply98)
                    {
                        ax *= g3dm;
                        ay *= g3dm;
                        az *= g3dm;
                    }
                    ORB_SLAM2::IMUData imudata(imuMsg->angular_velocity.x,imuMsg->angular_velocity.y,imuMsg->angular_velocity.z,
                                    ax,ay,az,imuMsg->header.stamp.toSec());
                    vimuData.push_back(imudata);
                    //ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
                }

                // Copy the ros image message to cv::Mat.
                cv_bridge::CvImageConstPtr cv_ptr0;
                cv_bridge::CvImageConstPtr cv_ptr1;
                try
                {
                    cv_ptr0 = cv_bridge::toCvShare(imageLeftMsg);
                    cv_ptr1 = cv_bridge::toCvShare(imageRightMsg);
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return -1;
                }

                // Consider delay of image message
                //SLAM.TrackMonocular(cv_ptr->image, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                cv::Mat iml = cv_ptr0->image.clone();
                cv::Mat imr = cv_ptr1->image.clone();
                {
                    // To test relocalization
                    static double startT=-1;
                    if(startT<0)
                        startT = imageMsg->header.stamp.toSec();
                    // Below to test relocalizaiton
                    //if(imageMsg->header.stamp.toSec() > startT+25 && imageMsg->header.stamp.toSec() < startT+25.3)
                    if(imageMsg->header.stamp.toSec() < startT+config._testDiscardTime)
                        im = cv::Mat::zeros(im.rows,im.cols,im.type());
                }
                SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                //SLAM.TrackMonoVI(cv_ptr->image, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                //cv::imshow("image",cv_ptr->image);

                // Wait local mapping end.
                bool bstop = false;
                while(!SLAM.bLocalMapAcceptKF())
                {
                    if(!ros::ok())
                    {
                        bstop=true;
                    }
                };
                if(bstop)
                    break;

            }

            //cv::waitKey(1);

            ros::spinOnce();
            r.sleep();
            if(!ros::ok())
                break;
        }

    }
    else
    {
        ROS_WARN("Run realtime");
        while( keepRunning )
        {
            bool bdata = msgsync.getRecentMsgs(imageMsg,vimuMsg);

            if(bdata)
            {
                std::vector<ORB_SLAM2::IMUData> vimuData;
                //ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
                for(unsigned int i=0;i<vimuMsg.size();i++)
                {
                    sensor_msgs::ImuConstPtr imuMsg = vimuMsg[i];
                    double ax = imuMsg->linear_acceleration.x;
                    double ay = imuMsg->linear_acceleration.y;
                    double az = imuMsg->linear_acceleration.z;
                    if(bAccMultiply98)
                    {
                        ax *= g3dm;
                        ay *= g3dm;
                        az *= g3dm;
                    }
                    ORB_SLAM2::IMUData imudata(imuMsg->angular_velocity.x,imuMsg->angular_velocity.y,imuMsg->angular_velocity.z,
                                    ax,ay,az,imuMsg->header.stamp.toSec());
                    vimuData.push_back(imudata);
                    //ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
                }

                // Copy the ros image message to cv::Mat.
                cv_bridge::CvImageConstPtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvShare(imageMsg);
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return -1;
                }

                // Consider delay of image message
                //SLAM.TrackMonocular(cv_ptr->image, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                cv::Mat im = cv_ptr->image.clone();
                {
                    // To test relocalization
                    static double startT=-1;
                    if(startT<0)
                        startT = imageMsg->header.stamp.toSec();
                    // Below to test relocalizaiton
                    //if(imageMsg->header.stamp.toSec() > startT+25 && imageMsg->header.stamp.toSec() < startT+25.3)
                    if(imageMsg->header.stamp.toSec() < startT+config._testDiscardTime)
                        im = cv::Mat::zeros(im.rows,im.cols,im.type());
                }
                SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                //SLAM.TrackMonoVI(cv_ptr->image, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                //cv::imshow("image",cv_ptr->image);

            }

            //cv::waitKey(1);

            ros::spinOnce();
            r.sleep();
            if(!ros::ok())
                break;
        }
    }


    SLAM.Shutdown();

    std::string map_file_updated_name = orb_settig_640["map_file_updated_name"];
    SLAM.SaveMap( map_file_updated_name );

    std::string path_to_save = orb_setting_640["path_to_save"];
    SLAM.SaveKeyFrameTrajectoryTUM( path_to_save );

    ros::shutdown();
    return 0;
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
