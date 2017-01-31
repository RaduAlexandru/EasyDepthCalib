#include <iostream>
#include <ros/ros.h>
#include <calibrationMatrix.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <boost/signals2.hpp>
#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

cv_bridge::CvImagePtr cvImageFromROS;
cv_bridge::CvImage out_msg;
cv_bridge::CvImagePtr diff;
image_transport::ImageTransport* it;
image_transport::ImageTransport* test;
image_transport::Publisher pub;
image_transport::Publisher pub2;
cv::Mat image;
cv::Mat DiffImage;
calibrationMatrix* multiplier;
ros::Publisher pub_info_left;



std::string _sub;
std::string _pub;
std::string _calib;


void callback(const sensor_msgs::ImageConstPtr &imgPtr){
    cvImageFromROS =cv_bridge::toCvCopy(imgPtr);
    diff =cv_bridge::toCvCopy(imgPtr);

    cvImageFromROS->image.copyTo(image);
    diff->image.copyTo(DiffImage);
    out_msg.header   = imgPtr->header;
    out_msg.encoding =  "16UC1"; //  sensor_msgs::image_encodings::MONO16; //"mono16";


    int cols=image.cols;
    int rows=image.rows;
    ushort v;
    cv::Point p;
    for(int i=0;i<cols;i++){
        for(int j=0;j<rows;j++){
            p.x=i;
            p.y=j;
            v=((float)image.at<ushort>(p));

            //cap the depth at 2 meters
            if (v>2000){
              v=0;
              image.at<ushort>(p)=(ushort)v;
              continue;
            }

            v*=multiplier->cell(p.y,p.x,v);
            image.at<ushort>(p)=(ushort)v;
        }
    }

    out_msg.image    =  image;
    pub.publish(out_msg.toImageMsg());
    out_msg.image    =  image-DiffImage;
    pub2.publish(out_msg.toImageMsg());
    //std::cout<<"global diff: "<< cv::sum(out_msg.image)/(out_msg.image.rows*out_msg.image.cols)<<std::endl;

}


int main(int argc, char **argv)
{
    if(argc<3){
        std::cout<<"This ROS NODE is intended to use in realtime with a /depth/image_raw topic"<<std::endl;
        std::cout<<"The node will subscribe to a DEPTH IMAGE RAW topic and will republish the undistorted data"<<std::endl;
        std::cout<<"Usage:"<<std::endl;
        std::cout<<"rosrun easydepthcalibration driver_node _sub:=/camera/depth/image_raw _pub:=/calibrated/ _calib:=calibration_filename.ext"<<std::endl;
        std::cout<<"Note:"<<std::endl;

        exit(1);
    }
    ros::init(argc, argv, "xtionDriver",ros::init_options::AnonymousName);
    ros::NodeHandle n("~");
    n.param("sub", _sub, string("/camera/depth/image_raw"));
    n.param("pub", _pub, string("/cameraCalib/depth"));
    n.param("calib", _calib, string("calib.mal"));

    multiplier = new calibrationMatrix(const_cast<char*>(_calib.c_str()));

    it= new image_transport::ImageTransport(n);
    test= new image_transport::ImageTransport(n);
    pub = it->advertise(_pub+"/image_raw", 10);
    pub2 = it->advertise(_pub+"/image_raw_diff", 10);
    std::cout<<"TOPIC SUBSCRIBED TO: "<<_sub <<std::endl;
    image_transport::Subscriber sub = test->subscribe(_sub, 1, &callback);
    ros::spin();
    return 1;
}
