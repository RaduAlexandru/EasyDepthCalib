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

#include "cv.h"

cv_bridge::CvImagePtr cvImageFromROS;
cv_bridge::CvImage out_msg;
cv_bridge::CvImagePtr diff;
image_transport::ImageTransport* it;
image_transport::ImageTransport* test;
image_transport::Publisher pub;
image_transport::Publisher pub2;
cv::Mat image;
cv::Mat image_undistorted;
cv::Mat DiffImage;
calibrationMatrix* multiplier;
ros::Publisher pub_info_left;

cv::Mat_<float> undistortDepthMapX, undistortDepthMapY;
cv::Mat_<float> id;
cv::Mat K;
cv::Mat DistCoef;



std::string _sub;
std::string _pub;
std::string _calib;

int numCols=640;
int numRows=480;
bool doUndistortion=true;

void initialize (){

  id = cv::Mat::eye(3, 3, CV_32FC1);
  K= cv::Mat ( 3, 3, CV_32FC1 ); K.setTo(0);
  K.at<float>(0,0) = 475.9396667480469;
  K.at<float>(1,1) = 475.9395446777344;
  K.at<float>(0,2) = 307.9508056640625;
  K.at<float>(1,2) = 245.95870971679688;
  K.at<float>(2,2) = 1.0f;

  DistCoef= cv::Mat( 5, 1, CV_32FC1 );
  // distortion cooefficients (rad tan)
  DistCoef.at<float>( 0 ) = 0.1646876037120819; // k1
  DistCoef.at<float>( 1 ) = -0.01863516867160797; // k2
  DistCoef.at<float>( 2 ) = 0.004235030617564917; // p1
  DistCoef.at<float>( 3 ) = 0.0034463282208889723; // p2
  DistCoef.at<float>( 4 ) = 0; // p3, can be 0.

  cv::initUndistortRectifyMap( K, DistCoef, id, K, cv::Size( numCols, numRows ), CV_32FC1, undistortDepthMapX, undistortDepthMapY );

  std::cout << "finished intiailization " << std::endl;



}


void undistort(cv::Mat img_depth,cv::Mat& image_undistorted){

    if ( doUndistortion )
    {

      //Convert to float
      img_depth.convertTo(img_depth, CV_32FC1);

      image_undistorted.create( img_depth.rows, img_depth.cols, img_depth.type());
      image_undistorted = 0.0f;

      // remap from distorted to undistorted image
      #pragma omp parallel for num_threads(6)
      for ( int i = 0; i < img_depth.rows; ++i )
      {
        for ( int j = 0; j < img_depth.cols; ++j )
        {
          int ux = std::floor(undistortDepthMapY(i,j));
          int vy = std::floor(undistortDepthMapX(i,j));
          if ( ux > 0 && vy > 0 && ux < img_depth.rows && vy < img_depth.cols )
            image_undistorted.at<ushort>(i,j) = img_depth.at<ushort>(ux,vy);
          else
            image_undistorted.at<ushort>(i,j) = 0;
        }
      }
    }
    else{
      img_depth.copyTo(image_undistorted);
    }


    //convert back to ushort
    image_undistorted.convertTo(image_undistorted, CV_16UC1);

    //std::cout << "finsished undistortion" << std::endl;



}


void callback(const sensor_msgs::ImageConstPtr &imgPtr){
    cvImageFromROS =cv_bridge::toCvCopy(imgPtr);
    diff =cv_bridge::toCvCopy(imgPtr);

    cvImageFromROS->image.copyTo(image);
    diff->image.copyTo(DiffImage);
    out_msg.header   = imgPtr->header;
    out_msg.encoding =  "16UC1"; //  sensor_msgs::image_encodings::MONO16; //"mono16";


    int cols=image.cols;
    int rows=image.rows;
    //std::cout << "is " << rows << " " << cols << std::endl;
    ushort v;
    cv::Point p;
    for(int i=0;i<cols;i++){
        for(int j=0;j<rows;j++){
            p.x=i;
            p.y=j;
            v=((float)image.at<ushort>(p));

            //cap the depth at 2 meters
            if (v>=2000){
              v=0;
              image.at<ushort>(p)=(ushort)v;
              continue;
            }

            v*=multiplier->cell(p.y,p.x,v);
            image.at<ushort>(p)=(ushort)v;
        }
    }


    //TODO Clean the horrible mess of a code
    numCols=cols;
    numRows=rows;
    undistort(image,image_undistorted);

    out_msg.image    =  image;
    pub.publish(out_msg.toImageMsg());

    //std::cout << "one is" << image_undistorted.rows << " " << image_undistorted.cols << std::endl;
    //std::cout << "one is" << DiffImage.rows << " " << DiffImage.cols << std::endl;

    out_msg.image    =  image_undistorted-DiffImage;
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
    initialize();

    it= new image_transport::ImageTransport(n);
    test= new image_transport::ImageTransport(n);
    pub = it->advertise(_pub+"/image_raw", 10);
    pub2 = it->advertise(_pub+"/image_raw_diff", 10);
    std::cout<<"TOPIC SUBSCRIBED TO: "<<_sub <<std::endl;
    image_transport::Subscriber sub = test->subscribe(_sub, 1, &callback);
    ros::spin();
    return 1;
}
