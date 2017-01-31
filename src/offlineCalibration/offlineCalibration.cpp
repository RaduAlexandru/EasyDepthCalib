
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>

#include "pointCloud.h"
#include "planeCloud.h"
#include "calibration.h"


//folder dump6 has good data in it

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(argc<3){
        std::cout<<"This node is intend to use offline"<<std::endl<<"provide a logfile (produced with the dumpernode) and the output calibration file you want"<<std::endl;
        std::cout<<"example:"<<std::endl<<"offlineCalibration <input log filename> <output calibration filename>"<<std::endl;
        return 0;
    }
    float c0,c1,c2,c3=0;
    c0=1.0f;
    if(argc==7){
        c3=atof(argv[3]);
        c2=atof(argv[4]);
        c1=atof(argv[5]);
        c0=atof(argv[6]);
        std::cout<<"polynomial coeffs: d^3*"<<c3<<" + d^2*"<<c2<<" + d*"<<c1<<" + "<<c0<<std::endl;
    }
    std::fstream log;
    log.open(argv[1]);
    string topic;
    int height;
    int width;
    log >> topic;
    log >> height;
    log >> width;
    Matrix3f k;
    log >> k(0,0);
    log >> k(0,1);
    log >> k(0,2);
    log >> k(1,0);
    log >> k(1,1);
    log >> k(1,2);
    log >> k(2,0);
    log >> k(2,1);
    log >> k(2,2);
    string filename;
    //calibrationMatrix c(480,640,8000,4,8);
    // int rows =240;
    // int cols = 320;
    // calibrationMatrix c(rows,cols,5000,2,4);

    int rows =480;
    int cols = 640;
    // calibrationMatrix c(rows,cols,2000,2,4);
    calibrationMatrix c(rows,cols,2000,4,8);


    //calibrationMatrix c(rows,cols,8000,4,64);
    while(!log.eof()){
        log>>filename;
        std::cout<< "opening "<<filename<<"\r"<<std::flush;
        cv::Mat data = cv::imread(filename,cv::IMREAD_ANYDEPTH);


        if(! data.data ) {
          cout <<  "Could not open or find the image" << std::endl ;
          return -1;
        }

        // cv::Mat img_cv_f;
        // data.convertTo(img_cv_f, CV_32FC1);
        // cv::normalize(img_cv_f, img_cv_f, 0, 1, cv::NORM_MINMAX, CV_32FC1);
        // cv::imshow("test",img_cv_f);
        // cv::waitKey(30);



        //std::cout << "calibration is" << k(0,0) << std::endl;

        pointCloud p(data,k);
        pcl::PointCloud<pcl::PointXYZ>* pcl = new pcl::PointCloud<pcl::PointXYZ>();
        pcl=p.pclCloud;



        //CENTER SQUARE CLOUD
        pcl::PointCloud<pcl::PointXYZ>* square= new pcl::PointCloud<pcl::PointXYZ>();
        calibration::computeCenterSquareCloud(data,k,square,cols/10,rows/10,c0,c1,c2,c3); //square will be 64x48
        std::cout << "center square has number of ponts: "  << square->size()<< std::endl;



        //CENTER PLANE
        Eigen::Vector4f centerModel;
        calibration::computerCenterPlane(square,centerModel,0.8);
        // planeCloud centerPlaneCloud;
        // centerPlaneCloud.model=centerModel;
        Eigen::Vector4f center;
        pcl::compute3DCentroid(*square,center);
        std::cout<<"CENTROID DISTANCE: "<<center[2]<<" ( "<<center.transpose()<<" )"<<std::endl;
        // centerPlaneCloud.com=center.head(3);


        //NORMALS
        pcl::PointCloud<pcl::PointXYZ>* p_normals = new pcl::PointCloud<pcl::PointXYZ>();
        pcl::copyPointCloud(*pcl, *p_normals);
        pcl::PointCloud<pcl::Normal>* normals= new pcl::PointCloud<pcl::Normal>();
        //calibration::computeNormals(p_normals,normals,100);

        //REJECTION
        Eigen::Vector3f ref;
        std::vector<bool> valid;
        ref<<centerModel[0],centerModel[1],centerModel[2];
        // pcl::PointCloud<pcl::PointXYZ> outFromNormalRejection;
        // pcl::PointCloud<pcl::PointXYZ>* tmpCloud = new pcl::PointCloud<pcl::PointXYZ>();
        // copyPointCloud(*pcl, *tmpCloud);
        // std::cout << "size of tmpcloud cloud is" << tmpCloud->size() << std::endl;
        //pcl::PointCloud<pcl::PointXYZ>* tmpCloud =p2;
        //calibration::pointrejection(&ref,0.7f,tmpCloud,normals,&outFromNormalRejection,&valid);

        //pointCloud outFromNormalRejectionCloud(&outFromNormalRejection);
        //ERROR PER POINT
        pcl::PointCloud<pcl::PointXYZ> error;
        calibration::computeErrorPerPoint(pcl,&error,center.head(3),ref,&valid);



        //COMPUTE CALIBRATION MATRIX
        calibration::computeCalibrationMatrix(*pcl,error,k,&valid,c);


        //CALIBRATE POINT CLOUD
        //pcl::PointCloud<pcl::PointXYZ> fixedCloud;
        //calibration::calibratePointCloudWithMultipliers(*cloudToCalibrate,fixedCloud,c,k);

        std::cout << "finished one loop clearing" << std::endl;

        error.clear();
        valid.clear();
        error.clear();
        p_normals->clear();
        delete p_normals;
        //tmpCloud->clear();
        //delete tmpCloud;
        //p2.cloud.clear();
        normals->clear();
        //outFromNormalRejection.clear();
        valid.clear();
        delete normals;


        square->clear();
        delete square;
        pcl->clear();
        delete pcl;
    }

    //fill the rest of the data in the calibration matrix that is missing
    // calibration::calibration_matrix_fill(c);
    //calibration::smooth_data(c);







    std::cout<<std::endl<<"saving "<<std::endl;
    c.dumpSensorImages();
    c.serialize(argv[2]);
    char nn[500];
    std::cout<<"saving nn"<<std::endl;
    sprintf(nn,"NN_%s",argv[2]);
    //c.serializeNN(nn);
    //calibrationMatrix* cc = c.downsample(2,2);
    //cc->serialize(argv[2]);

}
