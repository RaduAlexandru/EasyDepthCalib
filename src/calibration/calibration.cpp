#include "calibration.h"
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/convolution.h>

void calibration::voxelize(pcl::PointCloud<pcl::PointXYZ>* inCloud,pcl::PointCloud<pcl::PointXYZ>* outCloud, float voxelLeaf){
    //std::cout<<"IN has "<<inCloud->size();
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (inCloud->makeShared());
    float _voxelLeaf =voxelLeaf;
    sor.setLeafSize ((float)_voxelLeaf, (float)_voxelLeaf, (float)_voxelLeaf);
    sor.filter (*outCloud);
    //std::cout<<" OUT has "<<outCloud->size()<<std::endl;
}
void calibration::computeCenterSquareCloud(cv::Mat& depthImage,Eigen::Matrix3f k, pcl::PointCloud<pcl::PointXYZ>* outCloud, int width, int height, float c0, float c1, float c2, float c3){
    outCloud->clear();
    int cols=depthImage.cols;
    int rows=depthImage.rows;
    for(int i=cols/2-width;i<cols/2+width;i++){
        for(int j=rows/2-height;j<rows/2+height;j++){


            ushort depthPixel = depthImage.at<ushort>(j,i);

            if (depthPixel<=1){
              //std::cout << "computeCenterSquareCloud: skipped point with no depth" << std::endl;
              continue;
            }

            Eigen::Vector3f worldPoint;
            worldPoint <<i*depthPixel,j*depthPixel,depthPixel;
            worldPoint = k.inverse()*worldPoint;
            outCloud->push_back(pcl::PointXYZ(worldPoint[0],worldPoint[1],worldPoint[2]));
        }
    }
}
void calibration::computerCenterPlane(pcl::PointCloud<pcl::PointXYZ>* inCloud,Eigen::Vector4f& model, float distanceThreshold){
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (inCloud->makeShared ());
    seg.segment (*inliers, *coefficients);

    std::cout << "iliers is " << inliers->indices.size ()  << std::endl;

    model[0]=coefficients->values[0];
    model[1]=coefficients->values[1];
    model[2]=coefficients->values[2];
    model[3]=coefficients->values[3];
}

void calibration::computeNormals(pcl::PointCloud<pcl::PointXYZ>* inCloud, pcl::PointCloud<pcl::Normal>* normals, float searchRadius){
    // pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    // pcl::PointCloud<pcl::PointXYZ>::ConstPtr p(inCloud);
    // //todo: check here
    // ne.setInputCloud (p);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    // ne.setSearchMethod (tree);
    // ne.setRadiusSearch (searchRadius);
    // ne.compute (*normals);



    //For non voxelized points, therefore organized points
   pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
   pcl::PointCloud<pcl::PointXYZ>::ConstPtr p(inCloud);
   ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
   ne.setMaxDepthChangeFactor(10.0f);
   ne.setNormalSmoothingSize(10.0f);
   ne.setInputCloud(p);
   ne.compute(*normals);
}

void calibration::pointrejection(Eigen::Vector3f* referenceNormal,
                                 float rejectionValue,
                                 pcl::PointCloud<pcl::PointXYZ>* inCloud,
                                 pcl::PointCloud<pcl::Normal>* normals,
                                 pcl::PointCloud<pcl::PointXYZ>* outCloud,
                                 std::vector<bool>* validIndeces ){
    pcl::Normal n;
    for(unsigned int i=0; i<inCloud->size();i++){
        n=normals->at(i);
        Eigen::Vector3f n1(n.normal_x,n.normal_y,n.normal_z);
        Eigen::Vector3f cross=n1.cross(*referenceNormal);
        if(cross.norm()>rejectionValue){
            validIndeces->push_back(false);
            outCloud->push_back(pcl::PointXYZ(inCloud->at(i)));
        }
        else{
            validIndeces->push_back(true);
        }

    }
}
void calibration::computeErrorPerPoint(pcl::PointCloud<pcl::PointXYZ>* inCloud,
                                       pcl::PointCloud<pcl::PointXYZ>* errorCloud,
                                       Eigen::Vector3f planeOrigin,
                                       Eigen::Vector3f planeCoefficient,
                                       std::vector<bool>* validIndeces ){
    pcl::PointXYZ p;
    pcl::PointXYZ pp;
    Eigen::Vector3f res;
    for(unsigned int i=0; i<inCloud->size();i++){
        p=inCloud->at(i);


        pcl::geometry::project( Eigen::Vector3f(p.x,p.y,p.z),
                                planeOrigin,
                                planeCoefficient,
                                res);
        pp.x=res(0);
        pp.y=res(1);
        pp.z=res(2);
        // if(validIndeces->at(i)==false){
        //     pp.x=p.x;
        //     pp.y=p.y;
        //     pp.z=p.z;
        // }

        if (std::isnan(p.z)){
          //std::cout << "computeErrorPerPoint:: skipped because point with no depth" << std::endl;
          pp.x=p.x;
          pp.y=p.y;
          pp.z=p.z;
        }


        errorCloud->push_back(pp);

    }
}
void calibration::computeCalibrationMatrix(pcl::PointCloud<pcl::PointXYZ> &inCloud,
                                           pcl::PointCloud<pcl::PointXYZ> &errorCloud,
                                           Eigen::Matrix3f k,
                                           std::vector<bool>* validIndeces ,
                                           calibrationMatrix &cM){
    pcl::PointXYZ point;
    pcl::PointXYZ projected_point;
    for(unsigned int i=0; i<inCloud.size();i++){
        point=inCloud.at(i);


        if (std::isnan(point.z)){
          //std::cout << "computeCalibrationMatrix:: skipped point because it has no depth" << std::endl;
          continue;
        }

        projected_point=errorCloud.at(i);

        Eigen::Vector3f diff = point.getVector3fMap() - projected_point.getVector3fMap();
        Eigen::Vector3f e=projected_point.getVector3fMap();
        float measuredDistance = diff.norm();
        Eigen::Vector3f localPoint;
        localPoint = k*Eigen::Vector3f(point.getArray3fMap());
        //PIXEL COORDINATES (DEPTH IMAGE)
        localPoint[0]/=localPoint[2];
        localPoint[1]/=localPoint[2];

        //std::cout<<"MAX= "<<cM.maxDepth<<" CALIB: "<<localPoint.y()<<" "<<localPoint.x()<<" "<<localPoint.z()<<std::endl;
        if(localPoint.z()>=cM.maxDepth){
            std::cout<<" [X] "<<std::endl;
            return;
        }
        if(point.z<=projected_point.z){

          //std::cout << "adding smaller value" << (measuredDistance+localPoint.z())/localPoint.z() << std::endl;
            cM.cell(localPoint.y(),
                    localPoint.x(),
                    localPoint.z(),
                    (measuredDistance+localPoint.z())/localPoint.z());

        }
        if(point.z>projected_point.z){
          //std::cout << "adding bigger value"  << (localPoint.z()-measuredDistance)/localPoint.z() << std::endl;
            cM.cell(localPoint.y(),
                    localPoint.x(),
                    localPoint.z(),
                    (localPoint.z()-measuredDistance)/localPoint.z());

        }

        cM.increment(localPoint.y(),localPoint.x(),localPoint.z());

    }

}
void calibration::calibratePointCloudWithMultipliers(pcl::PointCloud<pcl::PointXYZ> &inCloud,
                                                     pcl::PointCloud<pcl::PointXYZ> &outCloud,
                                                     calibrationMatrix &cM,
                                                     Eigen::Matrix3f k){
    pcl::PointXYZ point;
    for(unsigned int i=0; i<inCloud.size();i++){

        point=inCloud.at(i);
        Eigen::Vector3f localPoint;
        localPoint = k*Eigen::Vector3f(point.getArray3fMap());
        localPoint[0]/=localPoint[2];
        localPoint[1]/=localPoint[2];
        localPoint[2]*=cM.cell(localPoint[1],localPoint[0],localPoint[2]);

        localPoint[0]*=localPoint[2];
        localPoint[1]*=localPoint[2];
        Eigen::Vector3f newLocal;

        newLocal=k.inverse()*localPoint;
        outCloud.push_back(pcl::PointXYZ(newLocal[0],newLocal[1],newLocal[2]));
    }





}

void calibration::calibration_matrix_fill (calibrationMatrix &cm){
  std::cout << "filling the calibration data with data where it was missed" << std::endl;
  int K = 10;
  // float thresh=255;
  // float sigma=6;

  float thresh=49;
  float sigma=3;

  //create a point cloud of the Calibration matrix, encode in the
  pcl::PointCloud<pcl::PointXYZ>::Ptr cm_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<float> multipliers;
  std::vector<float> hits;

  // float max_multi=-999;
  // float min_multi=9999;

  for (size_t i = 0; i < cm.rows; i++) {
    for (size_t j = 0; j < cm.cols; j++) {
      for (size_t k = 0; k < cm.layers; k++) {
        if (cm._hits[k][i][j]==1.0f){
          continue;
        }

        pcl::PointXYZ point;
        point.x=j;
        point.y=i;
        point.z=k;
        cm_cloud->push_back(point);
        multipliers.push_back(cm._data[k][i][j] / cm._hits[k][i][j]);
        hits.push_back(cm._hits[k][i][j]);

        // if( (cm._data[k][i][j] / cm._hits[k][i][j]) > max_multi){
        //   max_multi=cm._data[k][i][j] / cm._hits[k][i][j];
        // }
        // if( (cm._data[k][i][j] / cm._hits[k][i][j]) < min_multi){
        //   min_multi=cm._data[k][i][j] / cm._hits[k][i][j];
        // }


      }
    }
  }

  // std::cout << "min,max is " << min_multi << " " << max_multi << std::endl;

  std::cout << "cloud has size " << cm_cloud->size() << std::endl;


  //loop through the calibration matrix and for each cell that doesnt have any hits, try to infer the value from the neighbours
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cm_cloud);
  //kdtree.setEpsilon(1);

  for (size_t i = 0; i < cm.rows; i++) {
    for (size_t j = 0; j < cm.cols; j++) {
      for (size_t k = 0; k < cm.layers; k++) {
        if (cm._hits[k][i][j] == 1.0f){

          // if (k < 69 || k > 308){
          //   continue;
          // }

          //std::cout << " data with no hits" << std::endl;
          pcl::PointXYZ searchPoint;
          searchPoint.x = j;
          searchPoint.y = i;
          searchPoint.z = k;

          std::vector<int> pointIdxNKNSearch(K);
          std::vector<float> pointNKNSquaredDistance(K);

          float new_multi=0.0f;
          float weight_sum=0.0f;


          if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for (size_t k_idx = 0; k_idx < pointIdxNKNSearch.size (); ++k_idx){

              float dist= std::sqrt(pointNKNSquaredDistance[k_idx]);

              // float weight = std::exp(- (  pointNKNSquaredDistance[k_idx] / (2*sigma*sigma) ) );
              float weight=1.0/ (float)pointNKNSquaredDistance[k_idx];
              if (pointNKNSquaredDistance[k_idx]>thresh){
                continue;
              }

              //std::cout << "dist: " << pointNKNSquaredDistance[k_idx] << std::endl;

              // new_multi+= weight*multipliers[pointIdxNKNSearch[k_idx]]  * hits[pointIdxNKNSearch[k_idx]] ;
              new_multi+= weight*multipliers[pointIdxNKNSearch[k_idx]] ;
              weight_sum+=weight;
            }
          }

          if (weight_sum==0.0f){
            continue;
          }
          new_multi=new_multi/weight_sum;
          //std::cout << "new_nulti is " << new_multi << std::endl;
          cm._data[k][i][j]=new_multi*2;
          cm._hits[k][i][j]=2.0f;




        }

      }
      //std::cout << "did a column" << std::endl;
    }
    std::cout << "did a row: " << i  << std::endl;
   }


}


void calibration::smooth_data(calibrationMatrix &cm){
  std::cout << " smoothing data " << std::endl;


  // pcl::PointCloud<pcl::PointXYZ>::Ptr cm_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //
  // for (size_t i = 0; i < cm.rows; i++) {
  //   for (size_t j = 0; j < cm.cols; j++) {
  //     for (size_t k = 0; k < cm.layers; k++) {
  //       if (cm._hits[k][i][j]==1.0f){
  //         continue;
  //       }
  //
  //       pcl::PointXYZ point;
  //       point.x=j;
  //       point.y=i;
  //       point.z=k;
  //       cm_cloud->push_back(point);
  //
  //     }
  //   }
  // }
  //
  //
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  // pcl::filters::Convolution<pcl::PointXYZ, pcl::PointXYZ>  convolution;
  // Eigen::ArrayXf gaussian_kernel(5);
  // gaussian_kernel << 1.f/16, 1.f/4, 3.f/8, 1.f/4, 1.f/16;
  // convolution.setBordersPolicy(
  // pcl::filters::Convolution<pcl::PointXYZ, pcl::PointXYZ>::BORDERS_POLICY_IGNORE);
  // convolution.setDistanceThreshold (static_cast<float> (0.1));
  // convolution.setInputCloud (cm_cloud);
  // convolution.setKernel (gaussian_kernel);
  // convolution.convolve(*cloud);
  //
  //
  // for (size_t i = 0; i < cloud.size(); i++) {
  //   pcl::PointXYZ point;
  //   std::round (cloud.at(i).x);
  // }





}
