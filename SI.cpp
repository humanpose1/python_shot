#include <chrono>
//#include <pcl/io/ply_io.h>
// #include <pcl/io/pcd_io.h>

#include "core.h"



int main(int argc, char *argv[]) {

  std::string data_file = argv[1];
  float radiusSearch = 1.0f; // radius of SHOT
  std::vector<double> x;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  
  if (fileExist(data_file)) {
    happly::PLYData plyIn(data_file);
    getPointCloudFromPLY(plyIn, cloud);
  }
  else
    {
      std::cout << "Point cloud file does not exsist or cannot be opened!!" << std::endl;
      return 1;
    }

  std::cout << "File: " << data_file << std::endl;
  std::cout << "Number of Points: " << cloud->points.size() << std::endl;
  //std::cout << "Number of Points: " << x.size() << std::endl;
  for (int i = 0; i < 10; i++) {
    std::cout << i<<":"<<cloud->points[i].x<<" "<<cloud->points[i].y<<" " <<cloud->points[i].z<<" "<< std::endl;
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::Histogram<153> >::Ptr siFeatures(new pcl::PointCloud<pcl::Histogram<153> >);
  std::cout << siFeatures->size() << std::endl;
  computeSpinImageDescriptors(cloud, siFeatures, radiusSearch, 8, 0.5);
  //computeSHOTDescriptors(cloud, cloud_small, shotFeatures, radiusSearch);
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "time to compute SI Descriptor:" << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() <<"ms"<< std::endl;
  std::cout << siFeatures->size() << std::endl;
  std::cout << siFeatures->points[1621] << std::endl;

  std::cout << siFeatures->points[14621] << std::endl;

}
