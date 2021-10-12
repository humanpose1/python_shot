#include "core.h"



bool fileExist(const std::string& name){
  std::ifstream f(name.c_str());  
  return f.is_open();
}

void stringSplit(const std::string& data_file, const std::string& delimiter, std::vector<std::string>& listToken){
  listToken.clear();
  std::string::size_type n = 0;
  std::string::size_type new_n = 0;;
  while (n != std::string::npos) {
    new_n = (delimiter + data_file).find(delimiter,n+1);
    auto token = (delimiter + data_file).substr(n+1, new_n-n-1);
    listToken.push_back(token);
    n = new_n;
  }
}

std::pair<std::string, std::string> pathSplit(const std::string& data_file){
  std::vector<std::string> listToken;
  stringSplit(data_file, "/", listToken);
  std::string baseName;
  for (size_t i=0; i < listToken.size() - 1; i++) {
    baseName = baseName + listToken[i] + "/";
  }
  return std::make_pair(baseName, listToken[listToken.size()-1]);
}

void renameDataFile(const std::string& data_file, const std::string& suffix, std::string& data_out) {
  data_out.clear();
  std::vector<std::string> listToken;
  auto splitted = pathSplit(data_file);
  stringSplit(splitted.second, ".", listToken);
  data_out = splitted.first + listToken[0] + "." + suffix;
}

void getPointCloudFromPLY(happly::PLYData & plyIn, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
  auto x = plyIn.getElement("vertex").getProperty<double>("x");
  auto y = plyIn.getElement("vertex").getProperty<double>("y");
  auto z = plyIn.getElement("vertex").getProperty<double>("z");
  auto nx = plyIn.getElement("vertex").getProperty<double>("nx");
  auto ny = plyIn.getElement("vertex").getProperty<double>("ny");
  auto nz = plyIn.getElement("vertex").getProperty<double>("nz");
  cloud->width = x.size();
  cloud->height = 1;
  cloud->points.resize(x.size());
  for (int i = 0; i < x.size(); i++) {
    cloud->points[i].x = (float)x[i];
    cloud->points[i].y = (float)y[i];
    cloud->points[i].z = (float)z[i];
    cloud->points[i].normal_x = (float)nx[i];
    cloud->points[i].normal_y = (float)ny[i];
    cloud->points[i].normal_z = (float)nz[i];
    cloud->points[i].r = (std::uint8_t)(std::pow(std::abs(nz[i]), 25) * 255);
    cloud->points[i].g = (std::uint8_t)(std::pow(std::abs(nz[i]), 25) * 255);
    cloud->points[i].b = (std::uint8_t)(std::pow(std::abs(nz[i]), 25) * 255);
  }
}

void randomSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudOut, unsigned int new_size) {
  pcl::RandomSample <pcl::PointXYZRGBNormal> random;
  random.setInputCloud(cloudIn);
  random.setSeed (std::rand ());
  random.setSample(new_size);
  random.filter(*cloudOut);
}


void computeSHOTDescriptors(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr shotFeatures, float radiusSearch) {
  //auto shotEstimation = pcl::SHOTColorEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::SHOT352> (true, true, 8);
  
  auto shotEstimation = pcl::SHOTEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::SHOT352> ();
  shotEstimation.setSearchSurface(cloud);
  shotEstimation.setInputNormals(cloud);
  shotEstimation.setInputCloud(keypoints);
  
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  shotEstimation.setSearchMethod (tree);
  shotEstimation.setRadiusSearch (radiusSearch);
  
  //shotEstimation.setKSearch(10);
  shotEstimation.compute(*shotFeatures);
  
}

void computeSpinImageDescriptors(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, pcl::PointCloud<pcl::Histogram<153> >::Ptr spinImageFeatures, float radiusSearch, unsigned int imageWidth, double supportAngleCos) {
  pcl::SpinImageEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Histogram<153> > siEstimation (imageWidth, supportAngleCos, 16);
  siEstimation.setSearchSurface(cloud);
  siEstimation.setInputNormals(cloud);
  siEstimation.setInputCloud(cloud);
  
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  siEstimation.setSearchMethod (tree);
  siEstimation.setRadiusSearch (radiusSearch);
  
  //shotEstimation.setKSearch(10);
  siEstimation.compute(*spinImageFeatures);

}
