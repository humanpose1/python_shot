#pragma once
#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <utility>
#include <memory>
#include <cmath>
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot.h>
#include "pcl/features/shot_lrf_omp.h"
#include <pcl/features/shot_omp.h>
#include <pcl/features/spin_image.h>
#include <pcl/filters/random_sample.h>
#include <flann/flann.hpp>
#include "happly.h"

bool fileExist(const std::string& name);


void getPointCloudFromPLY(happly::PLYData& plyIn, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

void randomSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudOut, unsigned int new_size);

void computeSHOTDescriptors(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr shotFeatures, float radiusSearch);

void computeSpinImageDescriptors(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, pcl::PointCloud<pcl::Histogram<153> >::Ptr spinImageFeatures, float radiusSearch, unsigned int imageWidth, double supportAngleCos);

void stringSplit(const std::string& data_file, const std::string& delimiter, std::vector<std::string>& listToken);

std::pair<std::string, std::string> pathSplit(const std::string& data_file);

void renameDataFile(const std::string& data_file, const std::string& suffix, std::string& data_out);


