#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <memory>

#include "core.h"

using namespace pybind11::literals;

namespace py = pybind11;

void descriptor2ptr(pcl::PointCloud<pcl::SHOT352>::Ptr descriptor, float* ptr, size_t size){
  for (size_t i = 0; i < descriptor->size(); i++) {
    for (size_t j = 0; j < size; j++) {
      ptr[size*i + j] = descriptor->points[i].descriptor[j];
    }
  }
}

void ptr2pcl(float* pcdPtr, float* normalsPtr, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, py::ssize_t size) {
  cloud->width = size;
  cloud->height = 1;
  cloud->points.resize(size);
  for(py::ssize_t i =0; i < size; i++){
    cloud->points[i].x = pcdPtr[3*i + 0];
    cloud->points[i].y = pcdPtr[3*i + 1];
    cloud->points[i].z = pcdPtr[3*i + 2];
    cloud->points[i].normal_x = normalsPtr[3*i +0];
    cloud->points[i].normal_y = normalsPtr[3*i +1];
    cloud->points[i].normal_z = normalsPtr[3*i +2];
    cloud->points[i].r = (std::uint8_t)(std::pow(std::abs(normalsPtr[3*i +2]), 25) * 255);
    cloud->points[i].g = (std::uint8_t)(std::pow(std::abs(normalsPtr[3*i +2]), 25) * 255);
    cloud->points[i].b = (std::uint8_t)(std::pow(std::abs(normalsPtr[3*i +2]), 25) * 255);
  }
  
}



py::array_t<float> computeSHOT352(py::array_t<float> pcd, py::array_t<float> normals, py::array_t<float> small_pcd, py::array_t<float> small_normals, float radius_search) {
  if (pcd.ndim() != 2 || normals.ndim() != 2 || small_pcd.ndim() != 2)
    throw std::runtime_error("Number of dimensions must be two");
  
  if (pcd.size() != normals.size())
    throw std::runtime_error("Input shapes must match");

  if (pcd.shape()[1] != 3 || normals.shape()[1] != 3, small_pcd.shape()[1] != 3)
    throw std::runtime_error("It must be of dimension 3");
  
  auto pcdPtr =  static_cast<float *>(pcd.request().ptr);
  auto normalsPtr =  static_cast<float *>(normals.request().ptr);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  ptr2pcl(pcdPtr, normalsPtr, cloud, pcd.shape()[0]);
  

  auto smallPcdPtr =  static_cast<float *>(small_pcd.request().ptr);
  auto smallNormalsPtr =  static_cast<float *>(small_normals.request().ptr);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_small(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  ptr2pcl(smallPcdPtr, smallNormalsPtr, cloud_small, small_pcd.shape()[0]);
  

  auto output = py::array_t<float>({small_pcd.shape()[0], static_cast<py::ssize_t>(352)});
  auto outputPtr = static_cast<float *>(output.request().ptr);
  

  pcl::PointCloud<pcl::SHOT352>::Ptr shotFeatures(new pcl::PointCloud<pcl::SHOT352>);
  computeSHOTDescriptors(cloud, cloud_small, shotFeatures, radius_search);
  descriptor2ptr(shotFeatures, outputPtr, 352);
  
  return output;
}

PYBIND11_MODULE(handcrafted_descriptor, m) {
  m.def("compute_shot", &computeSHOT352,
        "compute SHOT descriptors: input are numpy array return shot descriptor for each support"
        "pcd: size N x 3 float"
        "normals: size N x 3 float"
        "small_pcd: size n x 3 float support point"
        "small_normals: size n x 3 float"
        "radius_search: size of the radius"
        , "pcd"_a, "normals"_a, "small_pcd"_a, "small_normals"_a, "radius_search"_a = 1.0f);
}


