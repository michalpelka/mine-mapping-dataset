#ifndef PROCESS_MINE_DATA_SAVELAZ_H
#define PROCESS_MINE_DATA_SAVELAZ_H
#include <iostream>
#include <vector>
#include <string>
#include "Eigen/Eigen"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
namespace SaveLAZ {

    bool exportLaz(const std::string& filename,
                   const std::vector<Eigen::Vector3f>& pointcloud,
                   const std::vector<uint16_t>& intensity =  std::vector<uint16_t>());

    bool exportPcd(const std::string& filename,
                   const std::vector<Eigen::Vector3f>& pointcloud,
                   const std::vector<uint16_t>& intensity =  std::vector<uint16_t>());

};


#endif //PROCESS_MINE_DATA_SAVELAZ_H
