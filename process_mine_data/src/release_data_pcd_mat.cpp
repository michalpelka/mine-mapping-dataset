#include <vector>
#include <iostream>
#include "SaveLAZ.h"
#include "math.h"
#include <cstring>
#include "utils.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <boost/program_options.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>


constexpr size_t kMaxNumberOfPoints = 0.25*1e6;
constexpr float kMaxRangeXY = 450.f;
constexpr float kMaxRangeZ = 450.f;


const Eigen::Vector3d offsetTo2000{ -86346,22674,110 };
Eigen::Affine3d start_integration(Eigen::Matrix4d::Zero());
Eigen::Affine3f start_integration_inv(Eigen::Matrix4f::Zero());

std::string savePointCloud(const std::string &out_dir_name, int laser_id, size_t &pointcloud_count,
                    std::vector<Eigen::Vector3f> &pointcloud,
                    std::vector<uint16_t> &intensity, float filtering) {
    assert(pointcloud.size() == intensity.size());
    if (pointcloud.size()==0){
        return "";
    }
    std::vector<Eigen::Vector3f> pointcloud_lowres;
    std::vector<uint16_t> intensity_lowres;
    if (filtering>0){
        const auto points_to_save = my_utils::downsample(pointcloud, filtering);
        assert(pointcloud.size() == points_to_save.size());
        pointcloud_lowres.reserve(kMaxNumberOfPoints);
        intensity_lowres.reserve(kMaxNumberOfPoints);
        for (size_t i =0; i < pointcloud.size(); i++){
            if (points_to_save[i]){
                pointcloud_lowres.push_back(pointcloud[i]);
                intensity_lowres.push_back(intensity[i]);
            }
        }
    }
    else
    {
        std::swap(pointcloud_lowres,pointcloud);
        std::swap(intensity_lowres,intensity);

    }

    std::stringstream oss;
    oss << std::setw(3) << std::setfill('0') << pointcloud_count;

    std::string filename(out_dir_name + "/" + configruation::kLaserNames.at(laser_id)+ "_" + oss.str());
    bool ok = SaveLAZ::exportPcd(filename+".pcd", pointcloud_lowres, intensity_lowres);
    if (ok) {
        std::cout << "Saved PCD" << std::endl;
    }
    pointcloud.clear();
    intensity.clear();
    pointcloud.reserve(kMaxNumberOfPoints);
    intensity.reserve(kMaxNumberOfPoints);
    pointcloud_count++;
    if (ok){
        return filename;
    }
    return "";
}

Eigen::Vector3f
convertPoint(int laser_id, const Eigen::Vector3f &src_p, const Eigen::Affine3f &odometry, float rot_angle = 0) {
    Eigen::Vector3f dst_p{0, 0, 0};
    if (laser_id == 0) {
        dst_p = odometry * src_p;
    }
    if (laser_id == 1) {
        dst_p = odometry * configruation::getVLP32Calib() * src_p;
    }
    if (laser_id == 2) {
        float s = sin(-rot_angle);
        float c = cos(-rot_angle);
        Eigen::Matrix4f r;
        r << c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
        dst_p = odometry * configruation::getUnitToRotationAxis_M1() * Eigen::Affine3f(r) *
                configruation::getUnitFromRotToVLP16_M2() * src_p;
    }
    return dst_p;
}
namespace po = boost::program_options;
int main(int argc, char *argv[]) {

    po::variables_map vm;
    try {
        po::options_description desc("Allowed options");
        desc.add_options()
                ("help", "produce help message")
                ("bags", po::value<std::string>(), "directory with bags (bag files)")
                ("csv", po::value<std::string>(), "directory with trajectory (csv files)")
                ("output", po::value<std::string>(), "output directory for pcd")
                ("filtering",  po::value<float>()->default_value(-1.0f), "downsampling");
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    }
    catch(const std::exception &e){
        std::cerr << e.what() << std::endl;
        return 1;
    }

    const std::string out_dir_name = vm["output"].as<std::string>();
    boost::filesystem::create_directory(out_dir_name);
    const float voxel_size = vm["filtering"].as<float>();


    std::vector<std::string> bag_files = my_utils::findFiles(vm["bags"].as<std::string>(), ".bag");
    std::vector<std::string> fns_trj = my_utils::findFiles(vm["csv"].as<std::string>(), ".csv");
    std::sort(bag_files.begin(), bag_files.end());

    std::map<double, Eigen::Matrix4d> trajectory;

    // parse csv and assemble maps
    for (auto fn : fns_trj) {
        std::fstream infile(fn);
        std::string line;
        std::getline(infile, line);
        while (std::getline(infile, line)) {
            auto r = my_utils::loadLineCsv(line);
            trajectory[r.first] = r.second;
        }
    }

    std::cout << "Trajectory info :\n";
    std::cout << "\t begin time :" << trajectory.begin()->first << "\n";
    std::cout << "\t end   time :" << trajectory.rbegin()->first << "\n";
    std::cout << "\t count      :" << trajectory.size() << "\n";

    auto process = [&](int laser_id) {
        std::vector<Eigen::Vector3f> pointcloud;
        std::vector<uint16_t> intensity;
        pointcloud.reserve(kMaxNumberOfPoints);
        intensity.reserve(kMaxNumberOfPoints);
        size_t pointcloud_count = 0;
        std::string laser_topic = configruation::kTopicNames.at(laser_id);

        for (const auto &p : bag_files) {
            std::cout << "processing " << p << std::endl;
            rosbag::Bag bag;
            bag.open(p);
            std::vector<std::string> topics{laser_topic};

            rosbag::View view(bag, rosbag::TopicQuery(topics));
            for (rosbag::MessageInstance const m: view) {
                auto pointcloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
                if (!pointcloud_ptr) {
                    continue;
                }
                pcl::PCLPointCloud2 pcl_pc2;
                pcl_conversions::toPCL(*pointcloud_ptr, pcl_pc2);
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr tempCloud(
                        new pcl::PointCloud<pcl::PointXYZINormal>);
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr tempCloudFilter(
                        new pcl::PointCloud<pcl::PointXYZINormal>);
                pcl::fromPCLPointCloud2(pcl_pc2, *tempCloud);

                if (tempCloud->empty()) {
                    continue;
                }

                pcl::RadiusOutlierRemoval<pcl::PointXYZINormal> sor;
                sor.setInputCloud (tempCloud);
                sor.setRadiusSearch(1.0);
                sor.setMinNeighborsInRadius(4);
//                sor.setMeanK (150);
//                sor.setStddevMulThresh (10.0);
                sor.filter (*tempCloudFilter);
                const Eigen::Affine3f local_laser{configruation::getLivoxCalib().cast<float>()};
                float time_stamp;
                for (auto &p : *tempCloudFilter) {
                    time_stamp = p.normal_y; // timestamp is kept as normal-Y PCD field
                    const float angle = p.normal_x; // angle of rotation is kept as normal-X PCD field
                    [[maybe_unused]] const float ring = p.normal_z; // ring is kept as normal-Z PCD field
                    const Eigen::Matrix4d mat = my_utils::getInterpolatedPose(trajectory, time_stamp);

                    if (!mat.isZero()) {
                        if (start_integration.matrix() == Eigen::Matrix4d::Zero()){
                            std::cout << "start aggregating"<<std::endl;
                            start_integration = mat;
                            start_integration_inv = start_integration.inverse().cast<float>();
                        }
                        const Eigen::Matrix4d mat_offset = start_integration.inverse()* mat;

                        const Eigen::Vector3f dst_p = convertPoint(laser_id, p.getVector3fMap(),
                                Eigen::Affine3f(mat_offset.cast<float>()), angle);
                        if (dst_p.head<2>().norm() < kMaxRangeXY && dst_p.norm() < kMaxRangeZ) {
                            pointcloud.push_back(dst_p);
                            intensity.push_back(p.intensity);
                        }
                    }
                }
                if (pointcloud.size() > kMaxNumberOfPoints) {
                    auto fn = savePointCloud(out_dir_name, laser_id, pointcloud_count, pointcloud, intensity, voxel_size);
                    if (fn.size()){
                        Eigen::Affine3d offsetMat{Eigen::Matrix4d::Identity()};
                        offsetMat.translate(offsetTo2000);
                        my_utils::saveMat(fn+".txt", (offsetMat * start_integration).matrix().cast<double>());
                        std::ofstream ts_f(fn+".ts");
                        ts_f << time_stamp << std::endl;
                    }
                    start_integration = Eigen::Affine3d(Eigen::Matrix4d::Zero());
                }
            }
            bag.close();
        }
        //savePointCloud(out_dir_name, laser_id, pointcloud_count, pointcloud, intensity, voxel_size);
    };

    std::vector<std::thread> ths;
    for (const int laser_id : { 0}) {
        ths.emplace_back(process, laser_id);
    }
    for (auto &t : ths) {
        t.join();
    }
}