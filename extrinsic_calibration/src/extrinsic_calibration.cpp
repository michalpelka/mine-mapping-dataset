#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <assert.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>

#include <GL/freeglut.h>

#include "imgui.h"
#include "imgui_impl_glut.h"
#include "imgui_impl_opengl2.h"

#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include "cost_fun.h"
#include "boost/property_tree/json_parser.hpp"
#include <boost/program_options.hpp>
#include <pcl/filters/radius_outlier_removal.h>

const unsigned int window_width = 1920;
const unsigned int window_height = 1080;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
float rotate_x = 0.0, rotate_y = 0.0;
float translate_z = -30.0;
float translate_x, translate_y = 0.0;
bool gui_mouse_down{false};

void display();
void reshape(int w, int h);
void mouse(int glut_button, int state, int x, int y);
void motion(int x, int y);
bool initGL(int *argc, char **argv);

float imgui_co_size{1.0f};
bool imgui_draw_co{true};

Eigen::Affine3d getCalibGt(){
    Eigen::Matrix4d t_co;
    t_co <<       0.126034,    0.992026,    0.000000,   -0.687902,
            -0.992026,    0.126034,   -0.000000,    0.653370,
            -0.000000,    0.000000,    1.000000, -262.172742,
            0.000000,    0.000000,    0.000000 ,   1.000000;
    return orthogonize(Eigen::Affine3d(t_co));



    Sophus::SE3d m(orthogonize(Eigen::Affine3d(t_co)).matrix());
    std::cout << "========================" << std::endl;
    std::cout << m.log() << std::endl;

    return orthogonize(Eigen::Affine3d(t_co));

}

std::vector< Eigen::Affine3d> getBackpackCalib() {
    Eigen::Matrix4d livox_co;
    Eigen::Matrix4d mast_base;
    Eigen::Matrix4d unit_base;
    Eigen::Matrix4d vlp32_co;
    Eigen::Matrix4d vlp16_co;
// -----

    livox_co << 0.000000, 0.000000, -1.000000, -0.057900,
            0.000000, -1.000000, -0.000000, -0.065000,
            -1.000000, 0.000000, -0.000000, -0.067986,
            0.000000, 0.000000, 0.000000, 1.000000;
    mast_base << 0.707107, -0.707107, 0.000000, 0.075000,
            0.707107, 0.707107, 0.000000, -0.075000,
            0.000000, 0.000000, 1.000000, 0.000000,
            0.000000, 0.000000, 0.000000, 1.000000;
    unit_base << 1.000000, 0.000000, 0.000000, -0.007000,
            0.000000, -1.000000, -0.000000, 0.065430,
            0.000000, 0.000000, -1.000000, 0.000000,
            0.000000, 0.000000, 0.000000, 1.000000;
    vlp32_co << -0.999995, 0.002365, -0.001985, 0.000922,
            0.003088, 0.766042, -0.642783, -0.032785,
            -0.000000, -0.642786, -0.766046, -0.474070,
            0.000000, 0.000000, 0.000000, 1.000000;
    vlp16_co << 0.000001, -0.000000, 1.000000, 0.000000,
            0.000000, 1.000000, 0.000000, 0.000000,
            -1.000000, -0.000000, 0.000001, 0.257200,
            0.000000, 0.000000, 0.000000, 1.000000;



    std::vector<Eigen::Affine3d> m{
            orthogonize(Eigen::Affine3d(livox_co)), orthogonize(Eigen::Affine3d(mast_base * vlp32_co)),
            orthogonize(Eigen::Affine3d(unit_base))
    };

    return m;
}
const std::vector< Eigen::Affine3d> laser_calib(getBackpackCalib());
const std::string TOPIC_LIVOX="/livox_raw";
const std::string TOPIC_IMU="/imu/data_hwts";
const std::string TOPIC_VELO_ROT="/velodyne_rot_raw";
const std::string TOPIC_VELO_STATIC="/velodyne_static_raw";

const double TIME_OFFSET_PCD= 1600000000.0;
const double VERTICAL_SPEED = 0.10;

//const std::vector<std::string> bag_files{
//
////        "/home/robot/logs/Day2_2021-10-30-10-33-12_3.bag"
////        "/home/robot/logs/Day2_2021-10-30-10-36-12_6.bag",
////        "/home/robot/logs/Day2_2021-10-30-10-37-12_7.bag"
//
//        "/home/michal/kopalnia_bags/bags/day2/Day2_2021-10-30-10-51-12_21.bag"
//
//
//};

struct IMU{
    Eigen::Affine3d ahrs;
};


float imgui_end=10;
float imgui_start=0;
float imgui_speed=0.156;
float imgui_nn=0.2;

float imgui_angle=M_PI;
bool imgui_draw_nn = true;
bool imgui_draw_gt = true;
bool imgui_draw_half = true;
int imgui_loops = 1;
std::array<bool,3> imgui_draw_lasers {true,true,true};


bool imgui_use_imu{true};

std::map<double, IMU> imu_map;
struct scan{
    int laser_id;
    IMU ahrs;
    Sophus::SE3d pose_optimized;
    Sophus::SE3d pose_initial;

    bool operator<(const scan &rhs) const {
        return timestamp < rhs.timestamp;
    }

    pcl::PointCloud<pcl::PointXYZINormal> cloud;
    double timestamp;
    double timestamp_offset;
    void reset(float velocity){
        pose_initial = Sophus::SE3d(ahrs.ahrs.matrix());
        pose_initial.translation().z() = velocity * timestamp_offset;
        pose_optimized = pose_initial;
    }
   std::string fn_cloud_full;
};
std::vector<scan> keyframes;

bool optimize_trajectory{false};
std::array<bool,3> laser_compute_nn{false};
std::array<bool,3> laser_to_apply{false};

struct NN{
    int keyframe_1_id;
    int keyframe_1_point_id;
    int keyframe_2_id;
    int keyframe_2_point_id;
    Eigen::Vector3f p1;
    Eigen::Vector3f p2;

};
std::vector<NN> nearest_nn;
std::vector<NN> nearest_gtnn;

std::vector<Sophus::SE3d> laser_se3d{};
pcl::PointCloud<pcl::PointXYZ> groundtruth;
Sophus::SE3d gt_se3 (getCalibGt().matrix());

Eigen::Affine3f getVelodyne1(){
    Eigen::Affine3f rot0 (Eigen::Affine3f::Identity());
    rot0.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f rot1 (Eigen::Affine3f::Identity());
    rot1.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitX()));

    //rot1.translate(Eigen::Vector3f (0, 0.003542, 0.072236));
    //rot1.rotate(Eigen::Quaternionf(-0.509959, -0.391302, -0.466336,0.607746));
    return rot1*rot0;
}
const Eigen::Affine3f getRotLaserLocCalib(){
    Eigen::Matrix4f d;
    d <<0.999921, -0.00638093,   0.0107958, -0.00731472,
            0.00634553,    0.999974,  0.00331037,   0.0463718,
            -0.0108166, -0.00324161,    0.999936,   0.0222408,
            0,           0,           0 ,          1;
    return Eigen::Affine3f (d);
}
Eigen::Affine3f getAngle(float angle){
    static Eigen::Affine3f staticTransform = getVelodyne1();
    Eigen::Affine3f rot2 (Eigen::Affine3f::Identity());
    rot2.rotate(Eigen::AngleAxisf(angle-5.53747-0.98*M_PI, Eigen::Vector3f::UnitZ()));
    return rot2*staticTransform*getRotLaserLocCalib();
}

pcl::PointCloud<pcl::PointXYZINormal> filterCloud(pcl::PointCloud<pcl::PointXYZINormal>& cloud, float filterSize)
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_temp0(new pcl::PointCloud<pcl::PointXYZL>());
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZL>());
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_temp2(new pcl::PointCloud<pcl::PointXYZL>());

    cloud_temp0->reserve(cloud.size());
    for (int i =0; i < cloud.size(); i++)
    {
        pcl::PointXYZL p;
        p.getArray3fMap() = cloud[i].getArray3fMap();
        if(p.x!=p.x ||p.y!=p.y||p.z!=p.z  )continue;
        p.label = i;
        cloud_temp0->push_back(p);
    }

    pcl::VoxelGrid<pcl::PointXYZL> sor;
    sor.setDownsampleAllData(true);
    sor.setInputCloud (cloud_temp0);
    sor.setLeafSize (filterSize,filterSize,filterSize);
//    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    pcl::PointCloud<pcl::PointXYZINormal> aggregate_filter;
    sor.filter(*cloud_temp1);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZL> sor2;
    sor2.setInputCloud (cloud_temp1);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (0.05);
    sor2.filter (*cloud_temp2);

    pcl::PointCloud<pcl::PointXYZINormal> ret;
    ret.resize(cloud_temp2->size());
    for (int i = 0 ; i< ret.size(); i++)
    {
        ret[i] = cloud[cloud_temp2->at(i).label];
    }
    return ret;
}



std::vector<scan> createKeyFrames(const std::vector<std::string>& bag_files, int laser_id, float filterSize){
    // load pointcloud
    std::vector<scan> keyframes;
    const double segment_time = 0.5;
    const double keyframe_time = 0.5;

    const std::vector<std::string> laser_topics{TOPIC_LIVOX, TOPIC_VELO_STATIC, TOPIC_VELO_ROT};
    const std::vector<float> laser_scale{1.0f,2.0f,1.0f};
    static int keyframe_id = 0;
    for (const auto &bag_fn: bag_files) {
        rosbag::Bag bag;
        bag.open(bag_fn, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery({laser_topics[laser_id]}));
        pcl::PointCloud<pcl::PointXYZINormal> aggregate;
        for (rosbag::MessageInstance const m: view) {
            const auto s = m.instantiate<sensor_msgs::PointCloud2>();
            if (!s)continue;
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*s, pcl_pc2);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr tempCloud(
                    new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::fromPCLPointCloud2(pcl_pc2, *tempCloud);
            const float scale = laser_scale[laser_id];
            pcl::transformPointCloud(*tempCloud,*tempCloud, Eigen::Affine3f(Eigen::Matrix4f::Identity() * scale));
            if (tempCloud->empty())continue;
            const double ts = (*tempCloud)[0].normal_y;
            if (!keyframes.empty() && ts - keyframes.back().timestamp < keyframe_time ){
                continue;
            }
            aggregate+=*tempCloud;
            if (aggregate.back().normal_y -  aggregate.front().normal_y > segment_time  )
            {
                const auto it = imu_map.lower_bound(ts);
                scan s;
                s.timestamp = ts;
                s.laser_id = laser_id;
                if (laser_id ==2 )
                {
                    s.cloud = filterCloud(aggregate, filterSize);
                }else
                {
                    s.cloud = filterCloud(aggregate, filterSize);
                }
                s.fn_cloud_full = "/tmp/cloud_"+std::to_string(keyframe_id)+".pcd";
                pcl::io::savePCDFile(s.fn_cloud_full , aggregate, true);
                s.timestamp_offset = ts - imu_map.begin()->first;
                s.ahrs = it->second;
                s.pose_optimized = Sophus::SE3d(s.ahrs.ahrs.matrix());
                s.pose_optimized.translation().z() = -imgui_speed * s.timestamp_offset;
                s.pose_initial = s.pose_optimized;
                keyframes.push_back(s);
                aggregate.clear();
                keyframe_id++;
            }
        }
    }
    return keyframes;
}
std::string json_filename_config;
int main(int argc, char *argv[]) {

    laser_se3d.clear();
    laser_se3d.push_back(Sophus::SE3d(laser_calib[0].matrix()));
    laser_se3d.push_back(Sophus::SE3d(laser_calib[1].matrix()));
    laser_se3d.push_back(Sophus::SE3d(laser_calib[2].matrix()));


    std::cout << "collect IMU data" << std::endl;
    double minimum_ts = std::numeric_limits<double>::max();

    namespace bo=boost::program_options;
    bo::variables_map vm;
    try
    {
        bo::options_description desc{"Options"};
        desc.add_options()
                ("gt", bo::value<std::string>(), "Ground truth")
                ("json", bo::value<std::string>(), "Json state")
                ("bags", bo::value<std::vector<std::string>>(), "Bags")
                ("filter", bo::value<float>()->default_value(0.1f), "filter");

        store(parse_command_line(argc, argv, desc), vm);
        notify(vm);
    }
    catch (const boost::program_options::error &ex)
    {
        std::cerr << ex.what() << '\n';
        return 1;
    }

    const std::string groundtruth_pcd_filename{vm["gt"].as<std::string>()};
    json_filename_config = {vm["json"].as<std::string>()};
    const std::vector<std::string> bag_files{vm["bags"].as<std::vector<std::string>>()};
    const float filterSize{vm["filter"].as<float>()};
    pcl::io::loadPCDFile(groundtruth_pcd_filename, groundtruth);

    for (const auto &bag_fn: bag_files) {
        std::cout << "openning " << bag_fn << std::endl;
        rosbag::Bag bag;
        bag.open(bag_fn, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery({TOPIC_IMU}));
        for (rosbag::MessageInstance const m: view) {
            const auto s = m.instantiate<sensor_msgs::Imu>();
            if (!s)continue;
            double ts = s->header.stamp.toSec() - TIME_OFFSET_PCD;
            if (minimum_ts == std::numeric_limits<double>::max()) {
                minimum_ts = ts;
                imgui_start = ts;
            }
            double delta_time = ts - minimum_ts;
            assert(delta_time >= 0);
            Eigen::Affine3d t(Eigen::Affine3d::Identity());
            t.rotate(Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()));
            Eigen::Quaterniond q{s->orientation.w, s->orientation.x, s->orientation.y, s->orientation.z};
            Eigen::Affine3d pose(Eigen::Affine3d::Identity());
            q.normalize();
            pose.rotate(q);

            imu_map[ts] = {(  pose )};
        }
    }

    assert(imu_map.size() > 2);
    std::cout << "Imu poses collected " << imu_map.size() << std::endl;
    std::cout << "Imu stream length " << imu_map.rbegin()->first - imu_map.begin()->first << std::endl;

    std::cout << "Imu timestamp start " << std::fixed << imu_map.begin()->first << std::endl;
    std::cout << "Imu timestamp start " << std::fixed << imu_map.rbegin()->first << std::endl;

    auto t0 = createKeyFrames(bag_files, 0, filterSize);
    keyframes.insert(keyframes.end(),t0.begin(),t0.end());
    auto t1 = createKeyFrames(bag_files, 1, filterSize);
    keyframes.insert(keyframes.end(),t1.begin(),t1.end());
    auto t2 = createKeyFrames(bag_files, 2, filterSize);
    keyframes.insert(keyframes.end(),t2.begin(),t2.end());

    std::stable_sort(keyframes.begin(),keyframes.end());
    //pcl::io::savePCDFile("/home/robot/test.pcd",aggregate, true );
    std::cout <<"Done" << std::endl;
    initGL(&argc, argv);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMainLoop();
    return 0;
}

void offset(int laser_id, float tx, float ty, float tz, float rx, float ry, float rz)
{
    Sophus::Vector6f u;
    u << tx,ty,tz,rx,ry,rz;
    Sophus::SE3d update = Sophus::SE3d::exp(u.cast<double>());
    if (laser_id == -1)
    {
        gt_se3 = update * gt_se3 ;
        std::cout << "gt_se3" << std::endl;
        std::cout << gt_se3.matrix() << std::endl;
        return;
    }
    laser_se3d[laser_id] = update * laser_se3d[laser_id];
    std::cout << "laser_se3d["<<laser_id<<"]"<< std::endl;

    std::cout << laser_se3d[laser_id] .matrix() << std::endl;
}


void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(translate_x, translate_y, translate_z);
    glRotatef(rotate_x, 1.0, 0.0, 0.0);
    glRotatef(rotate_y, 0.0, 0.0, 1.0);


    glLineWidth(3.0f);
    glBegin(GL_LINES);
    for (int keyframe1_id = 1; keyframe1_id < keyframes.size(); keyframe1_id++) {
//        if ( keyframes[keyframe1_id].timestamp > imgui_start && keyframes[keyframe1_id].timestamp < imgui_start + imgui_end) {
        //if (keyframes[keyframe1_id].laser_id != 0) continue;
        Eigen::Vector4d pt0 = keyframes[keyframe1_id].pose_optimized * Eigen::Vector4d(0, 0, 0, 1);

        Eigen::Vector4d pt1 = keyframes[keyframe1_id].pose_optimized * Eigen::Vector4d(.5, 0, 0, 1);
        Eigen::Vector4d pt2 = keyframes[keyframe1_id].pose_optimized * Eigen::Vector4d(0, .5, 0, 1);
        Eigen::Vector4d pt3 = keyframes[keyframe1_id].pose_optimized * Eigen::Vector4d(0, 0, .5, 1);

        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(pt0.x(), pt0.y(), pt0.z());
        glVertex3f(pt1.x(), pt1.y(), pt1.z());
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(pt0.x(), pt0.y(), pt0.z());
        glVertex3f(pt2.x(), pt2.y(), pt2.z());
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(pt0.x(), pt0.y(), pt0.z());
        glVertex3f(pt3.x(), pt3.y(), pt3.z());
//        }
    }

    glEnd();

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
    ImGui::Begin("Demo Window1");
    ImGui::Text("Text");
    ImGui::SliderFloat("imgui_start", &imgui_start,imu_map.begin()->first, imu_map.rbegin()->first );
    ImGui::SliderFloat("imgui_end", &imgui_end,0, 120 );
    ImGui::SliderFloat("imgui_speed", &imgui_speed,0.1, 0.2 );
    ImGui::SliderFloat("imgui_angle", &imgui_angle,-3.142,3.142 );
    ImGui::Checkbox("imgui_use_imu", &imgui_use_imu);
    ImGui::Checkbox("imgui_draw_nn", &imgui_draw_nn);
    ImGui::Checkbox("imgui_draw_gt", &imgui_draw_gt);
    ImGui::Checkbox("draw_l0", imgui_draw_lasers.data());
    ImGui::Checkbox("draw_l1", imgui_draw_lasers.data()+1);
    ImGui::Checkbox("draw_l2", imgui_draw_lasers.data()+2);
    ImGui::Checkbox("draw_half", &imgui_draw_half);
    ImGui::SliderInt("imgui_loops", &imgui_loops,0, 10);

    if (ImGui::Button("tz-")){offset(-1,0.f,0.f, -0.1f, 0.f,0.f,0.f);}
    ImGui::SameLine();
    if (ImGui::Button("tz+")){offset(-1,0.f,0.f, 0.1f, 0.f,0.f,0.f);}

    if (ImGui::Button("l_0z-")){offset(0,0.f,0.f, -0.1f, 0.f,0.f,0.f);}
    ImGui::SameLine();
    if (ImGui::Button("l_0z+")){offset(0,0.f,0.f, 0.1f, 0.f,0.f,0.f);}

    if (ImGui::Button("l_1z-")){offset(1,0.f,0.f, -0.1f, 0.f,0.f,0.f);}
    ImGui::SameLine();
    if (ImGui::Button("l_1z+")){offset(1,0.f,0.f, 0.1f, 0.f,0.f,0.f);}

    if (ImGui::Button("l_2z-")){offset(2,0.f,0.f, -0.1f, 0.f,0.f,0.f);}
    ImGui::SameLine();
    if (ImGui::Button("l_2z+")){offset(2,0.f,0.f, 0.1f, 0.f,0.f,0.f);}


    ImGui::SliderFloat("imgui_nn", &imgui_nn, 0.0f, 1.0f);

    if(ImGui::Button("reset"))
    {
        for (auto & k : keyframes){
            k.reset(-imgui_speed);
        }

    }

    if(ImGui::Button("load_state_trajectory"))
    {
        const auto deserialize = [](const std::string str){
            std::cout << "str " << str << std::endl;
            std::stringstream ss(str);
            Sophus::Vector6d l;
            ss >> l[0];ss >> l[1];ss >> l[2];
            ss >> l[3];ss >> l[4];ss >> l[5];
            return Sophus::SE3d::exp(l);
        };
        boost::property_tree::ptree  pt;
        boost::property_tree::read_json(json_filename_config,pt);
        std::string str;

        gt_se3 = deserialize(pt.get<std::string>("gt_se3d"));
        laser_se3d[0] = deserialize(pt.get<std::string>("laser_se3d_0"));
        laser_se3d[1] = deserialize(pt.get<std::string>("laser_se3d_1"));
        laser_se3d[2] = deserialize(pt.get<std::string>("laser_se3d_2"));

        for (int i =0; i < keyframes.size(); i++)
        {
            auto str = pt.get_optional<std::string>("keyframe_"+std::to_string(i));
            if (str){
                keyframes[i].pose_optimized = deserialize(*str);
            }

//            pt.get("keyframe_"+std::to_string(i), str);
//            keyframes[i].pose_optimized = deserialize(pt.get<std::string>("keyframe_"+std::to_string(i)));
        }

    }
    ImGui::SameLine();
    if(ImGui::Button("load_state"))
    {
        const auto deserialize = [](const std::string str){
            std::cout << "str " << str << std::endl;
            std::stringstream ss(str);
            Sophus::Vector6d l;
            ss >> l[0];ss >> l[1];ss >> l[2];
            ss >> l[3];ss >> l[4];ss >> l[5];
            return Sophus::SE3d::exp(l);
        };
        boost::property_tree::ptree  pt;
        boost::property_tree::read_json(json_filename_config,pt);
        std::string str;

        gt_se3 = deserialize(pt.get<std::string>("gt_se3d"));
        laser_se3d[0] = deserialize(pt.get<std::string>("laser_se3d_0"));
        laser_se3d[1] = deserialize(pt.get<std::string>("laser_se3d_1"));
        laser_se3d[2] = deserialize(pt.get<std::string>("laser_se3d_2"));
        for (int i =0; i < 3; i++)
        {
            std::cout << "  laser_se3d"<<i << std::endl;
            std::cout << laser_se3d[i].matrix() << std::endl;
        }
    }
    ImGui::SameLine();
    if(ImGui::Button("save_state"))
    {
        const auto serialize = [](Sophus::SE3d &m){
            std::stringstream ss;
            const auto m_l = m.log();
            ss << m_l[0] << " "<< m_l[1] << " "<< m_l[2] << " ";
            ss << m_l[3] << " "<< m_l[4] << " "<< m_l[5];
            return ss.str();
        };

        boost::property_tree::ptree  pt;
        pt.put("gt_se3d", serialize(gt_se3));
        pt.put("laser_se3d_0", serialize(laser_se3d[0]));
        pt.put("laser_se3d_1", serialize(laser_se3d[1]));
        pt.put("laser_se3d_2", serialize(laser_se3d[2]));
        for (int i =0; i < keyframes.size(); i++)
        {
            pt.put("keyframe_"+std::to_string(i), serialize(keyframes[i].pose_optimized));
        }
        boost::property_tree::write_json(json_filename_config, pt);

    }

    if(ImGui::Button("export_pcd"))
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>> pcd;
        pcd.resize(3);
        Eigen::Affine3f imu_offset(Eigen::Affine3d::Identity());
        imu_offset.rotate(Eigen::AngleAxisf(imgui_angle, Eigen::Vector3f::UnitZ()));
        for (int keyframe_id = 0; keyframe_id < keyframes.size(); keyframe_id++)
        {
            pcl::PointCloud<pcl::PointXYZINormal> cloud_full;
            pcl::io::loadPCDFile( keyframes[keyframe_id].fn_cloud_full, cloud_full);
            const auto laser_id = keyframes[keyframe_id].laser_id;
            Eigen::Affine3f laser_co {laser_se3d.at(laser_id).matrix().cast<float>()};
            const scan &keyframe = keyframes[keyframe_id];
            if ( keyframe.timestamp > imgui_start &&  keyframe.timestamp < imgui_start + imgui_end) {
                Eigen::Affine3f  pose =Eigen::Affine3f(keyframe.pose_optimized.matrix().cast<float>()) * imu_offset.inverse();

                pcl::PointCloud<pcl::PointXYZI> tempCloud;
                tempCloud.reserve(cloud_full.size());
                for (int i = 0; i <cloud_full.size(); i++) {
                    Eigen::Vector3f p = cloud_full[i].getArray3fMap();
                    pcl::PointXYZI pt;
                    pt.intensity = cloud_full[i].intensity;
                    if (laser_id == 2) {
                        pt.getArray3fMap() = pose * laser_co * getAngle( -cloud_full[i].normal_x)* p;
                    }else{
                        pt.getArray3fMap() = pose * laser_co * p;
                    }
                    tempCloud.push_back(pt);
                }
                pcl::RadiusOutlierRemoval<pcl::PointXYZI> sor;
                sor.setInputCloud (tempCloud.makeShared());
                sor.setRadiusSearch(1.0);
                sor.setMinNeighborsInRadius(4);
                pcl::PointCloud<pcl::PointXYZI> tempCloudFilter;
                sor.filter (tempCloudFilter);
                pcl::VoxelGrid<pcl::PointXYZI> sor2;
                sor2.setDownsampleAllData(true);
                sor2.setInputCloud (tempCloudFilter.makeShared());
                sor2.setLeafSize (0.01,0.01,0.01);
                pcl::PointCloud<pcl::PointXYZI> tempCloudFilter2;
                sor2.filter (tempCloudFilter2);

                pcd[laser_id]+=tempCloudFilter2;

            }
        }
        pcl::io::savePCDFile("/tmp/pc_laser_0.pcd", pcd[0],true);
        pcl::io::savePCDFile("/tmp/pc_laser_1.pcd", pcd[1],true);
        pcl::io::savePCDFile("/tmp/pc_laser_2.pcd", pcd[2],true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr gt_transformed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(groundtruth, *gt_transformed, gt_se3.matrix().cast<float>());
        pcl::io::savePCDFile("/tmp/pc_gt.pcd", *gt_transformed,true);
    }

    ImGui::Checkbox("l1_nn",&laser_compute_nn[0]);
    ImGui::SameLine();
    ImGui::Checkbox("l1_optimize",&laser_to_apply[0]);
    //
    ImGui::Checkbox("l2_nn",&laser_compute_nn[1]);
    ImGui::SameLine();
    ImGui::Checkbox("l2_optimize",&laser_to_apply[1]);
    ///
    ImGui::Checkbox("l3_nn",&laser_compute_nn[2]);
    ImGui::SameLine();
    ImGui::Checkbox("l3_optimize",&laser_to_apply[2]);
    ///
    ImGui::Checkbox("optimize_trajectory",&optimize_trajectory);


    if(ImGui::Button("optimize_gtnn"))
    {
        for (int iter = 0; iter < imgui_loops ; iter ++) {
            // find nn
            Eigen::Affine3f imu_offset(Eigen::Affine3d::Identity());
            imu_offset.rotate(Eigen::AngleAxisf(imgui_angle, Eigen::Vector3f::UnitZ()));
            nearest_gtnn.clear();
            pcl::PointCloud<pcl::PointXYZ>::Ptr gt_transformed(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(groundtruth, *gt_transformed, gt_se3.matrix().cast<float>());
            pcl::KdTreeFLANN<pcl::PointXYZ> gt_kdtree;
            gt_kdtree.setInputCloud(gt_transformed);
            for (int keyframe_id = 0; keyframe_id < keyframes.size(); keyframe_id++) {
                const scan &keyframe = keyframes[keyframe_id];
                const auto laser_id = keyframe.laser_id;
                if (!laser_compute_nn[0] && laser_id == 0){
                    continue;
                }
                if (!laser_compute_nn[1] && laser_id == 1){
                    continue;
                }
                if (!laser_compute_nn[2] && laser_id == 2){
                    continue;
                }

                Eigen::Affine3f laser_co{laser_se3d.at(laser_id).matrix().cast<float>()};
                if (keyframe.timestamp > imgui_start && keyframe.timestamp < imgui_start + imgui_end) {
                    Eigen::Affine3f pose =
                            Eigen::Affine3f(keyframe.pose_optimized.matrix().cast<float>()) * imu_offset.inverse();
                    for (int i = 0; i < keyframe.cloud.size(); i+=1) {
                        Eigen::Vector3f p = keyframe.cloud[i].getArray3fMap();
                        pcl::PointXYZ query_point;
                        if (laser_id == 2) {
                            query_point.getArray3fMap() = pose * laser_co * getAngle( -keyframe.cloud[i].normal_x)* p;
                        }else{
                            query_point.getArray3fMap() = pose * laser_co * p;
                        }

                        const int K = 1;
                        const float radius = imgui_nn;
                        std::vector<int> pointIdxKNNSearch(K);
                        std::vector<float> pointKNNSquaredDistance(K);

                        if (gt_kdtree.radiusSearch(query_point, radius, pointIdxKNNSearch, pointKNNSquaredDistance) >
                            0) {
                            NN found_nn;
                            found_nn.keyframe_1_id = keyframe_id;
                            found_nn.keyframe_2_id = -1;
                            found_nn.keyframe_1_point_id = i;
                            found_nn.keyframe_2_point_id = pointIdxKNNSearch[0];
                            found_nn.p1 = query_point.getArray3fMap();
                            found_nn.p2 = gt_transformed->at(found_nn.keyframe_2_point_id).getArray3fMap();
                            nearest_gtnn.push_back(found_nn);
                        }
                    }
                }
            }
            // optimize
            Eigen::Affine3d imu_offsetd(Eigen::Affine3d::Identity());
            imu_offsetd.rotate(Eigen::AngleAxisd(imgui_angle, Eigen::Vector3d::UnitZ()));
            ceres::Problem problem;
            Eigen::Affine3f transform_gt(gt_se3.matrix().cast<float>());
            for (int i = 0; i < keyframes.size(); i++) {
                problem.AddParameterBlock(keyframes[i].pose_optimized.data(), Sophus::SE3d::num_parameters,
                                          new LocalParameterizationSE3());
                if (!optimize_trajectory){
                    problem.SetParameterBlockConstant(keyframes[i].pose_optimized.data());
                }
            }

//            for (int i = 1; i < keyframes.size(); i++) {
//                ceres::LossFunction *loss = nullptr;
//                ceres::CostFunction *cost_function = RelativePose::Create(keyframes[i - 1].pose_initial,
//                                                                          keyframes[i].pose_initial);
//                problem.AddResidualBlock(cost_function, loss, (double *) keyframes[i - 1].pose_optimized.data(),
//                                         (double *) keyframes[i].pose_optimized.data());
//                if (!optimize_trajectory){
//                    problem.SetParameterBlockConstant( (double *) keyframes[i].pose_optimized.data());
//                }
//            }

            problem.AddParameterBlock((double *) laser_se3d[0].data(), Sophus::SE3d::num_parameters,
                                      new LocalParameterizationSE3());
            problem.AddParameterBlock((double *) laser_se3d[1].data(), Sophus::SE3d::num_parameters,
                                      new LocalParameterizationSE3());
            problem.AddParameterBlock((double *) laser_se3d[2].data(), Sophus::SE3d::num_parameters,
                                      new LocalParameterizationSE3());

            for (const auto &nn: nearest_gtnn) {
                const scan &keyframe1 = keyframes[nn.keyframe_1_id];
                const int laser1_id = keyframe1.laser_id;

                Eigen::Affine3f laser1_co{laser_se3d[laser1_id].matrix().cast<float>()};
                Eigen::Vector3f p1 = keyframe1.cloud[nn.keyframe_1_point_id].getArray3fMap();
                if (keyframe1.laser_id == 2) {
                    p1 =  getAngle( -keyframe1.cloud[nn.keyframe_1_point_id].normal_x)* p1;
                }
                Eigen::Vector3f p2 = transform_gt * groundtruth[nn.keyframe_2_point_id].getArray3fMap();
                ceres::LossFunction *loss = new ceres::CauchyLoss(imgui_nn/2);
                ceres::CostFunction *cost_function = costFunICPGT::Create(p1, p2, Sophus::SE3d(imu_offsetd.matrix()));

                problem.AddResidualBlock(cost_function, loss, (double *) keyframe1.pose_optimized.data(),
                                         (double *) laser_se3d[laser1_id].data());
            }

            if (!laser_to_apply[0]){
                problem.SetParameterBlockConstant((double*)laser_se3d[0].data());
            }
            if (!laser_to_apply[1]){
                problem.SetParameterBlockConstant((double*)laser_se3d[1].data());
            }
            if (!laser_to_apply[2]){
                problem.SetParameterBlockConstant((double*)laser_se3d[2].data());
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 25;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << "\n";
        }
    }

    ImGui::End();
    Eigen::Affine3f imu_offset(Eigen::Affine3d::Identity());
    imu_offset.rotate(Eigen::AngleAxisf(imgui_angle, Eigen::Vector3f::UnitZ()));
    if (imgui_draw_gt) {
        glBegin(GL_POINTS);
        glColor3f(0.5f, 0.5f, 0.5f);
        Eigen::Matrix4f transform = gt_se3.matrix().cast<float>();
        for (int i = 0; i < groundtruth.size(); i++) {
            Eigen::Vector4f p = groundtruth[i].getArray4fMap();
            Eigen::Vector4f pt = transform * p;
            if (imgui_draw_half && pt.x() > 0)continue;
            glVertex3f(pt.x(), pt.y(), pt.z());
        }
        glEnd();
    }
    if(imgui_draw_nn)
    {
        glBegin(GL_LINES);
        glColor3f(1.0f, 1.0f, 1.0f);

        Eigen::Matrix4f transform_gt = gt_se3.matrix().cast<float>();
        for (const auto&nn : nearest_gtnn) {
            const scan &keyframe1 = keyframes[nn.keyframe_1_id];
            const int laser1_id = keyframe1.laser_id;

            Eigen::Affine3f  pose1 =Eigen::Affine3f(keyframe1.pose_optimized.matrix().cast<float>()) * imu_offset.inverse();
            Eigen::Affine3f laser1_co {laser_se3d[laser1_id].matrix().cast<float>()};

            Eigen::Vector4f p1 = keyframe1.cloud[nn.keyframe_1_point_id].getArray4fMap();
            if (keyframe1.laser_id == 2) {
                p1 =  getAngle( -keyframe1.cloud[nn.keyframe_1_point_id].normal_x)* p1;
            }

            Eigen::Vector4f p2 = groundtruth[nn.keyframe_2_point_id].getArray4fMap();
            Eigen::Vector4f pt1 = pose1 * laser1_co * p1;
            Eigen::Vector4f pt2 = transform_gt * p2;
            glVertex3f(pt1.x(), pt1.y(), pt1.z());
            glVertex3f(pt2.x(), pt2.y(), pt2.z());
        }
        glEnd();

    }
    {
        glBegin(GL_POINTS);
        for (int keyframe_id = 0; keyframe_id < keyframes.size(); keyframe_id++) {
            const auto laser_id = keyframes[keyframe_id].laser_id;
            if (!imgui_draw_lasers[laser_id]) continue;
            //std::cout << "laser_id " << laser_id << std::endl;
            Eigen::Affine3f laser_co {laser_se3d.at(laser_id).matrix().cast<float>()};
            glColor3f(1,0,0);

            const scan &keyframe = keyframes[keyframe_id];
            if ( keyframe.timestamp > imgui_start &&  keyframe.timestamp < imgui_start + imgui_end) {
                Eigen::Affine3f  pose =Eigen::Affine3f(keyframe.pose_optimized.matrix().cast<float>()) * imu_offset.inverse();
                for (int i = 0; i < keyframe.cloud.size(); i++) {
                    Eigen::Vector3f p = keyframe.cloud[i].getArray3fMap();
                    Eigen::Vector3f pt;
                    float in = keyframe.cloud[i].intensity;
                    if (laser_id == 0 ) {
                        glColor3f(1.0f, 0.0f, 0.0f);
                    }else if (laser_id == 1 ) {
                        glColor3f(0.0f, 1.0f , 0.0f);
                    }else if (laser_id == 2 ) {
                        glColor3f(0.0f, 0.0f, 1.0f);
                    }
                    if (laser_id == 2) {
                        pt = pose * laser_co * getAngle( -keyframe.cloud[i].normal_x)* p;
                    }else{
                        pt = pose * laser_co * p;
                    }
                    if (imgui_draw_half && pt.x() > 0)continue;
                    glVertex3f(pt.x(), pt.y(), pt.z());
                }
            }
        }
        glEnd();

    }

    std::stringstream  ss0; ss0 << laser_se3d[0].log().matrix().transpose();
    std::stringstream  ss1; ss1 << laser_se3d[1].log().matrix().transpose();
    std::stringstream  ss2; ss2 << laser_se3d[2].log().matrix().transpose();

    ImGui::Text("L0 : %s", ss0.str().c_str());
    ImGui::Text("L1 : %s", ss1.str().c_str());
    ImGui::Text("L2 : %s", ss2.str().c_str());

    glEnd();
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glutSwapBuffers();
    glutPostRedisplay();

}

void mouse(int glut_button, int state, int x, int y) {
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);
    int button = -1;
    if (glut_button == GLUT_LEFT_BUTTON) button = 0;
    if (glut_button == GLUT_RIGHT_BUTTON) button = 1;
    if (glut_button == GLUT_MIDDLE_BUTTON) button = 2;
    if (button != -1 && state == GLUT_DOWN)
        io.MouseDown[button] = true;
    if (button != -1 && state == GLUT_UP)
        io.MouseDown[button] = false;

    if (!io.WantCaptureMouse)
    {
        if (state == GLUT_DOWN) {
            mouse_buttons |= 1 << glut_button;
        } else if (state == GLUT_UP) {
            mouse_buttons = 0;
        }
        mouse_old_x = x;
        mouse_old_y = y;
    }
}

void motion(int x, int y) {
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    if (!io.WantCaptureMouse)
    {
        float dx, dy;
        dx = (float) (x - mouse_old_x);
        dy = (float) (y - mouse_old_y);
        gui_mouse_down = mouse_buttons>0;
        if (mouse_buttons & 1) {
            rotate_x += dy * 0.2f;
            rotate_y += dx * 0.2f;
        } else if (mouse_buttons & 4) {
            translate_z += dy * 0.05f;
        } else if (mouse_buttons & 3) {
            translate_x += dx * 0.05f;
            translate_y -= dy * 0.05f;
        }
        mouse_old_x = x;
        mouse_old_y = y;
    }
    glutPostRedisplay();
}

void reshape(int w, int h) {
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat) w / (GLfloat) h, 0.01, 10000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

bool initGL(int *argc, char **argv) {
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow("perspective_camera_ba");
    glutDisplayFunc(display);
    glutMotionFunc(motion);

    // default initialization
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);

    // viewport
    glViewport(0, 0, window_width, window_height);

    // projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat) window_width / (GLfloat) window_height, 0.01,
                   10000.0);
    glutReshapeFunc(reshape);
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();
    return true;
}