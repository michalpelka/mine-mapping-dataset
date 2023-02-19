#include "../GL/glwrapper.h"
#include "utils.h"
#include <memory>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <boost/program_options.hpp>

#include <sophus/se3.hpp>
glm::vec2 clicked_point;
float rot_x =0.0f;
float rot_y =0.0f;
bool drawing_buffer_dirty = true;
glm::vec3 view_translation{ 0,0,-30 };

void cursor_calback(GLFWwindow* window, double xpos, double ypos)
{
    ImGuiIO& io = ImGui::GetIO();
    if(!io.WantCaptureMouse) {
        const glm::vec2 p{-xpos, ypos};
        const auto d = clicked_point - p;
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) {
            rot_x += 0.01 * d[1];
            rot_y += 0.01 * d[0];
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2) == GLFW_PRESS) {
            view_translation[2] += 0.02 * d[1];
        }
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_3) == GLFW_PRESS) {
            view_translation[1] += 0.01 * d[1];
            view_translation[0] -= 0.01 * d[0];
        }
        clicked_point = p;
    }
}


void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}
struct KeyFrame{

    KeyFrame(std::shared_ptr<float[]> data_p, int len, Eigen::Matrix4d mat, int laser_id):
        mat(mat),data(data_p), len(len), laser_id(laser_id),
        vb(data.get(), len* sizeof(float)),
        va()
    {
        timestamp = data[4];
        VertexBufferLayout layoutPc;
        layoutPc.Push<float>(3);
        layoutPc.Push<float>(1); // angle
        layoutPc.Push<float>(1); // ts
        layoutPc.Push<float>(1); // intensity
        va.AddBuffer(vb, layoutPc);
    }

    Eigen::Matrix4d mat;
    double timestamp;
    const std::shared_ptr<float[]> data;
    const int len;
    const int laser_id;
    VertexBuffer vb;
    VertexArray va;
};

std::map<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr> loadCloudsFromBag(
        const std::vector<std::string>& bag_files,
        const std::string & laser_id){
    std::map<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr> laser_clouds;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr aggregate(
            new pcl::PointCloud<pcl::PointXYZINormal>);
    for (const auto &bag_fn: bag_files) {
        rosbag::Bag bag;
        bag.open(bag_fn, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery({laser_id}));
        for (rosbag::MessageInstance const m: view) {
            const auto s = m.instantiate<sensor_msgs::PointCloud2>();
            if (!s)continue;
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*s, pcl_pc2);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr tempCloud(
                    new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::fromPCLPointCloud2(pcl_pc2, *tempCloud);
            if (tempCloud->empty())continue;
            *aggregate+=*tempCloud;
            if (aggregate->back().normal_y - aggregate->front().normal_y > 0.1)
            {
                double avg = (aggregate->back().normal_y + aggregate->front().normal_y)/2.0;
                laser_clouds[avg] = aggregate;
                aggregate = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
            }
        }
    }
    return laser_clouds;
}

std::shared_ptr<float[]> pclToBuffer(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, int &len, float scale)
{
    const int stride = 6;
    len = stride*cloud->size();
    std::shared_ptr<float[]> data (new float[len]);
    for (int i = 0; i < cloud->size();i++){
        data[stride*i+0] = scale*(*cloud)[i].x;
        data[stride*i+1] = scale*(*cloud)[i].y;
        data[stride*i+2] = scale*(*cloud)[i].z;
        data[stride*i+3] = (*cloud)[i].normal_x;
        data[stride*i+4] = (*cloud)[i].normal_y;
        data[stride*i+5] = (*cloud)[i].intensity;
    }
    return data;
}

int main(int argc, char **argv) {

    namespace bpo=boost::program_options;
    bpo::variables_map vm;
    try
    {
        bpo::options_description desc{"Options"};
        desc.add_options()
                ("gt", bpo::value<std::string>(), "Ground truth")
                ("bags", bpo::value<std::vector<std::string>>(), "Bags files")
                ("csv", bpo::value<std::vector<std::string>>(), "Corresponding trajectory files");
        store(parse_command_line(argc, argv, desc), vm);
        notify(vm);
    }
    catch (const boost::program_options::error &ex)
    {
        std::cerr << ex.what() << '\n';
        return 1;
    }

    const std::vector<std::string> bag_files = vm["bags"].as<std::vector<std::string>>();
    const std::vector<std::string> fns_trj = vm["csv"].as<std::vector<std::string>>();

    using CACHE_TYPE_MAP=std::map<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>;
    CACHE_TYPE_MAP livox_clouds = loadCloudsFromBag(bag_files, TOPIC_LIVOX);
    CACHE_TYPE_MAP vlp32_clouds = loadCloudsFromBag(bag_files, TOPIC_VELO_STATIC);
    CACHE_TYPE_MAP vlp16_clouds = loadCloudsFromBag(bag_files, TOPIC_VELO_ROT);

    float imgui_depth {250};
    float imgui_start {0};
    float imgui_len {1};

    std::array<bool,3> imgui_laser_vis{true,true,true};
    std::array<Sophus::Vector6f,4> imgui_corr{Sophus::Vector6f::Zero(),Sophus::Vector6f::Zero(),Sophus::Vector6f::Zero(),Sophus::Vector6f::Zero()};

    GLFWwindow *window;
    const char *glsl_version = "#version 130";
    if (!glfwInit())
        return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window = glfwCreateWindow(960, 540, "rgbd_demo", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, cursor_calback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glfwSwapInterval(1);
    if (glewInit() != GLEW_OK) { return -1; }

    GLCall(glClearColor(0.4, 0.4, 0.4, 1));

    Renderer renderer;


    VertexBufferLayout layout;
    layout.Push<float>(3);
    layout.Push<float>(3);


    VertexArray va;
    VertexBuffer vb(gl_primitives::coordinate_system_vertex.data(),
                    gl_primitives::coordinate_system_vertex.size() * sizeof(float));
    va.AddBuffer(vb, layout);
    IndexBuffer ib(gl_primitives::coordinate_system_indices.data(), gl_primitives::coordinate_system_indices.size());

    Shader shader(shader_simple_v, shader_simple_f);
    Shader shader_pc(shader_pc_intensity_v, shader_pc_intensity_f);
    Shader shader_pc_head(shader_pc_intensity_head_v, shader_pc_intensity_f);



    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, false);
    ImGui_ImplOpenGL3_Init(glsl_version);


    std::vector<std::pair<double, Eigen::Matrix4d>> trajectory;
    std::vector<std::unique_ptr<KeyFrame>> keyframes;

    // parse csv

    for (auto fn : fns_trj){
        std::fstream infile(fn);
        std::string line;
        std::getline(infile, line);
        while (std::getline(infile, line)){
            auto r = my_utils::loadLineCsv (line);
            trajectory.emplace_back(r);
        }
    }


    for (const auto &t : trajectory) {

        auto it = livox_clouds.lower_bound(t.first);
        if (it != livox_clouds.end()){
            if (it->second)
            {
                double err = (it->first -  t.first);
                //std::cout << "\tlivox cloud with err " << err << std::endl;
                int len = 0;
                std::shared_ptr<float[]> data = pclToBuffer(it->second, len, 1.0f);
                keyframes.emplace_back(std::make_unique<KeyFrame>(data, len,  t.second,0));
                it->second = nullptr;
            }
        }
    }
    for (const auto &t : trajectory) {

        auto it = vlp32_clouds.lower_bound(t.first);
        if (it != vlp32_clouds.end()){
            if (it->second)
            {
                double err = (it->first -  t.first);
                //std::cout << "\tVLP32 cloud with err " << err << std::endl;
                int len = 0;
                std::shared_ptr<float[]> data = pclToBuffer(it->second, len, 2.0f);
                keyframes.emplace_back(std::make_unique<KeyFrame>(data, len,  t.second,1));
                it->second = nullptr;
            }
        }
    }
    for (const auto &t : trajectory) {

        auto it = vlp16_clouds.lower_bound(t.first);
        if (it != vlp16_clouds.end()){
            if (it->second)
            {
                double err = (it->first -  t.first);
                //std::cout << "\tVLP16 cloud with err " << err << std::endl;
                int len = 0;
                std::shared_ptr<float[]> data = pclToBuffer(it->second, len, 1.0f);
                keyframes.emplace_back(std::make_unique<KeyFrame>(data, len,  t.second,2));
                it->second = nullptr;
            }
        }
    }




    std::vector<Eigen::Matrix4d> mats;
#define INITAL_CAD
#ifdef INITAL_CAD
    Eigen::Matrix4f vlp16_calib;
    vlp16_calib <<0.999921, -0.00638093,   0.0107958, -0.00731472,
            0.00634553,    0.999974,  0.00331037,   0.0463718,
            -0.0108166, -0.00324161,    0.999936,   0.0222408,
            0,           0,           0 ,          1;


    const double angular_encoder_offset =  -5.53747-0.98*M_PI;
    mats.push_back(Eigen::Matrix4d::Identity());
    Eigen::Matrix4d vlp32c;
    vlp32c << 0.0000,  0.6428,  0.7660,  0.2061,
			0.7049, -0.5433,  0.4559,  0.0325,
			0.7093,  0.5400, -0.4531, -0.1567,
			0.0000,  0.0000,  0.0000,  1.0000;
    Eigen::Matrix4d vlp16;
    vlp16 << 0.0000,  0.0000,  1.0000, -0.0678,
            -0.0000,  1.0000, -0.0000, -0.1292,
            -1.0000, -0.0000,  0.0000, -0.0584,
            0.0000,  0.0000, 0.0000,  1.0000;
    mats.push_back(vlp32c);
    mats.push_back(vlp16);

#endif
#ifdef IMPROVED_CALIBRATION
    Eigen::Matrix4f vlp16_calib;
    vlp16_calib <<0.999921, -0.00638093,   0.0107958, -0.00731472,
            0.00634553,    0.999974,  0.00331037,   0.0463718,
            -0.0108166, -0.00324161,    0.999936,   0.0222408,
            0,           0,           0 ,          1;

    const double angular_encoder_offset =  -5.53747-0.98*M_PI;
    std::vector< Eigen::Affine3d> laser_calib_phd;
    laser_calib_phd.resize(3);


    Sophus::Vector6d l1;
    Sophus::Vector6d l2;
    Sophus::Vector6d l3;

    l1 << 0.0772398, -0.139824, 0.0671538, 2.22144, 0, -2.22144;
    l2 << 0.492684, 0.304044, 0.264678, -1.0054, 2.46172, -0.854861;
    l3 << 0.182153, -0.0338997, -0.294753, 3.12548, 0.24284, -0.0213847;
    laser_calib_phd[0] = Sophus::SE3d::exp(l1).matrix();
    laser_calib_phd[1] = Sophus::SE3d::exp(l2).matrix();
    laser_calib_phd[2] = Sophus::SE3d::exp(l3).matrix();


    mats.push_back(Eigen::Matrix4d::Identity());
    mats.push_back((laser_calib_phd[0].inverse()*laser_calib_phd[1]).matrix());
    mats.push_back((laser_calib_phd[0].inverse()*laser_calib_phd[2]).matrix());
#endif


    while (!glfwWindowShouldClose(window)) {
        GLCall(glEnable(GL_DEPTH_TEST));
        GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiIO& io = ImGui::GetIO();
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        glm::mat4 proj = glm::perspective(30.f, 1.0f*width/height, 0.05f, 100.0f);
        glm::mat4 scale =  glm::scale(glm::mat4(1.0f), glm::vec3(0.1f, 0.1f, 0.1f));
        glm::mat4 model_translate = glm::translate(glm::mat4(1.0f), view_translation);
        glm::mat4 model_rotation_1 = glm::rotate(model_translate, rot_x, glm::vec3(1.0f, 0.0f, 0.0f));
        glm::mat4 model_rotation_2 = glm::rotate(model_rotation_1, rot_y, glm::vec3(0.0f, 0.0f, 1.0f));
        glm::mat4 model_rotation_3 = glm::rotate(model_rotation_2, (float)(0.5f*M_PI), glm::vec3(-1.0f, 0.0f, 0.0f));
        glm::mat4 model_rotation_4 = glm::translate(model_rotation_2,glm::vec3(0.0f, 0.0f, -imgui_depth));

        glm::mat4 scan2_cfg;
        shader.Bind(); // bind shader to apply uniform

        shader.setUniformMat4f("u_MVP", proj * model_rotation_2);
        renderer.Draw(va, ib, shader, GL_LINES);
        // draw reference frame
        for (const auto &k : trajectory) {
            glm::mat4 local;
            Eigen::Map<Eigen::Matrix4f> map_local(&local[0][0]);
            map_local = k.second.cast<float>();
            shader.setUniformMat4f("u_MVP", proj * model_rotation_4 * local * scale);
            renderer.Draw(va, ib, shader, GL_LINES);
        }
        for (const auto &k : keyframes){
            if (k->len != 0 && k->timestamp > imgui_start && k->timestamp < imgui_start+imgui_len ) {
                if (!imgui_laser_vis.at(k->laser_id)) continue;
                glm::mat4 local;
                glm::mat4 corr;
                Eigen::Map<Eigen::Matrix4f> map_corr(&corr[0][0]);
                Eigen::Map<Eigen::Matrix4f> map_local(&local[0][0]);
                map_local = k->mat.matrix().cast<float>();
                Sophus::SE3f gui_corr =  Sophus::SE3f::exp(imgui_corr[k->laser_id]);
                Sophus::SE3f gui_corr_local_head =  Sophus::SE3f::exp(imgui_corr.at(3));

                map_corr =  gui_corr.matrix() * mats[k->laser_id ].cast<float>();

                GLCall(glPointSize(1));

                if (k->laser_id == 0) {
                    shader_pc.Bind();
                    shader_pc.setUniformMat4f("u_MVPPC", proj * model_rotation_4 * local * corr);
                    shader_pc.setUniform4f("u_COLORPC", 1, 0, 0, 1);
                    renderer.DrawArray(k->va, shader_pc, GL_POINTS, k->len / 6);
                }else if (k->laser_id == 1) {
                    shader_pc.Bind();
                    shader_pc.setUniformMat4f("u_MVPPC", proj * model_rotation_4 * local * corr);
                    shader_pc.setUniform4f("u_COLORPC", 0, 1, 0, 1);
                    renderer.DrawArray(k->va, shader_pc, GL_POINTS, k->len / 6);
                }else if (k->laser_id == 2) {


                    Eigen::Affine3f rot0 (Eigen::Affine3f::Identity());
                    rot0.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()));
                    Eigen::Affine3f rot1 (Eigen::Affine3f::Identity());
                    rot1.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitX()));
                    Eigen::Affine3f staticTransform = rot1*rot0;

                    Eigen::Affine3f angularOffset (Eigen::Affine3f::Identity());
                    angularOffset.rotate(Eigen::AngleAxisf(angular_encoder_offset, Eigen::Vector3f::UnitZ()));
                    Eigen::Affine3f mc1 = Eigen::Affine3f (gui_corr.matrix() * mats[k->laser_id ].cast<float>())*angularOffset;
                    Eigen::Affine3f mc2 = staticTransform * Eigen::Affine3f(vlp16_calib);

                    map_corr = mc1.matrix();
                    glm::mat4 uniform_head_matrix;
                    Eigen::Map<Eigen::Matrix4f> map_uniform_head_matrix(&uniform_head_matrix[0][0]);
                    map_uniform_head_matrix = staticTransform * vlp16_calib * gui_corr_local_head.matrix() ;
                    shader_pc_head.Bind();
                    shader_pc_head.setUniformMat4f("u_MVPPC", proj * model_rotation_4 * local * corr );
                    shader_pc_head.setUniformMat4f("u_HEAD", uniform_head_matrix);
                    shader_pc_head.setUniform4f("u_COLORPC", 0, 0, 1, 1);
                    shader_pc_head.setUniform1f("u_AngOffset",0);
                    renderer.DrawArray(k->va, shader_pc_head, GL_POINTS, k->len / 6);
                }

            }
        }


        ImGui::Begin("Data View Demo");
        if(ImGui::Button("reset view")){
            rot_x =0.0f;
            rot_y =0.0f;
            view_translation = glm::vec3{ 0,0,-30 };
            view_translation = glm::vec3{ 0,0,-30 };
        }
        ImGui::SliderFloat("start", &imgui_start, trajectory.front().first, trajectory.back().first);
        ImGui::SliderFloat("len", &imgui_len, 0,120);

        ImGui::SliderFloat("depth", &imgui_depth, -20, 290);
        for (int i =0; i< imgui_laser_vis.size(); i++){
            ImGui::Checkbox(std::string("v"+std::to_string(i)).c_str(), &(imgui_laser_vis[i]));
        }

        for (int i =0; i< imgui_corr.size(); i++){
            for (int j =0; j< 6; j++) {
                ImGui::SliderFloat(("l" + std::to_string(i)+"-"+std::to_string(j)).c_str(), &(imgui_corr[i].data()[j]), -0.2,0.2);
            }
        }
        ImGui::End();

        ImGui::Render();

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
    return 0;

}