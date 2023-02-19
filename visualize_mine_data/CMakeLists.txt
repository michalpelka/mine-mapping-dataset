cmake_minimum_required(VERSION 3.0.2)
project(visualize_mine_data)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    link_libraries(${GLEW_LIBRARIES})
endif()

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS}
)

include_directories(3rd/Sophus)
include_directories(3rd/imgui 3rd/imgui/backends)
set(ImGui_Glut_src 3rd/imgui/imgui.cpp 3rd/imgui/imgui_demo.cpp 3rd/imgui/imgui_draw.cpp 3rd/imgui/imgui_widgets.cpp
        3rd/imgui/backends/imgui_impl_glut.cpp 3rd/imgui/backends/imgui_impl_opengl2.cpp)


include_directories(GL 3rd 3rd/imgui 3rd/imgui/backends)
add_library(GL_Codes STATIC
        # wrapers
        GL/IndexBuffer.cpp
        GL/Renderer.cpp
        GL/Texture.cpp
        GL/VertexBuffer.cpp
        GL/Shader.cpp
        GL/VertexArray.cpp
        GL/VertexBufferLayout.cpp
        # imgui
        3rd/imgui/imgui.cpp 3rd/imgui/imgui_demo.cpp 3rd/imgui/imgui.cpp
        3rd/imgui/imgui_draw.cpp 3rd/imgui/imgui_widgets.cpp
        3rd/imgui/backends/imgui_impl_opengl3.cpp 3rd/imgui/backends/imgui_impl_glfw.cpp
        )


catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


find_package(fmt)
add_executable(visualize_mine_data src/visualize_mine_data.cpp)
target_link_libraries(visualize_mine_data
         ${GLEW_LIBRARIES} ${CERES_LIBRARIES} ${OPENGL_LIBRARIES} glfw  fmt::fmt ${catkin_LIBRARIES} )
target_link_libraries(visualize_mine_data GL_Codes)

