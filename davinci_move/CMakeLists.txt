cmake_minimum_required(VERSION 2.8.3)
project(davinci_move)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(davinci_cart_move_as src/davinci_cart_move_as.cpp)
cs_add_executable(davinci_cart_move_client_example src/davinci_cart_move_client_example.cpp)
cs_add_executable(davinci_cart_move_client_svc src/davinci_cart_move_client_svc.cpp)
target_link_libraries(davinci_cart_move_as davinci_kinematics ${catkin_LIBRARIES})
target_link_libraries(davinci_cart_move_client_example ${catkin_LIBRARIES})
target_link_libraries(davinci_cart_move_client_svc ${catkin_LIBRARIES})

cs_add_executable(davinci_cart_move_as2 src/davinci_cart_move_as2.cpp)
cs_add_executable(davinci_cart_move_client_example2 src/davinci_cart_move_client_example2.cpp)

target_link_libraries(davinci_cart_move_as2 davinci_kinematics ${catkin_LIBRARIES})
target_link_libraries(davinci_cart_move_client_example2 ${catkin_LIBRARIES})

cs_add_executable(davinci_cart_move_as3 src/davinci_cart_move_as3.cpp)
cs_add_executable(davinci_cart_move_client_example3 src/davinci_cart_move_client_example3.cpp)

target_link_libraries(davinci_cart_move_as3 davinci_kinematics ${catkin_LIBRARIES})
target_link_libraries(davinci_cart_move_client_example3 ${catkin_LIBRARIES})

cs_add_executable(davinci_cart_move_as4 src/davinci_cart_move_as4.cpp)
cs_add_executable(davinci_cart_move_client_example4 src/davinci_cart_move_client_example4.cpp)

target_link_libraries(davinci_cart_move_as4 davinci_kinematics ${catkin_LIBRARIES})
target_link_libraries(davinci_cart_move_client_example4 ${catkin_LIBRARIES})


cs_install()
cs_export()
    
