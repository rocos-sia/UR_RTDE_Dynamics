cmake_minimum_required(VERSION 3.5)
project(ur_rtde_examples)

# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

set(ur_rtde_DIR "/usr/local/lib/cmake/ur_rtde/") # 设置RTDE第一搜索路径，找不到在搜索第二路径
find_package(ur_rtde REQUIRED)
find_package(Boost  REQUIRED COMPONENTS program_options)

# ===============  官方的例子放此位置  =========================#
set(List_official
    forcemode_example
    move_until_contact
    io_example
    move_path_async_example
    movej_path_with_blend_example
    servoj_example
    speedj_example
    move_async_example
    robotiq_gripper_example
    record_data_example
    )
    
# ========================================#

add_subdirectory(3_party_library/filter_iir)


# ===============  自己的例子放此位置  =========================#

set(List_own moving_fluid my_ur_friction  my_ur_gravity my_ur_gravity_friction_test)
# ========================================#

set(LIST ${List_official} ${List_own})

include_directories(3_party_library/plog 3_party_library/track_Ik/include
                    3_party_library/kdl/include  ${CMAKE_CURRENT_SOURCE_DIR})

link_directories(3_party_library/track_Ik/lib 3_party_library/kdl/lib)

foreach(list ${LIST})

  add_executable(${list} ${list}.cpp  interpolate.cc )
  target_link_libraries(${list} PUBLIC ur_rtde::rtde orocos-kdl.so.1 trac_ik  Boost::program_options )

endforeach()
