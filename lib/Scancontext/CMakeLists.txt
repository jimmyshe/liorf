ament_auto_add_library(scan_context Scancontext.cpp)
target_include_directories(scan_context PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(scan_context Eigen3::Eigen ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
