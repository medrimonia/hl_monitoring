cmake_minimum_required(VERSION 2.8)
project(hl_monitoring)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  hl_communication
  )

# Handling generated headers is based on Dirk Thomas answer on topic:
# - https://answers.ros.org/answers/123416/revisions/
catkin_destinations()
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set (PROTOBUF_MESSAGES
  proto/camera.proto
  )

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${PROTOBUF_MESSAGES})

set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS} PROPERTIES GENERATED TRUE)

add_custom_target(${PROJECT_NAME}_generate_headers ALL
  COMMAND cp ${PROTO_HEADERS} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  DEPENDS ${PROTO_HEADERS}
  )

set(${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_headers)

message(WARNING "Jsoncpp found: ${JSONCPP_FOUND}")
message(WARNING "Jsoncpp include dirs: ${JSONCPP_INCLUDE_DIRS}")
message(WARNING "Jsoncpp libraries: ${JSONCPP_LIBRARIES}")

set(DELEGATE_INCLUDE_DIRS
  include
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  ${OpenCV_DIR}
  ${JSONCPP_INCLUDE_DIRS})

set(DELEGATE_LIBRARIES
  ${OpenCV_LIBS}
  ${JSONCPP_LIBRARIES})

catkin_package(
  INCLUDE_DIRS ${DELEGATE_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${DELEGATE_LIBRARIES}
  )

include_directories(${DELEGATE_INCLUDE_DIRS} ${catkin_INCLUDR_DIRS} )

set(SOURCES_DIRECTORIES
  src/hl_monitoring
  )

#Include Sources sub sources
foreach (DIRECTORY ${SOURCES_DIRECTORIES})
  include (${DIRECTORY}/sources.cmake)
  set (PREFIXED_SOURCES)
  foreach (SOURCE ${SOURCES})
    set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
  endforeach (SOURCE)
  set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
endforeach (DIRECTORY)


set(LINKED_LIBRARIES ${catkin_LIBRARIES} ${DELEGATE_LIBRARIES})

#Build the kid_size main library
add_library (${PROJECT_NAME} SHARED ${PROTO_SOURCES} ${ALL_SOURCES})
target_link_libraries(${PROJECT_NAME} ${LINKED_LIBRARIES})


option(BUILD_HL_MONITORING_TOOLS "Building hl_monitoring tools" OFF)

if (BUILD_HL_MONITORING_TOOLS)
  add_executable(intrinsic_calibration tools/intrinsic_calibration.cpp)
  target_link_libraries(intrinsic_calibration ${PROJECT_NAME} ${LINKED_LIBRARIES})

  add_executable(static_calibration tools/static_calibration.cpp)
  target_link_libraries(static_calibration ${PROJECT_NAME} ${LINKED_LIBRARIES})
endif()
