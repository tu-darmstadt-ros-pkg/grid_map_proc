
file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

catch_add_test(
  ${PROJECT_NAME}_tests
  ${SOURCES}
)
target_link_libraries(
  ${PROJECT_NAME}_tests
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
