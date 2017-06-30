cmake_minimum_required(VERSION 2.8)

set(build_dir ${CMAKE_CURRENT_LIST_DIR}/../build_docker)
set(revolve_brain_dir /revolve/revolve-brain)

execute_process(
  COMMAND make test
  WORKING_DIRECTORY ${build_dir}
  RESULT_VARIABLE test_result
)

if(${test_result})
  message(FATAL_ERROR "test failed")
endif()
