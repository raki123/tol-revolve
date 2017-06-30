cmake_minimum_required(VERSION 2.8)

set(build_dir ${CMAKE_CURRENT_LIST_DIR}/../build_docker)
set(revolve_brain_dir /revolve/revolve-brain)

if(NOT EXISTS ${build_dir})
  file(MAKE_DIRECTORY ${build_dir})
endif()

execute_process(
  COMMAND git pull
  WORKING_DIRECTORY ${revolve_brain_dir}
  RESULT_VARIABLE git_update_result
)

if(${cmake_result})
  message(WARNING "git pull failed")
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} ../cpp -Dtest=ON -DREVOLVE_BUILD_PATH=/revolve/revolve/build -DREVOLVE_BRAIN_PATH=${revolve_brain_dir}/cpp

  WORKING_DIRECTORY ${build_dir}
  RESULT_VARIABLE cmake_result
)

if(${cmake_result})
  message(FATAL_ERROR "cmake failed")
endif()

execute_process(
  COMMAND make
  WORKING_DIRECTORY ${build_dir}
  RESULT_VARIABLE compilation_result
)

if(${compilation_result})
  message(FATAL_ERROR "compilation failed")
endif()

execute_process(
  COMMAND make test
  WORKING_DIRECTORY ${build_dir}
  RESULT_VARIABLE test_result
)

if(${test_result})
  message(FATAL_ERROR "test failed")
endif()
