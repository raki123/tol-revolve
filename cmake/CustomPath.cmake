# Quick custompath to include the Revolve CPP header, I'm going
# to figure out a better setup later.
# TODO Change for more flexibility later
# TODO Windows support if required
set(TOL_DEP_DIR "${CMAKE_SOURCE_DIR}/../..")
message("Dependency base directory: ${TOL_DEP_DIR}")

set(REVOLVE_INCLUDE_PATH "${TOL_DEP_DIR}/revolve/cpp")
set(REVOLVE_LIBRARIES "${TOL_DEP_DIR}/revolve/build/librevolve-gazebo.a")
