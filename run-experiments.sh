#!/usr/bin/env bash

# Handle keyboard interrupt
trap " exit " INT

PROGNAME=$(basename $0)

# Default argument list
robot_list=(robot_1 robot_2 robot_3 robot_4 robot_5 robot_6 robot_7 robot_8 robot_9 robot_10 robot_11 robot_12 robot_13 robot_14 robot_15 robot_16 robot_17 robot_18 robot_19 robot_20 robot_21 robot_22 robot_23 robot_24 robot_25 robot_26 robot_27 robot_28 robot_29 robot_30 robot_31 robot_32 robot_33 robot_34 robot_35 robot_36 robot_37 robot_38 robot_39 robot_40)

config=./res/rlpower_spline.cfg
gz_command=gzserver
load_controller=None
manager=./experiments/single_robot_manager.py
no_experiments=3
output=output
restore=restore
world=./res/worlds/gait-learning.world
REVOLVE_HOME='' #'/home/matteo/projects/revolve'

function recompile() {
  cur_dir=$PWD
  cd "${REVOLVE_HOME}/revolve/build"
  echo '-> BUILDING REVOLVE'
  make
  cd "${REVOLVE_HOME}/tol-revolve/build"
  echo '-> BUILDING TOL-REVOLVE'
  make
  cd ${cur_dir}
}


function error_exit() {
    echo "${PROGNAME}: ${1:-"Unknown Error"}" 1 >&2
    exit 1
}

function help() {
    echo "Usage: ${PROGNAME} [args...]"
    echo "Arguments:"
    echo " -c | --config          Path to a robot brain config file"
    echo " -g | --gzcommand       Gazebo command [gzserver|gazebo] Default: gzserver"
    echo " -h | --help            Help page"
    echo " -l | --load            Path to a robot controller file"
    echo " -m | --manager         Name of a script that controls robots and the environment"
    echo " -n | --no-experiments  Number of experiment repetitions"
    echo " -o | --output          Name of a output directory"
    echo " -r | --restore         Name of a restore directory"
    echo " -w | --world           Name of a world file"
    echo " --recompile            Path to the project folder containing revolve and tol-revolve projects"

    exit 0
}

function main() {

    # Read out the argument list
    if [ $# -gt 0 ]; then
        while [ "$1" != "" ]
        do
            local argument="$1";
            shift
            local parameter="$1";
            shift
            case ${argument} in
                -c | --config)
                    local config=${parameter}
                    if [ ! -s ${config} ]; then error_exit "Configuration file '${config}' does not exist."; fi
                    ;;
                -g | --gzcommand) local gz_command=${parameter} ;;
                -h | --help) help ;;
                -l | --load)
                    local load_controller=${parameter}
                    if [ ! -s ${load_controller} ]; then error_exit "Controller file '${load_controller}' does not exist."; fi
                    ;;
                -m | --manager)
                    local manager=${parameter}
                    if [ ! -s ${manager} ]; then error_exit "Manager file '${manager}' does not exist."; fi
                    ;;
                -n | --no-experiments) local no_experiments=${parameter} ;;
                -o | --output) local output=${parameter} ;;
                -r | --restore) local restore=${parameter} ;;
                -w | --world)
                    local world=${parameter}
                    if [ ! -s ${world} ]; then error_exit "World file '${world}' does not exist."; fi
                    ;;
                --recompile)
                    REVOLVE_HOME=${parameter}
                    recompile
                    ;;
                *) error_exit "${LINENO}: In ${FUNCNAME}() unknown argument ${argument}." ;;
            esac
        done
    fi

    # For each name in 'robot_list' run 'no_experiments' experiment
    for index in ${!robot_list[*]}
    do
        for (( i = 1; i <= ${no_experiments}; ++ i ))
        do
            python ./start.py \
                --load-controller ${load_controller} \
                --manager ${manager} \
                --world ${world} \
                --output ${output} \
                --restore ${restore} \
                --robot-name ./res/robots/${robot_list[$index]} \
                --experiment-round ${i} \
                --brain-conf-path ${config} \
                --gazebo-cmd ${gz_command}
        done
    done

    exit 0
}

main $@
