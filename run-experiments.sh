#!/usr/bin/env bash

# Handle keyboard interrupt
trap " exit " INT

PROGNAME=$(basename $0)

# Default argument list
#robot_list=(spider9 spider13 spider17 gecko7 gecko12 gecko17 snake5 snake7 snake9 babyA babyB babyC)
#robot_list=(babyA babyB babyC)
robot_list=(babyAC)

config=./res/rlpower_spline.cfg
gz_command=gzserver
load_controller=None
manager=./experiments/single_robot_manager.py
no_experiments=10
output=output
restore=restore
world=./res/worlds/gait-learning.world

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
