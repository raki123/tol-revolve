#!/usr/bin/env bash

# Handle keyboard interrupt
trap " exit " INT

PROGNAME=$(basename $0)

# Default argument list
robot_list=( spider9 spider13 spider17 gecko7 gecko12 gecko17 snake5 snake7 snake9 babyA babyB babyC )

manager=single_robot_manager.py
world=gait-learning.world
output=output
restore=restore
gz_command=gzserver
no_experiments=10

function error_exit() {
    echo "${PROGNAME}: ${1:-"Unknown Error"}" 1 >&2
    exit 1
}

function help() {
    echo "Usage: ${PROGNAME} [args...]"
    echo "Arguments:"
    echo " -g | --gzcommand       Gazebo command [gzserver|gazebo] Default: gzserver"
    echo " -h | --help            Help page"
    echo " -m | --manager         Name of script that controls robots and the environment"
    echo " -n | --no-experiments  Number of experiment repetitions"
    echo " -o | --output          Name of the output directory"
    echo " -r | --restore         Name of the restore directory"
    echo " -w | --world           Name of world file"

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
                -g | --gzcommand) local gz_command=${parameter} ;;
                -h | --help) help ;;
                -m | --manager) local manager=${parameter} ;;
                -n | --no-experiments) local no_experiments=${parameter} ;;
                -o | --output) local output=${parameter} ;;
                -r | --restore) local restore=${parameter} ;;
                -w | --world) local world=${parameter} ;;
                *) error_exit "${LINENO}: In ${FUNCNAME}() unknown argument ${argument}." ;;
            esac
        done
    fi

    # For each name in 'robot_list' run 'no_experiments' experiments
    for index in ${!robot_list[*]}
    do
        for (( i = 1; i <= ${no_experiments}; ++ i ))
        do
            python start.py \
                --manager ${manager} \
                --world ${world} \
                --output ${output} \
                --restore ${restore} \
                --robot-name robots/${robot_list[$index]} \
                --experiment-round ${i} \
                --gazebo-cmd ${gz_command}
        done
    done

    exit 0
}

main $@
