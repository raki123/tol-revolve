#!/usr/bin/env bash

MGR=single_robot_manager.py
WORLD=gait-learning.world
OUT=output
RST=restore
GZCMD=gzserver

RNAME=spider
RNUMBER=9

RND=1
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RNAME=spider
RNUMBER=13

RND=1
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RNAME=spider
RNUMBER=17

RND=1
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

RND=$(( RND + 1 ))
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}${RNUMBER} --experiment-round ${RND} --gazebo-cmd ${GZCMD}

