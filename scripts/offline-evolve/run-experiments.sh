#!/usr/bin/env bash

MGR=single_robot_manager.py
WORLD=gait-learning.world
OUT=output
RST=restore

RNAME=spider
python start.py --manager ${MGR} --world ${WORLD} --output ${OUT} --restore ${RST} --robot-name robots/${RNAME}9 --experiment-round