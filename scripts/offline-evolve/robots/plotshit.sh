#!/bin/bash
neato -Tpng -Goverlap=scale spider9_simple-$1-$2.dot > ind$2.png
xdg-open ind$2.png
