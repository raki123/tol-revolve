#!/bin/bash
set -e

BASE_DEPENDENCIES="build-essential \\
                   cmake           \\
                   cppcheck        \\
                   xsltproc        \\
                   python          \\
                   mercurial"

BASE_DEPENDENCIES=$(sed 's:\\ ::g' <<< $BASE_DEPENDENCIES)

apt-get update
apt-get -y install ${BASE_DEPENDENCIES}
apt-get -y install python
