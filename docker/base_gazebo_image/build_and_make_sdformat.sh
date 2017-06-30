#!/bin/bash

# DEPENDECIES
SDFORMAT_BASE_DEPENDENCIES="libboost-system-dev          \\
                            libboost-filesystem-dev      \\
                            libboost-program-options-dev \\
                            libboost-regex-dev           \\
                            libboost-iostreams-dev       \\
                            libtinyxml-dev               \\
                            libxml2-utils                \\
                            ruby-dev                     \\
                            ruby                         \\
                            libignition-math2-dev"
SDFORMAT_BASE_DEPENDENCIES=$(sed 's:\\ ::g' <<< $SDFORMAT_BASE_DEPENDENCIES)

apt-get -y install ${SDFORMAT_BASE_DEPENDENCIES}

# DOWNLOAD SDFORMAT
cd /tmp
tar -xaf /tmp/sdformat-3.1.1.tar.bz2
cd /tmp/sdformat-3.1.1

# BUILD SDFORMAT
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE="Release" \
         -DCMAKE_INSTALL_PREFIX=/usr \
         -DCMAKE_INSTALL_LIBDIR=lib
make

# INSTALL SDFORMAT
make install

rm -r /tmp/sdformat-3.1.1
