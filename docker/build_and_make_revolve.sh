#!/bin/bash
set -e

apt-get -y install git

# DOWNLOAD REVOLVE
cd /revolve

git clone https://github.com/ElteHupkes/revolve.git
cd revolve

#Disable `gz` target, it won't compile for some weird reason
echo >/revolve/revolve/setup.py "#!/usr/bin/env python
from distutils.core import setup

setup(name='revolve',
      version=0.2,
      description='Revolve: robot evolution framework',
      author='Elte Hupkes',
      author_email='elte@hupkes.org',
      url='https://github.com/ElteHupkes/revolve',
      packages=['revolve',
                'revolve.gazebo',
                'revolve.gazebo.manage',
                'revolve.build',
                'revolve.build.sdf',
                'revolve.build.sdf.body',
                'revolve.build.sdf.neural_net',
                'revolve.convert',
                'revolve.generate',
                'revolve.spec',
                'revolve.spec.msgs',
                'revolve.angle',
                'revolve.angle.manage',
                'revolve.angle.robogen',
                'revolve.angle.robogen.spec',
                'revolve.angle.robogen.body_parts',
                'revolve.util',
                'revolve.util.supervisor',
                'revolve.logging'],
      install_requires=['PyYAML', 'pygazebo', 'protobuf', 'sdfbuilder', 'numpy', 'psutil']
      )
"

# BUILD GAZEBO
mkdir build && cd build
cmake ../cpp -DCMAKE_BUILD_TYPE="Release" \
             -DLOCAL_BUILD_DIR=1
make -j4

# INSTALL GAZEBO
make install

apt-get -y purge git
