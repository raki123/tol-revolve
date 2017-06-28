#!/bin/bash
set -e

apt-get -y install git python-pip libgsl-dev libgsl2 libboost-python-dev libyaml-cpp-dev

# DOWNLOAD REVOLVE
cd /revolve
git clone https://github.com/portaloffreedom/revolve-brain.git
cd /revolve/revolve-brain
git checkout light-supg

cd /revolve
git clone https://github.com/portaloffreedom/tol-revolve.git
cd /revolve/tol-revolve
git checkout light-supg


# BUILD GAZEBO
mkdir build && cd build
cmake ../cpp -DCMAKE_BUILD_TYPE="Release" \
             -DREVOLVE_BUILD_PATH="`pwd`/../../revolve/build"
make -j4

# pip install
cd /revolve/tol-revolve
pip install -r requirements.txt

cd /revolve/tol-revolve/src/sdfbuilder/sdfbuilder
echo '--- element.py  2016-09-14 15:52:50.446557816 +0000
+++ element.py-fixed     2016-09-14 15:48:33.436156401 +0000
@@ -127,6 +127,8 @@
         method.
         :return:
         """
+        if self.attributes is None:
+            return {}
         return self.attributes.copy()

     def render_elements(self):
'| patch

# clean
apt-get -y purge git python-pip libgsl-dev
