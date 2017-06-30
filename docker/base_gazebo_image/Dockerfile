from ubuntu:xenial

RUN apt-get update && apt-get upgrade -y
COPY install_dependencies.sh /tmp/install_dependencies.sh
RUN /tmp/install_dependencies.sh

RUN apt-get install -y python-virtualenv

ADD http://osrf-distributions.s3.amazonaws.com/sdformat/releases/sdformat-3.1.1.tar.bz2 /tmp/
COPY build_and_make_sdformat.sh /tmp/build_and_make_sdformat.sh
RUN /tmp/build_and_make_sdformat.sh

COPY build_and_make_gazebo.sh /tmp/build_and_make_gazebo.sh
RUN /tmp/build_and_make_gazebo.sh

COPY build_and_make_revolve.sh /tmp/build_and_make_revolve.sh
RUN /tmp/build_and_make_revolve.sh

