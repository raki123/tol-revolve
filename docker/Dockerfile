from base_gazebo_image

COPY build_and_make_tol-revolve.sh /tmp/build_and_make_tol-revolve.sh
RUN /tmp/build_and_make_tol-revolve.sh

ENV PROJECT /project

RUN mkdir $PROJECT
WORKDIR $PROJECT
