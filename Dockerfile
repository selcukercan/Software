# Note: this container will have the name duckietown/rpi-duckiebot-base
FROM duckietown/rpi-ros-kinetic-base:master18


RUN [ "cross-build-start" ]

COPY requirements.txt /requirements.txt
COPY requirements_system_identification.txt /requirements_system_identification.txt
COPY requirements_system_identification_build.txt /requirements_system_identification_build.txt

# otherwise installation of Picamera fails https://github.com/resin-io-projects/resin-rpi-python-picamera/issues/8
ENV READTHEDOCS True

RUN pip install -r /requirements_system_identification_build.txt
RUN pip install -r /requirements_system_identification.txt
RUN pip install -r /requirements.txt


RUN mkdir /home/software
COPY . /home/software/
# COPY .git /home/software/.git/
COPY docker/machines.xml /home/software/catkin_ws/src/00-infrastructure/duckietown/machines

ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
RUN /bin/bash -c "cd /home/software/ && source /opt/ros/kinetic/setup.bash && catkin_make -j -C catkin_ws/"

RUN echo "source /home/software/docker/env.sh" >> ~/.bashrc


# We make sure that all dependencies are installed
# by trying to import the duckietown_utils package
RUN bash -c "source /home/software/docker/env.sh && python -c 'import duckietown_utils'"

# Most of these will fail, but might be useful to debug some issues.
# Leave it here to run it manually.
# RUN bash -c "source /home/software/docker/env.sh && /home/software/what-the-duck"


RUN [ "cross-build-end" ]

WORKDIR /home/software

CMD [ "/bin/bash" ]

ENV DISABLE_CONTRACTS=1

LABEL maintainer="Breandan Considine breandan.considine@umontreal.ca"
