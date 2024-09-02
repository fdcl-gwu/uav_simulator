FROM ros:iron-ros-core-jammy
RUN apt update
RUN apt -y install python3-pip tmux
RUN apt -y install python3-colcon-common-extensions
RUN apt -y install ros-iron-gazebo-ros-pkgs ros-iron-gazebo-ros2-control ros-iron-xacro
RUN python3 -m pip install pip --upgrade
RUN python3 -m pip install numpy pandas matplotlib
RUN python3 -m pip install PyQt5

RUN adduser --disabled-password sim
USER sim

WORKDIR /home
