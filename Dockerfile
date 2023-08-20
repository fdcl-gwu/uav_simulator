FROM ros:noetic-ros-core-focal
RUN apt update
RUN apt -y install python3-pip python3-gi python3-pip tmux
RUN apt -y install mesa-utils libgl1-mesa-glx dbus
RUN apt -y install libgtk-3-dev python3-gi gobject-introspection
RUN apt -y install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-xacro
RUN python3 -m pip install pip --upgrade
RUN python3 -m pip install numpy pandas matplotlib

RUN python3 -m pip install -U rosdep
RUN sudo rosdep init
RUN rosdep update

RUN adduser --disabled-password sim
USER sim

WORKDIR /home
