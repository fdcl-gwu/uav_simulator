FROM ros:iron-ros-core-jammy
RUN apt update
RUN apt -y install python3-pip python3-gi python3-pip tmux
RUN apt -y install mesa-utils libgl1-mesa-glx dbus
RUN apt -y install libgtk-3-dev python3-gi gobject-introspection python3-colcon-common-extensions
RUN apt -y install ros-iron-gazebo-ros-pkgs ros-iron-gazebo-ros2-control ros-iron-xacro
RUN python3 -m pip install pip --upgrade
RUN python3 -m pip install numpy pandas matplotlib

# RUN python3 -m pip install -U rosdep
# RUN sudo rosdep init
# RUN rosdep update

RUN adduser --disabled-password sim
USER sim

WORKDIR /home
