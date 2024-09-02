docker run \
    --name uav-simulator-container \
    --mount source="$(pwd)",target=/home/uav_simulator,type=bind \
    --net host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -u sim \
    -e DISPLAY=unix$DISPLAY \
    --privileged \
    -it uav_simulator bash

# For debugging: once the container is first run with the above command, you 
# can run the following command for the second and subsequent runs.
# docker start --interactive uav-simulator-container