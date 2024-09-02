echo "Starting docker image .."

if [ "$(docker ps -a -q -f name=uav-simulator-container)" ]; 
then
    echo "Previously created container exists, starting it .."
    docker start --interactive uav-simulator-container
else
    echo "No previously created container, creating a new one .."

    docker run \
        --name uav-simulator-container \
        --mount source="$(pwd)",target=/home/uav_simulator,type=bind \
        --net host \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -u sim \
        -e DISPLAY=unix$DISPLAY \
        --privileged \
        -it uav_simulator bash
fi

echo "Docker run complete"