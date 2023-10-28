PACKAGE=uav_gazebo
LAUNCH_FILE=simple_world.launch

.PHONY: launch

launch:
	@roslaunch $(PACKAGE) $(LAUNCH_FILE)

uav:
	@python3 ./scripts/main.py

env:
	@echo "cd devel && source setup.bash && cd ../"