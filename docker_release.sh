docker build --tag <tag-name>  -t uav_simulator .
docker tag uav_simulator kanishgama/uav_simulator:<tag-name>
docker push kanishgama/uav_simulator:<tag-name>