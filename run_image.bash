xhost local:root

XAUTH=/tmp/.docker.xauth

if [$1 = ''] ; then
	echo "Usage: ./run_image.bash <image_name>"
	exit 1
fi

docker run -it \
	-v $(pwd):/home/ros2_ws \
	--env="DISPLAY=$DISPLAY" \
	--env="QT_X!!_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--env="XAUTHORITY=$XAUTH" \
	--volume="$XAUTH:$XAUTH" \
	--net=host \
	--privileged \
	-w /home/ros2_ws \
	$1 \
	bash