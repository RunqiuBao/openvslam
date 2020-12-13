#catkin_make

sudo sh -c 'echo 8000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
source devel/setup.bash

rosrun publisher image
sleep 1
echo "publisher node starting success!"

#yolov3
rosrun yolov3 yolov3.py #修改后无须编译

#openvslam
rosrun openvslam run_slam -v /home/jiang/Desktop/bao/orb_vocab/orb_vocab.dbow2 -c /home/jiang/Desktop/bao/config.yaml
sleep 1
echo "openvslam node starting success!"

#nmeagen
rosrun nmeagen nmeagen_node

