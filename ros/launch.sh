source devel/setup.bash

rosrun publisher image
sleep 1
echo "publisher node starting success!"

rosrun openvslam run_slam -v /home/jiang/Desktop/bao/orb_vocab/orb_vocab.dbow2 -c /home/jiang/Desktop/bao/config.yaml
sleep 1
echo "openvslam node starting success!"
