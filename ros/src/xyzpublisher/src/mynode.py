#!/usr/bin/env python
import rospy
from xyzpublisher.msg import Kenkipos

def talker(trajlist, reflist):
    pub = rospy.Publisher('traj', Kenkipos, queue_size=10)
    pub2 = rospy.Publisher('gt', Kenkipos, queue_size=10)
    rospy.init_node('xyzpublisher', anonymous=True)

    rate = rospy.Rate(5) # 5Hz
    count = 0
    while not rospy.is_shutdown() and count<=1235:
        mymsg = Kenkipos()
        onetraj = trajlist[count].split(' ')
        stamp = onetraj[0].split('.')
        stampsecs = int(stamp[0])
        stampnsecs = int((float(onetraj[0])-float(stamp[0]))*1000000000)
        x = float(onetraj[1])
        y = float(onetraj[2])
        z = float(onetraj[3])
        mymsg.stamp.secs = stampsecs
        mymsg.stamp.nsecs = stampnsecs
        mymsg.x = x
        mymsg.y = y
        mymsg.z = z
        pub.publish(mymsg)

        mymsg = Kenkipos()
        oneref = reflist[count].split(' ')
        stamp = oneref[0].split('.')
        stampsecs = int(stamp[0])
        stampnsecs = int((float(oneref[0])-float(stamp[0]))*1000000000)
        x = float(oneref[1])
        y = float(oneref[2])
        z = float(oneref[3])
        mymsg.stamp.secs = stampsecs
        mymsg.stamp.nsecs = stampnsecs
        mymsg.x = x
        mymsg.y = y
        mymsg.z = z
        pub2.publish(mymsg)

        count+=1

        rate.sleep()

if __name__=='__main__':
    try:
        f = open('/home/jiang/Desktop/bao/openvslam/ros/src/xyzpublisher/testdata/traj_aligninplane.txt')
        trajdd = f.read()
        f.close()
        trajlist = trajdd.split('\n')
        
        f = open('/home/jiang/Desktop/bao/openvslam/ros/src/xyzpublisher/testdata/ref_aligninplane.txt')
        refdd = f.read()
        f.close()
        reflist = refdd.split('\n')

        print("finish reading!")

        talker(trajlist, reflist)
    except rospy.ROSInterruptException:
        pass

