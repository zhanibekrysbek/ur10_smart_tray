#!/usr/bin/env python3

import rospy
# from sound_play.msg import SoundRequest
# from sound_play.libsoundplay import SoundClient

from playsound import playsound

from std_msgs.msg import Header



# beepfile1 = "/home/cvrl20/catkin_ws/src/proactive_phri/ur10_smart_tray/data/beep/beep-07a.wav"
beepfile1 = "/home/cvrl20/catkin_ws/src/proactive_phri/ur10_smart_tray/data/beep/din-ding-89718.mp3"
beepfile2 = "/home/cvrl20/catkin_ws/src/proactive_phri/ur10_smart_tray/data/beep/ping-82822.mp3"
# beepfile2 = "/home/cvrl20/catkin_ws/src/proactive_phri/ur10_smart_tray/data/beep/beep-08b.wav"


if __name__ == '__main__':
    
    # rospy.init_node('beep_sound_play')
    rospy.init_node('BeepSoundPlay', anonymous=True)
    rospy.loginfo('Playing sound..')    

    pub = rospy.Publisher('beep', Header, queue_size = 10)
    # Import after printing usage for speed.
    # soundhandle = SoundClient()

    volume = 1.0

    msg = Header()

    rospy.sleep(.5)

    msg.seq = 1
    msg.stamp = rospy.get_rostime()
    msg.frame_id = 'Beep1'
    pub.publish(msg)
    # soundhandle.playWave(beepfile1, volume)
    playsound(beepfile1)

    msg.seq = 2
    msg.stamp = rospy.get_rostime()
    msg.frame_id = 'Beep11'
    pub.publish(msg)

    # Main pause between two beeps
    rospy.sleep(3)

    msg.seq = 3
    msg.stamp = rospy.get_rostime()
    msg.frame_id = 'Beep2'
    pub.publish(msg)
    # soundhandle.playWave(beepfile2, volume)
    playsound(beepfile2)

    msg.seq = 4
    msg.stamp = rospy.get_rostime()
    msg.frame_id = 'Beep22'
    pub.publish(msg)



    rospy.sleep(1)


