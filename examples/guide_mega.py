#!/usr/bin/python3

from hsrb_interface import Robot
import rospy

robot = Robot()
base = robot.try_get("omni_base")
tts = robot.try_get("default_tts")
whole_body = robot.try_get("whole_body")

def go_and_say(pos=(0,0,0), contents=""):
    try:
        base.go_abs(pos[0], pos[1], pos[2], 180)
    except:
        rospy.logerr("Failed to go to position: " + str(pos))
    
    tts.say(contents)
    rospy.sleep(5)


_SENARIO = [
    ((2.9, 0.36, -1.57), u'This is my favorite sofa. You can relax to see the TV. Although, I cannot sit.'),
    ((5.4, 0.07, -1.57), u'You can see sea of Odaiba from here. See how beautiful it is.'),
    ((5.7, 1.6, 3.14), u'This is a IH type cooking range. What shall we cook today.'),
    ((5.7, 2.6, 3.14), u'This is the sink. Although, we are out of water now.'),
    ((4.4, 5.8, -1.57), u'This is the place where we can join and eat.'),
    ((5.2, 6.3, 0.07), u'This is a future TV. We can control with a gesture. Great.'),
    ((1.5, 3.3, 1.57), u'This is my most recommended place, the curious tree. Feel calm.'),
    ((0.0, 0.0, 3.14), u'This is the end of my guide. Bye-bye.')]

if __name__ == "__main__":
    try:
        whole_body.move_to_go()
    except:
        rospy.logerr("Failed to move to go pose.")
    
    tts.say("hello")

    for unit in _SENARIO:
        print(unit)
        go_and_say(unit[0], unit[1])