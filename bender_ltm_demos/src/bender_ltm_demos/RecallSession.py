#!/usr/bin/env python
import rospy

# robot
from bender_skills import robot_factory

# LTM
from bender_ltm_plugins.msg import HumanEntity
from bender_ltm_plugins.msg import CrowdEntity


def main():
    robot = robot_factory.build(["tts", "ltm"], core=False)

    robot.say("Hello, i am bender")
    robot.tts.wait_until_done()
    robot.ltm.foo("blababala")


if __name__ == '__main__':
    rospy.init_node('ltm_demo__human_session')
    try:
        main()
    except rospy.ROSInterruptException:
        pass
