#!/usr/bin/env python
import rospy
import smach
import smach_ros
import os

#Robot building
if os.environ['UCHILE_ROBOT']=="bender":
    from bender_skills import robot_factory
else:
    from maqui_skills import robot_factory
    
# #State Machines
from uchile_states.perception import personal_information, facial_features_recognition, emotion_recognition
from uchile_states.perception import wait_face_detection

#States
from uchile_states.interaction.states import Speak
from uchile_states.head.states import LookFront, LookHome, LookPerson, LookCrowd

#LTM
# from bender_ltm_demos import AskNameSimple

#MSG
from bender_ltm_plugins.msg import HumanEntity



def getInstance(robot):
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    sm.userdata.operator_name = ""
    sm.userdata.features = ['gender','age']

    with sm:
        # smach.StateMachine.add('LOOKPERSON',LookPerson(robot),
        #     transitions={
        #         'succeeded':'WAIT_OPERATOR'
        #     }
        # )
        smach.StateMachine.add('WAIT_OPERATOR',wait_face_detection.getInstance(robot, timeout=20),
            transitions={
                'succeeded':'GREET_OPERATOR',
                'aborted' : 'GREET_OPERATOR',
                'preempted':'GREET_OPERATOR'
            }
        )
        smach.StateMachine.add('GREET_OPERATOR',Speak(robot,text="Hi operator"), #, please look into my eyes, so I can get a good look at you."),
            transitions={
                'succeeded':'succeeded'
            }
        )

        return sm


if __name__ == '__main__':

    rospy.init_node('INTERVIEW')

    #Only for testing 
    # robot = robot_factory.build(["tts","audition","facial_features","neck"], core=False)
    robot = robot_factory.build(["tts","audition"], core=False)

    robot.check()

    sm = getInstance(robot)
    sis = smach_ros.IntrospectionServer('interview_server', sm, '/INTERVIEW')
    sis.start()
    outcome = sm.execute()
