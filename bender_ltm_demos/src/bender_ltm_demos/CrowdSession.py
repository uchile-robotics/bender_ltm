#!/usr/bin/env python
import os
import rospy
import smach
import smach_ros

if os.environ['UCHILE_ROBOT']=="bender":
    from bender_skills import robot_factory
else:
    from maqui_skills import robot_factory

#States
from uchile_states.interaction.states import Speak
# from uchile_states.base.states import RotateState
# # from uchile_states.interaction.states import Tell
#from uchile_states.navigation.states import GoState
from uchile_states.misc.states import CodeRunner
# from uchile_robocup.SPR import BlindManGame, RiddleGame
# from uchile_states.head.states import LookFront, LookHome, LookPerson, LookCrowd, SetNeckJoint
from uchile_states.perception import crowd_information2 as crowd_information
# from uchile_states.base.states import SetSecurityShell
# from uchile_states.interaction.tablet_states import WaitTouchScreen, SubtitlesDouble

#State Machines
# from uchile_states.perception import wait_open_door

#MSG
from bender_ltm_plugins.msg import HumanEntity

#LTM
from bender_ltm_demos import CrowdInformationSimple


class PublishInformation(smach.State):
    def __init__(self,robot):
        smach.State.__init__(self, outcomes = ['succeeded','aborted','preempted'],
                    input_keys=['operator_name','features','facial_features', 'facial_features_image','face_emotion'],
                   )
        self.robot=robot
        self._ltm_topic = "/bender/ltm/entity/human/update"
        self.ltm_pub = rospy.Publisher(self._ltm_topic, HumanEntity, queue_size=10)


    def execute(self, userdata):

        entity = HumanEntity()
        entity.name = userdata.operator_name
        # entity.emotion = userdate.face_emotion

        self.ltm_pub.publish(entity)

        return 'succeeded'
        

def getInstance(robot):
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    WAVING = True
    PLACE_START = "gpsr_init"
    PLACE_EXIT = "bar"
    sm.userdata.recognized_sentence = ""
    sm.userdata.features = ['gender','age']# 'posture']

    sm.userdata.input_text = ""
    sm.userdata.neck_joints = []
    
    with sm:
        
        smach.StateMachine.add('GREET',Speak(robot,text="Hi friends!",gestures = True),
            transitions={
                'succeeded':'LOOKCROWD'
            }
        )

        smach.StateMachine.add('LOOKCROWD', CodeRunner('rospy.sleep(1)'),
            transitions={
                'succeeded':'CROWD_INFORMATION'
            }
        )
        smach.StateMachine.add('CROWD_INFORMATION',CrowdInformationSimple.getInstance(robot, map_filter=False, waving=WAVING),
            transitions={
                'succeeded':'PUBLISH_INFORMATION',
                'aborted'  :'aborted',
                'failed'   :'aborted'  }
        )
        smach.StateMachine.add('PUBLISH_INFORMATION', PublishInformation(robot),
            transitions={
                'succeeded':'FINAL_SPEECH'
            }
        )

        smach.StateMachine.add('FINAL_SPEECH',Speak(robot,text="thanks!, now i always remember all of you", gestures = True),
            transitions={
                'succeeded':'succeeded'
            }
        )


        # initial_data = smach.UserData()
        # sm.set_initial_state(['SETUP'],initial_data)# 

        return sm

if __name__ == '__main__':

    rospy.init_node('crowd_session')
    

    ###
    ### Skills Necesarias para la Prueba
    ###

    skills_base = [#"base",
                    #"waving_deep",
                    "tts",
                    # "AI",
                    "audition",
                    # "report_generator",
                    "facial_features",
                    # "person_detection",
                    #"knowledge",
                    # "neck",
                    # "sound_localization",
                    "face"]


    #constructor para bender
    if os.environ['UCHILE_ROBOT']=="bender":
        # skills = skills_base+only_bender
        robot = robot_factory.build( skills_base , core=False)
    
    robot.check()
    sm = getInstance(robot)
    sis = smach_ros.IntrospectionServer('CROWDSESSION_SM', sm, '/CROWDSESSION')
    sis.start()
    outcome = sm.execute()
    sis.stop()
