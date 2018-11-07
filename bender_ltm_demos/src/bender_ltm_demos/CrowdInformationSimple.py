#! /usr/bin/env python
import rospy
import smach
import cv2
import time

from smach import StateMachine
from smach_ros import ConditionState, IntrospectionServer

from geometry_msgs.msg import PoseStamped
from uchile_states.perception import person_detection, facial_features_recognition
# from uchile_states.knowledge.report_states import SaveFacialImages
from uchile_states.interaction.states import Speak
# from uchile_states.perception.states import FacialFeaturesSetup

# from uchile_states.perception import waving_deep_detector as waving_detector
# from uchile_states.perception.crowd_information2 import AnalizeTShirtInformation


#Crowd information
#crowd_size  : Size of the crowd
#crowd_male : Number of male people
#crowd_female : Number of female people
#crowd_children : Number of children
#crowd_adults: Number of adults


class AnalizeFaceInformation(smach.State):
    def __init__(self,robot):
        smach.State.__init__(self, outcomes = ['succeeded','failed','aborted','preempted'],
                    input_keys=['features','facial_features'],
                    output_keys=['n_people', 'n_male', 'n_female', 'n_children', 'n_adults', 'n_elders'])

 
        self.robot=robot.name

    def execute(self, userdata):
        try:

            if not userdata.facial_features or type(userdata.facial_features)==bool:
                return 'failed'

            total, n_woman, n_men, n_children, n_adults, n_elders = [0]*6

            if 'gender' in userdata.features:
                for gender in userdata.facial_features['gender']:
                    if gender.lower()  == "female":
                        n_woman+=1
                    else:
                        n_men+=1
                total = (n_men+n_woman)
            if 'age' in userdata.features:
                
                for age in userdata.facial_features['age']:
                    if isinstance(age, basestring):
                        if age == "(0, 2)" or  age == "(4, 6)" or  age == "(8, 12)":
                            n_children+=1
                        elif  age == "(60, 100)":
                            n_elders+=1
                        else:
                            n_adults+=1
                    else:
                        if age < 15:
                            n_children+=1
                        elif  age > 60:
                            n_elders+=1
                        else:
                            n_adults+=1
                
                if total == 0:
                    total = (n_children+n_adults+n_elders)
            
            userdata.n_people=total
            userdata.n_male=n_men
            userdata.n_female=n_woman
            userdata.n_children=n_children
            userdata.n_adults=n_adults
            userdata.n_elders=n_elders

            return 'succeeded'  

        except Exception as e:
            rospy.logerr(e)
            return 'aborted'



def getInstance(robot, map_filter = False, waving=False ):

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','failed'],
                output_keys=['n_people', 'n_male', 'n_female', 'n_children', 'n_adults', 'n_elders'])

    sm.userdata.features = ['gender','age']
    sm.waving_posestamped = PoseStamped()
    sm.userdata.input_text = ""



    with sm:

        smach.StateMachine.add('CROWD_INFO', facial_features_recognition.getInstance(robot),
                transitions={'succeeded': 'ANALIZE_FACE_INFORMATION'}
        )


        smach.StateMachine.add('ANALIZE_FACE_INFORMATION', AnalizeFaceInformation(robot),
            transitions={'succeeded': 'succeeded'})


    return sm

def main():


    import os
    if os.environ['UCHILE_ROBOT']=="bender":
        from bender_skills import robot_factory
    else:
        from maqui_skills import robot_factory

    rospy.init_node("crowd_information")

    robot = robot_factory.build([
        "facial_features",
        # "AI",
        "person_detector",
        # "knowledge",
        "tts"
    ], core=False)

    robot.check()
    sm = getInstance(robot, True)
    ud = smach.UserData()
    ud.features = ['gender','age']
    
    # introspection server
    sis = IntrospectionServer('crowd_information', sm, '/CROWD_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()

if __name__ == "__main__":
    main()
