#! /usr/bin/env python
import rospy
import smach
import cv2
import time

from smach import StateMachine
from smach_ros import ConditionState, IntrospectionServer

from geometry_msgs.msg import PoseStamped
from uchile_states.perception import person_detection, facial_features_recognition,shirt
from uchile_states.knowledge.report_states import SaveFacialImages
from uchile_states.interaction.states import Speak
from uchile_states.perception.states import FacialFeaturesSetup

# from uchile_states.perception import waving_deep_detector as waving_detector
# from uchile_states.perception.crowd_information2 import AnalizeTShirtInformation


#Crowd information
#crowd_size  : Size of the crowd
#crowd_gender : Number of male or female people
#crowd_male : Number of male people
#crowd_female : Number of female people
#crowd_children : Number of children
#crowd_adults: Number of adults


class AnalizeFaceInformation(smach.State):
    def __init__(self,robot):
        smach.State.__init__(self, outcomes = ['succeeded','failed','aborted','preempted'],
                    input_keys=['features','facial_features'],
                    output_keys=['total_people','facial_features','crowd_state_text'])

        # self.ai = robot.get("AI")
        self.robot=robot.name

    def execute(self, userdata):
        try:

            if not userdata.facial_features or type(userdata.facial_features)==bool:
                return 'failed'

            total, n_woman, n_men, n_children, n_adults, n_elders = [0]*6
            n_stand, n_sit, n_laying  = [0]*3
            sit_gen, lay_gen, sta_gen = ["male"]*3
            sit_p, stand_p, laying_p = [-1]*3
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
            

            if 'posture' in userdata.features:
                for i, post in enumerate(userdata.facial_features['posture']):
                    if post.lower()  == "stand":
                        n_stand+=1
                        stand_p = i
                    elif post.lower()  == "sit":
                        n_sit+=1
                        sit_p = i
                    else:
                        n_laying+=1
                        laying_p = i
                
                if total == 0:
                    total = (n_stand+n_sit+n_laying)


            crowd_information  = {'crowd_size': str(total),
                                    'crowd_men': str(n_men),                                 
                                    'crowd_women': str(n_woman), 
                                    'crowd_children': str(n_children),                                 
                                    'crowd_adults': str(n_adults),
                                    'crowd_elders': str(n_elders),
                                    'crowd_stand': str(n_stand), 
                                    'crowd_sit': str(n_sit),                                 
                                    'crowd_laying': str(n_laying)}
            print crowd_information


            # self.ai.addProperties(crowd_gender_information)                                
            # self.ai.addProperties(crowd_information)
            userdata.total_people = total
            userdata.crowd_state_text = "In the crowd there is {0} people. {1} females and {2} males.".format(total,n_woman, n_men)
            return 'succeeded'  

        except Exception as e:
            rospy.logerr(e)
            return 'aborted'



def getInstance(robot, map_filter = False, waving=False ):

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','failed'],
                    output_keys=['facial_features'])

    sm.userdata.features = ['gender','age']
    sm.waving_posestamped = PoseStamped()
    sm.userdata.input_text = ""

    """
    Pepper:
        Features: age/gender/recognize/
        Extra: posture in other state machine!

    Bender:
        Features: age/gender/recognize/posture/
        Extra: arm_position/waving

    """

    with sm:

        smach.StateMachine.add('CROWD_INFO', facial_features_recognition.getInstance(robot),
                transitions={'succeeded': 'ANALIZE_FACE_INFORMATION'}
        )


        smach.StateMachine.add('ANALIZE_FACE_INFORMATION', AnalizeFaceInformation(robot),
            transitions={'succeeded': 'succeeded'})


    # import os
    # if os.environ['UCHILE_ROBOT']=="bender":
    #     from bender_skills import robot_factory
    # else:
    #     from maqui_skills import robot_factory


    # if robot.name =="maqui":
    #     initial_state = ['SETUP'] #
    # else:
    #     initial_stat= ['CROWD_INFO']
    # sm.set_initial_state(initial_state)

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
        "AI",
        "person_detector",
        "knowledge",
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