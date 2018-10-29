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
from uchile_states.perception import personal_information, facial_features_recognition
from uchile_states.perception import wait_face_detection

#States
from uchile_states.interaction.states import Speak
from uchile_states.head.states import LookFront, LookHome, LookPerson, LookCrowd

#LTM
from ltm_demos import AskNameSimple

#MSG
from bender_msg.msg import 

class PublishInformation(smach.State):
    def __init__(self,robot):
        smach.State.__init__(self, outcomes = ['succeeded','failed','aborted','preempted'],
                    input_keys=['features','facial_features'],
                    output_keys=['features','facial_features'])
        self.robot=robot
        self._ltm_topic = "/asdssd_asdd"

    def execute(self, userdata):
        gender, phase = [0]*2
        age  = ""
                
        try:
            if 'gender' in userdata.features and len(userdata.facial_features['gender'])>0:
                gender = userdata.facial_features['gender'][0]
                gender = gender.lower() 

            if 'age' in userdata.features and len(userdata.facial_features['age'])>0:
                if self.robot.name == 'bender':
                    age_range = userdata.facial_features['age'][0]

                    age = age_range.strip("()")
                    age = age.replace(',','')
                    age = int(age.split(" ")[0]) + (int(age.split(" ")[1])-int(age.split(" ")[0]))/2
                    
                    if age_range == "(0, 2)" or  age_range == "(4, 6)" or  age_range == "(8, 12)":
                        phase="child"
                        if gender == "female":
                            phase="girl"
                    else:
                        phase="adult"
                else:
                    age = userdata.facial_features['age'][0]
                    if age < 13:
                        phase="child"
                        if gender == "female":
                            phase="girl"
                    else:
                        phase="adult"


            personal_information  = {'personal_gender': str(gender),
                                    'personal_age': str(age),                                 
                                    'personal_phase': str(phase)  }
            #print personal_information

            return 'succeeded'
        
        except Exception, e:
            print(e)
            return 'preempted'


def getInstance(robot):
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys=['features','facial_features'])

    sm.userdata.input_text = ""
    sm.userdata.features = ['gender','age']

    with sm:
        smach.StateMachine.add('LOOKPERSON',LookPerson(robot),
            transitions={
                'succeeded':'WAIT_OPERATOR'
            }
        )
        smach.StateMachine.add('WAIT_OPERATOR',wait_face_detection.getInstance(robot, timeout=20),
            transitions={
                'succeeded':'NOTIFY_OPERATOR',
                'aborted' : 'NOTIFY_OPERATOR',
                'preempted':'NOTIFY_OPERATOR'
            }
        )
        smach.StateMachine.add('NOTIFY_OPERATOR',Speak(robot,text="Hi operator"), #, please look into my eyes, so I can get a good look at you."),
            transitions={
                'succeeded':'NOTIFY_READY'
            }
        )
        #CAmbiar a que funcione por teclado o colocar alternativa
        smach.StateMachine.add('ASK_NAME',AskNameSimple.getInstance(robot),
            transitions={
                'succeeded': 'PERSONAL_INFORMATION'
            }
        ) #operator_name

        smach.StateMachine.add('GET_INFORMATION', facial_features_recognition.getInstance(robot),
            transitions={
                'succeeded':'ANALIZEFACEINFORMATION'})

        smach.StateMachine.add('PUBLISH_INFORMATION', PublishInformation(robot))

        return sm


if __name__ == '__main__':

    rospy.init_node('WAIT_OPERATOR')

    #Only for testing 
    robot = robot_factory.build(["tts","AI","audition","facial_features","neck"], core=False)

    robot.check()
    sm = getInstance(robot)
    outcome = sm.execute()
