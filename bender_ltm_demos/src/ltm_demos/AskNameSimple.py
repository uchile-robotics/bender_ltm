#!/usr/bin/env python
import rospy
import smach
import smach_ros

from uchile_states.interaction.states import Speak, Hear, Tell, NoiseCheck
from uchile_states.interaction        import Confirmation


class SaveNameAsInformation(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['recognized_sentence'],
                                    output_keys=['operator_name'])

    def execute(self,userdata):
        userdata.operator_name = userdata.recognized_sentence
        return "succeeded"
        

def getInstance(robot):
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'], output_keys=['operator_name'])
    sm.userdata.input_text = ""

    with sm:
        smach.StateMachine.add('INTRODUCE',Speak(robot,text="I am "+robot.name +". Can you tell me your name?. Please "),
            transitions={'succeeded':'HEAR_QUESTION'}
        )
        smach.StateMachine.add('HEAR_QUESTION',Hear(robot,dictionary='/basic/names',web=False,timeout=10),
            transitions={
                'succeeded':'NOISE_CHECK',
                'preempted':'SORRY'
            },
            remapping={
                'recognized_sentence':'recognized_sentence'
            }
        )
        smach.StateMachine.add('NOISE_CHECK',NoiseCheck(),
            transitions={
                'succeeded':'SAVE_NAME',
                'failed':'SORRY'
            },
            remapping={
                'input_text':'recognized_sentence'
            }
        )
        smach.StateMachine.add('CONFIRM_NAME',Confirmation.getInstance(robot,text="Your name is"),
            transitions={
                'yes'    :'SAVE_NAME',
                'no'     :'NAME_NO',
                'aborted':'SORRY'
            },
            remapping={
                'confirmation_text':'recognized_sentence'
            }
        )
        smach.StateMachine.add('NAME_NO',Speak(robot,text="So, what is your name?"),
            transitions={
                'succeeded':'HEAR_QUESTION'
            }
        )
        smach.StateMachine.add('SAVE_NAME',SaveNameAsInformation(),
            transitions={
                'succeeded':'succeeded'
            }
        )
        smach.StateMachine.add('SORRY',Speak(robot,text="I am sorry, I couldn't hear you. Can you repeat your name please?"),
            transitions={
                'succeeded':'HEAR_QUESTION'
            }
        )

    return sm



if __name__ == '__main__':

    import os
    if os.environ['UCHILE_ROBOT']=="bender":
        from bender_skills import robot_factory
    else:
        from maqui_skills import robot_factory

    rospy.init_node('ask_name')
    robot = robot_factory.build(["tts","audition"],core=False)

    robot.check()
    sm = getInstance(robot)
    outcome = sm.execute()
