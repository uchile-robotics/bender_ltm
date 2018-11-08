#!/usr/bin/env python
import rospy
import smach
import smach_ros

from uchile_states.interaction.states import Speak, KeyboardInput #, Hear, NoiseCheck


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
        smach.StateMachine.add('INTRODUCE', Speak(robot,text="I am "+robot.name +". Can you tell me your name?. Please "),
            transitions={'succeeded':'KEYBOARD_INPUT'}
            # transitions={'succeeded':'HEAR_QUESTION'}
        )
        smach.StateMachine.add('KEYBOARD_INPUT',KeyboardInput(robot),
            transitions={
                'succeeded':'SAVE_NAME'
            },
            remapping={
                'recognized_sentence':'recognized_sentence'
            }
        )

        smach.StateMachine.add('SAVE_NAME',SaveNameAsInformation(),
            transitions={
                'succeeded':'succeeded'
            }
        )
        
        # smach.StateMachine.add('HEAR_QUESTION',Hear(robot,dictionary='/basic/names'),
        #     transitions={
        #         'succeeded':'NOISE_CHECK',
        #         'preempted':'SORRY'
        #     },
        #     remapping={
        #         'recognized_sentence':'recognized_sentence'
        #     }
        # )
        # smach.StateMachine.add('NOISE_CHECK',NoiseCheck(),
        #     transitions={
        #         'succeeded':'SAVE_NAME',
        #         'failed':'SORRY'
        #     },
        #     remapping={
        #         'input_text':'recognized_sentence'
        #     }
        # )
        # smach.StateMachine.add('SORRY',Speak(robot,text="I am sorry, I couldn't hear you. Can you repeat your name please?"),
        #     transitions={
        #         'succeeded':'HEAR_QUESTION'
        #     }
        # )

    return sm



if __name__ == '__main__':

    from bender_skills import robot_factory

    rospy.init_node('ask_name_simple')
    # robot = robot_factory.build(["tts","audition"],core=False)
    robot = robot_factory.build(["tts"], core=False)

    robot.check()
    sm = getInstance(robot)
    outcome = sm.execute()

