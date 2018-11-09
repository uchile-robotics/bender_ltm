#!/usr/bin/env python
import rospy
import smach

from uchile_states.interaction.states import Speak, KeyboardInput


class SaveNameAsInformation(smach.State):

    def __init__(self, robot):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['recognized_sentence'],
            output_keys=['operator_name'])

        self.robot = robot

    def execute(self, userdata):
        name = userdata.recognized_sentence
        userdata.operator_name = name
        self.robot.say("Nice to meet you " + name + ".")
        return "succeeded"


def getInstance(robot):
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        output_keys=['operator_name'])

    with sm:
        smach.StateMachine.add(
            'INTRODUCE',
            Speak(robot, text="Can you tell me your name?"),
            transitions={'succeeded': 'KEYBOARD_INPUT'})

        smach.StateMachine.add(
            'KEYBOARD_INPUT',
            KeyboardInput(robot),
            transitions={'succeeded': 'SAVE_NAME'}
        )

        smach.StateMachine.add(
            'SAVE_NAME',
            SaveNameAsInformation(robot),
            transitions={'succeeded': 'succeeded'}
        )

    return sm


if __name__ == '__main__':

    try:
        import smach_ros
        from bender_skills import robot_factory

        rospy.init_node('ltm_demo_sm__ask_name')

        # robot = robot_factory.build(["tts","audition"],core=False)
        robot = robot_factory.build(["tts", "display_interface"], core=False)
        robot.check()

        # build machine
        sm = getInstance(robot)
        # ltm.setup(sm)

        # smach introspection server
        sis = smach_ros.IntrospectionServer('ltm_demo__human_session_sis', sm, '/SM_LTM_DEMO_HUMAN_SESSION')
        sis.start()

        # execute machine
        sm.execute()

        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()

    except rospy.ROSInterruptException:
        pass
