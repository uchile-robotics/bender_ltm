#!/usr/bin/env python
import rospy
import smach

from uchile_states.interaction.states import Speak, KeyboardInput


class SaveNameAsInformation(smach.State):

    def __init__(self, robot, text=None):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['recognized_sentence'],
            output_keys=['operator_name'])

        self.robot = robot
        self.text = text

    def execute(self, userdata):
        name = userdata.recognized_sentence
        userdata.operator_name = name
        speak_text = self.text if self.text else "nice to meet you " + name
        self.robot.say(speak_text)
        self.robot.tts.wait_until_done(timeout=4.0)
        return "succeeded"


def getInstance(robot, text=None, end_text=None):
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        output_keys=['operator_name'])

    speak_text = text if text else "Can you tell me your name?"
    with sm:
        smach.StateMachine.add(
            'INTRODUCE',
            Speak(robot, text=speak_text),
            transitions={'succeeded': 'KEYBOARD_INPUT'})

        smach.StateMachine.add(
            'KEYBOARD_INPUT',
            KeyboardInput(robot),
            transitions={'succeeded': 'SAVE_NAME'})

        smach.StateMachine.add(
            'SAVE_NAME',
            SaveNameAsInformation(robot, end_text))

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
