#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospy
import smach

# LTM
import ltm_addons.smach as ltm
from bender_ltm_plugins.msg import HumanEntity

# State Machines
from uchile_states.interaction.states import Speak
from uchile_states.joy.states import WaitForButtonA as WaitForButton

from bender_ltm_demos import Data


class RecordHuman(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            output_keys=['human'])

        self._ltm_topic = "/bender/ltm/entity/human/update"
        self.ltm_pub = rospy.Publisher(self._ltm_topic, HumanEntity, queue_size=10)

    def execute(self, ud):
        entity = HumanEntity()
        entity.name = Data.NAME
        self.ltm_pub.publish(entity)
        ud.human = entity
        return 'succeeded'


class RecordHappyHuman(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            io_keys=['human'])

        self._ltm_topic = "/bender/ltm/entity/human/update"
        self.ltm_pub = rospy.Publisher(self._ltm_topic, HumanEntity, queue_size=10)

    def execute(self, ud):
        entity = HumanEntity()
        entity.emotion = "happy"
        entity.name = ud.human.name
        self.ltm_pub.publish(entity)
        ud.human.emotion = entity.emotion
        return 'succeeded'


class DeliverCoffee(smach.State):

    def __init__(self, robot):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['human'])

        self.robot = robot

    def execute(self, ud):
        entity = ud.human
        self.robot.joy.wait_for_button('A')
        self.robot.tts.say("Hei " + entity.name + ", enjoy this coffee.")
        self.robot.tts.wait_until_done(timeout=10.0)
        return 'succeeded'


def build_get_order_sm(robot):
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        output_keys=['human'])
    ltm.register_state(sm, ["get_order"])

    wait_face_sm = WaitForButton(robot)
    approach_sm = WaitForButton(robot)
    ltm.register_state(approach_sm, ["wait_face"])
    ltm.register_state(approach_sm, ["approach_human"])

    with sm:
        smach.StateMachine.add(
            'WAIT_FACE',
            wait_face_sm,
            transitions={'succeeded': 'APPROACH_TO_HUMAN'}
        )
        smach.StateMachine.add(
            'APPROACH_TO_HUMAN',
            approach_sm,
            transitions={'succeeded': 'SAY_HELLO'}
        )
        smach.StateMachine.add(
            'SAY_HELLO',
            Speak(robot, text="Hello, " + Data.NAME + " i will prepare a coffee for you."),
            transitions={'succeeded': 'RECORD_HUMAN'}
        )
        smach.StateMachine.add(
            'RECORD_HUMAN',
            RecordHuman(),
            transitions={'succeeded': 'succeeded'})
    return sm


def build_prepare_coffee_sm(robot):
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    ltm.register_state(sm, ["deliver_order"])

    go_sm = WaitForButton(robot)
    prepare_sm = WaitForButton(robot)
    ltm.register_state(go_sm, ["go_to_kitchen"])
    ltm.register_state(prepare_sm, ["prepare_coffee"])

    with sm:
        smach.StateMachine.add(
            'GO_TO_KITCHEN',
            go_sm,
            transitions={'succeeded': 'PREPARE_COFFEE'})
        smach.StateMachine.add(
            'PREPARE_COFFEE',
            prepare_sm,
            transitions={'succeeded': 'succeeded'})
    return sm


def build_give_order_sm(robot):
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['human'])
    ltm.register_state(sm, ["deliver_order"])

    approach_sm = WaitForButton(robot)
    give_sm = DeliverCoffee(robot)
    ltm.register_state(approach_sm, ["approach_human"])
    ltm.register_state(give_sm, ["deliver_coffee"])

    with sm:
        smach.StateMachine.add(
            'APPROACH_TO_HUMAN',
            approach_sm,
            transitions={'succeeded': 'GIVE_COFFEE'})
        smach.StateMachine.add(
            'GIVE_COFFEE',
            give_sm,
            transitions={'succeeded': 'RECORD_HUMAN'})
        smach.StateMachine.add(
            'RECORD_HUMAN',
            RecordHappyHuman(),
            transitions={'succeeded': 'succeeded'})
    return sm


def getInstance(robot):
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    ltm.register_state(sm, ["ltm_demo", "coffee_session"])
    with sm:
        smach.StateMachine.add(
            'GET_ORDER',
            build_get_order_sm(robot),
            transitions={'succeeded': 'PREPARE_COFFEE'})

        smach.StateMachine.add(
            'PREPARE_COFFEE',
            build_prepare_coffee_sm(robot),
            transitions={'succeeded': 'GIVE_ORDER'})

        smach.StateMachine.add(
            'GIVE_ORDER',
            build_give_order_sm(robot),
            transitions={'succeeded': 'succeeded'})

        return sm


if __name__ == '__main__':

    try:
        import smach_ros
        from bender_skills import robot_factory

        rospy.init_node('ltm_demo__coffee_session')

        robot = robot_factory.build(["joy", "tts"], core=False)
        robot.check()

        # build machine
        sm = getInstance(robot)
        ltm.setup(sm)

        # smach introspection server
        sis = smach_ros.IntrospectionServer('ltm_demo__coffee_session_sis', sm, '/SM_LTM_DEMO_HUMAN_SESSION')
        sis.start()

        # execute machine
        sm.execute()

        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()

    except rospy.ROSInterruptException:
        pass
