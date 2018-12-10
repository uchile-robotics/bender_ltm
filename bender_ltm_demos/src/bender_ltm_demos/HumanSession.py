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
from uchile_states.perception import facial_features_recognition
from uchile_states.perception import emotion_recognition


HUMAN_NAME = "luz"
HUMAN_NAME = "matías"


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
        entity.name = HUMAN_NAME
        self.ltm_pub.publish(entity)
        ud.human = entity
        return 'succeeded'


class RecordHumanAgeAndGenre(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['facial_features', 'facial_features_image'],
            io_keys=['human'])

        self._ltm_topic = "/bender/ltm/entity/human/update"
        self.ltm_pub = rospy.Publisher(self._ltm_topic, HumanEntity, queue_size=10)

    def execute(self, ud):
        entity = HumanEntity()
        # entity.face = ud.facial_features_image
        # ud.human.face = entity.face
        try:
            if 'gender' in ud.facial_features and len(ud.facial_features['gender']) > 0:
                gender = ud.facial_features['gender'][0]
                gender = gender.lower()
                entity.gender = HumanEntity.MASCULINE if gender == "male" else HumanEntity.FEMININE
                ud.human.gender = entity.gender

            if 'age' in ud.facial_features and len(ud.facial_features['age']) > 0:
                age_range = ud.facial_features['age'][0]
                age_range = age_range.strip("()")
                age_range = age_range.replace(',', '')

                entity.age_bottom = int(age_range.split(" ")[0])
                entity.age_top = int(age_range.split(" ")[1])
                entity.age_avg = (entity.age_bottom + entity.age_top) / 2
                entity.live_phase = HumanEntity.CHILD if entity.age_bottom < 12 else HumanEntity.ADULT
                ud.human.age_bottom = entity.age_bottom
                ud.human.age_top = entity.age_top
                ud.human.age_avg = entity.age_avg
                ud.human.live_phase = entity.live_phase

            entity.name = ud.human.name
            self.ltm_pub.publish(entity)

        except Exception as e:
            rospy.logerr(e)
        return 'succeeded'


class RecordHumanEmotion(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['face_emotion'],
            io_keys=['human'])

        self._ltm_topic = "/bender/ltm/entity/human/update"
        self.ltm_pub = rospy.Publisher(self._ltm_topic, HumanEntity, queue_size=10)

    def execute(self, ud):
        entity = HumanEntity()
        entity.emotion = ud.face_emotion
        entity.name = ud.human.name
        self.ltm_pub.publish(entity)
        ud.human.emotion = entity.emotion
        return 'succeeded'


class SayHumanResults(smach.State):

    def __init__(self, robot):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['human'])

        self.robot = robot

    def execute(self, ud):
        entity = ud.human
        gender = "male" if entity.gender is HumanEntity.MASCULINE else "female"
        phase = "an adult"
        if entity.live_phase is HumanEntity.CHILD:
            phase = "a boy" if gender is "male" else "a girl"

        goodbye = "It seems you are a " + gender + " human"
        goodbye += " between " + str(entity.age_bottom) + " and " + str(entity.age_top) + " years"
        goodbye += ", so you are " + phase + " ."
        goodbye += " Thank you very much " + entity.name + ", see you later"
        self.robot.tts.say(goodbye)
        self.robot.tts.wait_until_done(timeout=10.0)
        return 'succeeded'


def build_wait_human_sm(robot):
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    ltm.register_state(sm, ["approach_human"])

    wait_face_sm = WaitForButton(robot, 15.0)
    approach_sm = WaitForButton(robot, 15.0)
    ltm.register_state(approach_sm, ["look_for_human"])
    ltm.register_state(approach_sm, ["approach_human"])
    with sm:
        smach.StateMachine.add(
            'LOOK_FOR_HUMAN',
            wait_face_sm,
            transitions={'succeeded': 'APPROACH_TO_HUMAN'}
        )
        smach.StateMachine.add(
            'APPROACH_TO_HUMAN',
            approach_sm,
            transitions={'succeeded': 'succeeded'}
        )
    return sm


def build_introduction_sm(robot):
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        output_keys=['human'])
    ltm.register_state(sm, ["hri_introduction"])
    with sm:
        smach.StateMachine.add(
            'SAY_HELLO',
            Speak(robot, text="Hello, i am bender ..."),
            transitions={'succeeded': 'ASK_NAME'})

        smach.StateMachine.add(
            'ASK_NAME',
            Speak(robot, text="Nice to meet you " + HUMAN_NAME),
            transitions={'succeeded': 'RECORD_HUMAN'})

        smach.StateMachine.add(
            'RECORD_HUMAN',
            RecordHuman(),
            transitions={'succeeded': 'succeeded'})
    return sm


def build_facial_features_sm(robot):
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['human'],
        output_keys=['human'])
    sm.userdata.features = ['gender', 'age']
    ltm.register_state(sm, ["facial_analysis"])

    with sm:
        smach.StateMachine.add(
            'INIT',
            Speak(robot, text="Please, look me in the eyes ..."),
            transitions={'succeeded': 'GET_AGE_AND_GENRE'})

        age_gender_sm = facial_features_recognition.getInstance(robot)
        ltm.register_state(age_gender_sm, ["age_estimation", "gender_estimation"])
        smach.StateMachine.add(
            'GET_AGE_AND_GENRE',
            age_gender_sm,
            transitions={
                'succeeded': 'RECORD_AGE_AND_GENRE',
                'failed': 'RECORD_AGE_AND_GENRE'
            })

        smach.StateMachine.add(
            'RECORD_AGE_AND_GENRE',
            RecordHumanAgeAndGenre(),
            transitions={'succeeded': 'GET_EMOTION'})

        emotion_sm = emotion_recognition.getInstance(robot)
        ltm.register_state(emotion_sm, ["emotion_estimation"])
        smach.StateMachine.add(
            'GET_EMOTION',
            emotion_sm,
            transitions={
                'succeeded': 'RECORD_EMOTION',
                'failed': 'RECORD_EMOTION'
            })

        smach.StateMachine.add(
            'RECORD_EMOTION',
            RecordHumanEmotion(),
            transitions={'succeeded': 'succeeded'})
    return sm


def build_goodbye_sm(robot):
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['human'])
    ltm.register_state(sm, ["hri_goodbye"])

    with sm:
        smach.StateMachine.add(
            'SAY_GOODBYE',
            SayHumanResults(robot),
            transitions={'succeeded': 'succeeded'})
    return sm


def getInstance(robot):
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    ltm.register_state(sm, ["ltm_demo", "human_session"])
    with sm:
        smach.StateMachine.add(
            'WAIT_HUMAN',
            build_wait_human_sm(robot),
            transitions={'succeeded': 'INTRODUCTION'})

        smach.StateMachine.add(
            'INTRODUCTION',
            build_introduction_sm(robot),
            transitions={'succeeded': 'ANALYZE_FACIAL_FEATURES'})

        smach.StateMachine.add(
            'ANALYZE_FACIAL_FEATURES',
            build_facial_features_sm(robot),
            transitions={'succeeded': 'GOODBYE'})

        smach.StateMachine.add(
            'GOODBYE',
            build_goodbye_sm(robot),
            transitions={'succeeded': 'succeeded'}
        )

        return sm


if __name__ == '__main__':

    try:
        import smach_ros
        from bender_skills import robot_factory

        rospy.init_node('ltm_demo__human_session')
        robot = robot_factory.build(["joy", "tts", "facial_features", "emotion_recognition"], core=False)
        robot.check()

        # build machine
        sm = getInstance(robot)
        ltm.setup(sm)

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
