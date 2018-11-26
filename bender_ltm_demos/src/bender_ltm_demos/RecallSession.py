#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospy

# robot
from bender_skills import robot_factory

# LTM
from bender_ltm_plugins.msg import HumanEntity
from bender_ltm_plugins.msg import CrowdEntity

# util
from RecallUtils import *


def stream_review(robot):
    """
    - known streams types
    - last recorded stream
    - Episode e1: saved streams
    - random what + stream
    - random when + stream
    - random where + stream
    """
    pass


def episode_review(robot):
    """
    - known tags (heavy)
    - by tag
    - by entity type
    - by entity type and uid
    - by stream type
    """
    print " ======== BY TAG =========="
    tag = "hri_introduction"
    episodes = robot.ltm.get_episodes_by_tag(tag=tag)
    print " == tag: " + tag + " ==="
    print_episodes(episodes)

    tag = "genre_estimation"
    episodes = robot.ltm.get_episodes_by_tag(tag=tag)
    print " == tag: " + tag + " ==="
    print_episodes(episodes)

    print " ======== BY ENTITY TYPE =========="
    episodes = robot.ltm.get_episodes_by_entity_type(entity_type="human")
    print " == entity_type: human ==="
    print_episodes(episodes)

    print " ======== BY ENTITY TYPE AND UID =========="
    human_uids = robot.ltm.query_entity_human("")
    entity_uid = human_uids[0]
    episodes = robot.ltm.get_episodes_by_entity_uid(entity_type="human", entity_uid=entity_uid)
    print " == type: human, entity_uid: " + str(entity_uid) + " ==="
    print_episodes(episodes)

    print " ======== BY STREAM TYPE =========="
    episodes = robot.ltm.get_episodes_by_stream_type(stream_type="images")
    print " == stream_type: images ==="
    print_episodes(episodes)

    print " ======== BY STREAM TYPE AND UID =========="
    images_uids = robot.ltm.query_stream_images("")
    stream_uid = images_uids[0]
    episodes = robot.ltm.get_episodes_by_stream_uid(stream_type="images", stream_uid=stream_uid)
    print " == type: images, stream_uid: " + str(stream_uid) + " ==="
    print_episodes(episodes)


def when_review(robot):
    """
    (What, Where, Entities, learned data)
    - First episode
    - Last episode
    - Middle time episode
    - time span
    """
    pass


def where_review(robot):
    """
    - known places (heavy)
    - by place: list what, when
    """
    episodes = robot.ltm.get_episodes_by_where(location="dining_room", area="inside_map")
    print " == location: dining_room, area: inside_map ==="
    print_episodes(episodes)

    print " == area: inside_map ==="
    episodes = robot.ltm.get_episodes_by_where(area="inside_map")
    print_episodes(episodes)

    print " == area: outside_map ==="
    episodes = robot.ltm.get_episodes_by_where(area="outside_map")
    print_episodes(episodes)


def entity_review(robot):
    """
    - A: Overview of known humans: name, age, genre, phase.
    - B: About H1 - First record: What, When, Where.
    - C: About H1 - Last record: What, When, Where.
    - D: About H1 - Knowledge at middle time: What, When, Where.
    - E: About H1: Field modification: When, value.
    """
    # - - - - - - - - - - - - - - - - - - - - - - - -
    # A.- Overview
    print "\n\n=== Test A ===\n"
    human_uids = robot.ltm.query_entity_human("")
    if not human_uids:
        print "I do not known any human yet."
    humans = robot.ltm.get_human_entities(human_uids)
    print "I remember " + str(len(human_uids)) + " humans."
    print_humans(humans)

    # names
    names = get_attr_lst(humans, "name")
    print names

    # - - - - - - - - - - - - - - - - - - - - - - - -
    # select first human
    a_human = humans[0]
    a_uid = a_human.meta.uid
    print_human(a_human)

    # - - - - - - - - - - - - - - - - - - - - - - - -
    # B.- First Record
    print "\n\n=== Test B ===\n"
    first_stamp = a_human.meta.init_stamp
    print "First record: "
    print_stamped_information(robot, a_uid, first_stamp)

    # - - - - - - - - - - - - - - - - - - - - - - - -
    # C.- Last Record
    print "\n\n=== Test C ===\n"
    last_stamp = a_human.meta.last_stamp
    print "Last record: "
    print_stamped_information(robot, a_uid, last_stamp)

    # - - - - - - - - - - - - - - - - - - - - - - - -
    # C.- Middle knowledge (in time)
    print "\n\n=== Test D ===\n"
    middle_stamp = rospy.Time()
    middle_stamp.secs = (first_stamp.secs + last_stamp.secs) / 2
    print "Middle record: "
    print_stamped_information(robot, a_uid, middle_stamp)

    # - - - - - - - - - - - - - - - - - - - - - - - -
    # E.- When some field was modified and field value
    print "\n\n=== Test E ===\n"
    field_name = "emotion"
    log_uids = robot.ltm.get_human_change_logs(a_uid, field_name)
    if log_uids:
        logs = robot.ltm.get_human_entity_logs(log_uids)
        trails = robot.ltm.get_human_entity_trails(log_uids)
        for log_uid in log_uids:
            log = next(x for x in logs if x.log_uid == log_uid)
            trail = next(x for x in trails if x.meta.log_uid == log_uid)

            when = fmt_stamp(log.timestamp)
            value = getattr(trail, field_name)
            print " - The field '" + field_name + "' was set to '" + str(value) + "' at " + when
    else:
        print " - the '" + field_name + "' field does not exist or has never changed."


def review_runner(label, robot, func):
    print "\n\n\n"
    print " ============================================= "
    print " - " + label.upper()
    print " ============================================= "
    func(robot)


def main():
    # robot = robot_factory.build(["tts", "ltm"], core=False)
    # robot.say("Hello, i am bender")
    # robot.tts.wait_until_done()
    robot = robot_factory.build(["ltm"], core=False)

    # TODO: Armar historia para mostrar    
    review_runner("where", robot, where_review)
    review_runner("when", robot, when_review)
    review_runner("entity", robot, entity_review)
    review_runner("stream", robot, stream_review)
    review_runner("episode", robot, episode_review)


if __name__ == '__main__':
    rospy.init_node('ltm_demo__human_session')
    try:
        main()
    except rospy.ROSInterruptException:
        pass
