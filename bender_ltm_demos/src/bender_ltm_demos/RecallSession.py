#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import time
import rospy

# robot
from bender_skills import robot_factory

# LTM
from bender_ltm_plugins.msg import HumanEntity
from bender_ltm_plugins.msg import CrowdEntity


def stream_review(robot):
    """
    - Lugar X, Fecha X: Streams
    - TAG X: Stream
    """
    pass


def episode_review(robot):
    """
    - Lugar X: Episodios
    - Fecha X: Episodios
    """
    pass


def when_review(robot):
    """
    - que pasó
    - donde estuvo
    - que aprendió durante el intervalo de tiempo X sobre humanos
    """
    pass


def where_review(robot):
    """
    - última fecha
    - primera vez
    - fecha particular
    """
    pass


def aggregation_review(robot):
    """
    - TEMPORAL
    - LUGAR
    - WHAT
    - ENTIDAD
    """
    pass


def print_human(human):
    genre = "unkown genre"
    if human.genre is human.MASCULINE:
        genre = "male"
    elif human.genre is human.FEMININE:
        genre = "female"

    phase = "unkown phase"
    if human.live_phase is human.CHILD:
        phase = "child"
    elif human.live_phase is human.ADULT:
        phase = "adult"

    age = str(human.age_avg) if human.age_avg is not 0 else "unknown age"
    print human.name + " (" + age + ") " + genre + ", " + phase


def print_humans(humans):
    print "---"
    for human in humans:
        print_human(human)
    print "---"


def get_attr_lst(lst, name):
    return map(lambda x: getattr(x, name), lst)


def fmt_stamp(stamp):
    return time.strftime("%a, %d %b %Y %H:%M:%S %Z", time.localtime(stamp.secs))


def print_stamped_information(robot, uid, stamp):
    human = robot.ltm.get_human_entity(uid, stamp)
    print_human(human)

    episodes = robot.ltm.get_episodes_by_stamp(stamp)
    episodes = robot.ltm.filter_children(episodes)
    tags = []
    for ep in episodes:
        tags += ep.tags
        tags += ep.children_tags
    tags = list(set(tags))

    episodes = robot.ltm.sort_by_duration(episodes)
    location = "unknown location"
    for ep in episodes:
        location = ep.where.location + " (" + ep.where.area + ")"
    print " - what : " + str(tags)
    print " - when : " + fmt_stamp(stamp)
    print " - where: " + location


def entity_review(robot):
    """
    - A: nombres de humanos que conoce
    - B: primera vez que lo vió (donde y cuando)
    - C: última vez que lo vió y (datos, donde y cuando)
    - D: conocimiento sobre él, en el intante X (cuando, donde, datos)
    """
    # known humans
    human_uids = robot.ltm.query_entity_human("")
    if not human_uids:
        print "I do not known any human yet."
    humans = robot.ltm.get_human_entities(human_uids)
    print "I remember " + str(len(human_uids)) + " humans."
    print_humans(humans)

    # names
    names = get_attr_lst(humans, "name")
    print names

    # select first human
    a_human = humans[0]
    a_uid = a_human.meta.uid
    print_human(a_human)

    # Episodic Information: First Time, WHAT, WHERE, WHEN
    first_stamp = a_human.meta.init_stamp
    print "\nFirst record: "
    print_stamped_information(robot, a_uid, first_stamp)

    # Episodic Information: Last Time, WHAT, WHERE, WHEN
    last_stamp = a_human.meta.last_stamp
    print "\nLast record: "
    print_stamped_information(robot, a_uid, last_stamp)

    # When some field was modified and field value
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


def main():
    # robot = robot_factory.build(["tts", "ltm"], core=False)
    # robot.say("Hello, i am bender")
    # robot.tts.wait_until_done()
    robot = robot_factory.build(["ltm"], core=False)

    # TODO: Armar historia para mostrar
    entity_review(robot)


if __name__ == '__main__':
    rospy.init_node('ltm_demo__human_session')
    try:
        main()
    except rospy.ROSInterruptException:
        pass
