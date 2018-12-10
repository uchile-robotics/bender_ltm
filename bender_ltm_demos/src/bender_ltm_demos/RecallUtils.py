#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import time
import rospy


def print_episode(ep):
    print "   - what           : " + str(ep.tags)
    print "   - where.location : " + ep.where.location
    print "   - where.area     : " + ep.where.area
    print "   - when.start     : " + fmt_stamp(ep.when.start)
    print "   - when.end       : " + fmt_stamp(ep.when.end)


def print_episodes(episodes):
    cnt = 1
    for ep in episodes:
        print " - [" + str(cnt) + "]: episode (" + str(ep.uid) + ")"
        print_episode(ep)
        cnt += 1


def print_human(human):
    gender = "unkown gender"
    if human.gender is human.MASCULINE:
        gender = "male"
    elif human.gender is human.FEMININE:
        gender = "female"

    phase = "unkown phase"
    if human.live_phase is human.CHILD:
        phase = "child"
    elif human.live_phase is human.ADULT:
        phase = "adult"

    age = str(human.age_avg) if human.age_avg is not 0 else "unknown age"
    print human.name + " (" + age + ") " + gender + ", " + phase


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
    min_stamp = rospy.Time.now()
    max_stamp = rospy.Time()
    tags = []
    for ep in episodes:
        tags += ep.tags
        tags += ep.children_tags
        min_stamp = ep.when.start if ep.when.start < min_stamp else min_stamp
        max_stamp = ep.when.end if ep.when.end > max_stamp else max_stamp

    tags = list(set(tags))

    episodes = robot.ltm.sort_by_duration(episodes)
    location = "unknown location"
    for ep in episodes:
        location = ep.where.location + " (" + ep.where.area + ")"
    print " - when : "
    print "   - req: " + fmt_stamp(stamp)
    print "   - min: " + fmt_stamp(min_stamp)
    print "   - max: " + fmt_stamp(max_stamp)
    print " - where: " + location
    print " - what : " + str(tags)
