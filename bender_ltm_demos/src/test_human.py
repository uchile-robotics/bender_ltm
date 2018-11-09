#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospkg
import rospy

from ltm.srv import *
from bender_ltm_plugins.msg import HumanEntity
from bender_ltm_plugins.srv import HumanEntitySrv, HumanEntitySrvRequest


class EntityInterface(object):

    def __init__(self):
        self.get_client = rospy.ServiceProxy('/bender/ltm/entity/human/get', HumanEntitySrv)
        self.query_client = rospy.ServiceProxy('/bender/ltm/db/query', QueryServer)

    def setup(self):
        self.get_client.wait_for_service()
        self.query_client.wait_for_service()

    def get(self, uids, stamps=None):
        req = HumanEntitySrvRequest()
        req.uids = uids
        if stamps:
            req.stamps = stamps
        resp = self.get_client(req)
        return resp

    def look_for(self, uid, stamp=None):
        stamps = None
        if stamp:
            stamps = [stamp]
        resp = self.get([uid], stamps)
        if len(resp.msgs) is not 1:
            print "ERROR: Human with uid: " + str(uid) + " not found."
            human = HumanEntity()
        else:
            human = resp.msgs[0]
            human.face = None
        return human

    def query(self, json):
        query = QueryServerRequest()
        query.target = "entity"
        query.semantic_type = "human"
        query.json = json
        resp = self.query_client(query)
        return resp

    def query_log(self, json):
        query = QueryServerRequest()
        query.target = "entity_trail"
        query.semantic_type = "human"
        query.json = json
        resp = self.query_client(query)
        print resp
        return resp.entities[0].uids, resp.entities_trail[0].uids


def show(name, uid, secs, nsecs, fields=[]):
    it = EntityInterface()
    t = rospy.Time(secs, nsecs)
    human = it.look_for(uid, t)

    print("======================================================================")
    print(name + ", stamp: " + str(secs) + ", log_uid:" + str(human.meta.log_uid) + ", human: " + human.name)
    print("----------------------------------------------------------------------")
    if not fields:
        print(human)
    else:
        for f in fields:
            print(f + ": " + str(getattr(human, f)))
    print("")


if __name__ == '__main__':
    rospy.init_node('test_ltm_human_entities')

    it = EntityInterface()
    it.setup()

    # ALL PEOPLE
    it.query('{}')

    static_f = ['name', 'age_bottom', 'age_top', 'age_avg', 'live_phase', 'genre', 'emotion', 'last_seen']

    uid = 1312696024
    show("COMPLETE", uid, 0, 0)
    show("INIT MSG", uid, 9806, 0, static_f)
    show("INIT MSG", uid, 9813, 0, static_f)
    show("INIT MSG", uid, 9815, 0, static_f)

    # it.query_log('{$query: {entity_uid: 2, timestamp: { $lte: 1540684542 }}, $orderby: { timestamp: -1}}')
