import sys
import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# print(BASE_DIR)
sys.path.append(BASE_DIR)

from dataclasses import dataclass
from cyclonedds.domain import DomainParticipant, Domain
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.pub import DataWriter
from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct
# from atom.msg import dds_
from rpc.callSwitchUpperLimbControl import JsonRpcClient

import threading
import logging
import subprocess
import re

from atom.msg import dds_

from DDSHelper import *

import numpy as np
import time

import json

logging.basicConfig(
    filename='log.log',
    level=logging.INFO,
    format='%(levelname).1s%(asctime)s %(filename)s:%(lineno)d] %(message)s',
    datefmt='%m%d %H:%M:%S'
)

log = logging.getLogger()


class Control_sim:
    def __init__(self):

        # self.domain = Domain(
        # domain_id=0, config="<CycloneDDS><Domain><General><Interface><NetworkInterface address='192.168.8.234'/></Interface></General></Domain></CycloneDDS>")
        self.participant = DomainParticipant()

        self.low_state_topic = Topic(self.participant, "rt/upper/state", dds_.UpperState_)
        self.low_cmd_topic = Topic(self.participant, "rt/upper/cmd", dds_.UpperCmd_)
        self.low_state_reader = DataReader(self.participant, self.low_state_topic)
        self.low_cmd_writer = DataWriter(self.participant, self.low_cmd_topic)

        self.inspire_state_topic = Topic(self.participant, "rt/hands/state", dds_.HandsState_)
        self.inspire_cmd_topic = Topic(self.participant, "rt/hands/cmd", dds_.HandsCmd_)
        self.inspire_state_reader = DataReader(self.participant, self.inspire_state_topic)
        self.inspire_cmd_writer = DataWriter(self.participant, self.inspire_cmd_topic)

        self.fsm_topic = Topic(self.participant, "set/fsm/id", dds_.SetFsmId_)
        self.fsm_cmd_writer = DataWriter(self.participant, self.fsm_topic)
        self.fsm_reader = DataReader(self.participant, self.fsm_topic)

        self.main_state_topic = Topic(self.participant, "rt/main/nodes/state", dds_.MainNodesState_)
        self.main_state_reader = DataReader(self.participant, self.main_state_topic)


        self.read_state_thread = threading.Thread(target=self.read_upper_state)
        self.read_state_thread.setDaemon(True)
        self.read_lock = threading.Lock()

        self.read_hand_thread = threading.Thread(target=self.read_hand_state)
        self.read_hand_thread.setDaemon(True)
        self.read_hand_lock = threading.Lock()

        self.fsm_msg = None
        self.fsm_read_cmd_thread = threading.Thread(target=self.listen_fsm)
        self.fsm_read_cmd_thread.setDaemon(True)
        self.fsm_cmd_lock = threading.Lock()
        
        self.main_state_msg = None
        self.main_state_thread = threading.Thread(target=self.listen_main_state)
        self.main_state_thread.setDaemon(True)
        self.main_state_lock = threading.Lock()

        self.msg = None
        self.hand_msg = None
        self.cmd_upper_msg = set_upper_state()
        self.cmd_hand_msg = get_hand_cmd_state()
        self.cmd_fsm = get_fsm()
        self.main_state = get_main_state()
        self.send_fsm_cmd()

        self.RPC = JsonRpcClient()

        self.controlParamsFile = BASE_DIR + '/controlParams_P2.json'
        # print("self.controlParamsFile", self.controlParamsFile)
        with open(self.controlParamsFile, 'r') as file:
            self.controlParams = json.load(file)


    def read_upper_state(self):
        for msg in self.low_state_reader.read_iter(timeout=duration(minutes=0.01)):
            with self.read_lock:
                self.msg = msg
                time.sleep(0.001)
                # print(msg)

    def read_hand_state(self):
        for msg in self.inspire_state_reader.read_iter(timeout=duration(minutes=0.01)):
            with self.read_hand_lock:
                self.hand_msg = msg
                time.sleep(0.001)
                # print(msg)

    def listen_fsm(self):
        for msg in self.fsm_reader.take_iter(timeout=duration(minutes=1)):
            with self.fsm_cmd_lock:
                self.fsm_msg = msg
                time.sleep(0.001)

    def listen_main_state(self):
        for msg in self.main_state_reader.take_iter(timeout=duration(minutes=1)):
            with self.main_state_lock:
                self.main_state_msg = msg
                time.sleep(0.001)

    def send_cmd(self, q, q_hand):
        for i in range(17):
            self.cmd_upper_msg.motor_cmd[i].q = q[i]
            self.cmd_upper_msg.motor_cmd[i].dq = 0.
            self.cmd_upper_msg.motor_cmd[i].tau = 0.
            self.cmd_upper_msg.motor_cmd[i].kp = self.controlParams[i]["kp"]
            self.cmd_upper_msg.motor_cmd[i].kd = self.controlParams[i]["kd"]

        for i in range(12):
            self.cmd_hand_msg.hands[i].q = q_hand[i]
            self.cmd_hand_msg.hands[i].dq = 0.
            self.cmd_hand_msg.hands[i].tau = 0.
            self.cmd_hand_msg.hands[i].kp = 10
            self.cmd_hand_msg.hands[i].kd = 0

        self.low_cmd_writer.write(self.cmd_upper_msg)
        self.inspire_cmd_writer.write(self.cmd_hand_msg)

    def send_vel_cmd(self, q, q_hand, dq,send_finger):
        for i in range(17):
            self.cmd_upper_msg.motor_cmd[i].q = q[i]
            self.cmd_upper_msg.motor_cmd[i].dq = dq[i]
            self.cmd_upper_msg.motor_cmd[i].tau = 0.
            self.cmd_upper_msg.motor_cmd[i].kp = self.controlParams[i]["kp"]
            self.cmd_upper_msg.motor_cmd[i].kd = self.controlParams[i]["kd"]
        self.low_cmd_writer.write(self.cmd_upper_msg)

        if send_finger:
            for i in range(12):
                self.cmd_hand_msg.hands[i].q = q_hand[i]
                self.cmd_hand_msg.hands[i].dq = 0.
                self.cmd_hand_msg.hands[i].tau = 0.
                self.cmd_hand_msg.hands[i].kp = 10
                self.cmd_hand_msg.hands[i].kd = 0
            self.inspire_cmd_writer.write(self.cmd_hand_msg)

    def send_fsm_cmd(self, fsm_id=2):
        self.cmd_fsm.id = fsm_id
        self.fsm_cmd_writer.write(self.cmd_fsm)

    def start(self):
        # print('Start simulation!')
        self.fsm_read_cmd_thread.start()
        print("Start dds")
        self.read_state_thread.start()
        self.read_hand_thread.start()
        self.main_state_thread.start()
        time.sleep(1)
        t = 0.
        T_tol = 5
        q_ref = {}
        q_hand = {}



        # while 1:
        #     theta = 0
        #     while t <= T_tol:
        #         t0 = time.time()
        #         t += 0.002
        #         for i in range(17):
        #             q_ref[i] = (theta - self.msg.motor_state[i].q) * t / T_tol + self.msg.motor_state[i].q
        #         for i in range(12):
        #             q_hand[i] = (theta - self.msg.motor_state[i].q) * t / T_tol + self.msg.motor_state[i].q
        #         self.send_cmd(q_ref,q_hand)

        #         t1 = time.time()
        #         simulation_time = t1 - t0
        #         sleep_time = 0.01 - simulation_time
        #         time.sleep(max(sleep_time, 0))


if __name__ == '__main__':
    main = Control_sim()
    # main.RPC.CallSwitchUpperLimbControl(True)
    main.start()
    # example
    print("右臂伺服报错:",main.main_state_msg.left_arm[0].error_code)