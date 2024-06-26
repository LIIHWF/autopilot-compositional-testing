#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#


import cyber_compatibility.exceptions
import cyber_compatibility.executors
import cyber_compatibility.node

from cyber_py3 import cyber, cyber_time


def init(name, args=None):
    cyber.init(name)

def ok():
    return not cyber.is_shutdown()

def shutdown():
    cyber.shutdown()

def waitforshutdown():
    cyber.waitforshutdown()

def get_timestamp(sec=0, nsec=0, from_sec=False):
    total = 0
    if from_sec:
        total = cyber_time.Time(float(sec)).to_nsec()
    else:
        total = cyber_time.Time(int(sec) * 1000000000 + int(nsec))
    secs = total / 1000000000
    nsecs = total - secs * 1000000000
    return {'secs': int(secs), 'nsecs': int(nsecs)}
