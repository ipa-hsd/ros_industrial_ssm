#!/usr/bin/env python

import rospy
import smach
import smach_ros
from airbus_ssm_core import ssm_state

#define class Foo
class Foo(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,
                             outcomes=['out1', 'out2'], io_keys=['counter'])

    def execution(self, ud):
        rospy.loginfo('Executing state FOO')

        if int(ud.counter) < 3:
            ud.counter = int(ud.counter) + 1
            return 'out1'
        else:
            return 'out2'

class Bar(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,
                             outcomes=['out1'], io_keys=['counter'])

    def execution(self, ud):
        rospy.loginfo('Executing state BAR %i', ud.counter)
        return 'out1'

