#!/usr/bin/env python

import rospy
import smach
import smach_ros

#define class Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['out1', 'out2'],
                             input_keys=['foo_count_in'],
                             output_keys=['foo_count_out'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if userdata.foo_count_in < 3:
            userdata.foo_count_out = userdata.foo_count_in + 1
            return 'out1'
        else:
            return 'out2'

class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['out1'],
                             input_keys=['bar_count_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR %s', userdata.bar_count_in)
        return 'out1'

def main():
    rospy.init_node('ssm_example')
    print 'Starting node'
    #create a state machine
    sm = smach.StateMachine(outcomes=['out4'])
    sm.userdata.sm_data = 0

    #open the container
    with sm:
        #add the states to the container
        smach.StateMachine.add('FOO', Foo(),
                               transitions={'out1':'BAR', 'out2':'out4'},
                               remapping={'foo_count_in':'sm_data',
                                          'foo_count_out':'sm_data'})
        smach.StateMachine.add('BAR', Bar(),
                               transitions={'out1':'FOO'},
                               remapping={'bar_count_in':'sm_data'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()
