#!/usr/bin/env python

import rospy
import smach
import smach_ros

#define class Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1', 'out2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'out1'
        else:
            return 'out2'

class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'out1'

def main():
    rospy.init_node('ssm_example')
    print 'Starting node'
    #create a state machine
    sm = smach.StateMachine(outcomes=['out4'])

    #open the container
    with sm:
        #add the states to the container
        smach.StateMachine.add('FOO', Foo(),
                               transitions={'out1':'BAR', 'out2':'out4'})
        smach.StateMachine.add('BAR', Bar(),
                               transitions={'out1':'FOO'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()
