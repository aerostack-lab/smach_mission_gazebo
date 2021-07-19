#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import threading

from smach_ros import ServiceState
from smach_ros import MonitorState

import behavior_execution_manager_msgs.srv as bsrv
from behavior_execution_manager_msgs.msg import BehaviorActivationFinished 

from SmachStates import * 

            
def call_service(service_name,service_spec,service_request):
        proxy=None
        response=None
        while proxy is None:
            if rospy.is_shutdown():
                return None
            try:
                rospy.wait_for_service(service_name,1.0)
                proxy = rospy.ServiceProxy(service_name, service_spec)
            except rospy.ROSException as ex:
                    rospy.logwarn("Still waiting for service '%s'..." % service_name)

        #Call service
        try:
            rospy.logwarn("Calling service %s with request:\n%s" % (service_name, str(service_request)))
            response = proxy(service_request)
        except rospy.ServiceException as ex:
            rospy.logerr("Exception when calling service '%s': %s" % (self._service_name, str(ex)))
            return None

        return response

class EXECUTING_TAKE_OFF(smach.State):

    def __init__(self):
        smach.State.__init__(self,outcomes=["take_off_completed"])
        self._trigger_event = threading.Event()


    def execute(self,ud):
        # declare subscriber
        self._sub = rospy.Subscriber("/drone111/behavior_activation_finished", BehaviorActivationFinished, self._cb)
        
        call_service("/drone111/quadrotor_motion_with_pid_control/behavior_take_off_with_pid/activate_behavior",
                    bsrv.ActivateBehavior,
                    bsrv.ActivateBehaviorRequest("",1000))
        
        # once you have called the service, wait until take_off is completed
        
        self._trigger_event.clear()
        
        # wait until msg has been received
        self._trigger_event.wait()
        
        self._sub.unregister()

        return "take_off_completed"

    def _cb(self,msg):
        # set true the varible -> if you were blocked, you can continue now
        self._trigger_event.set()


    


class EXECUTING_FOLLOW_PATH(smach.State):

    def __init__(self):
        smach.State.__init__(self,outcomes=["follow_path_completed"])
        self._trigger_event = threading.Event()


    def execute(self,ud):
        # declare subscriber
        self._sub = rospy.Subscriber("/drone111/behavior_activation_finished", BehaviorActivationFinished, self._cb)
        
        call_service("/drone111/quadrotor_motion_with_pid_control/behavior_follow_path/activate_behavior",
                    bsrv.ActivateBehavior,
                    bsrv.ActivateBehaviorRequest("path: [[1,0,1],[1,1,1],[0,1,1],[0,0,1]]",1000))
        
        # once you have called the service, wait until take_off is completed
        
        self._trigger_event.clear()
        
        # wait until msg has been received
        self._trigger_event.wait()
        
        self._sub.unregister()

        return "follow_path_completed"

    def _cb(self,msg):
        # set true the varible -> if you were blocked, you can continue now
        self._trigger_event.set()


class EXECUTING_LAND(smach.State):

    def __init__(self):
        smach.State.__init__(self,outcomes=["land_completed"])
        self._trigger_event = threading.Event()


    def execute(self,ud):
        # declare subscriber
        self._sub = rospy.Subscriber("/drone111/behavior_activation_finished", BehaviorActivationFinished, self._cb)
        
        call_service("/drone111/quadrotor_motion_with_pid_control/behavior_land_with_pid/activate_behavior",
                                    bsrv.ActivateBehavior,
                                    bsrv.ActivateBehaviorRequest("",1000))
        
        # once you have called the service, wait until take_off is completed
        
        self._trigger_event.clear()
        
        # wait until msg has been received
        self._trigger_event.wait()
        
        self._sub.unregister()

        return "land_completed"

    def _cb(self,msg):
        # set true the varible -> if you were blocked, you can continue now
        self._trigger_event.set()





def main():
    rospy.init_node('smach_example_state_machine')
    
    sm=smach.StateMachine(outcomes=['succeeded','aborted','preempted'])


    with sm:
                
        
            smach.StateMachine.add("ACTIVATING_SELF_LOCALIZE",ACTIVATING_SELF_LOCALIZE(),
                                    transitions={"self_localize_activated":"ACTIVATING_THRUST_CONTROL"})
            smach.StateMachine.add("ACTIVATING_THRUST_CONTROL",ACTIVATING_THRUST_CONTROL(),
                                    transitions={"thrust_control_activated":"ACTIVATING_PID_MOTION_CONTROL"})
            smach.StateMachine.add("ACTIVATING_PID_MOTION_CONTROL",ACTIVATING_PID_MOTION_CONTROL(),
                                    transitions={"pid_motion_control_activated":"EXECUTING_TAKE_OFF"})
            smach.StateMachine.add("EXECUTING_TAKE_OFF", EXECUTING_TAKE_OFF(),
                                    transitions={"take_off_completed":"EXECUTING_FOLLOW_PATH"})
	    smach.StateMachine.add("EXECUTING_FOLLOW_PATH",EXECUTING_FOLLOW_PATH(),
                                    transitions={"follow_path_completed":"EXECUTING_LAND"})
	    smach.StateMachine.add("EXECUTING_LAND", EXECUTING_LAND(),
                                    transitions={"land_completed":"DEACTIVATING_PID_CONTROL"})
            smach.StateMachine.add("DEACTIVATING_PID_CONTROL",DEACTIVATING_PID_CONTROL(),
                                    transitions={"pid_motion_control_deactivated":"DEACTIVATING_THRUST_CONTROL"})
            smach.StateMachine.add("DEACTIVATING_THRUST_CONTROL",DEACTIVATING_THRUST_CONTROL(),
                                    transitions={"thrust_control_deactivated":"DEACTIVATING_SELF_LOCALIZE"})
            smach.StateMachine.add("DEACTIVATING_SELF_LOCALIZE", DEACTIVATING_SELF_LOCALIZE(),
                                    transitions={'self_localize_deactivated':'succeeded'})
        
        
       	    """
		smach.StateMachine.add("TAKE_OFF",take_off_machine,
                                    transitions={"succeeded":"EXECUTING_FOLLOW_PATH"})

        
        


        	land_machine= smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

	        with land_machine:


            
        
        
	        smach.StateMachine.add("LAND",land_machine,
                                    transitions={"succeeded":"succeeded"})
	
  	   """
    

    sis = smach_ros.IntrospectionServer('smach_viewer', sm, '/SM_ROOT')
    
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
