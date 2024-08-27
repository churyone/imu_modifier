#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def get_plan(start, goal):
    rospy.wait_for_service('/move_base/NavfnROS/make_plan')
    try:
        make_plan = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
        plan = make_plan(start, goal)
        return plan.plan
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

if __name__ == "__main__":
    rospy.init_node('navfn_plan_only')
    
    start = PoseStamped()
    start.header.frame_id = "map"
    start.pose.position.x = 1.0
    start.pose.position.y = 1.0
    start.pose.orientation.w = 1.0
    
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = 5.0
    goal.pose.position.y = 5.0
    goal.pose.orientation.w = 1.0

    path = get_plan(start, goal)
    if path:
        rospy.loginfo("Path found with %d poses", len(path.poses))
    else:
        rospy.logwarn("No path found")
