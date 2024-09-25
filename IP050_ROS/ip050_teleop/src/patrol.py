#!/usr/bin/env python3
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

class ProjectNormal:
# A, B, C, D, E 지점의 좌표와 회전을 저장합니다.
    def __init__(self):
        rospy.init_node('project_normal')

        self.waypoints = [
            {'position': {'x': 0.0, 'y': 0.0}, 'orientation': {'z': 0.0, 'w': 0.0}}
            for _ in range(5)
        ]
        self.get_location()

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
    def get_location(self):
        for i in range(len(self.waypoints)):
            point_list = ['a', 'b', 'c', 'd', 'e']
            point = str(point_list[i])
            path_x = '~/params' + '_' + point + '/goal_position/x'
            path_y = '~/params' + '_' + point + '/goal_position/y'
            path_z = '~/params' + '_' + point + '/goal_position/z'
            path_w = '~/params' + '_' + point + '/goal_position/w'

            self.waypoints[i]['position']['x'] = rospy.get_param(path_x)
            self.waypoints[i]['position']['y'] = rospy.get_param(path_y)
            self.waypoints[i]['orientation']['z'] = rospy.get_param(path_z)
            self.waypoints[i]['orientation']['w'] = rospy.get_param(path_w)

    def move_base_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint['position']['x']
        goal.target_pose.pose.position.y = waypoint['position']['y']
        goal.target_pose.pose.orientation.z = waypoint['orientation']['z']
        goal.target_pose.pose.orientation.w = waypoint['orientation']['w']
        return goal
    
    def move_robot_sequence(self):
        self.clear_costmap()

        point_list = ['A', 'B', 'C', 'D', 'E']
        for i, waypoint in enumerate(self.waypoints):
            goal = self.move_base_goal(waypoint)
            self.client.send_goal(goal)
            self.client.wait_for_result()

            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"{point_list[i]} 목적지에 도착: {waypoint}")
                self.clear_costmap()

            else:
                rospy.logerr(f"{point_list[i]} 목적지에 이동 실패: {waypoint}")
                self.clear_costmap()

    def clear_costmap(self):
        try:
            self.clear_costmaps_service()
            rospy.loginfo("Successfully clear costmap.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        pn = ProjectNormal()
        pn.move_robot_sequence()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
