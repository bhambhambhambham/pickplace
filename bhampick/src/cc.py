#!/usr/bin/env python3

import rospy
from moveit_commander import PlanningSceneInterface

if __name__ == "__main__":
    rospy.init_node('delete_collision_objects_example')
    scene = PlanningSceneInterface()
    collision_objects = scene.get_known_object_names()
    for object_name in collision_objects:
        scene.remove_world_object(object_name)
    rospy.loginfo("All collision objects have been deleted.")
    rospy.signal_shutdown("Service completed")