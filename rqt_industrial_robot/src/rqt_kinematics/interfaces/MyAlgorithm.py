import threading
import numpy


class Algorithm:
    def __init__(self):
        self.is_on = False

    def set_pick_and_place(self, pick_place):
        self.pick_place = pick_place

    def myalgorithm(self, stopevent, pauseevent):
        self.pick_place.back_to_home()

        # insert following two lines where you want to stop the algorithm 
        # with the stop button in GUI
        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        # pick blue ball
        object_name = "blue_ball"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)

        grasp = self.pick_place.generate_grasp(object_name, "vertical", pose.position, 0.27, length=0.145)
        self.pick_place.pickup(object_name, [grasp])

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        goal_position = self.pick_place.get_target_position("blue_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        # pick yellow box
        object_name = "yellow_box"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)

        grasp = self.pick_place.generate_grasp(object_name, "vertical", pose.position, 0.55, yaw = 90, length=0.16)
        self.pick_place.pickup(object_name, [grasp])

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        goal_position = self.pick_place.get_target_position("yellow_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        # pick red box
        object_name = "red_box"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)

        grasp = self.pick_place.generate_grasp(object_name, "vertical", pose.position, length=0.16)
        self.pick_place.pickup(object_name, [grasp])

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        goal_position = self.pick_place.get_target_position("red_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        # pick yellow ball
        object_name = "yellow_ball"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)

        grasp = self.pick_place.generate_grasp(object_name, "vertical", pose.position, 0.55, length = 0.155)
        self.pick_place.pickup(object_name, [grasp])

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        goal_position = self.pick_place.get_target_position("yellow_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        # pick green cylinder
        object_name = "green_cylinder"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.z += 0.02

        grasp = self.pick_place.generate_grasp(object_name, "horizontal", pose.position, 0.3, length = 0.13)
        self.pick_place.pickup(object_name, [grasp])

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        goal_position = self.pick_place.get_target_position("green_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        # pick red cylinder
        object_name = "red_cylinder"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.z += 0.01

        grasp = self.pick_place.generate_grasp(object_name, "horizontal", pose.position, 0.45, length=0.14)
        self.pick_place.pickup(object_name, [grasp])

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        goal_position = self.pick_place.get_target_position("red_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        # pick blue box
        object_name = "blue_box"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.x -= 0.02

        grasp = self.pick_place.generate_grasp(object_name, "horizontal", pose.position, 0.4, pitch = 60, length = 0.16)
        self.pick_place.pickup(object_name, [grasp])

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        goal_position = self.pick_place.get_target_position("blue_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()

        # pick green ball
        object_name = "green_ball"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)

        grasp = self.pick_place.generate_grasp(object_name, "horizontal", pose.position, 0.37, pitch = 80, length=0.145)
        self.pick_place.pickup(object_name, [grasp])

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        goal_position = self.pick_place.get_target_position("green_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()

        self.pick_place.send_message("Algorithm finished")