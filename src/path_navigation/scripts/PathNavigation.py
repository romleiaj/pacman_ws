#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from math import pi
import numpy as np
import tf  # transform?
import pdb
from OdomReader import OdomReader


class PathNavigation():
    def __init__(self, cmd_vel_topic, odom_topic):
        """
        init
        """
        self.PAUSE = 1  # How long to move
        self.SMALL_WAIT = 0.01
        self.SMALL_MOVE = 0.03
        # The distance considered sufficiently close to the goal position
        self.MOVE_THRESHOLD = 0.1
        self.TURN_THRESHOLD = 4
        self.RAMP_DOWN = .1  # what distance to start moving slower
        # how much of the move to complete prior to checking the orientation
        self.MOVE_AND_CHECK = 0.33

        self.TURN_DISTANCE = 0.25
        self.SPEED = 0.02
        self.ANGULAR_VELOCITY = 1

        self.FORWARD_THRESHOLD = .25  # the distance away from a wall to trigger hitwall
        
        # Twist constant for rotating about z
        self.LEFT = Twist(Vector3(0, 0, 0), Vector3(
            0, 0, self.ANGULAR_VELOCITY))
        self.RIGHT = Twist(Vector3(0, 0, 0), Vector3(
            0, 0, -self.ANGULAR_VELOCITY))
        self.FORWARD = Twist(Vector3(self.SPEED, 0, 0), Vector3(0, 0, 0))
        self.BACKWARD = Twist(
            Vector3(-1.0 * self.SPEED, 0, 0), Vector3(0, 0, 0))
        self.orientation = 0  # in radians, [-pi, pi]
        self.STOP = Twist()
        rospy.init_node("PathNavigation")  # this is required for any ros thing to work
        self.pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.pub.publish(self.RIGHT)  # the first time rarely works
        self.odom_reader = OdomReader(odom_topic)
        rospy.sleep(1)

    def get_orientation(self):
        init_loc = self.get_location()
        self.move("forward", self.SMALL_MOVE)
        final_loc = self.get_location()
        self.move("backward", self.SMALL_MOVE)
        movement = np.asarray(final_loc) - np.asarray(init_loc)
        # the value in rads, [-pi, pi]
        self.orientation = np.arctan2(movement[1], movement[0])
        return self.orientation

    def get_location(self):
        """
        return the (x, y) location as a list
        """
        return [self.odom_reader.get_x(), self.odom_reader.get_y()]

    def turn(self, direction='left', distance=90):
        """
        turn the spherio robot in place

        args
        ----------
        direction : str
            The direction to turn, either 'left' or 'right'

        returns
        ----------
        none

        TODO
        ----------
        Figure out the units of the distance so it rotates right angles
        """
        if distance is None:  # implies the function was called with one or zero arguements
            distance = self.TURN_DISTANCE  # this is the default value

        if direction == 'left':
            self.pub.publish(self.LEFT)
        elif direction == 'right':
            self.pub.publish(self.RIGHT)
        else:
            # this could be an error
            print("Error: rotate was expected to be 'left' or 'right' but was {}".format(
                direction))

        initial_yaw = self.odom_reader.get_yaw()  # get the starting rotation
        print("in turn the initial yaw is {}".format(initial_yaw))

        new_yaw = self.odom_reader.get_yaw()
        while(abs(initial_yaw - new_yaw) < distance):  # check if it's moved enough
            rospy.sleep(self.SMALL_WAIT)  # Maybe not needed
            new_yaw = self.odom_reader.get_yaw()  # get the new rotation
            #print("requested {}, difference {}".format(distance, abs(new_yaw - initial_yaw)))
        self.pub.publish(self.STOP)  # stop the rotation

    def move(self, direction, distance=0.5):
        """
        Move foward or backward
        args
        ----------
        direction : str
            either 'forward', or 'backward'

        distance : float
            the distance in meters to move

        returns
        ----------
        success : Bool
            did the move execute without hitting a wall

        TODOgit config --global user.name
        ----------
        Make the backtracking closed loop
        """
        if direction == 'forward':
            self.pub.publish(self.ramp_down(self.FORWARD, distance))
        elif direction == 'backward':
            self.pub.publish(self.ramp_down(self.BACKWARD, distance))
        elif direction == 'left':
            raise NotImplementedError()
            self.pub.publish(self.LEFT)
        elif direction == 'right':
            raise NotImplementedError()
            self.pub.publish(self.RIGHT)
        else:
            # this could be an error
            print("Error: rotate was expected to be 'foward' or 'backward' but was {}".format(
                direction))

        # here again there is going to be a loop which runs until it has moved the requesite distance
        # get the starting rotation
        initial_loc = np.asarray(self.get_location())
        #print("in move the initial location is {}".format(initial_loc))
        new_loc = self.get_location()

        num_steps = 0  # The number of times we've checked the distances

        while(np.linalg.norm(initial_loc - new_loc) < distance):  # check if it's moved enough
            rospy.sleep(self.SMALL_WAIT)  # Maybe not needed
            if num_steps < 0:
                print("backtracked to approximately starting location")
                self.pub.publish(self.STOP)
                return False  # the action wasn't executed successfully
            num_steps -= 1  # go back the same number of steps, only an aproximation

            new_loc = self.get_location()  # get the new rotation
        movement = initial_loc - new_loc
        self.orientation = np.arctan2(movement[1], movement[0])
        self.pub.publish(self.STOP)  # make sure it stops at the end
        return True  # successfully completed the move

    def go_to_point(self, goal_location, direction="forward"):
        """
        Move to an x, y location in world space

        args
        ----------
        location : ArrayLike
            The x, y location in world space as an indexable object

        return
        ----------
        success : Bool
            Whether it hit a wall on the last move
        """

        loop_iters = 0
        initial_loc = self.get_location()

        while True:  # will break out
            current_loc = self.get_location()
            goal_direction = np.asarray(
                goal_location) - np.asarray(current_loc)
            goal_dist = self.vec_magnitude(goal_direction)
            if goal_dist < self.MOVE_THRESHOLD:
                break  # leave the loop
            goal_angle = self.vec_to_angle(goal_direction)
            self.turn_to_angle(goal_angle)
            # move halfway to the goal
            success = self.move(direction, goal_dist * self.MOVE_AND_CHECK)
            if not success:
                self.go_to_point(initial_loc)
                return False  # success

        return True  # success

    def ramp_down(self, t, distance):
        """
        scale the speed down if the distance it has to travel is slow
        """
        if distance < self.RAMP_DOWN:
            assert type(self.RAMP_DOWN) == float
            sf = distance / self.RAMP_DOWN  # scale factor
            linear = Vector3(t.linear.x * sf, t.linear.y * sf, t.linear.z * sf)
            angular = Vector3(t.angular.x * sf,
                              t.angular.y * sf, t.angular.z * sf)
            twist = Twist(linear, angular)
            return twist
        else:
            return t

    def vec_to_angle(self, direction_vector):
        """
        returns as degrees
        """
        return np.arctan2(direction_vector[1], direction_vector[0]) / (2*pi)*360

    def vec_magnitude(self, direction_vector):
        """
        Get the magnitude of an ArrayLike vector
        """
        return np.linalg.norm(direction_vector)

    def turn_to_angle(self, goal_angle):
        """
        Turn a specified angle in the shorter direction

        args
        ----------
        angle : float
            This direction can be unbounded but needs to be in degrees
            It represents the absolute direction in degrees
        """

        goal_angle = goal_angle % 360  # convert to [0, 360]
        if goal_angle > 180:
            goal_angle -= 360  # put in into the range [-180, 180]

        turn_angle = self.shortest_angle(goal_angle, self.get_orientation())
        #print("amount to turn {}".format(turn_angle))

        if turn_angle < 0:  # which turning direction is quicker
            self.pub.publish(self.LEFT)
        else:
            self.pub.publish(self.RIGHT)

        while True:
            current_orientation = self.get_orientation()
            #print("angular error in turn to angle is {}".format(goal_angle - current_orientation))
            rospy.sleep(self.SMALL_WAIT)  # Maybe not needed
            if np.abs(goal_angle - current_orientation) < self.TURN_THRESHOLD:
                break  # mimic do while
        self.pub.publish(self.STOP)  # stop the rotation

    def shortest_angle(self, angle1, angle2):
        """
        https://stackoverflow.com/questions/28036652/finding-the-shortest-distance-between-two-angles/28037434
        """
        diff = (angle2 - angle1 + 180) % 360 - 180
        return diff + 360 if diff < -180 else diff


if __name__ == "__main__":
    
    cmd_vel_topic = rospy.get_param('cmd_vel_topic', 'pacman_equiv/cmd_vel')
    odom_topic = rospy.get_param('odom_topic', 'pacman_equiv/odom')
    print(cmd_vel_topic)

    mover = PathNavigation(cmd_vel_topic, odom_topic)
    mover.go_to_point([0.755172, -0.013588])
    mover.go_to_point([0.755172, -0.013588])
    mover.go_to_point([2.012774, -0.070005])
    mover.go_to_point([4.350890, -0.339451])
    mover.go_to_point([4.098817, -1.434626])
    mover.go_to_point([3.490398, -3.885729])
    mover.go_to_point([3.420172, -5.579248])
    mover.go_to_point([3.863448, -7.039472])
    mover.go_to_point([4.524018, -8.021641])
    mover.go_to_point([5.680047, -8.769141])
    mover.go_to_point([7.505334, -8.786546])
    mover.go_to_point([8.774353, -7.700043])
    mover.go_to_point([9.617440, -6.335436])
    mover.go_to_point([9.878201, -5.057729])
    mover.go_to_point([9.678280, -3.536668])
    mover.go_to_point([9.000335, -2.467563])
    mover.go_to_point([7.696537, -1.350658])
    mover.go_to_point([6.001646, -0.672696])
    mover.go_to_point([4.332815, -0.290254])
    mover.go_to_point([4.515331, 0.804919])
    mover.go_to_point([4.715252, 2.282535])
    mover.go_to_point([4.567483, 3.690624])
    mover.go_to_point([3.950347, 4.490277])
    mover.go_to_point([2.837802, 4.942240])
    mover.go_to_point([0.491005, 5.107382])
    mover.go_to_point([-2.142627, 5.950500])
    mover.go_to_point([-3.194343, 6.611070])
    mover.go_to_point([-4.246068, 7.488265])
    mover.go_to_point([-6.106102, 9.096958])
    mover.go_to_point([-3.072665, 6.524170])
    mover.go_to_point([-1.421202, 5.724518])
    mover.go_to_point([-0.395565, 5.011765])
    mover.go_to_point([-2.429456, 4.690167])
    mover.go_to_point([-4.585040, 4.046970])
    mover.go_to_point([-6.914442, 3.021342])
    mover.go_to_point([-9.209095, 1.309050])
    mover.go_to_point([-10.425963, -0.090338])
    mover.go_to_point([-11.086534, -1.315889])
    mover.go_to_point([-11.521123, -3.106407])
    mover.go_to_point([-11.251677, -5.235918])
    mover.go_to_point([-10.478114, -7.261125])
    mover.go_to_point([-9.200408, -8.895178])
    mover.go_to_point([-7.601103, -9.964276])
    mover.go_to_point([-5.593270, -10.459704])
    mover.go_to_point([-3.515941, -10.311967])
    mover.go_to_point([-1.812329, -9.686143])
    mover.go_to_point([-0.308641, -8.695256])
    mover.go_to_point([0.543160, -7.730462])
    mover.go_to_point([0.812607, -6.687454])
    mover.go_to_point([0.343249, -5.861741])
    mover.go_to_point([-0.847534, -4.983845])
    mover.go_to_point([-3.003112, -4.288496])
    mover.go_to_point([-5.601987, -4.105980])
    mover.go_to_point([-7.236040, -3.471470])
    mover.go_to_point([-8.174774, -2.463213])
    mover.go_to_point([-8.461595, -1.359348])
    mover.go_to_point([-8.148683, -0.281560])
    mover.go_to_point([-6.757986, 0.552859])
    mover.go_to_point([-5.254298, 0.787537])
    mover.go_to_point([-3.255182, 0.344254])
    mover.go_to_point([-1.638494, 0.005272])
    mover.go_to_point([0.012957, -0.012112])
