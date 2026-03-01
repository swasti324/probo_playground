"""
A simulation environment for a mobile robot operating in two dimensions.

The Environment class models the world that the robots navigate in. The world is continuous and two-dimensional. The world possesses an outer border, internal obstacles, and identifiable landmarks. The world also manages the passage of time and the motion of robotic agents within the world over time.

Critically, the environment tracks the robot's state. In this case, the robot's state is a vector that includes three state variables: x position, y position, and heading.
"""

from utils import Position, Pose, Bounds, Landmark, BearingRange
import pandas as pd
import math

class Environment:
    """
    A class that models the world simulation environment and the robot's state.

    Attributes:
        dimensions: the horizontal and vertical size of the world
        dt: the length of each timestep, in seconds
        obstacles: a list of obstacles
        landmarks: a list of landmarks
        robot_pose: the position and heading of the robot in the world
    """

    def __init__(
        self,
        dimensions: Bounds,
        dt: float,
        obstacles: list[Bounds],
        landmarks: list[Landmark],
        robot_starting_pose: Pose,
        lm_range: float = 10.0,

    ):
        """
        Initialize an instance of the Environment class.

        Args:
            dimensions: the horizontal and vertical size of the world
            dt: the length of each timestep, in seconds
            obstacles: a list of obstacles
            landmarks: a list of landmarks
            robot_starting_pose: the initial position and heading of the robot
        """
        # set the dimensions property to the parameter value
        self.DIMENSIONS = dimensions

        # set the timestep size property to the parameter value
        self.DT = dt

        # set the current time to zero
        self.time = 0

        # set the obstacles and landmarks properties to the parameter lists
        self.OBSTACLES = obstacles
        self.LANDMARKS = landmarks

        # set the robot pose property to the parameter value
        assert dimensions.within_bounds(robot_starting_pose.pos)
        self.robot_pose = robot_starting_pose
        
        for o in obstacles:
            assert dimensions.within_x(o.x_min)
            assert dimensions.within_x(o.x_max)
            assert dimensions.within_y(o.y_min)
            assert dimensions.within_y(o.y_max)
    def robot_step(self, dx: float, dy: float, dtheta: float):
        """
        Update the robot's position and heading in the world. The robot should not be able to pass through obstacles or outside of the world bounds.

        Args:
            dx: change in x position
            dy: change in y position
            dtheta: change in heading

        Returns:
            Nothing, but update the robot_pose property at the end
        """

        updated_pos = self.is_valid_position(dx, dy)
        updated_theta = self.robot_pose.theta + dtheta
        # math for updated theta turning it into radians?? want to look into this more
        updated_theta = (updated_theta + math.pi) % (2 * math.pi) - math.pi

        #step and update
        self.time = round(self.time + self.DT, 3)
        self.robot_pose = Pose(updated_pos, updated_theta)
        

    def is_valid_motion(self, dx: float, dy: float):
        """
        Given attempted x and y motion by the robot, determine what motion is physically possible (i.e. doesn't go through any obstacles or barriers). Return the actual motion that will be executed.

        Args:
            dx: attempted change in x position
            dy: attempted change in y position

        Returns:
            dx: change in x position that should be executed
            dy: change in y position that should be executed
        """
        
        x_new = self.robot_pose.x
        if self.is_valid_position(
            Position(self.robot_pose.pos.x + dx, self.robot_pose.pos.y)
        ):
            x_new = self.robot_pose.pos.x + dx
        
        y_new = self.robot_pose.y
        if self.is_valid_position(
            Position(x_new, self.robot_pose.pos.y + dy)
        ):
            y_new = self.robot_pose.pos.y + dy

        return Position(x_new, y_new)
    def is_valid_position(self, position: Position):
        """
        Check if a given robot position is valid; i.e. not out-of-bounds or within an obstacle. Return a boolean representing whether or not this condition is true.

        Args:
            position: the robot position

        Returns:
            true if the position is valid and false otherwise
        """
        
        if self.DIMENSIONS.within_bounds(position):
            result = True
            for o in self.OBSTACLES:
                result = result and not o.within_bounds(position)
            return result
        return False

    def get_robot_pose(self):
        """
        Return the true robot pose.
        """
        return self.robot_pose

    def get_proximity_to_landmarks(self):
        """
        Return a list of the robot's true range and bearing to all landmarks.
        """
        measurements = pd.DataFrame()
        for l in self.LANDMARKS:
            x_diff = l.pos.x - self.robot_pose.pos.x
            y_diff = l.pos.y = self.robot_pose.pos.y
            #some math i want to understand why we are using here
            range = math.sqrt(x_diff**2 + y_diff**2)
            bearing = math.atan2(y_diff, x_diff) - self.robot_pose.theta
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
            measurements[f"Landmark{l.id}"] = [BearingRange(bearing, range)]
        return measurements
    def take_state_snapshot(self):
        """
        Return true state information about this timestep, including time, robot position, and the robot's bearing/range to landmarks, in a table format.
        """
        df1 = pd.DataFrame(
            {
                "Time": [self.time],
                "RobotPose": [self.robot_pose],
            }
        )
        # landmark hell
        gt_to_lms = self.get_proximity_to_landmarks()
        df2 = pd.DataFrame()
        for lm in gt_to_lms.columns:
            df2[lm] = [gt_to_lms[lm].values[0]]

        # combine and return
        return pd.merge(
            df1,
            df2,
            left_index=True,
            right_index=True,
        )

    def get_environment_info(self):
        """
        Return static information about the environment, including dimensions, timestep size, locations and dimensions of obstacles, and locations of landmarks.
        """
        return {
            "Dimensions": self.DIMENSIONS.to_dict(),
            "Obstacles": [obs.to_dict() for obs in self.OBSTACLES],
            "Landmarks": [l.to_dict() for l in self.LANDMARKS],
            "Timestep": self.DT,
            "Pinger Range": self.lm_range,
        }


#not sure why thesea rent included here...

def get_landmark_by_id(self, id: int):
        """
        Retrieve a landmark object using its id number.
        """
        for l in self.LANDMARKS:
            if l.id == id:
                return l
        return None

def get_landmarks_by_pos(self, pos: Position):
        """
        Retrieve any landmarks at a given position.
        """
        lms = []
        for l in self.LANDMARKS:
            if l.pos == pos:
                lms.append(l)
        return lms
