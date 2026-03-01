"""
A simulated robotic agent with teleoperation and sensing capabilities.

The Robot class models the robotic agent that explores the world. The robot is remote-controlled by angular and linear velocity commands read from an external file. The robot can execute motor commands to move, and can sense both externally (GPS, landmarks, obstacles) and internally (odometry, IMU).
"""
from utils import NEAR_ZERO, floating_mod_zero, SEED
from environment import Environment
from sensors import SensorInterface, LandmarkPinger, GPS, Odometry
import random
import math
import pandas as pd

class Robot:
    """
    A class that models a simulated robotic agent.

    Attributes:
        env: the environment this robot is operating in
        sensors: list of all robot sensors
    """

    def __init__(self, env: Environment):
        """
        Initialize an instance of the Robot class.

        Args:
            env: the environment this robot is operating in
        """
        
        # set the environment property to the parameter value
        self.env = env
        # commands from the controller
        self.cmd_lin_vel = 0.0 #m/s
        self.cmd_ang_vel = 0.0 #rad/s
        # noisy execution of controller commands
        self.actual_lin_vel = 0.0
        self.actual_ang_vel = 0.0
        # initialize the sensors property as an empty list
        self.sensors = []

        

        


    def robot_step_differential(self, lin_vel: float, ang_vel: float):
        """
        Differential-drive mode. Given forward linear and angular velocities, determine the robot's change in x, y, and heading and apply those changes in the environment.

        Args:
            lin_vel: input linear velocity command
            ang_vel: input angular velocity command

        Returns:
            dx: change in x position
            dy: change in y position
            d-theta: change in heading
        """
        # TODO: fill in the function
        pass

    def robot_step_translational(self, x_vel: float, y_vel: float, ang_vel: float):
        """
        Swerve-drive mode. Given x, y, and angular velocities, determine the robot's change in x, y, and heading and apply those changes in the environment.

        Args:
            x_vel: input x velocity command
            y_vel: input y velocity command
            ang_vel: input angular velocity command

        Returns:
            dx: change in x position
            dy: change in y position
            d-theta: change in heading
        """
        # TODO: fill in the function
         # pocket the cmds
        self.cmd_lin_vel = lin_vel
        self.cmd_ang_vel = ang_vel

        # noisify the execution proportionally
        lin_vel = lin_vel * (1 + random.gauss(0, self.EXECUTION_NOISE_LINEAR))
        ang_vel = ang_vel * (1 + random.gauss(0, self.EXECUTION_NOISE_ANGULAR))

        # pocket the actual
        self.actual_lin_vel = lin_vel
        self.actual_ang_vel = ang_vel

        # linear only; drive in a straight line
        if abs(ang_vel) < NEAR_ZERO:
            dx = lin_vel * self.env.DT * math.cos(self.env.agent_pose.theta)
            dy = lin_vel * self.env.DT * math.sin(self.env.agent_pose.theta)
            dtheta = 0.0
        # linear and angular; drive in an arc
        else:
            r = lin_vel / ang_vel
            dtheta = ang_vel * self.env.DT
            dx = r * (
                math.sin(self.env.agent_pose.theta + dtheta)
                - math.sin(self.env.agent_pose.theta)
            )
            dy = -r * (
                math.cos(self.env.agent_pose.theta + dtheta)
                - math.cos(self.env.agent_pose.theta)
            )

        # pass deltas to the env
        self.env.robot_step(dx, dy, dtheta)

    def take_sensor_measurements(self):
        """
        Return noisy sensor readings of the environment at this timestep, including data from all sensors, in a table format.
        """
        # TODO: fill in the function
        pass
