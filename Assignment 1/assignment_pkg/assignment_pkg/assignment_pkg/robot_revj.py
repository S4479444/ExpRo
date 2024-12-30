import rclpy
import numpy as np
import time

from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float64, Bool, Float64MultiArray
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor


class RobotController(Node):
    def __init__(self):
        super().init("robot_revj")

        # -------------------------------------- Variables ----------------------------------------- #

        self.current_angle = Float64()
        self.current_angle.data = 0.0

        self.target = Bool()
        self.target.data = False

        self.ready = Bool()
        self.ready.data = False

        self.dt = 0.1

        # Camera operating mode
        self.op_mode = Bool()
        self.op_mode.data = True

        self.waypoint_reached = Bool()
        self.waypoint_reached.data = False

        self.begin_compensation = Bool()
        self.begin_compensation.data = False

        self.angular_vel = Float64()
        self.angular_vel.data = 0.0

        self.sign = 1
        self.theta_goal = Float64()
        self.theta_goal = 0.0

        # Creating timer for the camera operating mode
        self.mode_timer = self.create_timer(self.dt, self.camera_mode)

        # PID controller for linear velocity and rotation
        self.ang_pid = PID(0.5, 0.01, 0.1)
        self.ang_pid.sample_time = self.dt

        # Position and angular error threshold value
        self.ang_threshold = 0.01

        # ----------------------------------------- Publishers ------------------------------------------------ #

        self.pub_rotation_goal = self.create_publisher(Float64, "camera_theta_goal", 10)
        self.pub_cmd_vel = self.create_publisher(Float64MultiArray, "/joint_cam_controller/commands", 10)

        # ----------------------------------------- Subscribers ----------------------------------------------- #

        self.sub_camera_mode = self.create_subscription(Bool, "camera_on_off", self.camera_mode_callback, 10)
        self.sub_rotation = self.create_subscription(Float64, "inverse_rotation", self.rotation_callback, 10)

        # ----------------------------------------------------------------------------------------------------- #
        self.get_logger().info("Revolute controller module initialized succesfully."        )
        self.get_logger().info("Scanning for nearby markers...")


    def rotation_callback(self, msg: Float64):
        self.angular_vel.data = msg.data


    def camera_mode_callback(self, msg: Bool):
        self.op_mode = msg.data

    def camera_mode(self):
        """ 
        CAMERA MODE
        -----------------------------------------------------------------------------------
        This function handles the camera logic, called every sampling period and checking
        whether the camer is rotating or not.
        If camera is rotating, then it checks for the current angle value (clipping it inside
        the interval (0, 2*pi] radians), and then publishes the command message; otherwise, if the camera is not rotating and the
        waypoint hasn't been reached yet, it publishes the command message and the goal angle,
        to the corresponding topics.

        """

        # SEARCH MODE
        if self.op_mode.data == True:

            self.begin_compensation.data = True


            # Clipping current angle value inside (0, 2*pi]
            if self.current_angle.data >= 2*np.pi:
                self.current_angle.data = 2*np.pi
                self.get_logger().error("Angle value is too high!")
            
            elif self.current_angle.data <= 0.01:
                self.current_angle.data = 0.02
                self.get_logger().error("Angle value is too low!")

            self.current_angle.data += 0.02 * self.sign
            self.current_angle.data = round(self.current_angle.data, 3)

            if (self.current_angle.data == 0.0) or (self.current_angle.data == 6.283):
                self.sign *= -1

            # Publish command to rotate the joint
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [self.current_angle.data]

            self.pub_cmd_vel.publish(cmd_msg)
            self.get_logger().ingo(f"Camera is rotating, current angle: {self.current_angle.data}")
        
        # COMPENSATION 
        elif self.op_mode.data == False:

            # Clipping values
            if (self.angular_vel.data < 0.0) and (self.current_angle.data < 6.27):
                self.sign = 1
                self.step = 0.025
                self.current_angle.data += self.step * self.sign
            
            elif (self.angular_vel.data > 0.0) and (self.current_angle.data > 0.01):
                self.sign = -1
                self.step = 0.025
                self.current_angle.data += self.step * self.sign

            # Publish command to rotate the joint
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [self.current_angle.data]
            self.pub_cmd_vel.publish(cmd_msg)

            # Publish the current angle as the theta goal angle (compensation)
            theta_goal_msg = Float64()
            theta_goal_msg.data = self.current_angle.data
            self.pub_rotation_goal.publish(theta_goal_msg)

            self.get_logger().info(f"Compensating angle, current angle: {self.current_angle.data}")

        else:
            self.get_logger().info("Warning: camera opeating mode is not working properly.")


    def control_loop_callback(self):
        """ 
        CONTROL LOOP CALLBACK
        --------------------------------------------------------------------------------------------------------------------
        This function operates the control loop of the camera angle, keeping the current angle locked to the camera.
        It corrects the current angle with a PID controller.
        
        """
        
        # Interrupt the cycle if the waypoint has been reached.
        if self.waypoint_reached.data == True:
            return
        
        # Otherwise compute the misalignment error between the desired and actual angles.
        mis_error = abs(self.current_angle.data - self.ang_pid.setpoint)

        self.get_logger().info(f"Angle: {self.current_angle.data}, Goal: {self.ang_pid.setpoint}, Error: {mis_error}\n")

        # Clipping values
        ang_control = np.clip(self.ang_pid(self.current_angle.data) * self.dt, -0.05, 0.05)

        if self.current_angle.data > 6.27:
            self.current_angle.data = 6.27
            self.get_logger().error("Angle value is too high!")
        
        elif self.current_angle.data <= 0.01:
            self.current_angle.data = 0.01
            self.get_logger().error("Angle value is too low!")

        # Increment angle (control law)
        self.current_angle.data += ang_control
        self.get_logger().warn(f"Current angle: {self.current_angle.data}")

        # Publish command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [self.current_angle.data]
        self.pub_cmd_vel.publish(cmd_msg)

        # Threshold value for reaching waypoints.
        if mis_error < self.ang_threshold:
            self.get_logger().info("Waypoint reached!")
            self.waypoint_reached.data = True
            return
        
def main(args = None):
    rclpy.init(args = args)

    try:
        controller_node = RobotController()
        executor = MultiThreadedExecutor()
        executor.add_node(controller_node)

        try:
            while rclpy.ok():
                executor.spin_once()
        
        finally:
            rclpy.shutdown()
    
    except Exception as e:
        print(f"Error in main: {str(e)}")

if __name__ == "__main__":
    main()