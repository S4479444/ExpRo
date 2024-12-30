import rclpy
import math
import time

from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

MAX_VEL = 0.5

class MotorControl(rclpy.node.Node):
    def __init__(self, dt):
        super().__init__('motor_control')

        # --------------------------- Subscriptions --------------------------------- #

        # Subscribe to <odom>
        self.subscription = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.subscription

        # Subscribe to <camera_theta_goal>
        self.subscription = self.create_subscription(Float64, "camera_theta_goal", self.theta_callback, 10)
        self.subscription

        # Subsrcribe to <marker_reached>
        self.subscription = self.create_subscription(Bool, "marker_reached", self.reached_callback, 10)
        self.subscription

        # --------------------------- Publications ---------------------------------- #
        
        # Publish to <cmd_vel>
        self.publisher_vel = self.create_publisher(Twist, "cmd_vel", 10)

        # Publish to <inverse_rotation>
        self.publisher_rotation = self.create_publisher(Float64, "inverse_rotation", 10)

        # --------------------------- Variables ------------------------------------- #
        self.theta = 0.0
        self.theta_goal = 0.0
        self.flag = 0.0
        self.dt = dt
        self.reached_marker = 0

        # ------------------------------ Control Loop timer ------------------------- #
        self.ctrl_timer = self.create_timer(self.dt, self.robot_motion)

    
    def robot_motion(self):
        """ 
        ROBOT CONTROL LOOP
        --------------------------------------------------------------------------------------
        This function checks for flag's value and updates the robot's behavour consequently.
                  
        """

        # Waiting for theta
        if self.flag == 0:
            time.sleep(0.5 * self.dt)
            self.get_logger().info("Waiting for theta value...")
        
        # Aligning camera with the marker
        elif self.flag == 1:
            self.camera_alignment()
            self.get_logger().info("Aligning camera...")
        
        # Moving forward
        elif self.flag == 2:
            self.forward(1)
            self.get_logger().info("Moving forward")
        
        # Moving backwards
        elif self.flag == 3:
            self.forward(-1)
            time.sleep(2 * self.dt)
            self.stop()
            self.flag = 0
            self.reached_marker += 1
            self.get_logger().info(f"Moving backwards")

        # Shutdown node
        if self.reached_marker == 4:
            self.get_logger().info("Mission success! Now shutting down.")
            self.destroy_node()
            rclpy.shutdown()

    def odom_callback(self, msg):
        """" 
        ODOMETRY CALLBACK 
        ---------------------------------------------------------------------------
        This function computes the angle theta from the quaternion representation,
        also normalizing its value in order to have it betweem 0 and 2*pi radians.

        """

        # Callback for robot orientation update, fetching pose expressed with quaternions
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Compute theta from quaternion representation
        self.theta = math.atan2(2 * (qx * qy + qz * qw),
                                qw**2 + qx**2 - qy**2 - qz**2)
        
        # Correct to values in [0, 2*pi]
        if self.theta < 0:
            self.theta = 2 * math.pi + self.theta

        
    def theta_callback(self, msg):
        """ 
        THETA CALLBACK 
        -----------------------------------------------------------------------------
        This function is called to update the value of theta, during the phase of goal
        orientation.

        """

        # Callback for goal orientation updates, fetching data and updating value
        if self.flag == 0:
            delta = msg.data
            self.theta_goal = self.theta + delta

            # Correct to values in [0, 2*pi]
            if self.theta_goal > 2*math.pi:
                self.theta_goal -= 2*math.pi

            self.flag += 1

    def reached_callback(self, msg):
        """
        REACHED MARKER CALLBACK 
        ----------------------------------------------------------------------------------
        This function is used to update the status of the internal flag, depending on the 
        value of the stop command sent to the robot.
                
        """

        stop = msg.data
        
        if stop == True and self.flag == 2:
            self.stop()
            time.sleep(2 * self.dt)
            self.flag += 1
        
        elif stop == False and self.flag == 2:
            self.stop()
            self.flag = 0

    def camera_alignment(self):
        """ 
        CAMERA ALIGNMENT FUNCTION
        --------------------------------------------------------------------------------------
        This function is used to align the camera with the position of the marker, by reading
        the goal value for theta, and rotate the camera depending on the misalignment error.
        If the error is inside a certain threshold, stop rotating.

        """
        msg = Float64()

        delta = (self.theta_goal - self.theta + math.pi) % math.pi

        if delta > math.pi:
            msg.data = -1.0
            self.rotate(-1)

        else:
            msg.data = 1.0
            self.rotate(1)
        
        if abs(self.theta_goal - self.theta) < 0.01:
            msg.data = 0.0
            self.stop()
            self.flag += 1

        self.publisher_rotation.publish(msg)
    
    def stop(self):
        """ 
        STOPPING THE ROBOT
        -----------------------------------------------------------------------------
        This function is used to stop the robot's motion, by setting the linear x 
        velocity and angular z velocity to zero.

        """

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher_vel.publish(cmd_vel)
        time.sleep(0.1)

    def forward(self, direction):
        """ 
        MOVING FORWARD
        ------------------------------------------------------------------------------
        This function drives the robot forward, by setting its linear x velocity, and
        reducing the angular velocity on z to zero.

        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5 * direction * MAX_VEL 
        cmd_vel.angular.z = 0.0
        self.publisher_vel.publish(cmd_vel)

    def rotate(self, direction):
        """"
        ROTATE THE ROBOT
        ------------------------------------------------------------------------------
        This function rotates the robot around the z axis, and sets the linear x
        velocity to zero.
        
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = direction * MAX_VEL
        self.publisher_vel.publish(cmd_vel)

def main(args = None):
    rclpy.init(args=args)

    try:
        motor_control = MotorControl(dt = 0.1)
        executor = MultiThreadedExecutor()
        executor.add_node(motor_control)

        try:
            while rclpy.ok():
                executor.spin_once()
        
        finally:
            rclpy.shutdown()
    
    except Exception as e:
        print(f"An error occurred: {str(e)}")

if __name__ == "__main__":
    main()