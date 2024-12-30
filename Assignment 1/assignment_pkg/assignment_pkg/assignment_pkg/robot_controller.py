import math
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Float32MultiArray
from ros2_aruco_interfaces.msg import ArucoMarkers

class RobotControl(rclpy.node.Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Subscriber to <aruco_markers>
        self.aruco_subscription = self.create_subscription(ArucoMarkers, "aruco_markers", self.aruco_callback, 10)
        self.aruco_subscription

        # Subscriber to <aruco_corners>
        self.subscription_corners = self.create_subscription(Float32MultiArray, "aruco_corners", self.corners_callback, 10)
        self.subscription_corners

        # Publisher to <camera_on_off> for its rotation
        self.publisher_cameraonoff = self.create_publisher(Bool, "camera_on_off", 10)

        # Publisher to <marker_reached> for the reached marker
        self.publisher_marker_reached = self.create_publisher(Bool, "marker_reached", 10)

        # Timer for the Controller Logic
        period = 0.1
        self.timer = self.create_timer(period, self.timer_callback)

        # -------------------------------------------------------- Variables -------------------------------------------------------------- #


        self.id_marker = 0                              
        self.position_marker = 0                       # marker's position in the list of already-seen markers
        self.corners_marker = []                       # corners of the searched marker, when one is found
        self.goal_markers = [11, 12, 13, 15]           # list of objectives
        self.reached_markers = 0                        # number of reached markers
        self.flag = 0                                  # useful flags
        self.flag_marker = 0                           
        self.last_marker_area = 300000.0               # value for comparing the area of the goal marker
        self.iteration = 0                             # wait for inaccurate measurement of the camera

    def timer_callback(self):
        """ 
        TIMER CALLBACK
        --------------------------------------------------------------------
        Function called to handle various cases:
            - destroy node if objective is reached;
            - select the ID of the marker to-be-reached and scan the area;
            - reach the marker in the camera FOV.
        """

        # Destroy node when all the markers are reached
        if self.reached_markers == 4:
            self.get_logger().info("All markers reached! Objective complete.")
            self.destroy_node()
            rclpy.shutdown()

        # Select the ID of the marker to-be-reached
        self.id_marker = self.goal_markers[self.reached_markers]
        if self.flag == 0:
            self.rotation_camera(True)
            self.aruco_controller_area()

        elif self.flag == 1:
            self.iteration = 0
            self.aruco_follow_marker()

    def rotation_camera(self, switch):
        """ Publisher to camera to rotate itself during scanning """
        msg = Bool()
        msg.data = switch
        self.publisher_cameraonoff.publish(msg)

    def marker_reached(self, ctrl):
        """ Publisher to motor to notify that a marker has been reached """
        msg = Bool()
        msg.data = ctrl
        self.publisher_marker_reached.publish(msg)

    def aruco_callback(self, msg):
        """ 
        ARUCO CALLBACK
        --------------------------------------------------------------------------------------------
        Function that selects the ID of the marker that is desired to be reached, and checks whether
        it's inside given list of markers. In positive case, update the internal variables, otherwise
        wait and check again after some time.
        """

        # Select the ID of the marker that is to be reached
        self.id_marker = self.goal_markers[self.reached_markers]

        # Take the marker's ID from the msg
        self.ids_marker = msg.marker_ids

        # Check if marker is already in listt
        if self.id_marker in self.ids_marker:
            self.flag_marker = 1
            self.position_marker = self.ids_marker.index(self.id_marker)
        else:
            # Else wait and then check again
            self.flag_marker = 0
    
    def corners_callback(self, msg):
        """ 
        MARKER'S CORNERS CALLBACK
        ----------------------------------------------------------------------------------------------
        This function looks for the marker's corners and put the coordinates in the list.
         
        """

        if self.flag_marker == 1:
            full_data_corners = msg.data    # Take the marker's corners          
            self.corners_marker = []        # Empty the list
            
            # Put all corners in the list
            for i in range(8):
                self.corners_marker.append(full_data_corners[i + 8*self.position_marker])
        else:
            self.corners_marker = []

    def aruco_controller_area(self):
        """ 
        ARUCO AREA CONTROLLER 
        -----------------------------------------------------------------------------------
        This function makes the robot wait for the marker to be in the area, and then
        moves it to the marker with the camera doing the motion.

        """

        # Check if the position of the marker is in area
        if self.flag_marker == 1 and len(self.corners_marker) != 0:
            # Compute the area of the rectangle between the marker's corners
            marker_area = self.compute_rect_area(self.corners_marker)
            # Check if marker area changes
            if marker_area > self.last_marker_area:
                self.iteration += 1
                if self.iteration >= 3:
                    self.rotation_camera(False)
                    self.flag = 1
                    return # Exit the loop
            self.last_marker_area = marker_area


    def aruco_follow_marker(self):
        """ FOLLOW MARKER 
        --------------------------------------------------------------------------------------------------------------
        This function is used to make the camera following the marker during the robot's motion.
        
        """

        # Check if the position of the marker is in area
        if self.flag_marker == 1 and len(self.corners_marker) != 0:
            # Compute the longest distance between the markers
            longest_side = self.compute_dist(self.corners_marker)
            # Threshold variable
            dist_check = 200

            if longest_side > dist_check:
                # Notify the motor control that it has reached the target marker
                self.rotation_camera(True)
                self.marker_reached(True)
                self.get_logger().info(f"Marker N° {self.id_marker} reached!")
                self.flag = 0
                self.flag_marker = 0
                self.reached_markers += 1

        else:
            # Else notify the motor control that marker has been lost, possibly outside the camera FOV
            self.marker_reached(False)
            self.flag = 0
            self.get_logger().error("Target marker is out of the camera range, scanning...")
 
    def compute_rect_area(self, coords):
        """ 
        MARKER AREA COMPUTATION
        ---------------------------------------------------------------------------------------------
        This function computes the area of the marker, given the coordinates of its corners.

        """
        # Comput the area of the bounding box around the marker
        x1, y1, x2, y2, x3, y3, x4, y4 = coords

        return 0.5 * abs((x1*y2 + x2*y3 + x3*y4 + x4*y1) - (y1*x2 + y2*x3 + y3*x4 + y4*x1))

    def compute_dist(self, coords):
        """ Compute cartesian distances between the markers """
        x1, y1, x2, y2, x3, y3, x4, y4 = coords
        dist1 = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        dist2 = math.sqrt((x3 - x1)**2 + (y3 - y1)**2)
        dist3 = math.sqrt((x4 - x1)**2 + (y4 - y1)**2)
        dist4 = math.sqrt((x3 - x2)**2 + (y3 - y2)**2)
        return max(dist1, dist2, dist3, dist4)
    

def main(args=None):
    rclpy.init(args = args)
    try:
        robot_control = RobotControl()
        print("Robot Controller Node started.")
        executor = MultiThreadedExecutor()
        executor.add_node(robot_control)
        print("Robot Controller Node added to the executor.")

        try:
            while rclpy.ok():
                executor.spin_once()

        finally:
            print("Destroying Robot Controller Node...")
            executor.shutdown()
            robot_control.destroy_node()
            rclpy.shutdown()
    
    except KeyboardInterrupt:
        print("Node interrupted by user.")
        robot_control.destroy_node()
        rclpy.shutdown()

    rclpy.spin(robot_control)

if __name__ == "__main__":
    main()