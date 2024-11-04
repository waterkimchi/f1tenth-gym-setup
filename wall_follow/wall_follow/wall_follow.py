import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Create subscribers and publishers
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # Set PID gains
        self.kp = 1.0
        self.kd = 0.1
        self.ki = 0.01
        self.desired_distance_to_wall = 0.8

        # Store history
        self.integral = 0.0
        self.prev_error = 0.0

        # Store any necessary values you think you'll need
        self.error = 0.0

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        angle_increment = (range_data.angle_max - range_data.angle_min) / len(range_data.ranges)
        index = int((angle - range_data.angle_min) / angle_increment)
        range_at_angle = range_data.ranges[index]

        if np.isnan(range_at_angle) or np.isinf(range_at_angle):
            return 0.0
        return range_at_angle

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        # Get the range measurements at 90 degrees (left) and 45 degrees (front-left)
        theta = np.radians(45)
        b = self.get_range(range_data, np.radians(90))
        a = self.get_range(range_data, np.radians(45))
        alpha = np.arctan2(a*np.cos(theta)-b,a*np.cos(theta))
        ab = b*np.cos(alpha)
        cd = ab+1.5*np.sin(alpha)
        error = 1.5-cd

        return -error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # PID control
        self.integral += error
        derivative = error - self.prev_error
        angle = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

        # Update previous error
        self.prev_error = error

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # Calculate the error using get_error()
          # Example desired distance to the wall in meters
        error = self.get_error(msg, self.desired_distance_to_wall)
        
        # Calculate desired car velocity based on error (you can adjust this logic as needed)
        velocity = 1.0  # Example constant velocity
        
        # Actuate the car with PID control
        self.pid_control(error, velocity)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()