import rclpy
import csv
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
# Import custom interface Age
from custom_interfaces.srv import PointName
from geometry_msgs.msg import PoseWithCovarianceStamped


class SaveSpot(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('save_spot_node')
        # create the publisher object
        self.save_spot_srv = self.create_service(PointName, 
                                                'save_spot',
                                                self.save_spot_callback)

        self.amcl_pose_subs = self.create_subscription(PoseWithCovarianceStamped, 
                                                      '/amcl_pose', 
                                                      self.amcl_callback,
                                                      10)

        self.points_data = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
    def save_spot_callback(self, request, response):
        # create an Age message
        if request.point_name != "end":
            self.points_data.append([request.point_name, self.robot_x, self.robot_y, self.robot_theta])
            self.get_logger().info('Saved point: "%s"' %request.point_name)
            self.get_logger().info('    X = "%s"' %str(self.robot_x))
            self.get_logger().info('    Y = "%s"' %str(self.robot_y))
            self.get_logger().info('    Orientation = "%s"' %str(self.robot_theta))
            self.get_logger().info(' ')

        else:
            filename = "/home/user/ros2_ws/src/Nav2_Project/project_localization/config/points.csv"
            with open(filename, 'w', newline='') as file:
                writer = csv.writer(file)
                for i in range(len(self.points_data)):
                    writer.writerow(self.points_data[i])
            self.get_logger().info('File saved with "%s" points' %str(len(self.points_data)))
        return response 

    def amcl_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = msg.pose.pose.orientation.z


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    save_spot_node = SaveSpot()
    rclpy.spin(save_spot_node)
    # Explicity destroy the node
    save_spot_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()