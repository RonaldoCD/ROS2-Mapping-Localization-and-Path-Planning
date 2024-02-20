import rclpy
import yaml
from rclpy.node import Node
# Import custom interface Age
from geometry_msgs.msg import PoseWithCovarianceStamped

class SpotRecorder(Node):

    def __init__(self):
        super().__init__('spot_recorder')
        
        self.pose_subs = self.create_subscription(PoseWithCovarianceStamped, 
                                                      '/initialpose', 
                                                      self.pose_callback,
                                                      10)

        self.points_names = ['corner1', 'corner2', 'pedestrian', 'spot4', 'spot5']
        self.points_recorded = 0
        #self.spots_dict = {}
        self.spots_dict = {"Nav_To_Pose_Action_Client":{"ros__parameters":{}}}

    def pose_callback(self, msg):
        spot_x = msg.pose.pose.position.x
        spot_y = msg.pose.pose.position.y
        spot_theta = msg.pose.pose.orientation.z
        self.add_spot(spot_x, spot_y, spot_theta)
        self.write_yaml_to_file("spots_data")
    
    def write_yaml_to_file(self, filename) :
        file_path = "/home/user/ros2_ws/src/Nav2_Project/project_path_planning/config/" + filename + ".yaml"
        with open(file_path, 'w') as f :
            yaml.dump_all([self.spots_dict], f, sort_keys=False)
            self.get_logger().info("written to file successfully")
            self.get_logger().info(' ')
    
    def add_spot(self, spot_x, spot_y, spot_theta):
        point_name = self.points_names[self.points_recorded]
        #self.spots_dict[point_name] = {"x": spot_x, "y": spot_y, "theta": spot_theta}
        self.spots_dict["Nav_To_Pose_Action_Client"]["ros__parameters"][point_name] = {"x": spot_x, "y": spot_y, "theta": spot_theta}
        
        self.points_recorded += 1
        self.get_logger().info('Saved point: "%s"' %point_name)
        self.get_logger().info('    X = "%s"' %str(spot_x))
        self.get_logger().info('    Y = "%s"' %str(spot_y))
        self.get_logger().info('    Orientation = "%s"' %str(spot_theta))
        

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    spot_recorder_node = SpotRecorder()
    rclpy.spin(spot_recorder_node)
    # Explicity destroy the node
    spot_recorder_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()