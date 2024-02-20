import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('Nav_To_Pose_Action_Client')
        self.declare_parameter('spot_name', 'corner1')
        self.spot_name = self.get_parameter('spot_name').value

        self.get_logger().info('Spot name: %s \n' % (self.spot_name))

        self.declare_parameter(self.spot_name+'.x', 1.0)
        self.declare_parameter(self.spot_name+'.y', 1.0)
        self.declare_parameter(self.spot_name+'.theta', 1.0)

        self.spot_x = self.get_parameter(self.spot_name+'.x').value
        self.spot_y = self.get_parameter(self.spot_name+'.y').value
        self.spot_theta = self.get_parameter(self.spot_name+'.theta').value
        self.get_logger().info('Spot:\n X : %f \n Y : %f \n Theta : %f' % (self.spot_x, self.spot_y, self.spot_theta))
        #self.get_logger().info('Spot:\n X : %s ' % str(self.spot))
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
    def send_goal(self):
        self.get_logger().info('sending goal to action server')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = self.spot_x
        goal_pose.pose.pose.position.y = self.spot_y
        goal_pose.pose.pose.position.z = self.spot_theta

        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('FEEDBACK:' + str(feedback) )

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()