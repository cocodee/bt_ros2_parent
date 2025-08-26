import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action.client import ClientGoalHandle, GoalStatus
import threading
import asyncio # 用于在 async 函数中非阻塞地等待

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# 导入我们新创建的 Action 接口
from my_robot_interfaces.action import SynchronizedMove

class SynchronizedMoveServer(Node):
    def __init__(self):
        super().__init__('synchronized_move_action_server')
        
        # --- 新增 ---
        # 线程锁，用于保护共享的 goal handle 变量
        self._lock = threading.Lock()
        # 存储当前正在执行的底层手臂的 goal handles
        self._current_goal_handles = {'left': None, 'right': None}
        
        self._action_server = ActionServer(
            self,
            SynchronizedMove,
            'synchronized_move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback)

        self._right_arm_client = ActionClient(self, FollowJointTrajectory, '/supre_robot_follower/right_arm_trajectory_controller/follow_joint_trajectory')
        self._left_arm_client = ActionClient(self, FollowJointTrajectory, '/supre_robot_follower/left_arm_trajectory_controller/follow_joint_trajectory')
        
        self.right_joint_names = [f'follower_right_arm_joint_{i}' for i in range(1, 7)]
        self.left_joint_names = [f'follower_left_arm_joint_{i}' for i in range(1, 7)]

        self.get_logger().info('同步双臂运动 Action 服务器已启动，等待请求...')

    def goal_callback(self, goal_request):
        """接受或拒绝新的 Goal 请求"""
        self.get_logger().info('收到新的同步运动请求...')
        if len(goal_request.left_arm_positions) != 6 or len(goal_request.right_arm_positions) != 6:
            self.get_logger().error('关节数量错误! 拒绝请求。')
            return GoalResponse.REJECT
        
        # 检查是否已有任务在执行 (简单抢占策略)
        with self._lock:
            if any(self._current_goal_handles.values()):
                self.get_logger().warn('已有任务在执行，但新任务将抢占它。')
        
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """当一个 goal 被接受后，立即开始执行它"""
        # 如果已有任务，先尝试取消旧任务
        # 注意：这是一个简单的抢占模型，更复杂的模型可能需要队列
        with self._lock:
            if any(self._current_goal_handles.values()):
                self.get_logger().info('抢占旧任务...')
                # 这里不直接取消，因为 execute_callback 的 try...finally 会处理
        
        # 启动一个新的后台任务来执行 goal
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """处理来自客户端的取消请求"""
        self.get_logger().info('收到取消请求！')
        
        # --- 补全的核心逻辑 ---
        # 将取消指令传递给底层的两个 Action Client
        with self._lock:
            if self._current_goal_handles['left']:
                self.get_logger().info('正在取消左臂的底层任务...')
                self._current_goal_handles['left'].cancel_goal_async()
            if self._current_goal_handles['right']:
                self.get_logger().info('正在取消右臂的底层任务...')
                self._current_goal_handles['right'].cancel_goal_async()
        
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """执行 Action 的核心逻辑"""
        self.get_logger().info('正在执行同步双臂运动...')
        
        try:
            left_pos = goal_handle.request.left_arm_positions
            right_pos = goal_handle.request.right_arm_positions
            time_from_start = goal_handle.request.time_from_start

            self._left_arm_client.wait_for_server(timeout_sec=2.0)
            self._right_arm_client.wait_for_server(timeout_sec=2.0)
            
            left_fjt_goal = self._create_fjt_goal(self.left_joint_names, left_pos, time_from_start)
            right_fjt_goal = self._create_fjt_goal(self.right_joint_names, right_pos, time_from_start)

            left_goal_handle_future = self._left_arm_client.send_goal_async(left_fjt_goal)
            right_goal_handle_future = self._right_arm_client.send_goal_async(right_fjt_goal)
            
            # 等待 goal handle 返回
            left_arm_goal_handle = await left_goal_handle_future
            right_arm_goal_handle = await right_goal_handle_future

            # --- 关键: 存储底层 goal handle ---
            with self._lock:
                self._current_goal_handles['left'] = left_arm_goal_handle
                self._current_goal_handles['right'] = right_arm_goal_handle

            if not left_arm_goal_handle.accepted or not right_arm_goal_handle.accepted:
                goal_handle.abort()
                return self._create_result(False, "底层手臂控制器拒绝了目标")
            
            left_result_future = left_arm_goal_handle.get_result_async()
            right_result_future = right_arm_goal_handle.get_result_async()
            
            # --- 关键: 循环等待，并检查取消状态 ---
            while not left_result_future.done() or not right_result_future.done():
                if goal_handle.is_cancel_requested:
                    # cancel_callback 已经被调用，并向底层发送了取消指令
                    # 我们在这里只需要更新顶层 Action 的状态并退出
                    goal_handle.canceled()
                    self.get_logger().info('同步双臂运动已被取消！')
                    return self._create_result(False, "Action was canceled")
                
                # 非阻塞等待一小段时间
                await asyncio.sleep(0.1)

            # 获取最终结果
            left_status = left_result_future.result().status
            right_status = right_result_future.result().status
            
            if left_status == GoalStatus.STATUS_SUCCEEDED and right_status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
                self.get_logger().info('同步双臂运动成功！')
                return self._create_result(True, "双臂运动成功")
            else:
                goal_handle.abort()
                error_msg = f"同步双臂运动失败: Left Status({left_status}), Right Status({right_status})"
                self.get_logger().error(error_msg)
                return self._create_result(False, error_msg)

        finally:
            # --- 关键: 无论如何，最后都要清理资源 ---
            with self._lock:
                self.get_logger().info('清理当前任务的 Goal Handles...')
                self._current_goal_handles['left'] = None
                self._current_goal_handles['right'] = None

    def _create_fjt_goal(self, joint_names, positions, time_from_start):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = time_from_start
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        return goal_msg

    def _create_result(self, success, message):
        """辅助函数创建 Result 消息"""
        result = SynchronizedMove.Result()
        result.success = success
        result.message = message
        return result

def main(args=None):
    rclpy.init(args=args)
    server = SynchronizedMoveServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()