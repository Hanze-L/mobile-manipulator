import rospy
import actionlib
from geometry_msgs.msg import Pose, Vector3, Quaternion
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
from zivid_camera.srv import CaptureAndSave
import numpy as np
import tf
import os

class HandEyeCalibration:
    def __init__(self):
        rospy.init_node("hand_eye_calibration", anonymous=True)
        
        # 初始化笛卡尔轨迹控制客户端
        self.trajectory_client = actionlib.SimpleActionClient(
            "pose_based_cartesian_traj_controller/follow_cartesian_trajectory",
            FollowCartesianTrajectoryAction,
        )
        
        # 等待动作服务器就绪
        if not self.trajectory_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("Could not connect to action server")
            raise RuntimeError("Action server not available")
            
        # 初始化相机服务
        rospy.wait_for_service("/zivid_camera/capture_and_save", 30.0)
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture_and_save", CaptureAndSave)
        
        # 初始化tf监听器
        self.tf_listener = tf.TransformListener()
        
        # 创建保存数据的目录
        self.save_dir = "calibration_data"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # 记录所有位姿
        self.robot_poses = []
        
    def move_to_cartesian_pose(self, position, orientation, duration=5.0):
        """移动机器人到指定的笛卡尔位姿"""
        goal = FollowCartesianTrajectoryGoal()
        point = CartesianTrajectoryPoint()
        
        # 设置目标位姿
        point.pose = Pose(
            position=Vector3(*position),
            orientation=Quaternion(*orientation)
        )
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()
        
        result = self.trajectory_client.get_result()
        return result.error_code == 0
        
    def capture_image(self, image_name):
        """使用Zivid相机捕获图像并保存"""
        file_path = os.path.join(self.save_dir, f"{image_name}.zdf")
        try:
            self.capture_service(file_path)
            rospy.loginfo(f"Image saved to: {file_path}")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Image capture failed: {e}")
            return False
            
    def get_current_pose(self):
        """获取当前末端执行器位姿"""
        try:
            self.tf_listener.waitForTransform('/base_link', '/tool0', rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get current pose: {e}")
            return None
            
    def run_calibration_sequence(self):
        """执行标定序列"""
        # 定义一系列标定位姿（位置和方向）
        # 位置使用[x, y, z]格式
        # 方向使用四元数[x, y, z, w]格式
        calibration_poses = [
            # 位置1：正上方
            ([0.4, -0.1, 0.5], [0, 0, 0, 1]),
            # 位置2：倾斜45度
            ([0.4, 0.1, 0.5], [0, 0, 0.383, 0.924]),
            # 位置3：另一个角度
            ([0.4, -0.1, 0.4], [0, 0, -0.383, 0.924]),
            # 位置4：前倾
            ([0.4, 0.1, 0.4], [0.383, 0, 0, 0.924]),
            # 可以添加更多位姿...
        ]
        
        for i, (position, orientation) in enumerate(calibration_poses):
            # 移动到标定位姿
            rospy.loginfo(f"Moving to calibration pose {i+1}")
            if self.move_to_cartesian_pose(position, orientation):
                rospy.sleep(1)  # 等待稳定
                
                # 捕获图像
                if self.capture_image(f"calibration_{i+1}"):
                    # 获取并保存机器人位姿
                    current_pose = self.get_current_pose()
                    if current_pose:
                        self.robot_poses.append({
                            'trans': current_pose[0],
                            'rot': current_pose[1]
                        })
                        rospy.loginfo(f"Captured pose {i+1}")
                    else:
                        rospy.logerr(f"Failed to get pose for position {i+1}")
                else:
                    rospy.logerr(f"Failed to capture image for position {i+1}")
            else:
                rospy.logerr(f"Failed to move to position {i+1}")
                
    def save_calibration_data(self):
        """保存标定数据"""
        import json
        
        data = {
            'poses': [
                {
                    'trans': pose['trans'],
                    'rot': pose['rot'],
                    'image_path': os.path.join(self.save_dir, f"calibration_{i+1}.zdf")
                }
                for i, pose in enumerate(self.robot_poses)
            ]
        }
        
        with open(os.path.join(self.save_dir, 'calibration_data.json'), 'w') as f:
            json.dump(data, f, indent=4)
            
        rospy.loginfo(f"Calibration data saved to {self.save_dir}/calibration_data.json")

def main():
    try:
        calibrator = HandEyeCalibration()
        # 执行标定序列
        calibrator.run_calibration_sequence()
        # 保存数据
        calibrator.save_calibration_data()
        rospy.loginfo("Calibration sequence completed")
    except Exception as e:
        rospy.logerr(f"Calibration failed: {e}")

if __name__ == "__main__":
    main()
