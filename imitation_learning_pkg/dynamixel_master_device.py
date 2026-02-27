import os, sys
from glob import glob

from dynamixel_sdk import *
import numpy as np
from scipy.spatial.transform import Rotation as R




current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
robosuite_repo_path = os.path.join(src_path, "robosuite")

if robosuite_repo_path not in sys.path:
    sys.path.insert(0, robosuite_repo_path)

from robosuite.devices import Device
from robosuite.utils.transform_utils import rotation_matrix

def deg2rad(x):
    y = x * np.pi / 180
    return y

class DynamixelMasterDevice(Device):
    def __init__(self, env, pos_sensitivity=1.0, rot_sensitivity=1.0):
        
        super().__init__(env)

        # robosuite
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        self.base_modes = ["0"] * len(self.env.robots)
        self.active_robot = 0

        self._reset_state = 0
        self._enabled = False

        # 다이나믹셀
        self.DEVICENAME = '/dev/ttyUSB0'
        self.BAUDRATE = 57600

        self.motor_ids = [1, 2, 3, 4, 5, 6, 7]

        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132

        self.PROTOCOL_VERSION = 2

        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.d_ur5e = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
        self.a_ur5e = [0, -0.425, -0.3922, 0, 0, 0]
        self.alpha_ur5e = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]

        if self.portHandler.openPort():
            print("포트 열기 성공")
        else:
            print("포트 열기 실패")
            exit()

        if self.portHandler.setBaudRate(57600):
            print("보드레이트 성공")
        else:
            print("보드레이트 실패")
            exit()

        self.BulkRead = GroupBulkRead(self.portHandler, self.packetHandler)
        self.BulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)

        for id in self.motor_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_TORQUE_ENABLE, 0)

        self.data_length_4byte = 4
        for i in range(7):
            self.BulkRead.addParam(self.motor_ids[i], self.ADDR_PRESENT_POSITION, self.data_length_4byte)

        # 6-DOF variables
        self.dynamixel_angle = np.zeros(6)

        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0

        self._control = [0, 0, 0, 0, 0, 0]
        self._reset_state = 0
        # self.rotation = np.array([[-1, 0, 0],
        #                           [0, 1, 0],
        #                           [0, 0, -1]])
        self.rotation = np.eye(3)
        self.last_pos = None
        self.last_ori = None

        self.q = np.zeros(6)
        self.current_q = np.zeros(6)

        # 절대로 시도
        self.abs_offset = None
        
    def trans_mat(self, theta, d, a, alpha):
        T = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        return T

    def forward_kinematics(self, theta, d, a, alpha):
        T = np.eye(4)
        for i in range(6):
            T_i = self.trans_mat(theta[i], d[i], a[i], alpha[i])
            T = T @ T_i
        return T
    
    def _reset_internal_state(self):
        super()._reset_internal_state()
        
        # self.rotation = np.array(
        #     [[-1.0, 0.0, 0.0],
        #      [0.0, 1.0, 0.0],
        #      [0.0, 0.0, -1.0]]
        # )
        self.rotation = np.eye(3)

        self.last_ori = None
        self.last_pos = None
        # 리셋 컨트롤
        self._control = np.zeros(6)

        self.abs_offset = None

    def start_control(self):
        self._reset_internal_state()
        self._reset_state = 0
        self._enabled = True
    
    def control_gripper(self, value):
        # close가 2048 open이 1200
        min_val = 1200
        max_val = 2048
        if value > max_val:
            normalized_val = 1
        elif value < min_val:
            normalized_val = 0
        else:
            normalized_val = (value - min_val) / (max_val - min_val)
            if normalized_val > 0.5:
                return 1
            else:
                return 0
        
    # 델타로 시도.
    # def get_controller_state(self):
    #     q = np.zeros(7)
        
    #     self.BulkRead.txRxPacket()
    #     for i in range(7):
    #         q[i] = self.BulkRead.getData(self.motor_ids[i], self.ADDR_PRESENT_POSITION, self.data_length_4byte)

    #     dir = [1, 1, -1, 1, 1, 1]
    #     offset = [0, -90, 0, -90, 0, 180]
    #     # offset = [0, 0, 0, 0, 0, 0]
    #     q_pose = []
    #     for i in range(6):
    #         q_pose.append((q[i] - 2048) * dir[i] * 360 / 4096 + offset[i])
    #     q_pose_rad = np.deg2rad(q_pose)
    #     self.current_q = q_pose_rad
        
    #     EE = self.forward_kinematics(q_pose_rad, self.d_ur5e, self.a_ur5e, self.alpha_ur5e)

    #     self.pos = EE[:3, 3]
    #     self.ori = EE[:3, :3]
    #     print(f"master pos = {self.pos}")

    #     if self.last_pos is None:
    #         self.last_pos = np.array(self.pos)
    #         self.last_ori = np.array(self.ori)
    #         return dict(
    #             dpos=np.zeros(3),
    #             rotation=self.ori,
    #             raw_drotation=np.zeros(3),
    #             grasp=0,
    #             reset=self._reset_state,
    #             base_mode=int(self.base_mode),
    #         )
        
    #     ori_obj = R.from_matrix(self.ori)
    #     last_ori_obj = R.from_matrix(self.last_ori)
    #     delta_r = ori_obj * last_ori_obj.inv()

    #     dpos = (self.pos - self.last_pos)
    #     drot = delta_r.as_rotvec()

    #     # 전 pos와 ori 저장
    #     self.last_pos = self.pos
    #     self.last_ori = self.ori

        

    #     pre_swapped_drot = np.array([
    #         drot[1],
    #         drot[0],
    #         -drot[2]
    #     ])
        
    #     return dict(
    #         dpos=dpos,
    #         rotation=self.ori,
    #         raw_drotation=pre_swapped_drot,
    #         grasp=self.control_gripper(q[6]),
    #         reset=self._reset_state,
    #         base_mode=int(self.base_mode),
    #     )
    
    # 절대로 시도
    def get_controller_state(self):
        q = np.zeros(7)
        
        self.BulkRead.txRxPacket()
        for i in range(7):
            q[i] = self.BulkRead.getData(self.motor_ids[i], self.ADDR_PRESENT_POSITION, self.data_length_4byte)

        dir = [1, 1, -1, 1, 1, 1]
        offset = [0, -90, 0, -90, 0, 180]
        # offset = [0, 0, 0, 0, 0, 0]
        q_pose = []
        for i in range(6):
            q_pose.append((q[i] - 2048) * dir[i] * 360 / 4096 + offset[i])
        q_pose_rad = np.deg2rad(q_pose)
        self.current_q = q_pose_rad
        
        EE = self.forward_kinematics(q_pose_rad, self.d_ur5e, self.a_ur5e, self.alpha_ur5e)

        self.pos = EE[:3, 3]
        self.ori = EE[:3, :3]

        obs = self.env._get_observations()

        base_pos = self.env.robots[0].base_pos
        base_ori_mat = self.env.robots[0].base_ori
        world_eef_pos = obs["robot0_eff_pos"]

        sim_eef_pos = base_ori_mat.T @ (world_eef_pos - base_pos)

        self.abs_offset = sim_eef_pos - world_eef_pos

        absolute_pos = self.abs_offset + self.pos

        self.last_pos = self.pos
        self.last_ori = self.ori

        return {
        "ee_pos": self.last_pos,               # 현재 위치
        "ee_ori": self.last_ori,           # 현재 회전
        "goal_pos": absolute_pos,         # 절대 제어 시 설정된 목표 위치
        "goal_ori": self.ori,     # 절대 제어 시 설정된 목표 회전
        "joint_pos": None, 
    }
    

    def _postprocess_device_outputs(self, dpos, drotation):
        drotation = drotation * 50
        dpos = dpos * 20

        dpos = np.clip(dpos, -1, 1)
        drotation = np.clip(drotation, -1, 1)
        return dpos, drotation
    
    # collect_human_demonstrations에서 초기 자세 잡으려고.
    def get_current_q(self):
        return self.current_q
    
    @property
    def control(self):
        return np.array(self._control)

        

def main():
    dynamixel_master_device = DynamixelMasterDevice()

if __name__ == '__main__':
    main()