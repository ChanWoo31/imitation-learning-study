# actions에서 

import h5py
import numpy as np

# 데이터셋 경로 설정
dataset_path = "/home/han/robomimic_ws/src/robomimic/datasets/v4_0312/processed_data.hdf5"

with h5py.File(dataset_path, 'a') as f:
    for demo_name in f['data'].keys():
        demo_group = f[f'data/{demo_name}']
        
        # 1. 기존 7차원 actions 삭제
        if 'actions' in demo_group:
            del demo_group['actions']
        
        # 2. action_dict에서 요소 가져오기
        rel_pos = demo_group['action_dict/rel_pos'][:]      # (N, 3)
        rel_rot_6d = demo_group['action_dict/rel_rot_6d'][:] # (N, 6)
        gripper = demo_group['action_dict/gripper'][:]       # (N, 1)
        
        # 3. 10차원 데이터로 결합 (N, 10)
        new_actions = np.concatenate([rel_pos, rel_rot_6d, gripper], axis=-1)
        
        # 4. 새로운 actions 데이터셋 생성
        demo_group.create_dataset('actions', data=new_actions)
        
        print(f"{demo_name} 변환 완료: 차원 {new_actions.shape}")

print("모든 데모의 Action이 10차원(6D Rotation)으로 교체되었습니다.")