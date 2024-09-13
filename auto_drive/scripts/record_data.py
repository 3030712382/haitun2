# -*-encoding:utf-8-*-
import time
import os
import numpy as np
import argparse
import matplotlib.pyplot as plt
import h5py
import IPython
e = IPython.embed

class hdf5data:
    def __init__(self,dataset_dir="/home/hp-t4/data/berthing",episode_idx=50,camera_names=["front"]):
        """
        For each timestep:
        - action                  (2,)         'float64'   [throttle rudder] 油门舵角
        - targetstate               (2,)         'float64'   [u  yaw] 期望航速航向
        observations:
        - images
            - each_cam_name     (480, 640, 3) 'uint8'
        - qpos                    (6,)         'float64' [x y z pitch roll yaw] 无人艇状态：三轴位置及角度
        - qvel                      (6,)         'float64' [u v r dpitch droll dyaw] 无人艇速度
        - motion                  (2,)         'float64'   [thrust rudder] 动力系统：转速舵角
        """
        self.dataset_dir = dataset_dir
        self.hd_dir = "episode_1"
        self.episode_idx = episode_idx
        self.camera_names = camera_names
        self.data_dict = {
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/observations/motion': [],
            '/action': [],
            '/targetstate': [],
        }
        for cam_name in self.camera_names:
            self.data_dict[f'/observations/images/{cam_name}'] = []
    def update_data(self,pos,vel,motion,action,targetstate,cameras):
            self.data_dict['/observations/qpos'].append(pos)
            self.data_dict['/observations/qvel'].append(vel)
            self.data_dict['/observations/motion'].append(motion)
            self.data_dict['/action'].append(action)
            self.data_dict['/targetstate'].append(targetstate)
            for cam_name in self.camera_names:
                self.data_dict[f'/observations/images/{cam_name}'].append(cameras[cam_name])
    def save_data(self):
        # HDF5
        t0 = time.time()
        max_timesteps = len(self.data_dict['/action'])
        dataset_path = os.path.join(self.dataset_dir,self.hd_dir)
        # dataset_path = os.path.join(self.dataset_dir, f'episode_{self.episode_idx}')#保存在指定路径下 第episode_idx轮数据
        with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            for cam_name in self.camera_names:
                _ = image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8',
                                         chunks=(1, 480, 640, 3), )
            # compression='gzip',compression_opts=2,)
            # compression=32001, compression_opts=(0, 0, 0, 0, 9, 1, 1), shuffle=False)
            qpos = obs.create_dataset('qpos', (max_timesteps, 6))
            qvel = obs.create_dataset('qvel', (max_timesteps, 6))
            motion = obs.create_dataset('motion', (max_timesteps, 2))
            action = root.create_dataset('action', (max_timesteps, 2))
            targetstate = root.create_dataset('targetstate', (max_timesteps, 2))

            for name, array in self.data_dict.items():
                # print(array)
                root[name][...] = array
                self.data_dict[name] = []
        print(f'Saved to {self.dataset_dir}')
        # self.data_dict.clear()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_name', action='store', type=str, help='task_name', required=True)
    parser.add_argument('--dataset_dir', action='store', type=str, help='dataset saving dir', required=True)
    parser.add_argument('--num_episodes', action='store', type=int, help='num_episodes', required=False)
    parser.add_argument('--onscreen_render', action='store_true')
    
    # main(vars(parser.parse_args()))

