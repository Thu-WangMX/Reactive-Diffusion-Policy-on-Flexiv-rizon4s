'''
Test the interpolation of a robot's trajectory using a command queue.
Visualize the interpolated TCP trajectory using rerun (only x, y, z).
Note: This is a test script, not intended for real robot control.
Note: Rerun depends on python 3.9+, Create a virtual environment with python 3.9+ 
'''

import numpy as np
import time
from collections import deque
from loguru import logger
import enum
import rerun as rr

from reactive_diffusion_policy.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from reactive_diffusion_policy.common.precise_sleep import precise_wait

logger.add("interpolation_debug.log", format="{time} {level} {message}", level="INFO", rotation="10 MB")

class Command(enum.Enum):
    SCHEDULE_WAYPOINT = 2

class FrankaInterpolationController:
    def __init__(self, frequency=50):
        '''
        frequency: Low-Level Control frequency in Hz, lower frequency for testing purposes.
        '''
        self.frequency = frequency
        self.control_cycle_time = 1.0 / self.frequency
        self.command_queue = deque(maxlen=256)
        self.pose_interp = None
        self.last_waypoint_time = None
        self.all_targets = []

    def generate_test_trajectory(self):
        '''
        Generate a simple trajectory: translation along x-axis.
        duration: 100 seconds, 500 points.
        High-level control frequency is 5Hz
        '''
        curr_pose = [0.13, 0.54, 0.44, -0.03, 0.68, 0.73] # (x, y, z, rx, ry, rz)
        duration = 10.0
        num_points = 50
        curr_time = time.monotonic()
        for i in range(num_points):
            target_pose = curr_pose.copy()
            dx = (i + 1) * 0.2 / num_points
            target_pose[0] = curr_pose[0] + dx
            target_time = curr_time + (i + 1) * duration / num_points
            self.command_queue.append({
                'cmd': Command.SCHEDULE_WAYPOINT.value,
                'target_pose': target_pose,
                'target_time': target_time
            })

    def process_commands(self):
        '''
        Main loop to process commands and interpolate poses, visualize with rerun.
        '''
        # Initialize interpolator
        if self.pose_interp is None:
            curr_flange_pose = [0.13, 0.54, 0.44, -0.03, 0.68, 0.73]
            curr_time = time.monotonic()
            self.pose_interp = PoseTrajectoryInterpolator(
                times=[curr_time],
                poses=[curr_flange_pose]
            )
            self.last_waypoint_time = curr_time

        t_start = time.monotonic()
        iter_idx = 0

        logger.info("Start interpolation and visualization. Press Ctrl+C to exit.")
        try:
            while True:
                t_now = time.monotonic()
                flange_pos = self.pose_interp(t_now).tolist() # (x, y, z, rx, ry, rz)
                pos = np.array(flange_pos[:3])
                logger.info(f"[INTERP] t_now={t_now:.4f}, flange_pos={flange_pos}")

                # Rerun: log the current TCP position as a point
                # rr.log("tcp_traj", rr.Points3D([pos], colors=[(255,0,0)]))

                try:
                    command = self.command_queue.popleft()
                    if command['cmd'] == Command.SCHEDULE_WAYPOINT.value:
                        target_pose = command['target_pose']

                        # visualize the target pose
                        target_pos = np.array(target_pose[:3])
                        # self.all_targets.append(target_pos)
                        # rr.log("target_points", rr.Points3D([target_pos], colors=[(0,0,255)]))


                        curr_time = t_now + self.control_cycle_time
                        target_time = command['target_time']
                        logger.info(f"[WAYPOINT] target_time={target_time:.4f}, target_pos={target_pos}")
                        self.pose_interp = self.pose_interp.schedule_waypoint(
                            pose=target_pose,
                            time=target_time,
                            curr_time=curr_time,
                            last_waypoint_time=self.last_waypoint_time
                        )
                        self.last_waypoint_time = target_time
                except IndexError:
                    pass

                # if self.all_targets:
                #     rr.log("target_points", rr.Points3D(self.all_targets, colors=[(0,0,255)]))

                t_wait_util = t_start + (iter_idx + 1) * self.control_cycle_time
                precise_wait(t_wait_util, time_func=time.monotonic)
                iter_idx += 1
        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt received, stopping visualization.")
        finally:
            logger.info("Interpolation test completed.")

    def run(self):
        self.generate_test_trajectory()
        self.process_commands()

if __name__ == "__main__":
    controller = FrankaInterpolationController()
    controller.run()