'''
A simplified teleop server, used for testing connection between teleop server and franka server.
This script sends commands to move the robot arm along the x-axis.
'''

import threading
import time
import uvicorn
import requests
import numpy as np
from loguru import logger

from reactive_diffusion_policy.common.precise_sleep import precise_wait
class TeleopServerTest:
    def __init__(self, 
                 robot_server_ip="0.0.0.0", 
                 robot_server_port=8092, 
                 step=1/2400, 
                 num_steps=600, 
                 interval=1/60):
        '''
        robot_server_ip: The IP address of the FASTAPI server running the teleoperation API.
        robot_server_port: The port number of the FASTAPI server running the teleoperation API.
        Initialize the teleoperation server test.
        step: The step size for each movement along the x-axis.
        num_steps: The number of steps to take along the x-axis.
        interval: The time interval between each step in seconds.
        '''
        self.robot_server_ip = robot_server_ip
        self.robot_server_port = robot_server_port
        self.control_cycle_time = interval
        self.step = step
        self.num_steps = num_steps
        self.session = requests.session()
        self._stop_flag = False
        
    def send_command(self, endpoint: str, data: dict = None):
        url = f"http://{self.robot_server_ip}:{self.robot_server_port}{endpoint}"
        if 'get' in endpoint:
            response = self.session.get(url)
        else:
            if 'move' in endpoint:
                try:
                    response = self.session.post(url, json=data, timeout=0.001)
                    logger.info(f"Request to {url} succeeded with data: {data}")
                except requests.exceptions.ReadTimeout as e:
                    logger.warning(f"Request to {url} timed out: {e}")
                    response = None
            else:
                response = self.session.post(url, json=data)

        if response is not None:
            response.raise_for_status()  
            return response.json()
        else:
            return dict()


    def process_cmd(self):
        '''
        Process the command to move the robot arm along the x-axis.
        '''
        logger.info("Waiting for the franka server to initialize...")
        time.sleep(2) 
        logger.info("Franka server initialized. Starting to move along x axis...")

        init_state = self.send_command('/get_current_tcp/left')  # (x, y, z, qw, qx, qy, qz), in flange coordinate
        init_tcp = np.array(init_state)
        if init_state is None:
            logger.error("Failed to get initial TCP, aborting test.")
            return
        
        t_start = time.monotonic()
        iter_idx = 0
        
        for i in range(self.num_steps):
            if self._stop_flag:
                break

            target_tcp = init_tcp.copy()
            target_tcp[0] += (i + 1) * self.step
            self.send_command('/move_tcp/left', {'target_tcp': target_tcp.tolist()})
            logger.info(f"Step {i+1}: Move to {target_tcp[:3]}")

            t_wait_util = t_start + (iter_idx + 1) * self.control_cycle_time
            precise_wait(t_wait_util, time_func=time.monotonic)
            iter_idx += 1


        logger.info("Test finished. Terminating policy...")

    def run(self):
        test_thread = threading.Thread(target=self.process_cmd, daemon=True)
        try:
            test_thread.start()
            while test_thread.is_alive():
                time.sleep(0.2)
        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt received, stopping test...")
            self._stop_flag = True
            test_thread.join()
        finally:
            logger.info("TeleopServerTest finished.")

if __name__ == "__main__":
    server = TeleopServerTest()
    server.run()