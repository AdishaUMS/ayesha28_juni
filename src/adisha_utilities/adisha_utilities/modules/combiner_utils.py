import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node



class CombinerUtils(Node):

    def __init__(self) -> None:
        super().__init__('CombinerUtils')
        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('dxl_id', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('dxl_num', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('dt_ms', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('q_prop', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ps_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('arm_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('leg_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('output_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('arm_id', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('leg_id', rclpy.Parameter.Type.INTEGER_ARRAY)

        self.ID             = self.get_parameter('id').value
        self.DXL_ID         = self.get_parameter('dxl_id').value
        self.DXL_NUM        = self.get_parameter('dxl_num').value
        self.DT_MS          = self.get_parameter('dt_ms').value
        self.Q_PROP         = self.get_parameter('q_prop').value
        self.PS_PATH        = self.get_parameter('ps_path').value
        self.ARM_PATH       = self.get_parameter('arm_path').value
        self.LEG_PATH       = self.get_parameter('leg_path').value
        self.OUTPUT_PATH    = self.get_parameter('output_path').value
        self.ARM_ID         = self.get_parameter('arm_id').value
        self.LEG_ID         = self.get_parameter('leg_id').value

        self.joint_data = dict(())

        for dxl_id in self.DXL_ID:
            self.joint_data.update({
                dxl_id: {
                    'angle': [],
                    'time': []
                }
            })

        with open(self.ARM_PATH, 'r') as file:
            self.arm_file = yaml.safe_load(file)['val']

        with open(self.LEG_PATH, 'r') as file:
            self.leg_file = yaml.safe_load(file)['val']

        for i in range(len(self.arm_file)):
            pose_fn     = self.arm_file[i][0]
            delay       = self.arm_file[i][1]
            duration    = self.arm_file[i][2]
            init_pose   = True if i == 0 else False

            with open(os.path.join(self.PS_PATH, pose_fn), 'r') as file:
                joint_val = list(yaml.safe_load(file)['val'])

            for dxl_id in range(self.ARM_ID[0], self.ARM_ID[1] + 1):
                if (not init_pose) and (delay > 0):
                    self.joint_data[dxl_id]['angle'].append(
                        self.joint_data[dxl_id]['angle'][-1]
                    )
                    self.joint_data[dxl_id]['time'].append(
                        self.joint_data[dxl_id]['time'][-1] + delay
                    )

                angle_val = joint_val[self.DXL_ID.index(dxl_id)]
                self.joint_data[dxl_id]['angle'].append(angle_val)

                if init_pose:
                    self.joint_data[dxl_id]['time'].append(0)

                else:
                    self.joint_data[dxl_id]['time'].append(
                        self.joint_data[dxl_id]['time'][-1] + duration
                    )

        for i in range(len(self.leg_file)):
            pose_fn     = self.leg_file[i][0]
            delay       = self.leg_file[i][1]
            duration    = self.leg_file[i][2]
            init_pose   = True if i == 0 else False

            if duration <= 0:
                self.get_logger().warn('Pose duration must be greater than 0')

            with open(os.path.join(self.PS_PATH, pose_fn), 'r') as file:
                joint_val = list(yaml.safe_load(file)['val'])

            for dxl_id in range(self.LEG_ID[0], self.LEG_ID[1] + 1):
                if (not init_pose) and (delay > 0):
                    self.joint_data[dxl_id]['angle'].append(
                        self.joint_data[dxl_id]['angle'][-1]
                    )
                    self.joint_data[dxl_id]['time'].append(
                        self.joint_data[dxl_id]['time'][-1] + delay
                    )

                angle_val = joint_val[self.DXL_ID.index(dxl_id)]
                self.joint_data[dxl_id]['angle'].append(angle_val)

                if init_pose:
                    self.joint_data[dxl_id]['time'].append(0)

                else:
                    self.joint_data[dxl_id]['time'].append(
                        self.joint_data[dxl_id]['time'][-1] + duration
                    )



    def calculateS(self, tk0:float, tk0p:float, tk1m:float, tk1:float, tau:float) -> float:
        c1 = -tk0 + 3*tk0p - 3*tk1m + tk1
        c2 = 3*tk0 - 6*tk0p + 3*tk1m
        c3 = -3*tk0 + 3*tk0p
        c4 = tk0 - tau

        s_n = 0.5
        for i in range(10):
            f_val       = c1*(s_n**3.0) + c2*(s_n**2.0) + c3*s_n + c4
            f_dot_val   = 3.0*c1*(s_n**2.0) + 2.0*c2*s_n + c3
            s_n         = s_n - f_val/f_dot_val

        return s_n



    def bezierInterpolate(self, angle:list, time_ms:list, q_proportion:float, dt_ms:float) -> tuple:
        if len(angle) != len(time_ms): raise Exception('"angle" and "time" lists length must be the same')

        num_data    = len(angle)
        timestamp   = np.arange(time_ms[0], time_ms[-1], dt_ms, dtype=int).tolist()
        tau_plus    = []
        tau_min     = []

        for i in range(num_data):
            if i == 0: continue
            tau_plus.append(time_ms[i - 1] + (time_ms[i] - time_ms[i - 1])*q_proportion)
            tau_min.append(time_ms[i - 1] + (time_ms[i] - time_ms[i - 1])*(1.0 - q_proportion))

        angle_res   = []
        idx         = 1
        for t in timestamp:
            if t > time_ms[idx]: idx += 1
            
            s_val = self.calculateS(
                time_ms[idx - 1], 
                tau_plus[idx - 1], 
                tau_min[idx - 1], 
                time_ms[idx], 
                t
            )

            angle_res.append(
                int(np.round(((1.0 - s_val)**3.0)*angle[idx - 1] + 
                (3.0*s_val*(1.0 - s_val)**2.0)*angle[idx - 1] + 
                (3.0*(s_val**2.0)*(1.0 - s_val))*angle[idx] + 
                (s_val**3.0)*angle[idx]))
            )

        return angle_res, timestamp
    


    def executeCombiner(self) -> None:
        result_data = {'dt_ms': self.DT_MS}
        
        for dxl_id in self.DXL_ID:
            angle_data, time_data = self.bezierInterpolate(
                angle           = self.joint_data[dxl_id]['angle'],
                time_ms         = self.joint_data[dxl_id]['time'],
                q_proportion    = self.Q_PROP,
                dt_ms           = self.DT_MS
            )

            result_data.update({
                dxl_id: {
                    'angle': angle_data.copy(),
                    'time': time_data.copy()
                }
            })

        with open(self.OUTPUT_PATH, 'w') as file:
            yaml.safe_dump(result_data, file)

        self.get_logger().info(f'Result saved at {self.OUTPUT_PATH}')