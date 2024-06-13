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
        self.declare_parameter('dxl_type', rclpy.Parameter.Type.STRING_ARRAY)
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
        self.DXL_TYPE       = self.get_parameter('dxl_type').value
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
        speed_res   = []
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
    


    def calculateSpeed(self, angle:list, dt_ms:float, dxl_type:str) -> list:
        XL320_DPB       = 0.29
        AX12A_DPB       = 0.29
        MX28_DPB        = 0.088
        XL320_DPSPB     = 0.666
        AX12A_DPSPB     = 0.666
        MX28_DPSPB      = 0.684
        XL320_SPEED_MAX = 1023
        AX12A_SPEED_MAX = 1023
        MX28_SPEED_MAX  = 1023

        data_len    = len(angle)
        speed_res   = []

        for i in range(data_len):
            if i == 0:
                speed_res.append(512)
                continue

            if dxl_type == 'XL320':
                DPB_CONST   = XL320_DPB
                DPSPB_CONST = XL320_DPSPB
                MAX_CONST   = XL320_SPEED_MAX
            elif dxl_type == 'AX12A':
                DPB_CONST   = AX12A_DPB
                DPSPB_CONST = AX12A_DPSPB
                MAX_CONST   = AX12A_SPEED_MAX
            elif dxl_type == 'MX28':
                DPB_CONST   = MX28_DPB
                DPSPB_CONST = MX28_DPSPB
                MAX_CONST   = MX28_SPEED_MAX

            angle_dist  = float(np.abs(angle[i] - angle[i - 1]))*DPB_CONST
            target_time = dt_ms*1e-3
            omega       = angle_dist/target_time
            angle_speed = int(np.round(omega/DPSPB_CONST))
            if angle_speed > MAX_CONST: angle_speed = MAX_CONST

            speed_res.append(angle_speed)

        return speed_res



    def executeCombiner(self) -> None:
        result_data = {'dt_ms': self.DT_MS}
        
        for i in range(self.DXL_NUM):
            angle_data, time_data = self.bezierInterpolate(
                angle           = self.joint_data[self.DXL_ID[i]]['angle'],
                time_ms         = self.joint_data[self.DXL_ID[i]]['time'],
                q_proportion    = self.Q_PROP,
                dt_ms           = float(self.DT_MS)
            )

            speed_data = self.calculateSpeed(
                angle       = angle_data,
                time_ms     = float(self.DT_MS),
                dxl_type    = self.DXL_TYPE[i]
            )

            if i == 0:
                points_num = len(angle_data)
                result_data.update({'points_num': points_num})

            result_data.update({
                self.DXL_ID[i]: {
                    'angle': angle_data.copy(),
                    'time': time_data.copy(),
                    'speed': speed_data.copy()
                }
            })

        with open(self.OUTPUT_PATH, 'w') as file:
            yaml.safe_dump(result_data, file)

        self.get_logger().info(f'Result saved at {self.OUTPUT_PATH}')