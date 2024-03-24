const rclnodejs = require('rclnodejs')
const path      = require('path')
const fs        = require('fs')
const jsyaml    = require('js-yaml')
const data      = require('./data')


class RclGateAway {
 
    constructor(data) {
        this.robot_info = data.robot_info
        this.joint_info = data.joint_info

        this.response_msg = {
            present_torque: null,
            present_position: null,
            present_velocity: null,
            present_load: null
        }
        this.request_msg = {
            goal_torque: {val: null},
            goal_position: {val: null},
            goal_velocity: {val: null}
        }

        this.set_torque = false
        this.set_pos    = false
        this.set_vel    = false
    }


    nodeInit() {
        rclnodejs.init().then(() => {

            const node = new rclnodejs.Node(`${this.robot_info.robot_id}_WebAppServerNode`)

            node.createSubscription(
                'adisha_interface/msg/JointTorque',
                `${this.robot_info.robot_id}/present_torque`,
                (msg) => {
                    this.response_msg.present_torque = msg.val
                }
            ) 

            node.createSubscription(
                'adisha_interface/msg/JointPosition',
                `${this.robot_info.robot_id}/present_position`,
                (msg) => {
                    this.response_msg.present_position = msg.val
                }
            )

            node.createSubscription(
                'adisha_interface/msg/JointVelocity',
                `${this.robot_info.robot_id}/present_velocity`,
                (msg) => {
                    this.response_msg.present_velocity = msg.val
                }
            )

            node.createSubscription(
                'adisha_interface/msg/JointLoad',
                `${this.robot_info.robot_id}/present_load`,
                (msg) => {
                    this.response_msg.present_load = msg.val
                }
            )

            const torque_pub = node.createPublisher(
                'adisha_interface/msg/JointTorque',
                `${this.robot_info.robot_id}/goal_torque`
            )

            const pos_pub = node.createPublisher(
                'adisha_interface/msg/JointPosition',
                `${this.robot_info.robot_id}/goal_position`
            )

            const vel_pub = node.createPublisher(
                'adisha_interface/msg/JointVelocity',
                `${this.robot_info.robot_id}/goal_velocity`
            )

            setInterval(() => {
                if(this.set_torque) {
                    this.set_torque = false
                    torque_pub.publish(this.request_msg.goal_torque)
                }

                if(this.set_pos) {
                    this.set_pos = false
                }

                if(this.set_vel) {
                    this.set_vel = false
                }
            }, 50);
            
            node.spin()
        })
    }


    getResponseMessage() {
        return this.response_msg
    }


    setTorque(value) {
        this.set_torque = true
        this.request_msg.goal_torque.val = value
    }


    savePose(filename, value) {
        const yaml_str  = jsyaml.dump({val: value})
        const file_path = path.join(data.adisha_data_path, `pose_studio/${filename}.yaml`)
        fs.writeFileSync(file_path, yaml_str)
    }


    getSavedPose() {
        const dir_path = path.join(data.adisha_data_path, 'pose_studio')
        try {
            return fs.readdirSync(dir_path)
        }
        catch(error) {
            return []
        }
    }
}


module.exports = RclGateAway