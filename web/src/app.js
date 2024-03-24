const path          = require('path')
const express       = require('express')
const data          = require('./js/data')
const rclgateaway   = require('./js/rclgateaway')

const app       = express()
const port      = 3000
const rclnode   = new rclgateaway(data)

app.set('view engine', 'ejs')
app.set('views', path.join(__dirname, '../views'))
app.use(express.static(path.join(__dirname, '../public')))
app.use(express.urlencoded({extended: false}))
app.use(express.json())

rclnode.nodeInit()  



app.get(`/${data.robot_info.robot_id}`, (req, res) => {
    res.render('pages/status', {
        page_name: 'Status',
        robot_info: data.robot_info,
        joint_info: data.joint_info
    })
})

app.get(`/${data.robot_info.robot_id}/posestudio`, (req, res) => {
    res.render('pages/pose_studio', {
        page_name: 'Pose Studio',
        robot_info: data.robot_info,
        joint_info: data.joint_info
    })
})

app.get(`/${data.robot_info.robot_id}/motionsequencer`, (req, res) => {
    res.render('pages/motion_sequencer', {
        page_name: 'Motion Sequencer',
        robot_info: data.robot_info,
        joint_info: data.joint_info
    })
})

app.get(`/${data.robot_info.robot_id}/tune`, (req, res) => {
    res.render('pages/tune', {
        page_name: 'Tune',
        robot_info: data.robot_info,
        joint_info: data.joint_info
    })
})

app.get(`/${data.robot_info.robot_id}/play`, (req, res) => {
    res.render('pages/play', {
        page_name: 'Play',
        robot_info: data.robot_info,
        joint_info: data.joint_info
    })
})



app.get(`/${data.robot_info.robot_id}/api/get_joint_status`, (req, res) => {
    res.json(rclnode.getResponseMessage())
})

app.post(`/${data.robot_info.robot_id}/api/set_torque`, (req, res) => {
    const json_data = req.body
    rclnode.setTorque(json_data.goal_torque)

    res.status(200)
    res.json({message: 'Torques set'})
})

app.post(`/${data.robot_info.robot_id}/api/save_pose`, (req, res) => {
    const json_data = req.body
    rclnode.savePose(json_data.filename, json_data.val)

    res.status(200)
    res.json({message: 'Pose saved'})
})

app.get(`/${data.robot_info.robot_id}/api/get_saved_pose`, (req, res) => {
    res.json({poses: rclnode.getSavedPose()})
})



app.listen(port, () => {
    console.log(`Adisha App is running on ${data.robot_info.robot_ip}:${port}`)
})