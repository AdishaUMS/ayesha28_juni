const fs        = require('fs')
const path      = require('path')
const jsyaml    = require('js-yaml')

const ADISHA_DATA_PATH      = path.join(__dirname, '../../../../../adisha_data')
const ROBOT_INFO_YAML_PATH  = path.join(ADISHA_DATA_PATH, 'config/robot_info.yaml')
const JOINT_INFO_YAML_PATH  = path.join(ADISHA_DATA_PATH, 'config/joint_info.yaml')
const CORE_INFO_YAML_PATH   = path.join(ADISHA_DATA_PATH, 'config/core_info.yaml')

const ROBOT_INFO    = jsyaml.load(fs.readFileSync(ROBOT_INFO_YAML_PATH, 'utf8'))
const JOINT_INFO    = jsyaml.load(fs.readFileSync(JOINT_INFO_YAML_PATH, 'utf8'))
const CORE_INFO     = jsyaml.load(fs.readFileSync(CORE_INFO_YAML_PATH, 'utf8'))

module.exports.adisha_data_path     = ADISHA_DATA_PATH
module.exports.robot_info_yaml_path = ROBOT_INFO_YAML_PATH
module.exports.joint_info_yaml_path = JOINT_INFO_YAML_PATH

module.exports.robot_info   = ROBOT_INFO
module.exports.joint_info   = JOINT_INFO
module.exports.core_info    = CORE_INFO