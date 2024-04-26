function status_table_update() {
    fetch(`${ADISHA_URL}/api/get_status`)
        .then(response => {
            if(!response.ok) {
                throw new Error(`Bad response from ${ADISHA_URL}/api/get_status`)
            }
            return response.json()
        })
        .then(data => {
            for(let i = 0; i < ADISHA_DXL_NUM; i++) {
                document.getElementById(`pos_${ADISHA_DXL_ID[i]}`).innerHTML = data.present_position[i]
                document.getElementById(`vel_${ADISHA_DXL_ID[i]}`).innerHTML = data.present_speed[i]
                document.getElementById(`temp_${ADISHA_DXL_ID[i]}`).innerHTML = data.joint_sensor.temperature[i].toFixed(2)
                document.getElementById(`volt_${ADISHA_DXL_ID[i]}`).innerHTML = data.joint_sensor.voltage[i].toFixed(2)
                document.getElementById(`load_${ADISHA_DXL_ID[i]}`).innerHTML = data.joint_sensor.load[i].toFixed(2)
            }
        })
        .catch(error => {
            console.error('Error fetching data: ', error)
        })
}

setInterval(status_table_update, ADISHA_MASTER_CLOCK_MS)