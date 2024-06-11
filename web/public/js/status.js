function onload_update() {
    fetch(`${ADISHA_URL}/api/get_status`)
        .then(response => {
            if(!response.ok) {
                throw new Error(`Bad response from ${ADISHA_URL}/api/get_status`)
            }
            return response.json()
        })
        .then(data => {
            for(let i = 0; i < ADISHA_DXL_NUM; i++) {
                document.getElementById(`torque_en_${ADISHA_DXL_ID[i]}`).checked = data.present_torque[i]
            }
        })
        .catch(error => {
            console.error('Error fetching data: ', error)
        })
}



function button_change_view() {
    document.getElementById('table_a1').classList.toggle('hidden')
    document.getElementById('table_a2').classList.toggle('hidden')
    document.getElementById('table_b1').classList.toggle('hidden')
    document.getElementById('table_b2').classList.toggle('hidden')
}



function button_enable_all_torque() {
    for(let i = 0; i < ADISHA_DXL_NUM; i++) {
        document.getElementById(`torque_en_${ADISHA_DXL_ID[i]}`).checked = true    
    }
}



function button_disable_all_torque() {
    for(let i = 0; i < ADISHA_DXL_NUM; i++) {
        document.getElementById(`torque_en_${ADISHA_DXL_ID[i]}`).checked = false
    }
}



function button_apply_torque() {
    var torque_en = []

    for(let i = 0; i < ADISHA_DXL_NUM; i++) {
        torque_en.push(
            (document.getElementById(`torque_en_${ADISHA_DXL_ID[i]}`).checked) ? 1 : 0
        )
    }

    fetch(`${ADISHA_URL}/api/set_torque`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({goal_torque: torque_en})
    })
        .then(response => response.json())
        .then(data => {
            console.log(data.message);
        })
        .catch(error => {
            console.error('Error:', error);
        });
}



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