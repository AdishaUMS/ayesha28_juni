function onload_update() {
    var torque_en = []

    for(let i = 0; i < ADISHA_DXL_NUM; i++) {
        torque_en.push(0)
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

    for(let i = 0; i < ADISHA_DXL_NUM; i++) {
        let tpos_element = document.getElementById(`tpos_${ADISHA_DXL_ID[i]}`)

        tpos_element.addEventListener('keydown', (event) => {
            if(event.key == 'ArrowUp') {
                tpos_element.value = parseInt(tpos_element.value, 10) + 1
            }
            else if(event.key == 'ArrowDown') {
                tpos_element.value = parseInt(tpos_element.value, 10) - 1
            }
            if(event.key == 'ArrowUp' || event.key == 'ArrowDown') {
                let goal_pos = []
        
                for(let i = 0; i < ADISHA_DXL_NUM; i++) {
                    let val = parseInt(document.getElementById(`tpos_${ADISHA_DXL_ID[i]}`).value, 10)
                    goal_pos.push(val)
                }
                
                fetch(`${ADISHA_URL}/api/set_position`, {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({
                        goal_position: goal_pos
                    })
                })
                    .then(response => response.json())
                    .then(data => {
                        console.log(data.message);
                    })
                    .catch(error => {
                        console.error('Error: ', error);
                    });
            }
        })
    }

    document.getElementById('filename_entry').addEventListener('keypress', (event) => {
        if(event.key == 'Enter') {
            event.preventDefault()
            save_pose()
        }
    })
}



function onclick_pose(fname) {
    fetch(`${ADISHA_URL}/api/get_pose_value`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            filename: fname
        })
    })
        .then(response => response.json())
        .then(data => {
            for(let i = 0; i < ADISHA_DXL_NUM; i++) {
                document.getElementById(`spos_${ADISHA_DXL_ID[i]}`).innerHTML = data.val[i]
            }
        })
        .catch(error => {
            console.error('Error: ', error);
        });

    document.getElementById('pose_select_overlay').classList.add('hidden')
    document.getElementById('pose_select').classList.add('hidden')
}



function button_show_saved_poses() {
    fetch(`${ADISHA_URL}/api/get_saved_poses`)
        .then(response => {
            if(!response.ok) {
                throw new Error(`Bad response from ${ADISHA_URL}/api/get_saved_poses`)
            }
            return response.json()
        })
        .then(data => {
            var poses   = data.poses
            var parent  = document.getElementById('pose_item')

            while(parent.firstChild) {
                parent.removeChild(parent.firstChild)
            }

            if(poses.length != 0) {
                for(let i = 0; i < poses.length; i++) {
                    let item    = document.createElement('tr')
                    let no      = document.createElement('th')
                    let file    = document.createElement('td')

                    item.className  = 'bg-gray-800 border-b border-gray-700 hover:bg-gray-600'
                    no.className    = 'px-3 py-3 font-medium text-white whitespace-nowrap'
                    file.className  = 'px-20 py-3 font-medium text-white'

                    item.addEventListener('click', () => {
                        onclick_pose(poses[i])
                    })

                    no.innerHTML    = `${i + 1}`
                    file.innerHTML  = `${poses[i]}`

                    item.appendChild(no)
                    item.appendChild(file)
                    parent.appendChild(item)
                }
            }

            else {
                parent.innerHTML += `
                    <tr class="bg-gray-800 border-b border-gray-700 hover:bg-gray-600">
                        <th scope="row" class="px-3 py-3 font-medium text-white whitespace-nowrap">
                            -
                        </th>
                        <td class="px-20 py-3 text-white">
                            No pose file exists
                        </td>
                    </tr>
                `
            }
        })
        .catch(error => {
            console.error('Error fetching data: ', error)
        })

    document.getElementById('pose_select_overlay').classList.remove('hidden')
    document.getElementById('pose_select').classList.remove('hidden')
}



function button_show_save_prompt() {
    document.getElementById('filename_overlay').classList.remove('hidden')
    document.getElementById('filename').classList.remove('hidden')
}



function button_play_pose() {
    for(let i = 0; i < ADISHA_DXL_NUM; i++) {
        document.getElementById(`tpos_${ADISHA_DXL_ID[i]}`).value = document.getElementById(`spos_${ADISHA_DXL_ID[i]}`).innerText
    }
    
    let goal_pos = []
        
    for(let i = 0; i < ADISHA_DXL_NUM; i++) {
        let val = parseInt(document.getElementById(`tpos_${ADISHA_DXL_ID[i]}`).value, 10)
        goal_pos.push(val)
    }
    
    fetch(`${ADISHA_URL}/api/set_position`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            goal_position: goal_pos
        })
    })
        .then(response => response.json())
        .then(data => {
            console.log(data.message);
        })
        .catch(error => {
            console.error('Error: ', error);
        });
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
    var torque_en           = []
    var check_num           = 0
    let play_pose_button    = document.getElementById("play_pose_button")

    for(let i = 0; i < ADISHA_DXL_NUM; i++) {
        let checked = document.getElementById(`torque_en_${ADISHA_DXL_ID[i]}`).checked
        check_num   = (checked) ? check_num + 1 : check_num
        torque_en.push((checked) ? 1 : 0)
    }

    if(check_num == ADISHA_DXL_NUM) {
        play_pose_button.disabled = false
        play_pose_button.classList.replace("text-gray-950", "text-white")
    }
    else {
        play_pose_button.disabled = true
        play_pose_button.classList.replace("text-white", "text-gray-950")
    }

    fetch(`${ADISHA_URL}/api/set_torque`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({goal_torque: torque_en})
    })
        .then(response => response.json())
        .then(data => {
            console.log(data.message);

            for(let i = 0; i < ADISHA_DXL_NUM; i++) {
                var tpos_entry = document.getElementById(`tpos_${ADISHA_DXL_ID[i]}`)

                if(torque_en[i] == 1) {
                    tpos_entry.value    = document.getElementById(`pos_${ADISHA_DXL_ID[i]}`).innerText
                    tpos_entry.disabled = false
                    tpos_entry.classList.replace('text-gray-600', 'text-white')
                }

                else {
                    tpos_entry.value    = 0
                    tpos_entry.disabled = true
                    tpos_entry.classList.replace('text-white', 'text-gray-600')
                }
            }
        })
        .catch(error => {
            console.error('Error:', error);
        });
}



function cancel_saved_pose() {
    document.getElementById("pose_select_overlay").classList.add("hidden");
    document.getElementById("pose_select").classList.add("hidden");
}



function save_pose() {
    var file_name   = document.getElementById("filename_entry").value
    var pres_pos    = []

    for(let i = 0; i < ADISHA_DXL_NUM; i++) {
        let val = parseInt(document.getElementById(`pos_${ADISHA_DXL_ID[i]}`).innerHTML, 10)
        pres_pos.push(val)
        document.getElementById(`spos_${ADISHA_DXL_ID[i]}`).innerHTML = val
    }

    fetch(`${ADISHA_URL}/api/save_pose`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            filename: file_name,
            val: pres_pos
        })
    })
        .then(response => response.json())
        .then(data => {
            console.log(data.message);
        })
        .catch(error => {
            console.error('Error: ', error);
        });

    
    document.getElementById("filename_overlay").classList.add("hidden");
    document.getElementById("filename").classList.add("hidden");
}



function cancel_save_pose() {
    document.getElementById("filename_overlay").classList.add("hidden");
    document.getElementById("filename").classList.add("hidden");
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
            }
        })
        .catch(error => {
            console.error('Error fetching data: ', error)
        })
}

setInterval(status_table_update, ADISHA_MASTER_CLOCK_MS)