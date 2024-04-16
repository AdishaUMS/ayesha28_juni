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

    if(!document.getElementById('play_pose_button').disabled) {
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
}

setInterval(status_table_update, ADISHA_MASTER_CLOCK_MS)