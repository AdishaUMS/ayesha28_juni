function on_load_update() {
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