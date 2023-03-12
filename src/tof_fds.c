/*******************************************************************************
    BSD 3-Clause License

    Copyright (c)  2021-present  Andrew Green

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *******************************************************************************/

#include "tof_fds.h"

#include <string.h>

#include "fds.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "timer_delay.h"

static fds_record_t record;
static fds_record_desc_t record_desc;

static bool fds_registered = false;
static bool fds_inited = false;

static volatile uint8_t m_tof_dfs_active = false;

static ret_code_t tof_fds_gc(void);

static const char* fds_evt_id_str(fds_evt_id_t evt_id){
    switch (evt_id){
        case FDS_EVT_INIT:
            return "init";
        case FDS_EVT_WRITE:
            return "write";
        case FDS_EVT_UPDATE:
            return "update";
        case FDS_EVT_DEL_RECORD:
            return "del_rec";
        case FDS_EVT_DEL_FILE:
            return "del_file";
        case FDS_EVT_GC:
            return "gc";
        default:
            return "unknown";
    }
}

static const char* fds_evt_result_str(ret_code_t status){
    switch (status){
        case NRF_SUCCESS:
            return "ok";
        case FDS_ERR_OPERATION_TIMEOUT:
            return "timeout";
        case FDS_ERR_NOT_INITIALIZED:
            return "init";
        case FDS_ERR_UNALIGNED_ADDR:
            return "unaligned";
        case FDS_ERR_INVALID_ARG:
            return "invalid arg";
        case FDS_ERR_NO_OPEN_RECORDS:
            return "no open rec";
        case FDS_ERR_NO_SPACE_IN_FLASH:
            return "no space in flash";
        case FDS_ERR_NO_SPACE_IN_QUEUES:
            return "no space in queue";
        case FDS_ERR_RECORD_TOO_LARGE:
            return "too large";
        case FDS_ERR_NOT_FOUND:
            return "not found";
        case FDS_ERR_NO_PAGES:
            return "no pages";
        case FDS_ERR_USER_LIMIT_REACHED:
            return "user limit";
        case FDS_ERR_CRC_CHECK_FAILED:
            return "crc err";
        case FDS_ERR_BUSY:
            return "busy";
        case FDS_ERR_INTERNAL:
            return "internal";
        default:
            return "unknown";
    }
}

// Simple event handler to handle errors during initialization.
static void fds_evt_handler(fds_evt_t const *p_fds_evt){
//    switch (p_fds_evt->id){
//        case FDS_EVT_INIT:
//            break;
//        case FDS_EVT_WRITE:
//        case FDS_EVT_UPDATE:
//            break;
//        case FDS_EVT_DEL_RECORD:
//        case FDS_EVT_DEL_FILE:
//            break;
//        case FDS_EVT_GC:
//            break;
//        default:
//            break;
//    }

    NRF_LOG_INFO("FDS: evt id: %s, status: %s", fds_evt_id_str(p_fds_evt->id), fds_evt_result_str(p_fds_evt->result));

    m_tof_dfs_active = false;
}

void tof_fds_init(void) {
    ret_code_t status = NRF_SUCCESS;

    if(!fds_registered){
        status = fds_register(fds_evt_handler);
        if (!status){
            fds_registered = true;
        }
        else{
            NRF_LOG_INFO("FDS: evt_handler register failed, status: %s", fds_evt_result_str(status));
        }
    }

    if(!fds_inited){
        m_tof_dfs_active = true;
        status = fds_init();
        if (!status){
            wfe(&m_tof_dfs_active);
            fds_inited = true;
        }
    }
}

ret_code_t tof_fds_write(uint16_t file_id, uint16_t record_key,uint8_t* data, size_t data_len) {
    uint16_t lfile_id = file_id;
    uint16_t lrecord_key = record_key;

    // Set up record.
    record.file_id = file_id;
    record.key = lrecord_key;
    record.data.p_data = data;
    // The following calculation takes into account any eventual remainder of the division
    record.data.length_words = (data_len + 3) / 4;;

    NRF_LOG_INFO("FDS: writing %d bytes to file 0x%X rec 0x%X", data_len, record.file_id, record.key);

    // It is required to zero the token before first use.
    fds_find_token_t ftok;
    memset(&ftok, 0x00, sizeof(fds_find_token_t));

    // Does record exists?
    ret_code_t status = fds_record_find(lfile_id, lrecord_key, &record_desc, &ftok);
    if (!status){
        // Yes - Use update instead of write
        m_tof_dfs_active = true;
        status = fds_record_update(&record_desc, &record);
        if (!status){
            wfe(&m_tof_dfs_active);
        }
        else{
            return status;
        }
    }
    else if(FDS_ERR_NOT_FOUND == status){
        // No - Create the record
        m_tof_dfs_active = true;
        status = fds_record_write(&record_desc, &record);
        if (!status){
            wfe(&m_tof_dfs_active);
        }
        else{
            return status;
        }
    }
    else{
        return status;
    }

    status = tof_fds_gc();

    return status;
}

ret_code_t tof_fds_read(uint16_t file_id, uint16_t record_key, uint8_t* data, size_t data_len) {
    uint16_t lfile_id = file_id;
    uint16_t lrecord_key = record_key;

    /* It is required to zero the token before first use. */
    fds_find_token_t ftok;
    memset(&ftok, 0x00, sizeof(fds_find_token_t));

    /* Find the record */
    ret_code_t status = fds_record_find(lfile_id, lrecord_key, &record_desc, &ftok);
    if (NRF_SUCCESS == status){
        /* Open the record */
        fds_flash_record_t flash_record;
        status = fds_record_open(&record_desc, &flash_record);
        if (NRF_SUCCESS == status) {
            NRF_LOG_INFO("FDS: opened file 0x%X rec 0x%X, status: %s", lfile_id, lrecord_key, fds_evt_result_str(status));
            memcpy(data, flash_record.p_data, data_len);
        }
        else{
            NRF_LOG_INFO("FDS: could not open file 0x%X rec 0x%X, status: %s", lfile_id, lrecord_key, fds_evt_result_str(status));
        }

        /* Close the record when done. */
        status = fds_record_close(&record_desc);
        if (NRF_SUCCESS != status) {
            NRF_LOG_INFO("FDS: could not close file 0x%X rec 0x%X, status: %s", lfile_id, lrecord_key, fds_evt_result_str(status));
        }
    }
    else{
        NRF_LOG_INFO("FDS: could not find file 0x%X rec 0x%X, status: %s", lfile_id, lrecord_key, fds_evt_result_str(status));
    }

    return status;
}

ret_code_t tof_fds_delete(uint16_t file_id, uint16_t record_key) {
    uint16_t lfile_id = file_id;
    uint16_t lrecord_key = record_key;

    /* It is required to zero the token before first use. */
    fds_find_token_t ftok;
    memset(&ftok, 0x00, sizeof(fds_find_token_t));

    ret_code_t status = NRF_SUCCESS;

    NRF_LOG_INFO("FDS: deleting file 0x%X rec 0x%X", lfile_id, lrecord_key);

    /* Find all records with file_id and delete them */
    while(NRF_SUCCESS == fds_record_find(lfile_id, lrecord_key, &record_desc, &ftok)){
        /* Delete the record */
        m_tof_dfs_active = true;
        status = fds_record_delete(&record_desc);
        if (!status){
            wfe(&m_tof_dfs_active);
            status = tof_fds_gc();
            if(status){
                break;
            }
        }
        else{
            break;
        }
    }

    return status;
}

static ret_code_t tof_fds_gc(void){
    m_tof_dfs_active = true;
    ret_code_t status = fds_gc();
    if (!status){
        wfe(&m_tof_dfs_active);
    }
    return status;
}
