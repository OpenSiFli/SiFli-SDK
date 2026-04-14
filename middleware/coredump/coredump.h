/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __COREDUMP_H__
#define __COREDUMP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "section.h"

/**
 * @brief Coredump error codes enumeration
 */
typedef enum
{
    /** No error */
    COREDUMP_ERR_NO   = 0,
    /** Backend is busy */
    COREDUMP_ERR_BUSY = 1,
    /** Backend is not ready */
    COREDUMP_ERR_BACKEND_NOT_READY  = 2,
    /** Invalid partition type */
    COREDUMP_ERR_INVALID_PART_TYPE  = 3,
    /** Erase operation failed */
    COREDUMP_ERR_ERASE_FAILED       = 4,
    /** Invalid parameter */
    COREDUMP_ERR_INVALID_PARAM      = 5,
    /** No space available */
    COREDUMP_ERR_NO_SPACE           = 6,
    /** File creation failed */
    COREDUMP_ERR_FILE_CREATE_FAILED = 7,
    /** Read operation failed */
    COREDUMP_ERR_READ_FAILED        = 8,
} coredump_err_code_t;

/**
 * @brief Backend state enumeration
 */
typedef enum
{
    COREDUMP_BACKEND_STATE_INVALID  = 0,
    /** backend is idle and ready to start write*/
    COREDUMP_BACKEND_STATE_IDLE     = 1,
    /** backend is busy and writing */
    COREDUMP_BACKEND_STATE_BUSY     = 2,
} coredump_backend_state_t;

/**
 * @brief Coredump type enumeration
 */
typedef enum
{
    /** Minimum coredump (minidump) - only essential crash information */
    COREDUMP_TYPE_MINIMUM = 0,
    /** Full coredump - complete memory dump */
    COREDUMP_TYPE_FULL
} coredump_type_t;

/**
 * @brief Query ID enumeration for backend information queries
 */
typedef enum
{
    /** query max size for write, return negative for error code,
     * arg is NULL.
     */
    COREDUMP_QUERY_MAX_SIZE,
    /** query remain size, return remain size or negative error code, arg is NULL */
    COREDUMP_QUERY_REMAIN_SIZE,
    /** query path for the backend, return path or NULL if not found,
     * arg is coredump_type_t variable to indicate coredump type
     */
    COREDUMP_QUERY_PATH,
    /** query whether coredump data is present, return max data size if present, 0 if not, negative is error code,
     * arg is NULL
     */
    COREDUMP_QUERY_DATA_PRESENT,
} coredump_query_id_t;

/**
 * @brief Memory region descriptor
 *
 * Defines a contiguous memory region to be included in the core dump.
 * Used for dumping static data regions like RW/ZI sections.
 */
typedef struct
{
    /** start address of the memory region */
    uint32_t start_addr;
    /** end address plus 1 of the memory region, for region addr 0x0 ~ 0x3, end_address is set to 0x4 */
    uint32_t end_addr;
} coredump_memory_region_t;

/**
 * @brief Register region descriptor
 *
 * Defines a peripheral register region to be captured in the core dump.
 */
typedef struct
{
    /** start address of the memory region */
    uint32_t start_addr;
    /** length of the register region in bytes*/
    size_t len;
} coredump_register_region_t;

/**
 * @brief Coredump data information structure
 */
typedef struct
{
    /** True if backend is file-based, false if partition-based */
    bool is_file;
    /** Address of minidump data */
    uint32_t minidump_addr;
    /** Size of minidump data in bytes */
    uint32_t minidump_size;
    /** Address of fulldump data, address of partition-based and file full path for file-based */
    uint32_t fulldump_addr;
    /** Size of fulldump data in bytes */
    uint32_t fulldump_size;
} coredump_data_t;

/**
 * @brief Backend initialization function pointer
 *
 * @param coredump_type Type of coredump (COREDUMP_TYPE_MINIMUM or COREDUMP_TYPE_FULL)
 * @return coredump_err_code_t Error code, COREDUMP_ERR_NO indicates success
 */
typedef coredump_err_code_t (*coredump_backend_init_t)(coredump_type_t coredump_type);

/**
 * @brief Backend start function pointer
 *
 * Starts the backend for coredump operations. Write operations are allowed after start.
 *
 * @return coredump_err_code_t Error code, COREDUMP_ERR_NO indicates success
 */
typedef coredump_err_code_t (*coredump_backend_start_t)(void);

/**
 * @brief Backend end function pointer
 *
 * Ends the backend for coredump operations.
 *
 * @return None
 */
typedef void (*coredump_backend_end_t)(void);

/**
 * @brief Backend write function pointer
 *
 * Writes buffer data to the backend.
 *
 * @param[in] buf Pointer to the data buffer to write
 * @param[in] len Length of data to write in bytes
 * @return size_t Number of bytes written
 */
typedef size_t (*coredump_backend_write_t)(uint8_t *buf, size_t len);

/**
 * @brief Backend read function pointer
 *
 * Reads data from the backend.
 *
 * @param[in] offset Offset from where to start reading
 * @param[out] buf Pointer to the data buffer to read into
 * @param[in] len Length of data to read in bytes
 * @return size_t Number of bytes read
 */
typedef size_t (*coredump_backend_read_t)(uint32_t offset, uint8_t *buf, size_t len);

/**
 * @brief Backend clear function pointer
 *
 * Clears all coredump data from the backend.
 *
 * @return coredump_err_code_t Error code, COREDUMP_ERR_NO indicates success
 */
typedef coredump_err_code_t (*coredump_backend_clear_t)(void);

/**
 * @brief Query information from the backend for the core dump
 *
 * @param id the query id to indicate what information to query
 * @param arg the argument for the query, ref to coredump_query_id_t
 *
 * @return query result, negative value is error code for non-address return value, non-negative value for query result
 */
typedef int32_t (*coredump_backend_query_t)(coredump_query_id_t id, void *arg);

/**
 * @brief Backend sync function pointer
 *
 * Synchronizes the backend to ensure all data is flushed to storage.
 *
 * @return None
 */
typedef void (*coredump_backend_sync_t)(void);

/**
 * @brief Set backend work mode, fulldump or minidump
 *
 * @param coredump_type Type of coredump (COREDUMP_TYPE_MINIMUM or COREDUMP_TYPE_FULL)
 * @return coredump_err_code_t Error code, COREDUMP_ERR_NO indicates success
 */
typedef coredump_err_code_t (*coredump_backend_set_mode_t)(coredump_type_t coredump_type);


/**
 * @brief Coredump backend structure
 *
 * Contains all function pointers for backend operations.
 */
typedef struct
{
    /** init backend for the core dump, no write is allowed after init */
    coredump_backend_init_t     init;
    /** start backend for the core dump, write could be called after start */
    coredump_backend_start_t    start;
    /** end backend for the core dump */
    coredump_backend_end_t      end;
    /** write buffer to the backend for the core dump */
    coredump_backend_write_t    write;
    /** query information from the backend for the core dump */
    coredump_backend_query_t    query;
    /** sync the backend for the core dump */
    coredump_backend_sync_t     sync;
    /** clear the backend for the core dump */
    coredump_backend_clear_t    clear;
    /** read dump data from backend according to current work mode */
    coredump_backend_read_t    read;
    /** set backend work mode, minidump or fulldump */
    coredump_backend_set_mode_t   set_mode;

} coredump_backend_t;

/** Partition-based backend for coredump storage */
extern const coredump_backend_t coredump_backend_partition;
/** File-based backend for coredump storage */
extern const coredump_backend_t coredump_backend_file;
/** memory region list, weak variable that could be overridden by user */
extern const coredump_memory_region_t coredump_memory_region_list[];
/** register region list, weak variable that could be overridden by user */
extern const coredump_register_region_t coredump_register_region_list[];

/**
 * @brief Perfrom core dump operation, such as save context to flash for offline analysis
 *
  * @return None
 */
void coredump(void);

/**
 * @brief Perfrom core dump extended operation which is implemented by user
 *
 * @return None
 */
void coredump_ext(void);

/**
 * @brief Perform minimum core dump operation (minidump)
 *
 * Saves essential crash information including coredump info, IRQ stack,
 * HCPU log and current thread stack to the backend for offline analysis.
 *
 * @return None
 */
void coredump_minimum(void);

/**
 * @brief Read dump data from backend
 *
 * Reads dump data from the backend at the specified offset into the provided buffer.
 *
 * @param[in] offset Offset from the start of dump data to begin reading
 * @param[out] buf Pointer to the buffer to store read data
 * @param[in] len Number of bytes to read
 * @return size_t Number of bytes actually read
 */
size_t coredump_read_dump(uint32_t offset, uint8_t *buf, size_t len);

/**
 * @brief Read minidump data from backend
 *
 * Reads minidump data from the backend at the specified offset into the provided buffer.
 *
 * @param[in] offset Offset from the start of minidump data to begin reading
 * @param[out] buf Pointer to the buffer to store read data
 * @param[in] len Number of bytes to read
 * @return size_t Number of bytes actually read
 */
size_t coredump_read_minidump(uint32_t offset, uint8_t *buf, size_t len);

/**
 * @brief Get coredump data information from backend
 *
 * Queries the backend for coredump data presence and fills the provided
 * data structure with addresses and sizes for minidump and fulldump.
 *
 * @param data Pointer to coredump_data_t structure to be filled with data info
 *
 * @return coredump_err_code_t Error code, COREDUMP_ERR_NO indicates success
 */
coredump_err_code_t coredump_get_data(coredump_data_t *data);

/**
 * @brief Get the backend type of coredump
 *
 * Returns the type of backend currently being used for coredump operations.
 *
 * @return 0 for partition backend, 1 for file backend
 */
int coredump_get_type(void);

/**
 * @brief Clear coredump data from backend
 *
 * Clears all coredump data stored in the current backend.
 *
 * @return 0 on success
 */
int coredump_clear(void);

/**
 * @brief Set or get coredump enable/disable flag
 *
 * When flag is 0: disable coredump
 * When flag is 1: enable coredump
 * When flag is 2: query current enable status
 *
 * @param flag Operation flag (0=disable, 1=enable, 2=query)
 *
 * @return 0 on success for set operations, current enabled flag (0/1) for query operation
 */
int coredump_set_onoff(int flag);

#ifdef __cplusplus
}
#endif


#endif /* __COREDUMP_H__ */

