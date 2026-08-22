/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * TLV Storage Library Header
 * Features:
 * - Full 4-byte alignment for all data structures and pointers
 * - Support for bit fields, nested structures, and dynamic arrays
 * - String pointer handling with alignment padding
 */

#ifndef NVM_STORAGE_H
#define NVM_STORAGE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    STORAGE_NVM,
    STORAGE_FILE
} nvm_tlv_storage_t;

/* TLV Type Definitions */
typedef enum
{
    TLV_TYPE_UINT8 = 1,       /* element is uint8_t */
    TLV_TYPE_UINT16,          /* element is uint16_t */
    TLV_TYPE_UINT32,          /* element is uint32_t */
    TLV_TYPE_INT8,            /* element is int8_t */
    TLV_TYPE_INT16,           /* element is int16_t */
    TLV_TYPE_INT32,           /* element is int32_t */
    TLV_TYPE_FLOAT,           /* element is float */
    TLV_TYPE_DOUBLE,          /* element is double */
    TLV_TYPE_BOOL,            /* element is bool */
    TLV_TYPE_BITFIELD,        /* element is bit field */
    TLV_TYPE_BYTES,           /* element is array of uint8_t or int8, and array_length is fixed length */
    TLV_TYPE_VAR_BYTES,       /* element is array of uint8_t or int8，and array_length is first 4 bytes */
    TLV_TYPE_STRUCT,          /* element is stuct */
    TLV_TYPE_STR_PTR,         /* element is string or (char *). str_ptr length is strlen*/
    TLV_TYPE_STRUCT_ARRAY,    /* TBD */
    TLV_TYPE_STR_PTR_ARRAY,   /* element is array of string or (char *) */
    TLV_TYPE_STRUCT_PTR_ARRAY /* element is array of (stuct *) */
} nvm_tlv_type_t;

/* TLV Header Structure with Alignment Info */
typedef struct
{
    uint8_t                     type;               /* TLV Type */
    uint8_t                     alignment;          /* Alignment (1=byte, 2=word, 4=dword) */
    uint16_t                    length;             /* Original data length */
} nvm_tlv_header_t;

#define TLV_ALIGNMENT 4         /* Default 4-byte alignment */

/* Bit Field Descriptor */
typedef struct
{
    uint32_t                    byte_offset;        /* Byte offset within the structure */
    uint8_t                     bit_offset;         /* Bit offset within the byte */
    uint8_t                     bit_count;          /* Number of bits */
} nvm_bit_filed_dsc_t;

/* TLV Field Descriptor */
typedef struct
{
    const char                 *name;               /* Field name (for debugging) */
    nvm_tlv_type_t              type;               /* Field type */
    uint32_t                    offset;             /* Offset within the structure */
    uint32_t                    element_size;       /* Size of each element (for arrays) */
    const struct nvm_schema_t  *schema;             /* Nested schema (for structs/arrays) */
    uint32_t                    user_data;          /* Bit field descriptor or user_data*/
} nvm_filed_dsc_t;

/* TLV Schema Definition */
typedef struct nvm_schema_t
{
    const char                 *name;               /* Schema name */
    uint32_t                    struct_size;        /* Size of the structure */
    const nvm_filed_dsc_t      *fields;             /* Array of field descriptors */
    uint32_t                    field_count;        /* Number of fields */
} nvm_schema_t;

/* TLV Container */
typedef struct _TLVContainer
{
    uint8_t                    *buffer;             /* Buffer to store TLV data */
    uint32_t                    buffer_size;        /* Total buffer capacity (bytes) */
    uint32_t                    buffer_used;        /* Used buffer size (bytes) */
} nvm_container_t;

/* Container Management */
nvm_container_t *nvm_create_container(void);
void nvm_destroy_container(nvm_container_t *container);

/* Serialization Functions */
bool nvm_serialize_field(nvm_container_t *container, const nvm_filed_dsc_t *field, const void *struct_ptr);
bool nvm_add_struct(nvm_container_t *container, const nvm_schema_t *schema, const void *struct_ptr);
bool nvm_add_array(nvm_container_t *container, const nvm_filed_dsc_t *field, const void *array_ptr, uint32_t count);
bool nvm_save_to_file(nvm_container_t *container, const char *path, nvm_tlv_storage_t storage_mode, uint32_t *pre_crc);

/* Deserialization Functions */
bool nvm_deserialize_field(const nvm_container_t *container, const nvm_filed_dsc_t *field, void *struct_ptr);
nvm_container_t *nvm_load_from_file(const char *path, nvm_tlv_storage_t storage_mode);
bool nvm_extract_struct(const nvm_container_t *container, const nvm_schema_t *schema, void *struct_ptr);
bool nvm_extract_array(const nvm_container_t *container, const nvm_filed_dsc_t *field, void *array_ptr, uint32_t count);
bool nvm_extract_crc(nvm_container_t *container, uint32_t *crc);

#ifdef __cplusplus
}
#endif

#endif /* NVM_STORAGE_H */
