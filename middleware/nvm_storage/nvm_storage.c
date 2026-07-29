/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * TLV Storage Library Implementation
 * Features:
 * - Full 4-byte alignment for all data structures and pointers
 * - Support for bit fields, nested structures, and dynamic arrays
 * - String pointer handling with alignment padding
 */

#include <rtthread.h>
#include <string.h>
#include <app_nvm.h>
#include <nvm_storage.h>
#include <dfs_posix.h>

#define NVM_MAGIC       0x5a5a5a5e

#define nvm_malloc(size)       rt_malloc(size)
#define nvm_calloc(nmemb, size) rt_calloc(nmemb, size)
#define nvm_free(ptr)          rt_free(ptr)

typedef struct
{
    uint32_t        magic;
    uint32_t        total_len;
} nvm_header_t;

static uint32_t nvm_calc_crc32(uint32_t crc, const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;

    crc = ~crc;
    while (len--)
    {
        crc ^= *p++;
        for (uint32_t i = 0; i < 8; i++)
        {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }

    return ~crc;
}

/*------------------------------*/
/* Memory Alignment Utilities */
/*------------------------------*/

/**
 * Calculate the aligned size for a given size and alignment.
 * @param size The original size.
 * @param alignment The alignment boundary (e.g., 4 for 4-byte alignment).
 * @return The aligned size.
 */
#define align_size(size, alignment)  ((size + alignment - 1) & ~(alignment - 1))

/**
 * Calculate the next aligned offset.
 * @param offset The current offset.
 * @param alignment The alignment boundary.
 * @return The next aligned offset.
 */
#define align_offset(offset, alignment) align_size(offset, alignment)


/*------------------------------*/
/* Container Management */
/*------------------------------*/

/**
 * Expand the container buffer to accommodate more data.
 * @param container The container to expand.
 * @param additional_size The additional size needed.
 * @return true on success, false on failure.
 */
static bool expand_buffer(nvm_container_t *container, uint32_t additional_size)
{
    if (!container)
    {
        return false;
    }

    uint32_t required_size = container->buffer_used + additional_size;
    if (required_size <= container->buffer_size)
    {
        return true;
    }

    if (required_size < 1024) required_size = 1024;

    // Calculate new size with growth factor
    uint32_t new_size = container->buffer_size + 8192;
    if (new_size < required_size)
    {
        new_size = required_size;
    }

    uint8_t *new_buffer = (uint8_t *)nvm_calloc(1, new_size);
    if (!new_buffer)
    {
        return false;
    }
    memcpy(new_buffer, container->buffer, container->buffer_size);
    nvm_free(container->buffer);

    container->buffer = new_buffer;
    container->buffer_size = new_size;
    return true;
}

/*------------------------------*/
/* Serialization Functions */
/*------------------------------*/

/**
 * Write a TLV header to the container.
 * @param container The container to write to.
 * @param type The TLV type.
 * @param length The data length.
 * @param alignment The required alignment.
 * @return true on success, false on failure.
 */
static bool write_tlv_header(nvm_container_t *container, nvm_tlv_type_t type, uint32_t length, uint8_t alignment)
{
    if (!expand_buffer(container, sizeof(nvm_tlv_header_t)))
    {
        return false;
    }

    nvm_tlv_header_t header =
    {
        .type = type,
        .length = (uint16_t)length,
        .alignment = alignment
    };

    memcpy(container->buffer + container->buffer_used, &header, sizeof(nvm_tlv_header_t));
    container->buffer_used += sizeof(nvm_tlv_header_t);
    return true;
}

/**
 * Write aligned data to the container.
 * @param container The container to write to.
 * @param data The data to write.
 * @param size The size of the data.
 * @param alignment The required alignment.
 * @return true on success, false on failure.
 */
static bool write_aligned_data(nvm_container_t *container, const void *data, uint32_t size, uint8_t alignment)
{
    // Align the current buffer offset
    uint32_t aligned_offset = align_offset(container->buffer_used, alignment);
    if (!expand_buffer(container, aligned_offset - container->buffer_used + size))
    {
        return false;
    }
    container->buffer_used = aligned_offset;
    // Write the data
    memcpy(container->buffer + container->buffer_used, data, size);
    container->buffer_used += size;
    return true;
}

/**
 * Write a string pointer with alignment.
 * @param container The container to write to.
 * @param str The string pointer.
 * @return true on success, false on failure.
 */
static bool write_aligned_string(nvm_container_t *container, const char *str)
{
    uint32_t len = str ? strlen(str) + 1 : 0;  // Include null terminator
    uint32_t padded_len = align_size(len, TLV_ALIGNMENT);

    if (!expand_buffer(container, padded_len))
    {
        return false;
    }

    if (str)
    {
        memcpy(container->buffer + container->buffer_used, str, len);
    }

    // Add padding to meet alignment requirements
    if (padded_len > len)
    {
        memset(container->buffer + container->buffer_used + len, 0, padded_len - len);
    }

    container->buffer_used += padded_len;
    return true;
}

/**
 * Serialize a bit field value.
 * @param container The container to write to.
 * @param bitfield The bit field descriptor.
 * @param struct_ptr Pointer to the structure containing the bit field.
 * @return true on success, false on failure.
 */
static bool serialize_bitfield(nvm_container_t *container, const nvm_bit_filed_dsc_t *bitfield, const void *struct_ptr)
{
    const uint8_t *byte_ptr = (const uint8_t *)struct_ptr + bitfield->byte_offset;
    uint32_t value = 0;

    // Extract the bit field value
    for (uint8_t i = 0; i < bitfield->bit_count; i++)
    {
        uint8_t byte_idx = (bitfield->bit_offset + i) / 8;
        uint8_t bit_idx = (bitfield->bit_offset + i) % 8;

        if (byte_ptr[byte_idx] & (1 << bit_idx))
        {
            value |= (1 << i);
        }
    }

    return write_tlv_header(container, TLV_TYPE_BITFIELD, sizeof(uint32_t), TLV_ALIGNMENT) &&
           write_aligned_data(container, &value, sizeof(uint32_t), TLV_ALIGNMENT);
}

/**
 * Read the next TLV header from the container.
 * @param container The container to read from.
 * @param offset The current offset in the container.
 * @param header Pointer to store the header.
 * @return true if a header was read, false otherwise.
 */
static bool read_tlv_header(const nvm_container_t *container, uint32_t *offset, nvm_tlv_header_t *header)
{
    if (!container || !offset || !header || *offset + sizeof(nvm_tlv_header_t) > container->buffer_size)
    {
        return false;
    }

    memcpy(header, container->buffer + *offset, sizeof(nvm_tlv_header_t));
    *offset += sizeof(nvm_tlv_header_t);
    return true;
}

/**
 * Read aligned data from the container.
 * @param container The container to read from.
 * @param offset The current offset in the container.
 * @param data Pointer to store the data.
 * @param size The size of the data to read.
 * @param alignment The required alignment.
 * @return true on success, false on failure.
 */
static bool read_aligned_data(const nvm_container_t *container, uint32_t *offset, void *data, uint32_t size, uint8_t alignment)
{
    // Align the current buffer offset
    uint32_t aligned_offset = align_offset(*offset, alignment);
    if (aligned_offset + size > container->buffer_size)
    {
        return false;
    }
    *offset = aligned_offset; //padding
    // Read the data
    memcpy(data, container->buffer + *offset, size);
    *offset += size;
    return true;
}

/**
 * Read an aligned string from the container.
 * @param container The container to read from.
 * @param offset The current offset in the container.
 * @param length The length of the string data (including padding).
 * @param out_str Pointer to store the allocated string.
 * @return true on success, false on failure.
 */
static bool read_aligned_string(const nvm_container_t *container, uint32_t *offset, uint32_t length, char **out_str)
{
    if (!container || !offset || !out_str || *offset + length > container->buffer_size)
    {
        return false;
    }

    // Find the actual string length (excluding padding)
    uint32_t str_len = 0;
    while (str_len < length && container->buffer[*offset + str_len] != '\0')
    {
        str_len++;
    }

    // Allocate and copy the string
    *out_str = (char *)nvm_calloc(1, str_len + 1);
    if (!*out_str)
    {
        return false;
    }

    memcpy(*out_str, container->buffer + *offset, str_len);
    (*out_str)[str_len] = '\0';

    // Skip to the next aligned offset
    *offset += length;
    return true;
}

/**
 * Deserialize a bit field value.
 * @param container The container to read from.
 * @param offset The current offset in the container.
 * @param bitfield The bit field descriptor.
 * @param struct_ptr Pointer to the structure to update.
 * @return true on success, false on failure.
 */
static bool deserialize_bitfield(const nvm_container_t *container, uint32_t *offset, const nvm_bit_filed_dsc_t *bitfield, void *struct_ptr, uint8_t  alignment)
{
    uint32_t value;
    if (!read_aligned_data(container, offset, &value, sizeof(uint32_t), alignment))
    {
        return false;
    }

    // Set the bit field value
    uint8_t *byte_ptr = (uint8_t *)struct_ptr + bitfield->byte_offset;

    for (uint8_t i = 0; i < bitfield->bit_count; i++)
    {
        uint8_t byte_idx = (bitfield->bit_offset + i) / 8;
        uint8_t bit_idx = (bitfield->bit_offset + i) % 8;

        if (value & (1 << i))
        {
            byte_ptr[byte_idx] |= (1 << bit_idx);
        }
        else
        {
            byte_ptr[byte_idx] &= ~(1 << bit_idx);
        }
    }

    return true;
}

/**
 * Serialize a single field.
 * @param container The container to write to.
 * @param field The field descriptor.
 * @param struct_ptr Pointer to the structure containing the field.
 * @return true on success, false on failure.
 */
bool nvm_serialize_field(nvm_container_t *container, const nvm_filed_dsc_t *field, const void *struct_ptr)
{
    const uint8_t *field_ptr = (const uint8_t *)struct_ptr + field->offset;

    switch (field->type)
    {
    case TLV_TYPE_UINT8:
    case TLV_TYPE_INT8:
    case TLV_TYPE_BOOL:
    {
        return write_tlv_header(container, field->type, 1, 1) &&
               write_aligned_data(container, field_ptr, 1, 1);
    }
    case TLV_TYPE_UINT16:
    case TLV_TYPE_INT16:
    {
        return write_tlv_header(container, field->type, 2, 2) &&
               write_aligned_data(container, field_ptr, 2, 2);
    }
    case TLV_TYPE_UINT32:
    case TLV_TYPE_INT32:
    case TLV_TYPE_FLOAT:
    {
        return write_tlv_header(container, field->type, 4, 4) &&
               write_aligned_data(container, field_ptr, 4, 4);
    }
    case TLV_TYPE_DOUBLE:
    {
        return write_tlv_header(container, field->type, 8, 4) &&
               write_aligned_data(container, field_ptr, 8, 4);
    }
    case TLV_TYPE_BYTES:
    {
        uint32_t len = field->element_size;
        uint32_t padded_len = align_size(len, TLV_ALIGNMENT);
        return write_tlv_header(container, field->type, padded_len, TLV_ALIGNMENT) &&
               write_aligned_data(container, field_ptr, len, TLV_ALIGNMENT);
    }

    case TLV_TYPE_STR_PTR:
    {
        const char *str = *(const char **)field_ptr;
        uint32_t len = str ? strlen(str) + 1 : 0;
        uint32_t padded_len = align_size(len, TLV_ALIGNMENT);
        return write_tlv_header(container, field->type, padded_len, TLV_ALIGNMENT) &&
               write_aligned_string(container, str);
    }
    case TLV_TYPE_VAR_BYTES:
    {
        uint32_t *ptr = *(uint32_t **)field_ptr;
        if (!ptr || 0  == *ptr)
        {
            return write_tlv_header(container, field->type, 0, TLV_ALIGNMENT);
        }
        uint32_t len = *ptr;
        uint32_t padded_len = align_size(len, TLV_ALIGNMENT);
        return write_tlv_header(container, field->type, padded_len, TLV_ALIGNMENT) &&
               write_aligned_data(container, ptr, len, TLV_ALIGNMENT);
    }
    case TLV_TYPE_STR_PTR_ARRAY:
    {
        const char **str_array = (const char **)field_ptr;
        uint32_t total_num = field->element_size / sizeof(char *); // Calculate element count
        uint32_t valid_count = 0;

        if (!write_tlv_header(container, field->type, field->element_size, TLV_ALIGNMENT))
        {
            return false;
        }

        // Count non-NULL strings
        for (uint32_t i = 0; i < total_num; i++)
        {
            if (str_array[i] != NULL) valid_count++;
        }

        // Write valid count
        if (!write_tlv_header(container, TLV_TYPE_UINT32, sizeof(uint32_t), TLV_ALIGNMENT) ||
                !write_aligned_data(container, &valid_count, sizeof(uint32_t), TLV_ALIGNMENT))
        {
            return false;
        }

        // Write each non-NULL string with its index
        for (uint32_t i = 0; i < total_num; i++)
        {
            if (str_array[i] != NULL)
            {
                // Write index
                if (!write_tlv_header(container, TLV_TYPE_UINT32, sizeof(uint32_t), TLV_ALIGNMENT) ||
                        !write_aligned_data(container, &i, sizeof(uint32_t), TLV_ALIGNMENT))
                {
                    return false;
                }

                // Write string
                uint32_t len = strlen(str_array[i]) + 1;
                uint32_t padded_len = align_size(len, TLV_ALIGNMENT);

                if (!write_tlv_header(container, TLV_TYPE_STR_PTR, padded_len, TLV_ALIGNMENT) ||
                        !write_aligned_string(container, str_array[i]))
                {
                    return false;
                }
            }
        }

        return true;
    }
    case TLV_TYPE_STRUCT_PTR_ARRAY:
    {
        const void **array = (const void **)field_ptr;
        uint32_t total_num = field->element_size / sizeof(void *); // Calculate element count
        uint32_t valid_count = 0;

        if (!write_tlv_header(container, field->type, field->element_size, TLV_ALIGNMENT))
        {
            return false;
        }

        // Count non-NULL pointers
        for (uint32_t i = 0; i < total_num; i++)
        {
            if (array[i] != NULL) valid_count++;
        }

        // Write valid count
        if (!write_tlv_header(container, TLV_TYPE_UINT32, sizeof(uint32_t), TLV_ALIGNMENT) ||
                !write_aligned_data(container, &valid_count, sizeof(uint32_t), TLV_ALIGNMENT))
        {
            return false;
        }

        // Write each non-NULL struct
        for (uint32_t i = 0; i < total_num; i++)
        {
            if (array[i] != NULL)
            {
                // Write index
                if (!write_tlv_header(container, TLV_TYPE_UINT32, sizeof(uint32_t), TLV_ALIGNMENT) ||
                        !write_aligned_data(container, &i, sizeof(uint32_t), TLV_ALIGNMENT))
                {
                    return false;
                }

                if (0 == field->schema->field_count)
                {
                    uint32_t len = field->user_data;
                    uint32_t padded_len = align_size(len, TLV_ALIGNMENT);

                    return write_tlv_header(container, field->type, padded_len, TLV_ALIGNMENT) &&
                           write_aligned_data(container, field_ptr, len, TLV_ALIGNMENT);
                }
                // Write struct
                else if (!nvm_add_struct(container, field->schema, array[i]))
                {
                    return false;
                }
            }
        }

        return true;
    }
    case TLV_TYPE_STRUCT:
    {
        return write_tlv_header(container, field->type, field->element_size, TLV_ALIGNMENT) &&
               nvm_add_struct(container, field->schema, field_ptr);
    }
    case TLV_TYPE_STRUCT_ARRAY:
    {
        // Arrays are handled separately
        return false;
    }
    case TLV_TYPE_BITFIELD:
    {
        return serialize_bitfield(container, (const nvm_bit_filed_dsc_t *) field->user_data, struct_ptr);
    }
    default:
        ;
    }

    return false;
}

/**
 * Deserialize a single field.
 * @param container The container to read from.
 * @param offset The current offset in the container.
 * @param field The field descriptor.
 * @param struct_ptr Pointer to the structure to update.
 * @return true on success, false on failure.
 */
bool nvm_deserialize_field(const nvm_container_t *container, const nvm_filed_dsc_t *field, void *struct_ptr)
{
    uint32_t *offset = &((nvm_container_t *) container)->buffer_used;

    nvm_tlv_header_t header;
    if (!read_tlv_header(container, offset, &header))
    {
        return false;
    }

    uint8_t *field_ptr = (uint8_t *)struct_ptr + field->offset;

    if (header.type != field->type)
    {
        // Type mismatch, attempt to recover by skipping
        *offset = align_offset(*offset + header.length, header.alignment);
        return false;
    }

    switch (field->type)
    {
    case TLV_TYPE_UINT8:
    case TLV_TYPE_INT8:
    case TLV_TYPE_BOOL:
    {
        return read_aligned_data(container, offset, field_ptr, 1, header.alignment);
    }
    case TLV_TYPE_UINT16:
    case TLV_TYPE_INT16:
    {
        return read_aligned_data(container, offset, field_ptr, 2, header.alignment);
    }
    case TLV_TYPE_UINT32:
    case TLV_TYPE_INT32:
    case TLV_TYPE_FLOAT:
        return read_aligned_data(container, offset, field_ptr, 4, header.alignment);

    case TLV_TYPE_DOUBLE:
    {
        return read_aligned_data(container, offset, field_ptr, 8, header.alignment);
    }
    case TLV_TYPE_BYTES:
    {
        return read_aligned_data(container, offset, field_ptr, field->element_size, header.alignment);
    }
    case TLV_TYPE_STR_PTR:
    {
        return read_aligned_string(container, offset, header.length, (char **)field_ptr);
    }
    case TLV_TYPE_STRUCT:
    {
        return nvm_extract_struct(container, field->schema, field_ptr);
    }
    case TLV_TYPE_STRUCT_ARRAY:
    {
        // Arrays are handled separately
        return false;
    }
    case TLV_TYPE_BITFIELD:
    {
        return deserialize_bitfield(container, offset, (const nvm_bit_filed_dsc_t *) field->user_data, struct_ptr, header.alignment);
    }
    case TLV_TYPE_VAR_BYTES:
    {
        if (0 == header.length)  return true;
        void **data = (void **) field_ptr;
        *data = nvm_malloc(header.length);
        RT_ASSERT(*data);
        return read_aligned_data(container, offset, (uint8_t *)*data, header.length, header.alignment);
    }

    case TLV_TYPE_STR_PTR_ARRAY:
    {
        char ***str_array_ptr = (char ***)field_ptr; // Pointer to the array pointer
        uint32_t total_num = field->element_size / sizeof(char *); // Element count
        uint32_t valid_count;

        // Read valid count
        nvm_tlv_header_t header;
        if (!read_tlv_header(container, offset, &header) ||
                header.type != TLV_TYPE_UINT32 ||
                !read_aligned_data(container, offset, &valid_count, sizeof(uint32_t), header.alignment))
        {
            return false;
        }

        // Allocate memory for the array of pointers
        *str_array_ptr = (char **)nvm_malloc(field->element_size);
        if (!*str_array_ptr) return false;

        // Initialize array with NULLs
        memset(*str_array_ptr, 0, field->element_size);

        // Read valid strings and their indices
        for (uint32_t i = 0; i < valid_count; i++)
        {
            uint32_t index;

            // Read index
            if (!read_tlv_header(container, offset, &header) ||
                    header.type != TLV_TYPE_UINT32 ||
                    !read_aligned_data(container, offset, &index, sizeof(uint32_t), header.alignment))
            {
                nvm_free(*str_array_ptr);  // Cleanup on failure
                return false;
            }

            // Validate index
            if (index >= total_num)
            {
                nvm_free(*str_array_ptr);
                return false;
            }

            // Read string
            if (!read_tlv_header(container, offset, &header) ||
                    header.type != TLV_TYPE_STR_PTR ||
                    !read_aligned_string(container, offset, header.length, &(*str_array_ptr)[index]))
            {
                nvm_free(*str_array_ptr);  // Cleanup on failure
                return false;
            }
        }

        return true;
    }
    case TLV_TYPE_STRUCT_PTR_ARRAY:
    {
        void ***array_ptr = (void ***)field_ptr;
        uint32_t total_num = field->element_size / sizeof(void *); // Element count
        uint32_t valid_count;

        // Read valid count
        nvm_tlv_header_t header;
        if (!read_tlv_header(container, offset, &header) ||
                header.type != TLV_TYPE_UINT32 ||
                !read_aligned_data(container, offset, &valid_count, sizeof(uint32_t), header.alignment))
        {
            return false;
        }

        // Allocate array of pointers and initialize to NULL
        *array_ptr = (void **)nvm_calloc(1, field->element_size);
        if (!*array_ptr) return false;

        // Read each struct
        for (uint32_t i = 0; i < valid_count; i++)
        {
            uint32_t index;

            // Read index
            if (!read_tlv_header(container, offset, &header) ||
                    header.type != TLV_TYPE_UINT32 ||
                    !read_aligned_data(container, offset, &index, sizeof(uint32_t), header.alignment))
            {
                nvm_free(*array_ptr);
                return false;
            }

            // Validate index
            if (index >= total_num)
            {
                nvm_free(*array_ptr);
                return false;
            }

            // Allocate and deserialize struct
            (*array_ptr)[index] = (void *)nvm_calloc(1, sizeof(field->user_data));
            if (!(*array_ptr)[index])
            {
                nvm_free(*array_ptr);
                return false;
            }

            if (0 == field->schema->field_count)
            {
                if (!read_tlv_header(container, offset, &header) ||
                        !read_aligned_data(container, offset, field_ptr, field->user_data, header.alignment))
                {
                    nvm_free((*array_ptr)[index]);
                    nvm_free(*array_ptr);
                    return false;
                }
            }
            else if (!nvm_extract_struct(container, field->schema, (*array_ptr)[index]))
            {
                nvm_free((*array_ptr)[index]);
                nvm_free(*array_ptr);
                return false;
            }
        }

        return true;
    }

    default:
        ;
    }

    return false;
}

/**
 * Add a structure to the container.
 * @param container The container to add to.
 * @param schema The schema for the structure.
 * @param struct_ptr Pointer to the structure.
 * @return true on success, false on failure.
 */
bool nvm_add_struct(nvm_container_t *container, const nvm_schema_t *schema, const void *struct_ptr)
{
    if (!container || !schema || !struct_ptr)
    {
        return false;
    }

    for (uint32_t i = 0; i < schema->field_count; i++)
    {
        const nvm_filed_dsc_t *field = &schema->fields[i];

        if (field->type == TLV_TYPE_STRUCT_ARRAY)
        {
            // Handle arrays separately
            continue;
        }

        if (!nvm_serialize_field(container, field, struct_ptr))
        {
            return false;
        }
    }

    return true;
}

/**
 * Add an array to the container.
 * @param container The container to add to.
 * @param field The field descriptor for the array.
 * @param array_ptr Pointer to the array.
 * @param count The number of elements in the array.
 * @return true on success, false on failure.
 */
bool nvm_add_array(nvm_container_t *container, const nvm_filed_dsc_t *field, const void *array_ptr, uint32_t count)
{
    if (!container || !field || !array_ptr)
    {
        return false;
    }

    // Write array length as a separate TLV
    if (!write_tlv_header(container, TLV_TYPE_UINT32, sizeof(uint32_t), TLV_ALIGNMENT) ||
            !write_aligned_data(container, &count, sizeof(uint32_t), TLV_ALIGNMENT))
    {
        return false;
    }

    // Write each array element
    const uint8_t *element_ptr = (const uint8_t *)array_ptr;
    for (uint32_t i = 0; i < count; i++)
    {
        // Primitive array elements
        if (!nvm_serialize_field(container, field, element_ptr))
        {
            return false;
        }

        element_ptr += field->element_size;
    }

    return true;
}

/**
 * Extract a structure from the container.
 * @param container The container to read from.
 * @param schema The schema for the structure.
 * @param struct_ptr Pointer to the structure to populate.
 * @return true on success, false on failure.
 */
bool nvm_extract_struct(const nvm_container_t *container, const nvm_schema_t *schema, void *struct_ptr)
{
    if (!container || !schema || !struct_ptr)
    {
        return false;
    }

    for (uint32_t i = 0; i < schema->field_count; i++)
    {
        const nvm_filed_dsc_t *field = &schema->fields[i];

        if (field->type == TLV_TYPE_STRUCT_ARRAY)
        {
            // Arrays are handled separately
            continue;
        }

        if (!nvm_deserialize_field(container, field, struct_ptr))
        {
            return false;
        }
    }

    return true;
}

/**
 * Extract an array from the container.
 * @param container The container to read from.
 * @param field The field descriptor for the array.
 * @param array_ptr Pointer to the array to populate.
 * @param count The number of elements in the array.
 * @return true on success, false on failure.
 */
bool nvm_extract_array(const nvm_container_t *container, const nvm_filed_dsc_t *field, void *array_ptr, uint32_t count)
{
    if (!container || !field || !array_ptr)
    {
        return false;
    }

    uint32_t *offset = &((nvm_container_t *) container)->buffer_used;

    // Read array length (should match 'count')
    nvm_tlv_header_t header;
    if (!read_tlv_header(container, offset, &header) || header.type != TLV_TYPE_UINT32 || header.length != sizeof(uint32_t))
    {
        return false;
    }

    uint32_t array_count;
    if (!read_aligned_data(container, offset, &array_count, sizeof(uint32_t), header.alignment))
    {
        return false;
    }

    if (array_count != count)
    {
        return false;
    }

    // Read each array element
    uint8_t *element_ptr = (uint8_t *)array_ptr;
    for (uint32_t i = 0; i < count; i++)
    {
        // Primitive array elements
        if (!nvm_deserialize_field(container, field, element_ptr))
        {
            return false;
        }

        element_ptr += field->element_size;
    }

    return true;
}

/**
 * Create a new TLV container.
 * @return A pointer to the new container, or NULL on failure.
 */
nvm_container_t *nvm_create_container(void)
{
    nvm_container_t *container = (nvm_container_t *)nvm_calloc(1, sizeof(nvm_container_t));
    if (!container)
    {
        return NULL;
    }

    container->buffer = NULL;
    container->buffer_size = 0;
    container->buffer_used = sizeof(nvm_header_t); //reserved for header.

    return container;
}

/**
 * Destroy a TLV container and free its resources.
 * @param container The container to destroy.
 */
void nvm_destroy_container(nvm_container_t *container)
{
    if (container)
    {
        if (container->buffer)
        {
            nvm_free(container->buffer);
        }
        nvm_free(container);
    }
}

/**
 * Save the TLV container to a file.
 * @param container The container to save.
 * @param path Path to file or nvm key name
 * @param storage_mode file or NVM
 * @return true on success, false on failure.
 */
bool nvm_save_to_file(nvm_container_t *container, const char *path, nvm_tlv_storage_t storage_mode, uint32_t *pre_crc)
{
    if (!container || !container->buffer) return false;

    if (container->buffer_size < sizeof(nvm_header_t))
    {
        return false;
    }

    /**
     * calculate crc and compare nvm changed.
     */
    nvm_header_t *header = (nvm_header_t *) container->buffer; /* nvm_create_container reserved header */
    header->magic = NVM_MAGIC;
    // crc : nvm_tlv_header_t + uint32_t.
    header->total_len = ((container->buffer_used + sizeof(nvm_tlv_header_t) + 3) >> 2 << 2) + sizeof(uint32_t);
    uint32_t crc = nvm_calc_crc32(0, container->buffer, container->buffer_used);
    if (*pre_crc == crc) return true;

    /* add crc */
    nvm_filed_dsc_t dsc = {"nvm_crc", TLV_TYPE_UINT32, 0, sizeof(uint32_t), NULL, 0 };
    nvm_serialize_field(container, &dsc, &crc);
    *pre_crc = crc;
    bool ret = false;

    if (STORAGE_NVM == storage_mode)
    {
        if (RT_EOK == app_nvm_write(path, container->buffer, container->buffer_used))
        {
            ret = true;
        }
    }
    else
    {
        struct dfs_fd fd = {0};
        if (0 == dfs_file_open(&fd, path, O_CREAT | O_RDWR | O_TRUNC))
        {
            if (dfs_file_write(&fd, container->buffer, container->buffer_used) == container->buffer_used)
            {
                ret = true;
            }
            dfs_file_close(&fd);
        }
    }

    return ret;
}

/*------------------------------*/
/* Deserialization Functions */
/*------------------------------*/

/**
 * Load a TLV container from a file.
 * @param container The container to save.
 * @param path Path to file or nvm key name
 * @param storage_mode file or NVM
 * @return A pointer to the loaded container, or NULL on failure.
 */
nvm_container_t *nvm_load_from_file(const char *path, nvm_tlv_storage_t storage_mode)
{
    // Create container and allocate buffer
    nvm_container_t *container = nvm_create_container();
    if (!container) return NULL;
    nvm_header_t header = {0};

    if (STORAGE_NVM == storage_mode)
    {
        if (sizeof(nvm_header_t) != app_nvm_read(path, &header, sizeof(nvm_header_t)) ||
                NVM_MAGIC != header.magic)
        {
            nvm_destroy_container(container);
            return NULL;
        }

        container->buffer = (uint8_t *)nvm_malloc(header.total_len);
        RT_ASSERT(container->buffer);
        if (header.total_len != app_nvm_read(path, container->buffer, header.total_len))
        {
            nvm_destroy_container(container);
            return NULL;
        }

        container->buffer_size = header.total_len;
        container->buffer_used = sizeof(nvm_header_t);
    }
    else
    {
        struct dfs_fd fd = {0};
        if (dfs_file_open(&fd, path, O_RDONLY) != 0)
        {
            nvm_destroy_container(container);
            return NULL;
        }

        uint32_t file_size = fd.size;

        container->buffer = (uint8_t *)nvm_malloc(file_size);
        if (!container->buffer)
        {
            nvm_destroy_container(container);
            dfs_file_close(&fd);
            return NULL;
        }

        container->buffer_size = file_size;
        container->buffer_used = sizeof(nvm_header_t);
        int read_size = dfs_file_read(&fd, container->buffer, file_size);
        dfs_file_close(&fd);

        if (read_size != file_size)
        {
            nvm_destroy_container(container);
            return NULL;
        }
        //header = *((nvm_header_t *) container->buffer);
    }

    return container;
}

/**
 * Extract and verify the CRC32 checksum from the TLV container.
 * This function calculates the CRC32 of the container's buffer content and compares it
 * against the stored CRC value. If they match, it indicates the container data is valid.
 * @param container Pointer to the TLV container.
 * @param crc       Output parameter to store the calculated CRC32 value.
 * @return          true if the calculated CRC matches the stored CRC; false otherwise.
 *                  If verification fails, the container is destroyed to prevent invalid usage.
 */
bool nvm_extract_crc(nvm_container_t *container, uint32_t *crc)
{
    /* Calculate CRC32 over the entire container buffer to validate data integrity */
    *crc = nvm_calc_crc32(0, container->buffer, container->buffer_used);

    /* Read the stored CRC32 value from the container */
    uint32_t read_crc = 0;
    nvm_filed_dsc_t dsc = {"nvm_crc", TLV_TYPE_UINT32, 0, sizeof(uint32_t), NULL, 0 };

    /* Verify the stored CRC matches the calculated CRC */
    if (!nvm_deserialize_field(container, &dsc, &read_crc) || read_crc != *crc)
    {
        /* CRC mismatch or deserialization failure indicates corrupted data */
        nvm_destroy_container(container);
        *crc = 0;
        return false;
    }

    /* CRC verification passed - container data is valid */
    return true;
}
