#ifndef _PCB405_INTFX_H
#define _PCB405_INTFX_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "intfx_defs.h"

/* Macro for merge two uint8_t vars to uint16_t */
#define INTFX_MERGE_UINT8(msb, lsb) (uint16_t)((msb << 8) | lsb)

#ifndef PACKAGE_MAX_SIZE
  /*
   * Maximum packet length in bytes.
   * Can be overwritten by the build system
   */
  #define PACKAGE_MAX_SIZE 256
#endif

#define INTFX_OK               0x00
#define INTFX_E_NULL_PTR       0x01
#define INTFX_E_CRC_MISMATCH   0x02
#define INTFX_E_INVALID_INDEX  0x03

#define CRC8_MAX_PACKAGE_SIZE  15
#define CRC16_MAX_PACKAGE_SIZE 4095

PACKED_STRUCT_START

typedef union packed {
  struct packed {
    uint8_t address;
    uint8_t size;
    uint8_t data_type;
    uint8_t payload[];
  } package;

  uint8_t raw[PACKAGE_MAX_SIZE];
} SerializedPackage;

PACKED_STRUCT_END

// clang-format off
#define SerializedPackage_init {0}
// clang-format on

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Serializes data into a destination buffer
 *
 * @param data_type The type of the data being serialized
 * @param dst The destination buffer where the serialized data will be stored
 * @param src The source buffer containing the data to be serialized
 * @param size The size of the data to be serialized
 *
 * @return The length of the serialized data in bytes. If the destination buffer or
 * source buffer is NULL, or if the input size is zero, the function returns zero.
 */
uint8_t intfx_serialize(const uint8_t address, const uint8_t data_type, uint8_t *dst, const uint8_t *src, uint8_t size);

/**
 * @brief Deserialize a serialized package from the source buffer into the destination structure.
 *
 * @param dst Pointer to the destination SerializedPackage structure
 * @param src Pointer to the source buffer containing the serialized data
 *
 * @return Result of the deserialization process
 * @retval INTFX_OK: Deserialization was successful
 * @retval INTFX_E_CRC_MISMATCH: CRC check failed, indicating a data corruption
 * @retval INTFX_E_NULL_PTR: Either the destination or source pointer is NULL
 */
uint8_t intfx_deserialize(SerializedPackage *dst, uint8_t *src);

/**
 * @brief Composes a 9-bit value from an array of 8-bit values
 *
 * @param dst Pointer to the destination array of 16-bit values
 * @param src Pointer to the source array of 8-bit values
 * @param size Size of the source array
 * @param b9set_indx Index of the element in the source array to set the 9th bit
 *
 * @return INTFX_OK if successful, otherwise an error code
 */
uint8_t intfx_9bit_compose(uint16_t *dst, uint8_t *src, uint8_t size, uint8_t b9set_indx);

#ifdef __cplusplus
}
#endif
#endif /* _PCB405_INTFX_H */
