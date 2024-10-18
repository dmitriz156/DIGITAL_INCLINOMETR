#include "intfx.h"

uint8_t crc8(uint8_t *pcBlock, uint8_t len);
uint16_t crc16(uint8_t *pcBlock, uint16_t len);

uint8_t intfx_serialize(const uint8_t address, const uint8_t data_type, uint8_t *dst, const uint8_t *src, uint8_t size)
{
  uint8_t len = 0;

  if (dst != NULL && src != NULL) {
    SerializedPackage serialized_packet = SerializedPackage_init;
    serialized_packet.package.address   = address;
    serialized_packet.package.size      = size;
    serialized_packet.package.data_type = data_type;
    memcpy(serialized_packet.package.payload, src, size);

    len = sizeof(serialized_packet.package) + size;

    if (size > 15) {
      uint16_t crc = crc16(serialized_packet.raw, len);

      serialized_packet.package.payload[size]     = (uint8_t)(crc & 0xFF);  // lsb
      serialized_packet.package.payload[size + 1] = (uint8_t)(crc >> 8);    // msb
      len += 1;
    } else {
      serialized_packet.package.payload[size] = crc8(serialized_packet.raw, len);
    }
    len += 1;

    memcpy(dst, serialized_packet.raw, len);
  }

  return len;
}

uint8_t intfx_deserialize(SerializedPackage *dst, uint8_t *src)
{
  uint8_t rslt = INTFX_OK;

  if (dst != NULL && src != NULL) {
    uint8_t crc_check = 0;

    SerializedPackage serialized_packet = SerializedPackage_init;

    uint8_t len = sizeof(serialized_packet.package);
    memcpy(serialized_packet.raw, src, len);

    len += serialized_packet.package.size;

    if (serialized_packet.package.size > 15) {
      uint16_t crc = INTFX_MERGE_UINT8(*(uint8_t *)(src + len + 1), *(uint8_t *)(src + len));

      crc_check = (crc16(src, len) == crc);

      len += 1;
    } else {
      crc_check = (crc8(src, len + 1) == 0x0);
    }

    len += 1;

    if (crc_check == 1) {
      memmove(dst, src, len);
    } else {
      rslt = INTFX_E_CRC_MISMATCH;
    }
  } else {
    rslt = INTFX_E_NULL_PTR;
  }

  return rslt;
}

uint8_t intfx_9bit_compose(uint16_t *dst, uint8_t *src, uint8_t size, uint8_t b9set_indx)
{
  if (dst == NULL || src == NULL) {
    return INTFX_E_NULL_PTR;
  }

  if (b9set_indx >= size) {
    return INTFX_E_INVALID_INDEX;
  }

  for (size_t i = 0; i < size; i++) {
    if (i == b9set_indx) {
      dst[i] = (uint16_t)src[i] | 0x0100;
    } else {
      dst[i] = (uint16_t)src[i];
    }
  }

  return INTFX_OK;
}

/**
 * @brief Computes CRC-8 for the given block of data
 * @param pcBlock Pointer to the start of the data block
 * @param len Length of the data block in bytes
 * @return Computed CRC-8 value.
 *
 * @details:
 * - Poly  : 0x31 x^8 + x^5 + x^4 + 1
 * - Init  : 0xFF
 * - Revert: false
 * - XorOut: 0x00
 * - Check : 0xF7 ("123456789")
 * - MaxLen: 15 bytes (127 bits)
 */
uint8_t crc8(uint8_t *pcBlock, uint8_t len)
{
  uint8_t crc = 0xFF;
  uint8_t i;

  while (len--) {
    crc ^= *pcBlock++;

    for (i = 0; i < 8; i++)
      crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
  }

  return crc;
}

/**
 * @brief Computes CRC-16 CCITT for the given block of data
 * @param pcBlock Pointer to the start of the data block
 * @param len Length of the data block in bytes
 * @return Computed CRC-16 CCITT value.
 *
 * @details:
 * - Poly  : 0x1021 x^16 + x^12 + x^5 + 1
 * - Init  : 0xFFFF
 * - Revert: false
 * - XorOut: 0x0000
 * - Check : 0x29B1 ("123456789")
 * - MaxLen: 4095 bytes (32767 bits)
 */
uint16_t crc16(uint8_t *pcBlock, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  uint8_t i;

  while (len--) {
    crc ^= *pcBlock++ << 8;

    for (i = 0; i < 8; i++)
      crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}
