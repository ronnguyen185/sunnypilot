"""CRC-16/CCITT-FALSE for Gate Protocol v0 (poly 0x1021, init 0xFFFF, no reflection)."""


def crc16_ccitt_false(data: bytes | None) -> int:
  crc = 0xFFFF
  if data is None:
    return crc
  for b in data:
    crc ^= b << 8
    for _ in range(8):
      if crc & 0x8000:
        crc = ((crc << 1) ^ 0x1021) & 0xFFFF
      else:
        crc = (crc << 1) & 0xFFFF
  return crc
