"""Fingerprints for the comma Gate SOC2 link (no OEM chassis required)."""

from opendbc.car.gate.values import CAR

# GATE_STATUS 0x500, 16-byte CAN-FD — present when gate-panda is live on the SOC2 bus.
_GATE_LINK_FINGERPRINT = {
  0x500: 16,
}

FINGERPRINTS = {
  CAR.COMMA_GATE: [_GATE_LINK_FINGERPRINT],
}

FW_VERSIONS = {
  CAR.COMMA_GATE: {},
}
