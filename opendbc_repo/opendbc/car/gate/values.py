from opendbc.car import CarSpecs, PlatformConfig, Platforms


class CAR(Platforms):
  COMMA_GATE = PlatformConfig(
    [],
    CarSpecs(mass=2000, wheelbase=2.8, steerRatio=14),
    {},
  )


class CANBUS:
  """C3X panda bus wired to gate-panda bus 1 (SOC2 link)."""
  gate = 2       # production harness: C3X bus 2 → gate bus 1
  chassis = 0    # optional chassis feedback (not required in bench mode)
