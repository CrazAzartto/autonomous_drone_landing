"""
Pluto-specific layer: velocity set-points (drone body frame) -> RC channel values -> MSP over Wi-Fi.

  command_mapper.py  BodyVelocity + estimated velocity -> RC (roll, pitch, throttle, yaw), limits, slew rate
  msp_parser.py      parses the MSP telemetry stream (attitude, altitude, RC echo, battery)
  adapters.py        DryRunAdapter (nothing is sent), PlutoAdapter (your plutocontrol library), SimAdapter

Facts used here are taken from the Pluto firmware source (MagisV2, DronaAviation) and from the
plutocontrol library on your laptop; see docs/07_pluto_command_interface.md for the citations.
"""
