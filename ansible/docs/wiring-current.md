# Current wiring notes

Update this whenever the wiring reality changes.

- Laser power is on its own fused 24V feed, not the general 24V bus.
- 12V rail is fused, currently parked/reserved for future front panel use.
- PSU moved underneath the board for clearance and cooler air.
- RJ45 fanout removed.
- PowerCON white connectors are switched 120V outputs for DRY and FAN.
- Verify K2/K4 mapping in both code and `group_vars/all.yaml` before deployment.
