# Laser Enclosure Fire Control Diagrams

This repository contains Graphviz DOT diagrams for documenting the fire suppression logic and relay control layout in a Raspberry Pi-controlled laser enclosure project.

## Included Files

### `fire_state_machine.dot`
- Describes the FIRE state logic.
- **ARMED**: Tank is pressurized (system ready).
- **FIRE**: Triggered by pressure drop, CO₂ valve opens, gates close.
- **RESET**: Performed via E-stop twist.
- Use `dot -Tpng fire_state_machine.dot -o fire_state_machine.png` to render.

### `power_distribution.dot`
- Updated relay layout with sequential relay numbering:
  - `K1`: Laser
  - `K2`: High Pressure Air (HPA)
  - `K3`: CO₂ Valve
  - `K4`: Blast Gates (intake/exhaust)
  - `K5`: Low Pressure Air (LPA)
  - `K6`: Heater Fan
  - `K7`: Exhaust Fan
  - `K8`: PTC Heater (AC)
- Use `dot -Tpng relay_layout_k3_k4_separated.dot -o relay_layout_k3_k4_separated.png` to render.

## Notes

- Gates are gravity-closing and do not remain activated after a FIRE event.
- CO₂ valve remains open until a manual reset is performed.
- The system arms based on tank pressure and triggers on pressure drop.
- Relays are powered from 24V, 12V, and AC lines as appropriate.

## Rendering

Install [Graphviz](https://graphviz.org/download/) and run:

```bash
dot -Tpng fire_state_machine.dot -o fire_state_machine.png
dot -Tpng power_distribution.dot -o power_distribution.png
```

Or use online editors like [edotor.net](https://edotor.net/) or [dreampuf.github.io](https://dreampuf.github.io/GraphvizOnline/).

---

**Enginerd-built. Fire-tested.** 🔥🛠️
