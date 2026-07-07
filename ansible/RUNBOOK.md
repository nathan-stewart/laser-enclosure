# Laser Enclosure Ansible Runbook

This folder intentionally has only three normal commands.

```bash
cd ansible
make ping
make app
make site
```

## Normal app/code update

Use this after changing HAL/control code, systemd unit files, GPIO boot block, or the halctl plugin source copied with the app.

```bash
make app DRY=1
make app
```

## Full box update

Use this for a rebuild or when base OS/kiosk/MeerK40t setup may need to be reapplied.

```bash
make site DRY=1
make site
```

## One-off narrower runs

Do not add a Makefile target for every Ansible feature. Use `EXTRA` when needed:

```bash
make site EXTRA='--tags meerk40t'
make site EXTRA='--tags base'
make app EXTRA='--tags gpio-config'
make app EXTRA='-vv'
```

## Live HAL commands

```bash
halctl get
halctl exhaust on
halctl exhaust off
halctl exhaust 600
```

## Current hardware notes

- Laser has its own fused 24 V feed.
- 12 V rail is fused and currently parked/reserved for future front panel.
- Spare moved from K4 to K2; verify code and GPIO comments match actual wiring.
- RJ45 fanout removed.
- PSU moved underneath board for space and airflow.
- White PowerCONs are switched 120 VAC outputs for DRY and FAN.
