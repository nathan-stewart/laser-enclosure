# meerk40t_halctl.py

from .core import run_halctl, HalCtlError


def plugin(kernel, lifecycle):
    if lifecycle != "register":
        return

    @kernel.console_command(
        "halctl",
        help="HAL control. Usage: halctl purge on|off|SECONDS",
    )
    def halctl_cmd(command, channel, _, args=tuple(), **kwargs):
        try:
            run_halctl(list(args), source="meerk40t", emit=channel)
        except HalCtlError as e:
            channel(f"halctl: {e}")