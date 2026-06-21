import json

from .core import run_halctl, HalCtlError


def plugin(kernel, lifecycle=None):
    if lifecycle != "register":
        return

    @kernel.console_command("halctl", help="Send command to laser HAL")
    def halctl_cmd(command, channel, _, args=(), **kwargs):
        try:
            result = run_halctl(list(args), source="meerk40t")
        except HalCtlError as e:
            channel(f"halctl: {e}")
            return

        if result is not None:
            channel(json.dumps(result, indent=2, sort_keys=True))