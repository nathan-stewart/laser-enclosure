#!/usr/bin/env python3
import sys

sys.argv = [sys.argv[0], "--log", "DEBUG"]
import service.control as control
control.main(sys.argv)