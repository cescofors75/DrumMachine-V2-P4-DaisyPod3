"""Use the installed Python module for image generation on Windows.

Some Windows Application Control policies reject pip-generated .exe launchers
even when their Python modules can run. Keep the same esptool and arguments.
"""
import sys

Import("env")

if sys.platform == "win32":
    command = '"{}" -m esptool'.format(sys.executable)
    env.Replace(OBJCOPY=command, ERASETOOL=command)
