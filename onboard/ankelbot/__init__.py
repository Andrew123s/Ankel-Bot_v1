"""Ankel-Bot onboard software.

Runs entirely on the vessel's companion computer: camera -> YOLO26 -> size
estimation -> tracking -> mission supervisor -> thrust control. No network
connection is needed at any point; see ``sync.py`` for the optional uplink.
"""

__version__ = "0.1.0"
