from .vrep import VREP
from .prosthetic_hand import PROSTHETIC_HAND

try:
    from .pygame import PyGame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False
