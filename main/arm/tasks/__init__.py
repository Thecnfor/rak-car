"""main/arm/tasks"""
from .go_home import go_home
from .pick_left import pick_left
from .pick_right import pick_right
from .release import release

__all__ = ["go_home", "pick_left", "pick_right", "release"]
