"""race_planner/scensim.py of this checkout, loaded by its path: the studies (and tools/regress.py)
run against another revision's race_planner package too (RP_ROOT), which may predate it."""
import importlib.util
import os

_p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "race_planner", "scensim.py")
_s = importlib.util.spec_from_file_location("_race_planner_scensim", _p)
scensim = importlib.util.module_from_spec(_s)
_s.loader.exec_module(scensim)
