"""Where race_planner finds scensim's files: the car's vehicle spec, Isaac's surface classes and
the car's ECU (sim_vehicle_ecu), which the planner's models and the regression harness read.

scensim (formerly scenario-world-bridge, swb) is installed with pip (`pip install --user
scensim; scensim install`), which unpacks its release under ~/.local/share/scensim/current/sim;
that is the default. A source checkout is used instead when $SWB names one: a scensim checkout,
or an old scenario-world-bridge one, for work on scensim's source or to compare against it.

    SWB            a scensim (or scenario-world-bridge) checkout; first when set
    SCENSIM_SIM    the release's simulation side (default ~/.local/share/scensim/current/sim)
    SIM_VEHICLE_ECU  a sim_vehicle_ecu checkout of its own (e.g. a worktree), first of all for the ECU
"""
import os

SWB = os.environ.get("SWB") or None
SIM = os.environ.get("SCENSIM_SIM") or os.path.expanduser("~/.local/share/scensim/current/sim")
# where studies write their results when they are not told: the checkout's play_log, as before,
# else one of the user's own
PLAY_LOG = os.path.join(SWB, "play_log") if SWB else os.path.expanduser("~/.local/share/race_planner/play_log")


class NotFound(FileNotFoundError):
    pass


def _first(what, cands):
    if SWB and not os.path.isdir(SWB):
        raise NotFound(f"$SWB={SWB}: no such directory")
    for p in cands:
        if os.path.exists(p):
            return p
    raise NotFound(f"{what}: none of {', '.join(cands)} (install scensim with pip and run "
                   "`scensim install`, or set SWB to a checkout)")


def spec(name="drift_mustang"):
    """A car's sim_vehicle_spec file."""
    rel = os.path.join("sim_vehicle_spec", "vehicles", f"{name}.yaml")
    cands = [os.path.join(SWB, "src", rel)] if SWB else []
    return _first(f"vehicle spec {name}", cands + [os.path.join(SIM, "share", rel)])


def surface_classes():
    """Isaac's surface classes: scensim_world_server's since scensim 086 C2 (main, after 0.2.1),
    scensim_isaac_world's config before."""
    ws = "scensim_world_server"
    cands = []
    if SWB:
        cands += [os.path.join(SWB, "src", "scensim_world_protocol", ws, ws, "surface_classes.yaml")]
        cands += [os.path.join(SWB, "src", w, "config", "surface_classes.yaml")
                  for w in ("scensim_isaac_world", "swb_isaac_world")]
    return _first("surface classes",
                  cands + [os.path.join(SIM, "share", w, "config", "surface_classes.yaml")
                           for w in (ws, "scensim_isaac_world")])


def ecu_path():
    """The directory to put on sys.path for `import sim_vehicle_ecu`."""
    cands = [os.environ["SIM_VEHICLE_ECU"]] if os.environ.get("SIM_VEHICLE_ECU") else []
    if SWB:
        cands.append(os.path.join(SWB, "src", "sim_vehicle_ecu"))
    cands += [os.path.join(SIM, "lib", f"python{v}", "site-packages") for v in ("3.10", "3.12")]
    return _first("sim_vehicle_ecu", [c for c in cands
                                      if os.path.isdir(os.path.join(c, "sim_vehicle_ecu"))] or cands)
