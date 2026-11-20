# Build every package against Autoware 1.5.0
build:
    #!/usr/bin/env bash
    set -e
    source /opt/autoware/1.5.0/setup.bash
    colcon build --symlink-install --base-paths src

# The packages' own tests
test:
    cd src/control/drift_controller && python3 -m pytest -q test
    cd src/launcher/drift_launch && python3 -m pytest -q test

clean:
    rm -rf build install log

# race_planner's regression set at this checkout, outcomes only (tools/regress.py; scensim from pip, or $SWB)
regress:
    cd src/planning/race_planner && env -u PYTHONPATH .venv/bin/python tools/regress.py run --base HEAD

# race_planner against a base revision, bit for bit; exit 1 on a difference (README, "Before pushing to main").
# `just regress-check main` or `just regress-check base=main`
regress-check base="origin/main":
    b='{{base}}'; cd src/planning/race_planner && env -u PYTHONPATH .venv/bin/python tools/regress.py check --base "${b#base=}"
