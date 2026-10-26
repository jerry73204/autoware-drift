# Build every package against Autoware 1.5.0
build:
    #!/usr/bin/env bash
    set -e
    source /opt/autoware/1.5.0/setup.bash
    colcon build --symlink-install --base-paths src

# The packages' own tests
test:
    cd src/control/drift_controller && python3 -m pytest -q test

clean:
    rm -rf build install log
