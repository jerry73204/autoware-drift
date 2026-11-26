from setuptools import setup

setup(
    name="race_planner_ros",
    version="0.1.0",
    packages=["race_planner_ros"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/race_planner_ros"]),
        ("share/race_planner_ros", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["race_planner = race_planner_ros.node:main"]},
)
