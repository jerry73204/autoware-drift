from setuptools import setup

setup(
    name="drift_controller",
    version="0.1.0",
    packages=["drift_controller"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/drift_controller"]),
        ("share/drift_controller", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    license="Apache-2.0",
    tests_require=["pytest"],
)
