from glob import glob
import os

from setuptools import find_packages, setup


package_name = "drone_link"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="batuhan",
    maintainer_email="batuhan@todo.todo",
    description="Socket bridge between the onboard-local ROS graph and the GCS-local ROS graph.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "pi_link_bridge = drone_link.pi_link_bridge:main",
            "gcs_link_bridge = drone_link.gcs_link_bridge:main",
        ],
    },
)
