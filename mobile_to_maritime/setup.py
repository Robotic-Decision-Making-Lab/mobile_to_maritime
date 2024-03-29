import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mobile_to_maritime"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Evan Palmer",
    maintainer_email="evanp922@gmail.com",
    description="Nodes and launch files that transform ENU/FLU messages to NED/FSD messages",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mobile_to_maritime_twist = mobile_to_maritime.mobile_to_maritime:main_mobile_to_maritime_twist",
            "mobile_to_maritime_twist_stamped = mobile_to_maritime.mobile_to_maritime:main_mobile_to_maritime_twist_stamped",
        ],
    },
)
