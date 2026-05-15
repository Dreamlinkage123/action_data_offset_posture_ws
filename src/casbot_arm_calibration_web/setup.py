from glob import glob
import os
from setuptools import setup

package_name = "casbot_arm_calibration_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["web/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob("resource/**", recursive=True)
            if os.path.isfile(f)
        ],
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob("offest_data/**", recursive=True)
            if os.path.isfile(f)
        ],
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob("new_offset_data/**", recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=["setuptools", "flask>=2.0", "numpy>=1.20,<2"],
    zip_safe=True,
    maintainer="casbot",
    maintainer_email="casbot@example.com",
    description="CASBOT arm pose calibration web UI for ROS2",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_calibration_web = casbot_arm_calibration_web.web_node:main",
        ],
    },
)
