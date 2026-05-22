from setuptools import setup

package_name = "feixi_trajectory"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Trajectory and joint command publishers for the Feixi arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "send_feixi_demo_trajectory = feixi_trajectory.send_feixi_demo_trajectory:main",
            "send_feixi_reference_trajectory = feixi_trajectory.send_feixi_reference_trajectory:main",
            "stream_feixi_joint_commands = feixi_trajectory.stream_feixi_joint_commands:main",
        ],
    },
)
