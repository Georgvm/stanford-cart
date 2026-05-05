from setuptools import setup

package_name = "cart_dbw"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial", "odrive"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "odrive_steering_node = cart_dbw.odrive_steering_node:main",
            "mega_pedals_node = cart_dbw.mega_pedals_node:main",
            "cmd_vel_to_dbw = cart_dbw.cmd_vel_to_dbw:main",
        ],
    },
)
