from setuptools import setup

package_name = "cart_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "opencv-python", "pyserial", "pynmea2",
                      "transformers", "torch", "pillow"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "usb_camera_node = cart_sensors.usb_camera_node:main",
            "gps_mega_node = cart_sensors.gps_mega_node:main",
            "depth_anything_node = cart_sensors.depth_anything_node:main",
        ],
    },
)
