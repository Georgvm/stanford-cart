from setuptools import setup

package_name = "cart_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "ultralytics", "transformers", "torch", "pillow", "pyyaml"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "safety_gate = cart_safety.safety_gate:main",
            "perception_estop = cart_safety.perception_estop:main",
            "demo_runner = cart_safety.demo_runner:main",
            "engage_button = cart_safety.engage_button:main",
        ],
    },
)
