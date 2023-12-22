from setuptools import find_packages, setup

package_name = "controls_core"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="advaypakhale",
    maintainer_email="advay.pakhale@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "moveLeft = controls_core.movement_test:moveLeft",
            "moveRight = controls_core.movement_test:moveRight",
            "moveFront = controls_core.movement_test:moveFront",
            "moveBack = controls_core.movement_test:moveBack",
            "moveUp = controls_core.movement_test:moveUp",
            "moveDown = controls_core.movement_test:moveDown",
            "attitudeControl = controls_core.attitude_control_test:main",
            "imu_zeroing = controls_core.imu_zeroing:main",
            "moveLeftPID = controls_core.move_pid_test:moveLeft",
            "moveRightPID = controls_core.move_pid_test:moveRight",
            "moveFrontPID = controls_core.move_pid_test:moveFront",
            "moveBackPID = controls_core.move_pid_test:moveBack",
            "steer = controls_core.steer:main",
        ],
    },
)
