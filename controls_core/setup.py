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
            # "moveLeft = controls_core.move_test:moveLeft",
            # "moveRight = controls_core.move_test:moveRight",
            # "moveFront = controls_core.move_test:moveFront",
            # "moveBack = controls_core.move_test:moveBack",
            # "moveUp = controls_core.move_test:moveUp",
            # "moveDown = controls_core.move_test:moveDown",
            # "moveLeftPID = controls_core.move_pid_test:moveLeft",
            # "moveRightPID = controls_core.move_pid_test:moveRight",
            # "moveFrontPID = controls_core.move_pid_test:moveFront",
            # "moveBackPID = controls_core.move_pid_test:moveBack",
            # "steer = controls_core.steer:main",
            "attitudeControl = controls_core.attitude_control_test:main",
            "depthControl = controls_core.depth_control_test:main",
            "qualiSequence = controls_core.quali_sequence:main",
            "imu_zeroing = controls_core.imu_zeroing:main",
            "teleop = controls_core.teleop:main",
            "pass_through_gate_test = controls_core.pass_through_gate_test:main",
            "knock_down_flare_test = controls_core.knock_down_flare_test:main",
            "obstacle_avoidance = controls_core.obstacle_avoidance_test:main",
        ],
    },
)
