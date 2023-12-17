from setuptools import find_packages, setup

package_name = "tasks"

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
    maintainer="averageandy",
    maintainer_email="averageandyyy@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_publish_global_pose = tasks.test_publish_global_pose_task:main",
            "test_publish_current_state = tasks.test_publish_current_state:main",
            "movement_test = tasks.movement_test:main",
        ],
    },
)
