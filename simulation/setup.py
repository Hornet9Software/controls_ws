from setuptools import find_packages, setup

package_name = "simulation"

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
            "dummy_data = simulation.dummy_data:main",
            "movement_test_sim = simulation.movement_test_sim:main",
            "pid_manager_sim = simulation.pid_manager_sim:main",
            "simulation = simulation.simulation:main",
        ],
    },
)
