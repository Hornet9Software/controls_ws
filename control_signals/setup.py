from setuptools import find_packages, setup

package_name = "control_signals"

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
            # "cv_signals = control_signals.cv_processing:main",
            "cv_signals = control_signals.cv_processing_new:main",
            "imu_signals = control_signals.imu_integration:main",
        ],
    },
)
