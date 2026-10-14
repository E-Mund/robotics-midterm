from glob import glob

from setuptools import find_packages, setup

package_name = "midterm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models", glob("models/*.onnx")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ethan",
    maintainer_email="ewm58972@uga.edu",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "sub = midterm.sub:main",
            "human_detector = midterm.human_detector:main",
        ],
    },
)
