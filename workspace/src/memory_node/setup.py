from setuptools import setup

package_name = "memory_node"
submodules = package_name + "/submodules"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, submodules],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="leevi",
    maintainer_email="48809753+Le36@users.noreply.github.com",
    description="Manages saving the QR codes",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "listener = memory_node.memory_node:main",
        ],
    },
)
