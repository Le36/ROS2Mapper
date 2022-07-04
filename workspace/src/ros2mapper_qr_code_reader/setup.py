from setuptools import setup

package_name = "ros2mapper_qr_code_reader"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bntti",
    maintainer_email="roysko.juho@gmail.com",
    description="Listens for images from the image topic and publishes the QR code"
    "data to /qr_code_found topic when recognizing a QR code.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "launch = ros2mapper_qr_code_reader.ros2mapper_qr_code_reader:main"
        ],
    },
)
