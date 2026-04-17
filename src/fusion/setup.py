from setuptools import setup

package_name = "fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "scipy",
        "filterpy",
    ],
    zip_safe=True,
    maintainer="meshkah",
    maintainer_email="research@example.com",
    description="Fusion core for CUAS-SIM meshkah prototype.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fusion_node = fusion.fusion_node:main",
        ],
    },
)
