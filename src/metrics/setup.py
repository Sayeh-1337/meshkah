from setuptools import setup

package_name = "metrics"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="meshkah",
    maintainer_email="research@example.com",
    description="Metrics package for CUAS-SIM meshkah.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "evaluator = metrics.evaluator:main",
        ],
    },
)
