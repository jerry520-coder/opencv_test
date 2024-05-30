from setuptools import setup

package_name = "image_to_occupancy_grid"

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
    maintainer="binghe",
    maintainer_email="17371046931@163.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_to_occupancy_grid = image_to_occupancy_grid.image_to_occupancy_grid:main"
        ],
    },
)
