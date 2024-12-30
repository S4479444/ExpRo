from setuptools import find_packages, setup

pkg_name = "assignment_pkg"

setup(
    name = pkg_name,
    version = "0.0.0",
    packages = find_packages(exclude=["test"]),
    data_files = [
        ("share/ament_index/resource_index/packages", ["resource/" + pkg_name]),
        ("share/" + pkg_name, ["package.xml"])
    ],
    install_requires = ["setuptools"],
    zip_safe = True,
    maintainer = "Emanuele Giordano",
    maintainer_email = "s4479444@studenti.unige.it",
    description = "TODO: Package description",
    license = "TODO: Licence declaration",
    tests_require = ["pytest"],
    entry_points = {
        "console_scripts": [
            "motor_control = assginment_pkg.motor_control:main",
            "robot_controller = assignment_pkg.robot_controller:main",
            "robot_revj = assignment_pkg.robot_revj:main"
        ]
    }
)