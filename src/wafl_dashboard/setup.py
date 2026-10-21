import os
from glob import glob
from setuptools import find_packages, setup

package_name = "wafl_dashboard"
setup(name=package_name, version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"]),
                  (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py"))] +
                 [(os.path.join("share", package_name, root),
                   [os.path.join(root, name) for name in files])
                  for root, _dirs, files in os.walk("static") if files],
      # Runtime libraries are installed from Ubuntu packages (see README); do not
      # let ament/setuptools try to fetch them into Ubuntu's managed Python.
      install_requires=["setuptools"], zip_safe=True,
      entry_points={"console_scripts": ["dashboard_server = wafl_dashboard.server:main"]})
