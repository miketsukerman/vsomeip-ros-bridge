# Franca IDL to ROS2 IDL translator

Helper tools used to convert FIDL files to ROS2 IDLs.

# Dependencies

* argparse
* jinja2
* pyfranca

# Installation

To install call

    python3 setup.py install

or

    pip3 install -e <src-path>

# Usage

    fidl2ros -f <path-to-fidl> -o <output-directory>

with include path

    fidl2ros -i <include-path> -f <path-to-fidl> -o <output-directory>
