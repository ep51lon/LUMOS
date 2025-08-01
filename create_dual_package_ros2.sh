#!/bin/bash

# Check for package name argument
if [ -z "$1" ]; then
  echo "Usage: $0 <package_name>"
  exit 1
fi

PACKAGE_NAME=$1
BUILD_TYPE="ament_cmake"
DEPENDENCIES="rclcpp rclpy std_msgs"

echo "Creating ROS 2 package: $PACKAGE_NAME"

# Step 1: Create the package
ros2 pkg create $PACKAGE_NAME --build-type ${BUILD_TYPE} --dependencies ${DEPENDENCIES}

# Step 2: Navigate into package
cd $PACKAGE_NAME || exit 1

# Step 3: Create Python directory structure
mkdir -p $PACKAGE_NAME
touch $PACKAGE_NAME/__init__.py

mkdir -p scripts
cat <<EOL > scripts/py_node.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('py_node')
        self.get_logger().info('Python node has started.')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOL

chmod +x scripts/py_node.py

# Step 4: Create setup.py
cat <<EOL > setup.py
from setuptools import setup

package_name = '${PACKAGE_NAME}'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS2 package with both Python and C++ nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_node = '${PACKAGE_NAME}'.py_node:main',
        ],
    },
)
EOL

# Step 5: Create resource and marker files
mkdir -p resource
echo $PACKAGE_NAME > resource/$PACKAGE_NAME

# Step 6: Create a sample Python node
cat <<EOL > $PACKAGE_NAME/py_node.py
def main():
    print("Hello from Python node in package $PACKAGE_NAME!")
EOL

# Step 7: Update CMakeLists.txt
sed -i '/^find_package.*$/a find_package(rclpy REQUIRED)' CMakeLists.txt
sed -i '/^install.*$/a install(DIRECTORY '$PACKAGE_NAME'/ DESTINATION lib/${PROJECT_NAME})' CMakeLists.txt

# Step 8: Update package.xml
# Remove duplicate <depend> tags (if any)
sed -i '/<depend>rclpy<\/depend>/d' package.xml

# Add only exec_depend entries
sed -i '/<\/package>/i \
  <exec_depend>rclpy</exec_depend>\
  <exec_depend>python3</exec_depend>\
' package.xml

# Step 9: Done
echo "Package $PACKAGE_NAME created with C++ and Python support."