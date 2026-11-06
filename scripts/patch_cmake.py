#!/usr/bin/env python3
"""Patch unitree_go CMakeLists.txt to remove Connext DDS dependency."""
import re

cmake_path = "/home/dakshi/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/CMakeLists.txt"

with open(cmake_path, 'r') as f:
    content = f.read()

# Remove the rosidl_generator_dds_idl find_package and related sections
# Remove: find_package(rosidl_generator_dds_idl REQUIRED)
content = content.replace('find_package(rosidl_generator_dds_idl REQUIRED)\n', '')

# Remove the rosidl_generate_dds_interfaces block and the add_dependencies block
content = re.sub(
    r'rosidl_generate_dds_interfaces\(.*?\)\n',
    '',
    content,
    flags=re.DOTALL
)
content = re.sub(
    r'add_dependencies\(\s*\$\{PROJECT_NAME\}\s*\$\{PROJECT_NAME\}__dds_connext_idl\s*\)\n',
    '',
    content,
    flags=re.DOTALL
)

with open(cmake_path, 'w') as f:
    f.write(content)

print("CMakeLists.txt patched successfully")
print("--- New content ---")
with open(cmake_path, 'r') as f:
    print(f.read())

