[Back to Top page](../../README.md)

# Initial procedure
## Package creation

```shell
$ colcon_cd
$ cd <ws>
$ ros2 pkg create <package> --build-type ament_cmake
$ mkdir <package>/msg
$ mkdir <package>/srv
```

## Editing package.xml
Excerpt related parts.

```xml
<package format="3">
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

## Editing CMakeLists.txt
Excerpt related parts.

```txt
find_package(rosidl_default_generators REQUIRED)

set(msg_files
#  "msg/<file name>"
)
set(srv_files
#  "srv/<file name>"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)
```

# Procedure in production
## Adding message file
Add file name in parentheses of `set(msg_files )` or `set(srv_files )` in CMakeLists.txt when making msg or srv files.
