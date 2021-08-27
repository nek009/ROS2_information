# 手順
## パッケージの作成

```shell
$ colcon_cd
$ cd <ws>
$ ros2 pkg create <package> --build-type ament_cmake
$ mkdir <package>/msg
$ mkdir <package>/srv
```

## package.xmlの編集
関係部分のみ抜粋

```xml
<package format="3">
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

## CMakeLists.txtの編集
関係部分のみ抜粋

```txt
find_package(rosidl_default_generators REQUIRED)

set(msg_files
)
set(srv_files
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)
```

msgファイルやsrvファイルを作成したら`set(*_files)`の中に追加．
