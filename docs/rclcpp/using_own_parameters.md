[Back to Top page](../../README.md)

Usage of parameters is roughly classified into two.
One is a way to declare parameters and use them by itself.
Another is a way to use other node's parameters.<br>
Here first case is introduced considering a setting and a use parameters on a component node.
So refering [making_package_of_component_node.md](making_package_of_service_node.md) and making a component node before start.

# Initial procedure
Several parameters are made as example.
I explain terms or names of them roughly at each part, or there are few explanations.

## Flow of making a code for a service

**In hpp files**
1. Declare callback function(s) for changes of parameters
  * No need if lambda functions are used.
  * No need if no special procedures for changes of parameters

**In cpp files**

1. Use ParameterDescriptor
  * For detailed setting of parameters. No need if no detailed setting.
1. Declare parameters
1. Define callback function(s) for changes of parameters
  * No need if no special procedures for changes of parameters
1. Use Parameter

**yaml files**
If you want to set parameters value with yaml file, prepare it.
No need to prepare it for all parameters.
No read and set parameters which are not described in yaml file.

If use yaml file, use `ros2 run` by addin options `--ros-args --params-file <yaml file>`.

```shell
$ ros2 run <package name> <target name> --ros-args --params-file <yaml file>
```

## Main functions

* declare_parameter
* undeclare_parameter
* has_parameter
* set_parameters
* get_parameter
* get_parameters
* list_parameters
* add_on_set_parameters_callback
* remove_on_set_parameters_callback

Detailed Information is in [Official api reference](https://docs.ros2.org/foxy/api/rclcpp/index.html).

## Example of coding
Added comments `// Added below` at the point to be noticed.

```c++
#include <rclcpp/rclcpp.hpp>

namespace param_pkg{

class ParamTestNode : public rclcpp::Node{
private:
  // Added below
  OnSetParametersCallbackHandle::SharedPtr reset_param_callback_function_handler_;
  rcl_interfaces::msg::SetParametersResult
    reset_param_callback_function_(const std::vector<rclcpp::Parameter>& params);

public:
  PARAM_PKG_PUBLIC
  ParamTestNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  PARAM_PKG_PUBLIC
  ParamTestNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  PARAM_PKG_PUBLIC
  ~virtual ParamTestNode();
};

} // end of namespace
```

```c++
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "param_pkg/param_test_node.hpp"

namespace srv_pkg{

ParamTestNode::ParamTestNode(
  const rclcpp::NodeOptions& options
): ParamTestNode("",options){}

ParamTestNode::ParamTestNode(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("param_test_node","",options){

  // Added below
  // Use ParameterDescriptor
  rcl_interface::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true; // set read only

  // Declare parameters
  this->declare_parameter("param1",1.0);
  this->declare_parameter("param2","ok",descriptor);
  this->declare_parameter("param3.age",10);
  this->declare_parameter("param3.name","nek",descriptor);
  this->declare_parameter("param4", true);
  std::vector<uint8_t> param5(2,0); # example 1, makeing value and use it to declare
  this->declare_parameter("param5",param5);
  this->declare_parameter("param6",std::vector<bool>(3,true)); # example 2, make vector when declare
  this->declare_parameter("param7",std::vector<int64_t>(1,0));
  this->declare_parameter("param8",std::vector<double>(4,0.0));
  this->declare_parameter("param9",std::vector<string>(4,""));

  using namespace std::placeholders;
  // Register callback function
  reset_param_callback_function_handler_ = this->add_on_set_parameters_callback(
    std::bind(&ParamTestNode::reset_param_callback_function_, this, _1)
  );
  //Use parameter
  auto a1 = this->get_parameter("param1").as_int();
  auto a2 = this->get_parameter("param2").as_double();
  auto a3 = this->get_parameter("param3").as_string();
  auto a4 = this->get_parameter("param4").as_bool();
  auto a5 = this->get_parameter("param5").as_byte_array();
  auto a6 = this->get_parameter("param6").as_bool_array();
  auto a7 = this->get_parameter("param7").as_integer_array();
  auto a8 = this->get_parameter("param8").as_double_array();
  auto a9 = this->get_parameter("param9").as_string_array();
  auto results = this->set_parameters({
    rclcpp::Parameter("param2","ng"),
    rclcpp::Parameter("param3.age",11)
  });
}

// Added below
// Define callback function
rcl_interfaces::msg::SetParametersResult
ParamTestNode::reset_param_callback_function_(const std::vector<rclcpp::Parameter>& params){
  auto results = std::make_shared<rcl_interface::msg::SetParametersResult>();
  results->successful = true;
  results->reason="";

  for(auto&& param : params){
    // recogize param, process it.
    if(param.get_name() == "param1"){
      double tmp = param.as_double();
      // process something what you want.
    }else if(param.get_name() == "param2"){
      if(/* something to be wrong */){
        results->successful = false;
        results->reason = "hogehoge was hoihoi.";
        return *results;
      }
    }...
  }
  return *results;
}
...

} // end of namespace
RCLCPP_COMPONENTS_REGISTER_NODE(srv_pkg::ParamTestNode)
```

An example of yaml file is as follows.

```yaml
# ns_name: # if use namespace
param_test_node: # node name
  ros__parameters:
    param1: 10 # int
    param2: "ok" # string
    param3: # nest parameter
      age: 10
      name: "neknek"
    param4: true # bool
    param5: [10,20] # byte array
    param6: [true, false, false] # bool array
    param7: [30] # interger array, if only one value
    param8: [20.0, 30.0, 40.0, 50.0] # double array
    param9: ["ok", "ng", "hoge", "really?"] # string array
```

# Information
## add_on_set_parameters_callback function
### Declaration of callback function

Callback functions which shuld be registered in add_on_set_parameters_callback function must be defined as `OnParametersSetCallbackType`.
This type is defined as follows.

```c++
using OnParametersSetCallbackType =
  std::function<
  rcl_interfaces::msg::SetParametersResult(const std::vector<rclcpp::Parameter> &)
  >;
```

So callback functions must be defined as follows.

* arguments
  * const std::vector\<rclcpp::Parameter\>&
* return value
  * rcl_interface::msg::SetParametersResult

`rcl_interface::msg::SetParametersResult` has two attributes as follows.

* bool successful
  * result of setting of parameters
* string reason
  * reason of a result of setting of parameters, especially when successful == false

Plural callback functions can be set.
If one callback funciton returns successful == false, process of changes of parameters will stop and process of other callback functions which are not executed yet will be discontinued.

See [rclcpp::Node::add_on_set_parameters_callback in Official api reference](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html#a12d535bced9f26b65c0a450e6f40aff8).

### Functions for `param` in callback functions
In example, `param.as_double()` is used.
The usable member functions of `param` shows in [rclcpp::Parameter in official api reference](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Parameter.html).


### Timing of execution of callback funciton

1. execute declare_parameter()
1. read yaml file for parameters
1. <font color="red">call callback function</font>
1. execute set_parameters(), or set parameters by console
1. <font color="red">call callback function</font>

## ParameterDescriptor
Properties of parameters can be controled by ParameterDescriptor.

Reference:

* [ParameterDescriptor.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterDescriptor.msg)
* [ParameterType.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterType.msg)
* [rclcpp/rclcpp/doc/notes_on_statically_typed_parameters.md](https://github.com/ros2/rclcpp/blob/master/rclcpp/doc/notes_on_statically_typed_parameters.md)
* [example](https://github.com/ros2/rclcpp/blob/master/rclcpp/test/rclcpp/test_node.cpp#L1078-L1084)

### Basic usage of ParameterDescriptor
Choose attributes what you want to set and set them.
And declare parameters with it.

```c++
// In constructor
  ...
  rcl_interface::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true; // set read only
  descriptor.integer_range.resize(1);
  auto& range = descriptor.integer_range.at(0);
  range.from_value=1;
  range.to_value=4;
  range.step=2;

  this->declare_parameter("param",0,descriptor);
```

attributes are as follows.

* string name
  * the name of parameter
* uint8 type
  * usable type is written in [rcl_interface::msg::ParameterType.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterType.msg)
* string description
  * like memo
string additional_constraints
  * like memo about constraints what you consider
  * never restrict something, just work as memo
* bool read_only
  * default value is `false`, which means parameter is writable
* bool dynamic_typing
  * if it is true, type of parameter can change after declaration
  * default value is `false` after galactic, and `true` before `foxy`
* About range of value of parameter
  * related attirbutes
    * FloatingPointRange[<=1] floating_point_range
    * IntegerRange[<=1] integer_range
  * They consist of a `from_value`, `to_value` and `a step`.
  * Notice
    * Both attributes are mutually exclusive.
    * Usage of them is same.  
    * If both are set and not empty, the constraint apply this parameter, and opposite.

### FloatingPointRange and IntegerRange
The explanation of usage of them is done by examples.
The examples are in the case of FloatingPointRange, the same is in the case of IntegerRange.

* from_value=1.0, to_value=2.0,step=0.5
  * valid values: {1.0, 1.5, 2.0}
* from_value=1.0, to_value=2.0,step=0.8
  * valid values: {1.0, 1.8, 2.0}
* from_value=1.0, to_value=2.0,step=3.0
  * valid values: {1.0, 2.0}

## Gouping parameters
An following example is in yaml file.

```yaml
param_test_node: # node name
  ros__parameters:
    param:
      age: 10
      name: "nek"
```

So they can be used in source code by combining with `.`.

```c++
  this->declare_parameter("param.age",11);
  this->declare_parameter("param.name","kura");
...
  auto a = node->get_parameter("param.age").as_int();
```
