[Topページへ](../../README_JP.md)

パラメータの使用としては，自分自身(ノード)に宣言し使用する場合と他のノードのものを使用する場合が考えられる．
ここでは前者を対象として，その方法を述べる．
コンポーネントノードに実装するので，まず[making_package_of_component_node.md](making_package_of_service_node.md)を見てコンポーネントノードを作成しておくこと．

# 初期手順
いつくかのパラメータを使用するが，その名前などについては都度簡単に説明するかほとんど説明しないので適当に読み替えるように．

## serviceノードのコーディングの流れ

**In hpp file**

1. パラメータ変更に対するコールバック関数の宣言
   * lambda関数を使用する場合不必要．
   * パラメータ変更に対する特段の処理がない場合不必要

**In cpp file**

1. ParameterDescriptorの使用
   * 細かいパラメータ設定で使用．もし細かく設定しないなら不必要．
1. パラメータの宣言
1. パラメータ変更に対するコールバック関数の実装
1. パラメータの使用

1. Use ParameterDescriptor
   * For detailed setting of parameters. No need if no detailed setting.
1. Declare parameters
1. Define callback function(s) for changes of parameters
   * パラメータ変更に対する特段の処理がない場合不必要
1. Use Parameter

**yaml files**
もしyamlファイルで設定したいパラメータがあればyamlファイルを用意する．
すべてのパラメータについて書かなくてもよい．
書いたものだけ読み込まれる．

もしyamlファイルを用いる場合，以下のように`--ros-args --params-file <yaml file>`を使う．

```shell
$ ros2 run <package name> <target name> --ros-args --params-file <yaml file>
```

## 主要関数

* declare_parameter
* undeclare_parameter
* has_parameter
* set_parameters
* get_parameter
* get_parameters
* list_parameters
* add_on_set_parameters_callback
* remove_on_set_parameters_callback

詳細な情報は[公式のapi reference](https://docs.ros2.org/foxy/api/rclcpp/index.html)を参照のこと．

## コーディング例
追加したところに`// Added below`のコメントを追加しているので参考に．

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
      auto tmp = param.as_double();
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




# 情報
## add_on_set_parameters_callback関数について
### コールバック関数の宣言
add_on_set_parameters_callback関数に登録されるコールバック関数は`OnParametersSetCallbackType`型でなければならない．
この方は以下のように定義されている．

```c++
using OnParametersSetCallbackType =
  std::function<
  rcl_interfaces::msg::SetParametersResult(const std::vector<rclcpp::Parameter> &)
  >;
```

よってコールバック関数を以下のように定義するとよい．

* 引数
  * const std::vector\<rclcpp::Parameter\>&
* 返り値
  * rcl_interface::msg::SetParametersResult

また`rcl_interface::msg::SetParametersResult`は以下の二つの属性を持つ．

* bool successful
  * バラメータ設定の結果(成功か失敗か)
* string reason
  * 設定の結果の理由(特に失敗時)

このようなコールバック関数は複数設定できる．
ここで，あるひとつのコールバック関数が`successful==false`(失敗)を返したら，他の未実行なコールバック関数も処理をやめ，バラメータ変更自体が中止となる．

詳しい情報は[公式apiのrclcpp::Node::add_on_set_parameters_callback](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html#a12d535bced9f26b65c0a450e6f40aff8)を参照のこと．

### コールバック関数内の`param`で使える関数群
例では`param.as_double()`を使用している．
このような使用できる関数は[公式apiのrclcpp::Parameter](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Parameter.html)を参照のこと．

### コールバック関数実行のタイミング

1. declare_parameter()の呼び出し
1. パラメータ設定のためのyaml fileの読み込み
1. <font color="red">コールバック関数の呼び出し</font>
1. set_parameters()やコンソールからのパラメータ設定
1. <font color="red">コールバック関数の呼び出し</font>

## ParameterDescriptor
ParameterDescriptorによってパラメータのプロパティを設定することができる．

参照:

* [ParameterDescriptor.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterDescriptor.msg)
* [ParameterType.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterType.msg)
* [rclcpp/rclcpp/doc/notes_on_statically_typed_parameters.md](https://github.com/ros2/rclcpp/blob/master/rclcpp/doc/notes_on_statically_typed_parameters.md)
* [example](https://github.com/ros2/rclcpp/blob/master/rclcpp/test/rclcpp/test_node.cpp#L1078-L1084)

### ParameterDescriptorの基本的な使い方
使用したい属性に対して設定し，declareで使用すればよい．

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

使用できる属性は以下の通り．

* string name
  * パラメータの名前
* uint8 type
  * バラメータの型(使用できる型の一覧は[rcl_interface::msg::ParameterType.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterType.msg)を参照のこと)
* string description
  * メモみたいなもの
string additional_constraints
  * メモみたいなもの
    * 特に制限に関するメモ
  * メモであって実際に制限の効力はない
* bool read_only
  * デフォルトは`false`(書き込み可)
* bool dynamic_typing
  * 使用中に型を変更できるか否か
  * デフォルトは`false`(galactic以降)，`true`(foxy以前)
* パラメータの値の制限
  * 関連する属性
    * FloatingPointRange[<=1] floating_point_range
    * IntegerRange[<=1] integer_range
  * `from_value`, `to_value`, `a step`を設定可能
  * 注意
    * どっちかしか使用できない
    * 使用方法はどちらも同じである
    * 設定したら制限がかかるし設定しなければ制限はかからない

### FloatingPointRangeとIntegerRange
以下の説明は`FloatingPointRange`を対象に例を用いて行う．
`IntegerRange`の場合も同様となる．

* from_value=1.0, to_value=2.0,step=0.5
  * 有効な値 : {1.0, 1.5, 2.0}
* from_value=1.0, to_value=2.0,step=0.8
  * 有効な値 : {1.0, 1.8, 2.0}
* from_value=1.0, to_value=2.0,step=3.0
  * 有効な値 : {1.0, 2.0}
* from_value=1.0, to_value=0.0,step=0.5
  * 有効な値 : {1.0, 0.5, 0.0}
* from_value=1.0, step=0.5
  * 有効な値 : {1.0, 0.5, 0.0}
  * 設定しないと'0'になる？

## バラメータのグループ化
yamlファイル例で紹介する．

```yaml
param_test_node: # node name
  ros__parameters:
    param:
      age: 10
      name: "nek"
```

この時，paramによってageとnameがグループ化される．使用するときには`.`でつなげればよい．

```c++
  this->declare_parameter("param.age",11);
  this->declare_parameter("param.name","kura");
...
  auto a = node->get_parameter("param.age").as_int();
```
