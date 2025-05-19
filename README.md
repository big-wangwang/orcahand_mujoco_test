<div style="display: flex; gap: 10px;"> <img src=".docs/orcahand.png" alt="ORCA机械手" height="300"> <img src=".docs/orcahand_combined_extended.png" alt="组合扩展版ORCA机械手" height="300"> </div>
ORCA机械手 模型描述
本仓库包含ORCA机械手的模型描述文件。当前仅提供MJCF格式的模型描述文件。其中'extended'扩展版本包含附加组件（含惯性属性），例如相机支架、U2D2控制板和散热风扇。

URDF格式描述文件即将推出——目前可使用转换工具如 mjcf_urdf_simple_converter 或 mjcf2urdf 将MJCF描述转换为URDF格式。

使用示例
克隆本仓库：

bash
git clone git@github.com:orcahand/orcahand_description.git
cd orcahand_description
安装依赖项：

bash
pip install mujoco
在MuJoCo中运行仿真：

bash
python3 test1.py
文件说明
mjcf/: 包含不同版本的MJCF模型文件

basic/: 基础版机械手模型

extended/: 扩展版模型（含附加组件）

.docs/: 存放说明文档图片

test1.py: MuJoCo仿真示例脚本

技术特性
精确的关节参数配置

符合实际物理特性的质量惯性参数

支持MuJoCo物理引擎的接触力学模拟

模块化设计便于扩展修改

如有任何使用问题，请通过GitHub Issues提交反馈。
