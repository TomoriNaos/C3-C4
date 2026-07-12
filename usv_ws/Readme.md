# USV Fog Simulation Project

本工程用于琼州海峡大雾低能见度场景下的无人船多模态感知仿真。系统在 ROS 2 Humble + Gazebo Classic 中融合普通相机、门控相机、深度相机、毫米波雷达、声呐和无人机观测，实现海上目标 YOLO 识别、融合跟踪、跟随与避障评估。

## 从零部署

当前项目以 `usv_ws` 为工程根目录。最终上传/交付时只需要保留这个目录：

```text
usv_ws/
├── Readme.md
├── src/
├── scripts/
└── eval_outputs/
```

`build/`、`install/`、`log/` 是编译后自动生成的目录，不作为源码包内容上传；如果刚下载工程时没有这些目录是正常的，执行后文编译命令后会重新生成，其中 `install/setup.bash` 只有编译成功后才会出现。

辅助查看模型与类别的本地样例数据：

```text
~/c3example/
```

该目录由 `scripts/capture_c3example_dataset.py` 生成，包含普通去雾相机、门控伪彩色相机、无人机俯视相机的单目标和复杂场景截图，标注时优先看这个目录里的真实仿真截图。
如需输出到其他位置，可在运行脚本前设置 `C3EXAMPLE_DIR=/path/to/output`。

下面命令先进入 `usv_ws` 根目录，再设置工程路径变量。后续命令都默认已经执行过这两行：

```bash
cd ~/usv_ws
export USV_WS="$(pwd)"
```

如果你的工程不在 `~/usv_ws`，把第一行改成实际路径，例如：

```bash
cd /path/to/usv_ws
export USV_WS="$(pwd)"
```

推荐环境：

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Classic 11
Python 3.10
ONNX Runtime 1.23.2
```

安装基础依赖：

```bash
sudo apt update
sudo apt install -y \
  git wget python3-pip python3-colcon-common-extensions python3-rosdep \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-rviz2 \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs
```

初始化 `rosdep`：

```bash
sudo rosdep init || true
rosdep update
```

安装 ONNX Runtime。推荐放在用户目录，例如 `~/onnxruntime-linux-x64-1.23.2`：

```bash
cd ~
wget https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/onnxruntime-linux-x64-1.23.2.tgz
tar -xzf onnxruntime-linux-x64-1.23.2.tgz
```

如果 ONNX Runtime 在其他目录，编译时指定：

```bash
colcon build --symlink-install --cmake-args -DONNXRUNTIME_ROOT=/your/onnxruntime/path
```

### VRX / WAM-V 资源

本工程内置了简化 WAM-V URDF 和自定义海上目标，主仿真不直接 `find_package(vrx)`；但新机器经常缺 WAM-V/VRX/Gazebo 海上模型资源，建议部署时下载 VRX。两种方式二选一。

方式 1：完整编译 VRX 工作区：

```bash
mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src
git clone -b humble https://github.com/osrf/vrx.git
cd ~/vrx_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

方式 2：只下载 VRX Gazebo models：

```bash
mkdir -p ~/.gazebo/models
cd /tmp
wget -q https://github.com/osrf/vrx/archive/humble.tar.gz -O vrx.tar.gz
mkdir -p vrx_models
tar -xzf vrx.tar.gz -C vrx_models --strip-components=2 vrx-humble/models
cp -r vrx_models/* ~/.gazebo/models/
```

如果编译了 VRX，新终端先 source ROS 和 VRX；本工程的 `install/setup.bash` 需要等本工程编译成功后再 source：

```bash
source /opt/ros/humble/setup.bash
source ~/vrx_ws/install/setup.bash
```

获取并编译本工程。以下命令需要在 `usv_ws` 根目录执行：

```bash
cd ~/usv_ws
export USV_WS="$(pwd)"
rosdep install --from-paths src --ignore-src -r -y
./scripts/build_clean_env.sh
source install/setup.bash
```

注意：`install/setup.bash` 是 `colcon build` 成功后生成的文件。第一次部署时，如果还没编译过，`install/` 目录不存在是正常现象；先运行 `./scripts/build_clean_env.sh`，成功后再 `source install/setup.bash`。

## 快速使用

编译：

```bash
cd ~/usv_ws
export USV_WS="$(pwd)"
./scripts/build_clean_env.sh
source install/setup.bash
```

如果提示 `./scripts/build_clean_env.sh: 没有那个文件或目录`，说明你当前不在 `usv_ws` 根目录，或者 `${USV_WS}` 没有正确设置。先检查：

```bash
pwd
ls scripts/build_clean_env.sh
```

正确的 `pwd` 应类似 `$HOME/usv_ws`，并且能够看到 `src/`、`scripts/`、`Readme.md`。如果进入工程目录后提示符变成了 `~`，通常是工程路径变量为空或路径写错；请重新设置：

```bash
cd ~/usv_ws
export USV_WS="$(pwd)"
```

启动：

```bash
ros2 launch usv_bringup sim.launch.py
```

如果 Gazebo 报 multicast 网卡错误，用本地通信启动：

```bash
./scripts/launch_sim_localhost.sh gui:=false rviz:=true
```

常用参数：

```bash
ros2 launch usv_bringup sim.launch.py target_model:=moving_vessel
ros2 launch usv_bringup sim.launch.py target_model:=small_fishing_boat
ros2 launch usv_bringup sim.launch.py c3_multimodal_fusion:=true
ros2 launch usv_bringup sim.launch.py c3_mmwave:=true
ros2 launch usv_bringup sim.launch.py pseudocolor_gated_yolo:=true
ros2 launch usv_bringup sim.launch.py uav:=false
```

### 当前探测范围

当前 C3 热力图和融合候选区域使用无人船 `base_link` 坐标系：

```text
x: 0 ~ 300 m
y: -150 ~ 150 m
```

含义是：

```text
前向 300 m
后向 0 m
左侧 150 m
右侧 150 m
```

为了避免热力图边角目标被距离过滤提前裁掉，相关距离上限同步放大：

```text
c3_multimodal_buffer_fusion.max_range_m: 360 m
radar_sonar_tracker.max_tracking_range: 360 m
usv_target_follower.max_follow_range: 360 m
uav_gated_camera_recognizer.gate_far: 120 ~ 370 m
UAV 门控相机 Gazebo far clip: 380 m
```

船载普通相机、船载门控相机和深度相机仍主要负责近中距离识别；300m 级远距离候选主要依赖毫米波雷达、声呐点云和无人机门控二次确认。

## 模型与数据

当前模型分工：

| 模型 | 相机类型 | 默认节点 | 实时融合阈值 |
| --- | --- | --- | --- |
| `src/usv_bringup/models/best.onnx` | 普通相机 RGB 图像 | `gated_camera_recognizer` | `confidence_threshold: 0.10` |
| `src/usv_bringup/models/best.onnx` | 深度相机 RGB 图像 | `depth_camera_recognizer` | `confidence_threshold: 0.08` |
| `src/usv_bringup/models/best1.onnx` | 船载门控伪彩色、无人机门控伪彩色 | `pseudocolor_gated_camera_recognizer`、`uav_gated_camera_recognizer` | `confidence_threshold: 0.08` |

两个 ONNX 的类别顺序一致：

```text
0 small_fishing_boat
1 moving_vessel
2 research_platform
3 service_boat
4 survey_boat
5 cargo_ship_far
6 anchored_tanker
7 obstacle
```

点云输出里的 `class_id` 已经和 ONNX 类别顺序保持一致。
所以 `usv_target_follower.follow_class_ids: [0.0, 1.0, 2.0, 3.0, 4.0]` 表示只优先跟踪 `small_fishing_boat`、`moving_vessel`、`research_platform`、`service_boat`、`survey_boat` 这五类；`cargo_ship_far`、`anchored_tanker` 和 `obstacle` 不作为主动跟踪目标。

注意：如果 `best.onnx` 或 `best1.onnx` 仍是旧六分类模型，必须用新的 8 类数据重新训练并替换模型；否则代码虽然能运行，但类别编号会和真实语义不一致。

当前工程只保留运行所需的 ONNX 模型和可视化结果，不再随仓库附带原始 YOLO 训练数据集或 Roboflow 导出目录。仿真运行不需要外部数据集；重新训练或重新做数据集级准确率评估时，需要自行提供 YOLO 格式数据。

### 新 8 类标注截图

本地已生成一套新类别标注参考图，默认位置：

```text
~/c3example/
├── classes.txt
├── manifest.csv
├── normal/single/        # 普通去雾相机，单目标多角度
├── gated/single/         # 门控伪彩色相机，单目标多角度
├── normal/complex/       # 普通去雾相机，100 张复杂场景
├── gated/complex/        # 门控伪彩色相机，100 张复杂场景
├── normal/uav_topdown/   # 无人机俯视普通相机，单目标多角度
└── gated/uav_topdown/    # 无人机俯视门控伪彩色，单目标多角度
```

生成规则：

1. `0~6` 类每个目标按 60 度间隔生成 6 张单目标图片。
2. `obstacle` 类中每一种仿真物体生成 2 张单目标图片。
3. `complex_000.png` 到 `complex_099.png` 是普通相机和门控相机一一对应的复杂场景图片，文件编号相同就表示同一时刻、同一场景。
4. 复杂场景每张通常 2~3 个目标，重点覆盖 `small_fishing_boat`、`moving_vessel`、`research_platform`、`service_boat`、`survey_boat` 五类；其他类只少量出现，避免训练集被非跟踪目标占满。
5. 复杂场景会轻微改变初始视角、目标距离、横向位置和遮挡关系，但不做过度拥挤的堆叠。

普通相机和门控相机要按各自图片分别标注：如果某个目标在普通相机里很清楚，但在门控伪彩色图里几乎不可辨认，门控图不要强行标；如果门控图里能看出稳定轮廓，即使颜色弱一些也应该标。需要增强门控可标注性时，优先调整采图脚本或 `perception.yaml` 中的 `gate_near/gate_mid/gate_far`，不要把不可见目标当作正样本。

这些图片建议先作为训练集使用；验证集不要从同一批高度相似图片里随机抽，最好重新用不同距离、不同随机种子、不同遮挡关系再生成一批，或者手动截取另一组场景。Roboflow 中可以在生成 Dataset Version 时设置 Train/Valid/Test 比例，也可以在数据管理页面把图片手动分配到 train、valid、test。若本地训练 YOLO，则按下面结构组织即可：

```text
dataset/
├── images/train
├── images/val
├── labels/train
├── labels/val
└── data.yaml
```

当前模型是 YOLO 目标检测模型，标注请用矩形框，不需要轮廓标注。中心点坐标由程序直接取 `bbox_cx/bbox_cy`，再结合深度估计三维位置；只有后续改成 YOLO-seg 或实例分割时，才需要轮廓标注。

复现截图前先启动无界面仿真，再运行采图脚本：

```bash
cd ~/usv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch usv_bringup sim.launch.py gui:=false rviz:=false perception:=false dynamic_targets:=false usv_follow:=false uav:=false c3_mmwave:=false rgbd_dehaze:=false

# 另开终端
cd ~/usv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/capture_c3example_dataset.py
```

无人机俯视数据不建议和船载水平视角“无脑混用”。它们可以共用同一套 8 类类别名，但视角差异很大：如果要训练同一个 YOLO，需要同时放入船载视角和俯视视角，并单独验证俯视子集；如果无人机识别效果要求更高，建议给无人机门控相机单独训练一个俯视模型。

### 仿真模型名与类别映射

主场景 `src/usv_bringup/worlds/ocean_fog.world` 中的可评估目标如下。`model_name` 是 Gazebo `/model_states` 中的真实模型名，可作为仿真真值；`class_id` 是识别模型输出的 8 类编号。

| Gazebo model_name | 类别 | class_id | 说明 |
| --- | --- | ---: | --- |
| `small_fishing_boat` | `small_fishing_boat` | 0 | 小型渔船 |
| `moving_vessel` | `moving_vessel` | 1 | 默认跟踪目标船 |
| `research_platform` | `research_platform` | 2 | 海上平台，当前属于重点跟踪类 |
| `service_boat` | `service_boat` | 3 | 作业船 |
| `survey_boat` | `survey_boat` | 4 | 测量船 |
| `cargo_ship_far` | `cargo_ship_far` | 5 | 远距离货船/大船，主要给远程探测使用 |
| `anchored_tanker` | `anchored_tanker` | 6 | 锚泊油轮/大船，当前不作为主动跟踪目标 |
| `fishnet_buoy` | `obstacle` | 7 | 渔网浮标，归入障碍物 |
| `channel_buoy_north` | `obstacle` | 7 | 航道浮标，归入障碍物 |
| `channel_buoy_south` | `obstacle` | 7 | 航道浮标，归入障碍物 |
| `navigation_marker_port` | `obstacle` | 7 | 左舷侧标，归入障碍物 |
| `navigation_marker_starboard` | `obstacle` | 7 | 右舷侧标，归入障碍物 |
| `drift_debris` | `obstacle` | 7 | 漂浮碎片/箱体，归入障碍物 |
| `floating_container` | `obstacle` | 7 | 漂浮集装箱，归入障碍物 |
| `floating_obstacle` | `obstacle` | 7 | 漂浮障碍物 |
| `net_line_a` | `obstacle` | 7 | 网线/漂浮障碍 |

模型名评估的基本口径：

1. 评估真值来自 Gazebo `/model_states`，不是相机图片标注。
2. 模型在本船坐标系前方或近后方，即 `x > -5m`，且距离不超过 `c3_multimodal_buffer_fusion.max_range_m`，才进入可评估真值集合。
3. 一个已确认目标在真值模型 8m 内，算定位命中；类别也一致时，算分类正确。
4. 真值模型进入评估范围但 8m 内没有已确认目标，算漏检。
5. 已确认目标不在任何真值模型 8m 内，且也不属于非当前评价目标附近，算误检。
6. 当前 launch 默认按 `target_model` 统计总 Precision/Recall，例如默认只统计 `moving_vessel`；`/c3/perception_metrics` 的 `model_eval` 字段会额外列出每个场景模型的命中、漏检和分类状态。
7. 当前为了保证仿真跟踪稳定，`enable_sim_truth_confirmation` 默认开启，会用 Gazebo 真值辅助二次确认；做严格识别准确率实验时应临时关闭它，否则模型名评估结果会偏乐观。

标注数据集时必须按上表区分 0~6 类船/平台；其余浮标、漂浮物、集装箱、网线等都标成 `obstacle`。复杂图中只要目标可见，就应该全部标注；漏标的可见物体会被 YOLO 当成背景学习，后面会直接增加漏检和误检。

### 当前视觉识别方法

| 方法 | 当前识别方式 | 定位方式 | 数据集要求 |
| --- | --- | --- | --- |
| 船载门控相机 | `best1.onnx` 对门控伪彩色图做 YOLO | 用 YOLO bbox 中心区域的深度和相机内参/视场角解算 `base_link` 三维坐标 | 使用门控伪彩色图训练，覆盖近/中/远距离、遮挡、雾、不同朝向；每张图里所有可见目标都要标注 |
| 普通相机 | `best.onnx` 对去雾增强后的普通图像做 YOLO | 同样用 bbox + 深度估计坐标 | 如果推理前去雾，训练/验证图也建议使用同样去雾流程，避免训练图和推理图分布不一致 |
| 深度相机 | `best.onnx` 对 RGB 图像做 YOLO | bbox 中心深度中值测距，输出 `PointCloud2` 检测点 | 2D YOLO 仍需要 RGB/去雾 RGB 标注；如果后续改成点云检测，则需要 3D 框或仿真真值生成的 3D 标签 |
| 无人机门控相机 | `best1.onnx` 对无人机门控伪彩色图做 YOLO | 俯视 bbox + 深度确认远距离候选点 | 俯视图和船载水平视角差异大，建议单独验证或单独训练 |

门控相机和深度相机发布的 `detection_points` 不是整幅密集深度点云，而是目标检测后的定位点云。为了避免每个 bbox 只有一个中心点导致融合过稀，当前每个检测框默认用 `bbox_support_grid_side: 3` 在框内采样中心点和周围支持点；如果点云仍太少，优先检查 YOLO 是否检出目标，其次再调低 `confidence_threshold`、减小 `depth_roi_shrink` 或增大 `bbox_support_grid_side`。

当前工程已经删除 BEV 几何旁路、三切片几何识别和 YOLO 空框轮廓兜底；传统几何不再输出类别。C3 热力图只做多模态位置投票和候选点生成，不负责判断目标类别。

建议重做数据集时分成两套：`best.onnx` 使用普通/深度相机 RGB 或去雾 RGB 数据；`best1.onnx` 使用门控伪彩色数据。无人机俯视图可以作为同类别补充，但最好单独做验证集。

### YOLO 之外的可选识别方法

| 方法 | 适合输入 | 优点 | 数据集要求 |
| --- | --- | --- | --- |
| PointPillars | 门控/深度相机反投影得到的伪点云 | 速度快，适合船体、平台、漂浮物几何检测 | 需要 3D 框；仿真中可用 Gazebo 模型位姿和模型尺寸自动生成初始标签 |
| SECOND | 伪点云或稀疏点云 | 比 PointPillars 更重，但空间特征更强 | 需要 3D 框，最好有多距离、多遮挡样本 |
| PV-RCNN | 伪点云/深度点云 | 精度高，适合复杂目标几何 | 数据要求最高，需要较准的 3D 标注 |
| VoteNet / PointNet++ | 点云目标 proposal 或点云分割 | 不依赖图像纹理，适合雾天几何识别 | 需要 3D 框、点级标签或实例分割标签 |
| Range-view CNN/Transformer | 门控近/中/远切片按通道输入 | 保留门控相机“距离切片”的物理特性 | 可用 2D 框训练，但最好保留三切片原始通道 |
| 3D-CNN / Gated3D | 近/中/远切片组成小体数据 | 能学习距离维度上的回波变化，理论上比伪 RGB 更贴近门控相机 | 需要切片序列和目标框/掩码；数据量要比普通 YOLO 更多 |
| 几何聚类 + EKF/JPDA | 深度点云、雷达/声呐点 | 不需要大量标注，适合只做位置跟踪兜底 | 主要调参和真值评估，不解决细分类问题 |
| 雷达/声呐 ROI 反投影 + YOLO 二次检测 | 多模态融合后的候选点 | 能把远距离小目标从整图搜索变成局部搜索，降低漏检 | 仍需要 2D 框，但对远距离样本数量要求低一些 |

当前阶段最稳妥的路线是先把新 8 类 YOLO 数据补好；如果后续要明显提高雾天定位和遮挡鲁棒性，再从 PointPillars 或“雷达/声呐 ROI + YOLO 二次检测”开始升级。

## 检测准确度

旧六分类的历史准确率不能继续作为新 8 类模型的准确率。重新训练 `best.onnx` 和 `best1.onnx` 后，必须用同一套 8 类 `data.yaml` 重新评估普通相机、门控相机、深度相机和无人机俯视子集。

实际仿真融合时为了降低漏检，普通相机、深度相机、船载门控和无人机门控默认使用较低 YOLO 阈值；正式报告的 Precision/Recall/F1 应以验证集脚本输出为准，不要直接用融合阈值代替模型准确率。

已保留的可视化图片：

```text
eval_outputs/camera_detection_accuracy_summary.jpg
eval_outputs/normal_camera_accuracy_sheet.jpg
eval_outputs/gated_camera_accuracy_sheet.jpg
eval_outputs/camera_threshold_sweep_summary.jpg
eval_outputs/camera_selected_class_metrics.png
eval_outputs/camera_selected_detection_examples.png
```

这些图片用于说明旧阶段模型效果和系统消融结果；新 8 类数据重做后，应重新生成同名或新目录下的评估图，不能直接沿用旧图作为新类别结论。

截图颜色：

```text
绿色 GT 框：被匹配
红色 GT 框：漏检
黄色预测框：正确预测
紫色预测框：误检
```

如果你后来重新准备了 YOLO 格式数据，可以用下面脚本重新评估。`--data` 改成自己的 `data.yaml`：

```bash
cd ~/usv_ws
export USV_WS="$(pwd)"

python3 scripts/evaluate_yolo_onnx_dataset.py \
  --model src/usv_bringup/models/best.onnx \
  --data /path/to/normal_camera_yolo/data.yaml \
  --split val \
  --conf 0.15 \
  --save-examples 24 \
  --save-dir eval_outputs/current_accuracy/normal_camera

python3 scripts/evaluate_yolo_onnx_dataset.py \
  --model src/usv_bringup/models/best1.onnx \
  --data /path/to/gated_camera_yolo/data.yaml \
  --split val \
  --conf 0.20 \
  --save-examples 24 \
  --save-dir eval_outputs/current_accuracy/gated_camera
```

评估脚本默认优先保存 FP/FN 较多的诊断样例；增加 `--example-order best` 可优先保存 FP/FN 较少的正确检测样例。两种排序只影响截图顺序，不改变准确率计算。

### 一键生成展示图片

展示图可以用总脚本统一复现：

```bash
cd ~/usv_ws
export USV_WS="$(pwd)"

python3 scripts/generate_presentation_visuals.py --skip-yolo
```

`--skip-yolo` 表示不依赖外部 YOLO 数据集，只重绘系统结构图、热力图、指标卡和消融图。若你另行准备了数据集，也可以去掉 `--skip-yolo` 并通过 `--yolo-dir` 指向数据目录，让脚本重新生成相机验证样例。若当前没有 `sonar_capture.json`，脚本会使用固定参考数据生成说明书/PPT 结构图和消融图；正式实验结论仍以 rosbag、实际 JSON 或重新评估得到的 `summary.json` 为准。

`--skip-yolo` 模式会重绘的主要图片：

```text
eval_outputs/c3_system_method_flow.png
eval_outputs/message_interface_alignment.png
eval_outputs/mmwave_radar_effect.png
eval_outputs/heatmap_mmwave_only.png
eval_outputs/heatmap_sonar_only.png
eval_outputs/heatmap_gated_camera.png
eval_outputs/heatmap_depth_camera.png
eval_outputs/heatmap_integrated_multi.png
eval_outputs/target_window_metrics.png
eval_outputs/radar_target_window_metrics.png
eval_outputs/sonar_target_window_metrics.png
eval_outputs/gated_camera_target_window_metrics.png
eval_outputs/depth_camera_target_window_metrics.png
eval_outputs/overall_target_window_metrics.png
```

下面这些相机检测图片是工程中保留的历史结果；没有外部 YOLO 数据集时不会重新计算：

```text
eval_outputs/camera_selected_class_metrics.png
eval_outputs/camera_selected_detection_examples.png
eval_outputs/camera_detection_accuracy_summary.jpg
eval_outputs/normal_camera_accuracy_sheet.jpg
eval_outputs/gated_camera_accuracy_sheet.jpg
eval_outputs/camera_threshold_sweep_summary.jpg
```

这些图片用于复现展示样例、系统框架、消息对齐、热力图和消融效果。完整 8 类准确率应以自行重新评估得到的 `summary.json` 为准。

## 实时输出话题

普通检测结果：

```text
/gated_camera/detections
/gated_camera/detection_points
/gated_camera/detection_details
/gated_camera/annotated
```

门控伪彩色检测结果：

```text
/gated_camera/pseudocolor/detections
/gated_camera/pseudocolor/detection_points
/gated_camera/pseudocolor/detection_details
/gated_camera/pseudocolor/annotated
```

无人机门控检测结果：

```text
/uav/gated_camera/detections
/uav/gated_camera/detection_points
/uav/gated_camera/detection_details
/uav/gated_camera/annotated
```

深度相机 YOLO 检测结果，使用 `best.onnx`：

```text
/depth_camera/detections
/depth_camera/detection_points
/depth_camera/detection_details
/depth_camera/annotated
```

### 原始检测点云字段

普通相机、门控伪彩色、无人机门控、深度相机 YOLO 输出 `sensor_msgs/msg/PointCloud2`。每个点 `point_step=36` 字节，字段全部是 `FLOAT32`：

| 字段 | offset | 含义 |
| --- | ---: | --- |
| `x` | 0 | 目标相对 `base_link` 的前向距离，单位 m |
| `y` | 4 | 目标相对 `base_link` 的横向距离，左正右负，单位 m |
| `z` | 8 | 目标高度，单位 m |
| `intensity` | 12 | 检测置信度，也就是 score/confidence |
| `class_id` | 16 | 类别编号，和 ONNX 类别顺序一致 |
| `bbox_cx` | 20 | 图像 bbox 中心点 x，像素坐标 |
| `bbox_cy` | 24 | 图像 bbox 中心点 y，像素坐标 |
| `bbox_w` | 28 | 图像 bbox 宽度，像素 |
| `bbox_h` | 32 | 图像 bbox 高度，像素 |

`class_id` 编号：

```text
0 small_fishing_boat
1 moving_vessel
2 research_platform
3 service_boat
4 survey_boat
5 cargo_ship_far
6 anchored_tanker
7 obstacle
```

有 bbox 的点云：

```text
/gated_camera/detection_points
/gated_camera/pseudocolor/detection_points
/uav/gated_camera/detection_points
/depth_camera/detection_points
```

### C3 缓存融合点云字段

`c3_multimodal_buffer_fusion` 会把毫米波雷达、声呐、视觉融合模态和深度点云统一成标准点云。每个点 `point_step=40` 字节：

| 字段 | offset | 含义 |
| --- | ---: | --- |
| `x` | 0 | 目标/点相对 `base_link` 的前向距离，单位 m |
| `y` | 4 | 横向距离，左正右负，单位 m |
| `z` | 8 | 高度，单位 m |
| `intensity` | 12 | 该点置信度或模态加权强度 |
| `class_id` | 16 | 物体类别 ID；未知时为 `-1` |
| `source_id` | 20 | 来源编号 |
| `bbox_cx` | 24 | 图像 bbox 中心 x；非图像来源为 `0` |
| `bbox_cy` | 28 | 图像 bbox 中心 y；非图像来源为 `0` |
| `bbox_w` | 32 | 图像 bbox 宽度；非图像来源为 `0` |
| `bbox_h` | 36 | 图像 bbox 高度；非图像来源为 `0` |

`source_id` 编号：

```text
1  mmWave radar
2  sonar
4  depth cloud
31 normal camera YOLO
32 gated pseudo-color YOLO
35 UAV gated camera
36 depth camera YOLO
```

C3 缓存池输出：

```text
/c3/buffer/radar_cloud
/c3/buffer/sonar_cloud
/c3/buffer/vision_cloud
/c3/buffer/depth_cloud
/c3/buffer/integrated_cloud
```

热力图、飞控目标、已确认目标和实时指标：

```text
/c3/heatmap/image
/c3/drone/goal
/c3/detected_objects
/c3/perception_metrics
/c3/perception_markers
```

新增的 `detection_details` 是 `std_msgs/String` JSON，尽量详细输出实时检测数据，包括：

```text
stamp
frame_id
output_prefix
image_topic
depth_topic
model_path
backend
detection_input
confidence_threshold
nms_threshold
dehaze_enabled
image_width / image_height
detection_count / point_count
max_model_score
detections[]:
  index
  label
  class_id
  score
  bbox:
    cx/cy/w/h     目标框中心、宽、高，像素坐标
    x1/y1/x2/y2   目标框左上角和右下角，像素坐标
points[]:
  index
  label
  class_id
  score
  x/y/z
  bbox_cx/bbox_cy/bbox_w/bbox_h
```

查看：

```bash
ros2 topic echo /gated_camera/detection_details
ros2 topic echo /gated_camera/pseudocolor/detection_details
ros2 topic echo /uav/gated_camera/detection_details
ros2 topic echo /depth_camera/detection_details
```

旧融合跟踪和评估仍然保留：

```text
/tracked_objects
/tracked_object_poses
/tracked_objects_text
/usv_follow_status
/tracking_metrics
```

## 检验方法

编译检查：

```bash
cd ~/usv_ws
export USV_WS="$(pwd)"
./scripts/build_clean_env.sh
```

Launch 参数检查：

```bash
cd ~/usv_ws
source install/setup.bash
ros2 launch usv_bringup sim.launch.py --show-args
```

无界面启动检查：

```bash
ros2 launch usv_bringup sim.launch.py gui:=false rviz:=false
```

查看四模态缓存池：

```bash
ros2 topic echo /c3/buffer/radar_cloud --once
ros2 topic echo /c3/buffer/sonar_cloud --once
ros2 topic echo /c3/buffer/vision_cloud --once
ros2 topic echo /c3/buffer/depth_cloud --once
ros2 topic echo /c3/buffer/integrated_cloud --once
```

查看热力图候选点、无人机目标和已确认目标：

```bash
ros2 topic echo /c3/drone/goal
ros2 topic echo /c3/detected_objects
ros2 topic echo /c3/perception_metrics
```

### 指标输出

`/c3/perception_metrics` 会实时输出 PPT 和实验记录常用指标。评价对象由启动参数 `target_model` 同步给 C3，默认只统计 `moving_vessel`；场景中的浮标、平台等真实模型不会被误算为漏检或假阳性。

```text
detection_precision        目标检测精度，按仿真真值匹配统计
false_positive_rate        误检率
miss_rate                  漏检率
classification_accuracy    已匹配目标的分类准确率
model_eval                 按 Gazebo model_name 展开的明细，包含每个模型的 class_id、range、matched、status、match_distance、detected_class_id
evaluation_scope           指标统计口径，默认 target_models
evaluation_target          当前评价目标，默认 moving_vessel
scene_ground_truth_objects 场景内全部真值数量，仅供参考
ground_truth_objects       当前评价口径内的真值数量
single_frame_processing_ms 单帧处理时间
process_memory_mb          当前进程内存占用
detected_objects           已确认目标累计数量
active_detected_objects    最近仍被观测到的有效目标数量
integrated_points          当前融合点云点数
heatmap_best_probability   热力图最大概率
candidate_x / candidate_y  当前热力图候选坐标
radar_points               当前帧毫米波雷达点数
sonar_points               当前帧声呐点数
normal_camera_points       普通相机 YOLO 点数
gated_camera_points        船载门控 YOLO 点数
uav_gated_points           无人机门控 YOLO 点数
depth_camera_yolo_points   深度相机 YOLO 点数
depth_points               深度点云点数
```

当前 `moving_vessel` 跟踪实验的 PPT 可视化结果保存在：

```text
eval_outputs/vessel_metrics_dashboard.png
eval_outputs/vessel_modality_ablation.png
eval_outputs/vessel_tracking_timeline.png
eval_outputs/target_window_metrics.png
eval_outputs/sonar_ablation_overview.png
eval_outputs/sonar_sector_health.png
eval_outputs/sonar_optimization_before_after.png
eval_outputs/live_multimodal_diagnostic.png
eval_outputs/c3_system_method_flow.png
eval_outputs/message_interface_alignment.png
eval_outputs/method_formula_summary.md
```

这些图片由 `scripts/generate_presentation_visuals.py` 统一生成。若目录中存在实际 `sonar_capture.json`，脚本会优先使用真实统计；若没有 JSON，则使用固定参考数据生成可复现的说明图，避免 README 和 PPT 图路径失效。

### 降低 moving_vessel 漏检的优先顺序

1. 补齐 `moving_vessel` 的远距离、小目标、遮挡、逆光和浓雾样本，并确保每张训练图中所有可见目标都被标注；漏标会直接把真实目标训练成背景。
2. 按航次而不是随机图片划分训练集和验证集，避免相邻帧泄漏造成虚高准确率；单独报告 `moving_vessel` 的 Precision、Recall 和 PR 曲线。
3. 对远距离图像使用切片推理或提高 YOLO 输入分辨率，使目标在网络输入中保留足够像素；同时加入与真实运行一致的门控伪彩色、雾浓度和重影分布。
4. 使用毫米波雷达热区反投影到相机图像形成 ROI，再对 ROI 做 YOLO 二次检测；它比整幅图盲检更适合远处小船。
5. 保持当前 30 秒 EKF 轨迹窗口，并采用“连续两帧低置信检测也可维持轨迹”的 track-before-detect 策略，减少偶发无框导致的跟丢。
6. 标定相机与雷达外参并检查时间戳。空间偏差超过匹配门限时，即使两个模态都检测到目标，融合层仍会把它们视为不同物体。
7. 最后再调整 `confidence_threshold` 和 NMS。阈值降低只能找回边缘预测，也会增加误检，不能替代补标和数据增强。

`/c3/detected_objects` 中的关键字段：

```text
object_id      已确认目标计数 ID，和类别 class_id 无关
class_id       8 类物体类别 ID
name           物体名称
x/y/z          EKF 滤波后的当前位置
vx/vy/vz       EKF 估计速度
predicted_x/y  下一时刻预测位置
confidence     综合置信度
updates        该目标被确认更新的次数
```

## 跟踪目标切换

启动时切换：

```bash
ros2 launch usv_bringup sim.launch.py target_model:=moving_vessel
ros2 launch usv_bringup sim.launch.py target_model:=small_fishing_boat
ros2 launch usv_bringup sim.launch.py target_model:=research_platform
ros2 launch usv_bringup sim.launch.py target_model:=survey_boat
ros2 launch usv_bringup sim.launch.py target_model:=service_boat
```

`target_model` 会同步给动态目标控制、C3 指标统计、无人机远程探查和本船跟随器。默认目标是 `moving_vessel`。

如果只想换“类别优先级”而不是 Gazebo 中的目标模型，改 `src/usv_bringup/config/perception.yaml`：

```yaml
usv_target_follower:
  ros__parameters:
    follow_class_id: 1.0
    follow_class_ids: [0.0, 1.0, 2.0, 3.0, 4.0]
```

当前默认只跟踪 0/1/2/3/4 五类。若临时只跟踪单个类别，例如 `moving_vessel`，可改成 `[1.0]`；若临时跟踪 `research_platform`，可改成 `[2.0]`。当前没有运行时热切换服务，建议停掉 launch 后改参数或换 `target_model:=...` 重新启动。

本船跟随器的目标来源优先级：

```text
1. /c3/detected_objects          C3 已确认目标，优先使用
2. /tracked_objects_text         传统雷达/声呐/视觉跟踪保留路径
3. /tracked_object_poses         旧 PoseArray 兼容路径
4. /uav/remote_target_status     无人机二次确认回传
5. Gazebo target_model fallback  仿真兜底，避免短时漏检导致船完全不动
```

运行过程中如果当前目标短时丢失，跟随器会在融合轨迹中自动重选目标，默认只在 `class_id=0/1/2/3/4` 中选择候选。

## 工程结构

### 交付目录约定

最终上传或交给别人复现时，保留 `usv_ws` 一个目录即可：

```text
usv_ws/
├── Readme.md
├── src/
├── scripts/
└── eval_outputs/
```

`usv_ws` 内不要上传 `build/`、`install/`、`log/`、`__pycache__/` 这类生成目录。重新编译时运行 `./scripts/build_clean_env.sh` 即可恢复。原始训练数据集和临时截图数据不再作为本工程交付内容；运行仿真只依赖 `src/usv_bringup/models/` 中的 ONNX 模型。

### usv_ws 根目录

```text
usv_ws/
├── Readme.md              # 部署、运行、话题、评估和目录说明
├── 点云信息.png           # 点云字段说明参考图
├── src/                   # ROS 2 源码包
├── scripts/               # 构建、评估、采图和展示图生成脚本
├── eval_outputs/          # 已生成的准确率、消融和 PPT 展示图片
└── example/               # 8 类目标的仿真截图参考图
```

当前工作区没有必需的 `docs/` 目录；说明性内容集中在本 README 和 `eval_outputs/` 的可视化结果中。

### ROS 2 源码包

```text
src/
├── usv_bringup/
│   ├── launch/sim.launch.py                 # 主仿真 launch
│   ├── config/perception.yaml               # 感知、融合、跟随、评估参数
│   ├── worlds/ocean_fog.world               # 琼州海峡大雾海面主场景
│   ├── worlds/annotation_targets.world      # 标注/截图辅助场景
│   ├── models/best.onnx                     # 普通相机和深度相机 YOLO
│   ├── models/best1.onnx                    # 船载门控和无人机门控 YOLO
│   └── docker/Dockerfile                    # 可选容器环境
├── usv_description/
│   ├── urdf/wamv_base.urdf.xacro            # WAM-V 船体和传感器布置
│   └── rviz/default.rviz                    # RViz 默认显示配置
├── usv_perception/
│   ├── include/usv_perception/common.hpp    # 感知节点公共结构
│   ├── scripts/mmwave_scan_converter.py     # 多高度/多扇区毫米波 scan 转点云
│   ├── scripts/mmwave_detection_debug.py    # 毫米波点云调试打印
│   └── src/                                 # 感知、融合、控制、评估节点
└── depth_image_to_pointcloud2/
    ├── launch/depth_camera_dehaze.launch.py # RGB-D 单独测试 launch
    ├── src/depth_image_to_pointcloud2_node.cpp
    └── worlds/fog_depth_camera.world        # 深度相机单独测试场景
```

### usv_perception 节点分工

| 路径 | 作用 |
| --- | --- |
| `src/usv_perception/src/gated_camera_recognizer.cpp` | 普通/门控 YOLO 识别、详细检测话题、点云输出 |
| `src/usv_perception/src/c3_multimodal_buffer_fusion.cpp` | 四模态缓存池、点云对齐、热力图、二次确认、`detected` 目标库、EKF 预测和 C3 指标输出 |
| `src/usv_perception/src/radar_sonar_tracker.cpp` | 传统雷达、声呐、视觉融合跟踪保留路径 |
| `src/usv_perception/src/usv_target_follower.cpp` | 本船跟随和避障控制 |
| `src/usv_perception/src/uav_patrol_controller.cpp` | 无人机远程探查，并接收 `/c3/drone/goal` 做二次确认飞行 |
| `src/usv_perception/src/dynamic_target_controller.cpp` | Gazebo 动态目标运动 |
| `src/usv_perception/src/tracking_evaluator.cpp` | 误检、漏检、CPA/TCPA、ID 切换评估 |
| `src/usv_perception/src/wave_buoyancy_node.cpp` | 船体浮力和海浪扰动仿真 |

### 脚本目录

```text
scripts/
├── build_clean_env.sh                 # 清理环境变量后编译，推荐构建入口
├── launch_sim_localhost.sh            # 本机 Gazebo/ROS 通信启动，规避 multicast 网卡报错
├── capture_c3example_dataset.py       # 生成 ~/c3example 新 8 类普通/门控/俯视截图
├── generate_presentation_visuals.py   # 一键复现 C3 系统图、热力图、指标图和消融图
├── evaluate_yolo_onnx_dataset.py      # 可选：有外部数据集时评估 ONNX
├── test_yolo_onnx_images.py           # 可选：对自备图片文件夹做 ONNX 可视化检测
└── coco_to_yolo_subset.py             # 可选历史辅助：COCO 子集转 YOLO
```

### 输出结果目录

```text
eval_outputs/
├── camera_detection_accuracy_summary.jpg
├── camera_selected_class_metrics.png
├── camera_selected_detection_examples.png
├── c3_system_method_flow.png
├── message_interface_alignment.png
├── method_formula_summary.md
├── mmwave_radar_effect.png
├── heatmap_*.png
├── *_target_window_metrics.png
├── sonar_ablation_overview.png
├── sonar_sector_health.png
├── sonar_optimization_before_after.png
└── live_multimodal_diagnostic.png
```

常用输出文件：

| 路径 | 作用 |
| --- | --- |
| `eval_outputs/camera_selected_class_metrics.png` | 普通/门控相机较稳定类别 Precision/Recall |
| `eval_outputs/camera_selected_detection_examples.png` | 普通/门控相机检测样例拼图 |
| `eval_outputs/c3_system_method_flow.png` | C3 总体方法流程图 |
| `eval_outputs/message_interface_alignment.png` | 多模态消息接口、缓存池和时间对齐结构图 |
| `eval_outputs/method_formula_summary.md` | 当前代码使用的公式、算法和方法汇总 |
| `eval_outputs/mmwave_radar_effect.png` | 纯毫米波雷达效果示意图 |
| `eval_outputs/heatmap_mmwave_only.png` | 纯毫米波雷达热力图 |
| `eval_outputs/heatmap_sonar_only.png` | 纯声呐热力图 |
| `eval_outputs/heatmap_gated_camera.png` | 门控相机热力图 |
| `eval_outputs/heatmap_depth_camera.png` | 深度相机热力图 |
| `eval_outputs/heatmap_integrated_multi.png` | 多模态融合热力图 |
| `eval_outputs/target_window_metrics.png` | C3 目标窗口召回率、漏检率、分类准确率、耗时 |
| `eval_outputs/radar_target_window_metrics.png` | 毫米波雷达单模态指标卡 |
| `eval_outputs/sonar_target_window_metrics.png` | 声呐单模态指标卡 |
| `eval_outputs/gated_camera_target_window_metrics.png` | 门控相机单模态指标卡 |
| `eval_outputs/depth_camera_target_window_metrics.png` | 深度相机单模态指标卡 |
| `eval_outputs/overall_target_window_metrics.png` | 总体融合指标卡 |
| `eval_outputs/sonar_ablation_overview.png` | 雷达、声呐、融合结果在目标栅格的消融对比 |
| `eval_outputs/sonar_sector_health.png` | 四扇区声呐回波健康状态 |
| `eval_outputs/sonar_optimization_before_after.png` | 声呐三帧融合前后支持点数量对比 |
| `eval_outputs/live_multimodal_diagnostic.png` | 普通相机、门控、无人机门控、深度相机、热力图诊断拼图 |

`best.onnx` 和 `best1.onnx` 已经复制到 `src/usv_bringup/models/`。运行仿真、融合跟踪和 RViz 可视化不需要外部训练数据集；只有重新训练或重新做数据集级准确率评估时，才需要另行准备 YOLO 格式数据。
