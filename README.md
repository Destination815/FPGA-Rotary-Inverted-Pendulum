# FPGA旋转倒立摆实时控制系统

基于J280开发板与旋转倒立摆套件的实时控制工程。FPGA完成传感器采集、状态估计、自动起摆、直立保持、受扰恢复、定位与正弦轨迹控制；上位机负责参数配置、曲线显示、数据记录及回放。

本仓库采用最终参赛的R43方案，包含正式FPGA工程、完整上位机、控制模型与仿真源码，以及现场使用的位流、实测样例和硬件资料。**直接使用已提供的位流即可运行，正常演示无需重新编译。**工程可解压或克隆到自选目录，入口从自身位置寻找文件。

## 先选入口

|想做什么|使用入口或文件|
|---|---|
|给开发板下载正式程序|`bitstream/j280_draft.fs`，使用Gowin Programmer|
|连接套件、校准并运行|双击`start_R43_windows.cmd`|
|不接实物，直接查看演示记录|双击`replay_demo_windows.cmd`|
|提前准备Windows离线安装依赖|联网时双击`prepare_offline_windows.cmd`|
|查看Python、依赖、工具和位流位置|双击`environment_info_windows.cmd`；此入口不连接硬件|
|将一次CSV/实验包整理为角度制曲线数据|把CSV或实验ZIP拖到`export_run_windows.cmd`；直接双击则处理演示样例|
|下载板端保存的1kHz原始采样|停止并关闭GUI后双击`download_passive_windows.cmd`，保持板端供电|
|重新综合、布局布线并生成位流|双击`compile_R43_windows.cmd`|

## 现场快速上手

### 出发前准备一次

1. 在实际参赛电脑安装64位Python3.12、Gowin EDA 1.9.12.03及其Programmer，并安装板卡所需的下载与USB串口驱动。驱动和工具安装方法见`hardware/`中的上手手册。Gowin EDA授权与驱动需提前配置好。
2. 把完整工程放到自选目录，联网双击`start_R43_windows.cmd`。首次启动在项目内建立`host/.venv_gui/`并安装锁定版本的依赖；已装好后，再次启动无需重新下载。
3. 双击`replay_demo_windows.cmd`，确认真实演示曲线能够显示。这个入口不连接设备，也不发送控制命令。
4. 如需备用电脑在无网络时首次安装，提前双击`prepare_offline_windows.cmd`。它下载Windows/Python3.12对应的依赖到`offline/wheels/windows-py312/`；把该目录随整个工程一起带走。备用电脑安装相同版本Python后，正常启动入口会自动使用此缓存。

依赖缓存通常需要数百MB，因此由准备入口按需下载；提交包不包含Python、Gowin安装器或已安装虚拟环境。缓存用于Windows x64，不能拿macOS的虚拟环境目录代替。使用其他受支持Python版本时，可执行`python tools/offline_dependencies.py --python-version 3.11`准备对应缓存。

### 到场下载与运行

1. 固定套件，连接12V电源、传感器与电机线路。**XH1为电机与编码器接口，XH3为摆杆角度传感器接口。**具体引脚与接法查`hardware/J280姿态控制系统开发套件用户手册V1.pdf`及底板原理图。
2. 使用板卡下载接口连接电脑，在Gowin Programmer中选择对应器件并加载`bitstream/j280_draft.fs`，按上手手册完成下载。该文件来自已通过时序的正式实现，可以复制到任意位置使用。
3. 连接USB串口，启动`start_R43_windows.cmd`，选择实际COM口并连接。正常板端标识为`4A070257 / XH1 / −1`。
4. 保持输出停止。沿用队伍确定的机械零位：**摆臂放到旧零位正对面约180°，扶杆竖直向上、静止后点击校准，建立本次新0°。**这一步同时校准摆杆竖直零点和摆臂位置零点；界面显示的机械行程采用装置实际已确认的值。
5. 选择本组速度，点击“应用本组速度并准备”或界面对应的准备入口；让摆杆自然下垂、摆臂回到新零位附近，再“使能并起摆”。后续定位和正弦运动均从原GUI操作。
6. 停止输出用STOP或Esc，板上SW4也可停止。“结束本组并导出”会停止并保存记录，窗口底部显示实际输出目录。

### 常用演示顺序

首次起摆并保持平衡 → 两个方向的轻推干扰 → 跌倒恢复 → 定位+90°、−90°、0° → 正弦运动。使用15、30、45、60°/s四组速度时，按原界面逐组选择；正弦半幅与周期可单独修改。

`examples/demo/run.csv`对应最终演示记录：定位速度60°/s，正弦半幅60°、周期6.28s。`examples/speed15/`至`speed60/`保存另外四组原始实测包；各组正弦设置以包内记录为准。载入历史数据属于回放，不会把记录中的参数自动下发给实物。

## 回放、记录和现场分析

- **回放CSV或实验包：**未连接串口时，点击“打开日志/实验包”，选择CSV或`run_*.zip`。拖动进度条查看起摆、干扰、定位与正弦区间；可用GUI保存曲线图片。`replay_demo_windows.cmd`已预选真实演示CSV。
- **保存本次操作：**通过GUI的“结束本组并导出”保留遥测CSV、参数/命令记录及实验包。界面底部显示日志位置，常用输出位于工程的`logs/`与`r20_ui/logs/`。
- **导出数据用于画图：**把CSV或`run_*.zip`拖到`export_run_windows.cmd`。默认输出到`logs/analysis/记录名/`，包括`curves_degrees.csv`和`summary.json`。角度及角速度分别转为°和°/s，输出为百分比；原始记录不改写。
- **下载板端原始记录：**先停止、关闭GUI，保持板卡供电，再运行`download_passive_windows.cmd`。按提示选择串口，生成`logs/passive/`下的采样包。GUI与下载脚本轮流使用同一串口。

命令行也可使用：

```sh
python tools/export_run.py examples/demo/run.csv --out logs/demo_analysis
python tools/analyze_passive.py examples/speed60/passive_20261031_001116_388087.zip --build 4A070257 --out logs/passive_summary.json
```

原始CSV中的角度按弧度保存，图表导出工具负责转换为角度；分析时沿用`host/coordinates.py`的坐标定义。1kHz板端记录与50HzGUI遥测分别保留，不能通过改时间轴把后者当作1kHz数据。

## 重新编译

已有位流可直接用于现场运行。需要重新生成时：

1. 安装Gowin EDA 1.9.12.03与Python3，双击`compile_R43_windows.cmd`。
2. 脚本自动查找Gowin；自定义安装位置可设置环境变量`GOWIN_GW_SH`为`gw_sh.exe`完整路径，或执行`python tools/compile_r40_once.py --gowin "实际路径/gw_sh.exe"`。
3. 输出默认位于工程同级、同盘的`R43_vendor_runs/`。结束时窗口显示位流完整路径：本次目录下的`work/fpga/impl/pnr/j280_draft.fs`；同时保留综合、布局布线和时序报告。
4. 按本次报告确认完整实现与时序结果，再使用本次生成的位流。打开`fpga/j280_draft.gprj`可查看工程，综合顶层为`j280_top`。

正式源码已在`fpga/src/`内，直接编译即可。根目录编译入口统一设置综合与布局布线选项；`fpga/configure_project.tcl`保留工程配置。无需先运行模型拟合、微码生成或历史变更脚本。

## 工程目录与源码分工

|目录|内容与使用方式|
|---|---|
|`bitstream/`|正式R43的`j280_draft.fs`，用于直接下载|
|`fpga/`|完整Verilog、引脚与时序约束、Gowin工程及配置|
|`host/`|完整串口协议、Qt GUI、日志/回放、坐标换算及启动依赖|
|`capture_ui/`、`target_ui_fix/`|正式GUI的捕获、定位、调速与正弦操作界面|
|`calibration/`|编码器坐标标定配置|
|`hybrid/`|多模型控制、定点计算、微码与计算单元生成源码|
|`observer/`、`reference/`、`swing/`、`tracking/`|状态观测、轨迹参考、起摆及反馈控制参考实现|
|`models/`|非线性模型、实测局部模型、标定参数及模型验证代码|
|`sim/`|RTL仿真接口、存储器仿真模型与测试平台|
|`tools/r43/`|本方案的定位、正弦、恢复及控制计算验证工具|
|`tools/`其余文件|编译启动、数据采集/分析、完整RTL构建与被上述工具复用的依赖|
|`evidence/r43/integration/`|已完成的代表性RTL仿真轨迹与指标，可直接查看|
|`evidence/r32_swing_selected/`|完整RTL仿真使用的动力学模型与场景输入|
|`examples/`|演示CSV、四档速度的原始运行包与1kHz采样包|
|`hardware/`|上手手册、用户手册、核心板/底板原理图、器件数据手册及赛题指南|
|`implementation/accepted_R43/`|已通过的综合网表、实现中间文件、资源与时序报告|

部分文件名保留原开发阶段编号，是正式工程的依赖或模型来源；运行入口统一为R43。模型、C++参考与生成器便于理解、仿真和后续开发；**最终下载所用实现以`fpga/src/`与本包位流为准**。生成器输出属于设计中间结果，直接覆盖最终RTL会丢失后续实现优化。

`hardware/coreboard_20230817.pdf`为核心板原理图，`hardware/Balance_Ctrl_20260618.pdf`为底板原理图。`implementation/`保留的网表、数据库及`.bin/.binx`用于查看既有实现，直接下载请选择`bitstream/`中的`.fs`。

## 可选模型与RTL仿真

硬件运行不需要安装仿真环境。在Linux、macOS或Windows的WSL中安装Verilator、C++17编译器及make，另建Python环境安装`requirements_offline.txt`；不要把模型环境的NumPy版本覆盖进GUI环境。

从工程根目录执行，`--out`使用新目录：

```sh
# 构建完整控制器RTL动态库
python tools/verify_r01.py --build-only
# 运行定位、正弦或跌倒恢复代表场景
python tools/r43/integration/run_position.py --project . --out build/position_demo --speed 45
python tools/r43/integration/run_sine.py --project . --out build/sine_demo --speed 60 --amplitude 60 --period 6.28
python tools/r43/integration/run_recovery.py --project . --out build/recovery_demo --case-index 0 --ms 18000
# 检查输出提交与停止行为
python tools/test_r40_fixed_commit.py --out build/commit_check
```

仿真输出轨迹、状态变化与数值指标，保存在指定的`build/`子目录。模型参数和输入场景随包保留；已有代表性结果在`evidence/r43/integration/`，查看它们无需重复运行长仿真。实物效果可对照`examples/`中的原始记录。

## 常见操作问题

|现象|处理方法|
|---|---|
|首次启动找不到Python|安装64位Python3.12，勾选PATH或安装Python Launcher；再运行启动入口|
|离线电脑缺少界面依赖|用联网电脑运行离线准备入口，带上匹配版本的`offline/wheels/windows-py312/`；重新启动即可本地安装|
|依赖缓存目录存在但安装失败|检查缓存是否复制完整、Python版本是否一致；需要重新联网安装时先移走该缓存目录|
|串口列表没有设备|检查USB串口线、驱动与实际使用的USB接口；刷新列表。下载接口和串口接口的用途不同|
|串口被占用|关闭另一个GUI、串口助手或采样下载窗口，再连接|
|连接后显示不匹配的板号|下载本包`bitstream/j280_draft.fs`，重新连接确认标识|
|按钮灰色|先看界面给出的原因；依次确认连接、板端身份、校准、速度组准备与使能状态。回放模式下运动按钮灰色是正常显示|
|提示资格文件缺失或无效|使用完整解压目录，保留`source_manifest.json`、`host/qualification.json`、`host/sta_review.json`及配套源码，不要仅复制GUI子目录|
|停止后想保存1kHz记录|保持板卡供电、关闭GUI后运行原始采样下载入口|
|运行后找不到数据|查看GUI底部路径及工程内`logs/`、`r20_ui/logs/`；编译产物在工程同级的`R43_vendor_runs/`|

macOS可执行`zsh host/start_gui_mac.command`；Linux可安装`host/requirements_gui.txt`后运行`python target_ui_fix/launch_target.py`。Windows是队伍原实测平台。

## 实现参数与报告入口

器件`GW2A-LV55PG484C8/I7`，主时钟50MHz，控制周期1ms，PWM频率20kHz。已通过实现的资源占用为Logic42740/54720、寄存器17547/42000、BSRAM125/140、DSP14/20；50MHz下Setup裕量0.785ns、Hold裕量0.076ns，均无违例。

- 综合资源：`implementation/accepted_R43/gwsynthesis/j280_draft_syn_resource.html`
- 布局布线报告：`implementation/accepted_R43/pnr/j280_draft.rpt.html`
- 时序报告：`implementation/accepted_R43/pnr/j280_draft.tr.html`，同目录两个配套HTML文件一并保留

上位机所需的三份运行元数据已随本工程配套，无需登记电脑、盘符或位流存放位置。本次整理扩充现场资料与辅助入口，正式控制算法、FPGA源码、串口控制行为和实测记录保持原样。
