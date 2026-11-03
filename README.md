# FPGA旋转倒立摆实时控制系统

基于J280开发板与旋转倒立摆套件的实时控制工程。FPGA完成传感器采集、状态估计、自动起摆、直立保持、受扰恢复、定位与正弦轨迹控制；上位机负责参数配置、运行曲线、数据记录及回放。

本仓库对应最终参赛的R43实现，包含完整RTL、约束、Gowin工程、上位机源码、控制计算与模型源码、仿真接口。文件按原有模块关系组织，可解压或克隆到自选目录。

## 目录

|目录/文件|用途|
|---|---|
|`fpga/`|可直接综合的完整Verilog源码、器件工程、引脚/时序约束和实现选项|
|`host/`|串口协议、坐标换算、Qt界面、日志记录和回放|
|`capture_ui/`、`target_ui_fix/`|正式界面的捕获、定位、调速与正弦操作入口|
|`calibration/`|编码器坐标标定配置|
|`hybrid/`|起摆计算、定点微码、模型参数和计算单元生成源|
|`observer/`、`reference/`、`swing/`、`tracking/`|观测反馈、轨迹参考、起摆及定点控制参考实现|
|`models/`|非线性模型、局部实测模型、参数和物理坐标适配|
|`sim/`|完整控制器RTL仿真接口及存储器仿真模型|
|`tools/`|Gowin编译启动、RTL仿真库构建及代表性验证脚本|
|`start_R43_windows.cmd`|Windows上位机入口|
|`compile_R43_windows.cmd`|Windows完整综合、布局布线和位流生成入口|

## 硬件与环境

- 开发板：J280，FPGA器件`GW2A-LV55PG484C8/I7`。
- 套件：旋转倒立摆、电机及正交编码器、WDD35D4角位移传感器。
- 接口：XH1连接电机与编码器，XH3连接角度传感器；USB串口连接电脑，套件使用12V供电。
- FPGA工具：Gowin EDA 1.9.12.03。设计主时钟50MHz，控制周期1ms，PWM频率20kHz。
- 上位机：Python3.12推荐，依赖为PySide6、pyqtgraph、NumPy、pyserial，版本见`host/requirements_gui.txt`。

## 启动上位机

**Windows：**安装Python3.12后，双击根目录`start_R43_windows.cmd`。首次启动自动创建项目内的Python虚拟环境并安装界面依赖，需要联网。此后从同一入口启动即可。

**macOS：**在终端执行`zsh host/start_gui_mac.command`。Linux可创建虚拟环境，安装`host/requirements_gui.txt`后运行`python target_ui_fix/launch_target.py`。

不连接设备时，可用“打开日志/实验包”载入CSV查看已有记录，或用“示例回放”了解界面。示例数据仅用于软件演示。

连接设备后，选择USB串口并连接；板端标识应为`4A070257 / XH1 / −1`。使用实际机械行程配置，保持输出停止，将摆臂置于本次选定的零位，扶杆竖直向上、静止后点击校准。校准同时建立摆杆竖直零点与摆臂位置零点。随后选择定位速度，按界面“准备→使能并起摆”操作；进入平衡后可定位或设置正弦半幅、周期。停止输出使用界面STOP或Esc，板上SW4也可停止。

运行日志保存在工程内的`logs/`等输出目录，界面底部会显示实际路径。“结束本组并导出”生成本次记录包。上位机参数用于配置板端，1ms实时控制在FPGA内部执行。

## 编译与下载

1. 安装Gowin EDA 1.9.12.03与Python3。
2. 双击`compile_R43_windows.cmd`。脚本自动寻找Gowin；也可通过环境变量`GOWIN_GW_SH`指定`gw_sh.exe`完整路径，或运行`python tools/compile_r40_once.py --gowin "实际路径/gw_sh.exe"`。
3. 编译输出默认位于工程同级的`R43_vendor_runs/`，结束时终端显示位流完整路径。各次工程位流在`work/fpga/impl/pnr/j280_draft.fs`，完整报告与日志也保留在该次目录。
4. 使用Gowin Programmer选择对应器件并下载生成的`.fs`文件，再启动上位机。

手动查看工程可打开`fpga/j280_draft.gprj`；综合顶层为`j280_top`，实现参数在`fpga/configure_project.tcl`。推荐使用根目录编译入口，使综合与布局布线选项一致。直接编译已提供的`fpga/src/`，无需先运行控制生成脚本。

## 可选离线仿真

在Linux、macOS或Windows的WSL环境安装Verilator、C++17编译器及make，另建Python环境安装`requirements_offline.txt`。以下命令从工程根目录执行：

```sh
python tools/verify_r01.py --build-only
python tools/test_r40_fixed_commit.py --out build/commit_check
python tools/r43/integration/run_sine.py --project . --out build/sine_demo --speed 60 --amplitude 60 --period 6.28
```

第一条构建完整控制器RTL动态库；第二条检查实际控制输出的帧边界提交与停止行为；第三条运行控制器RTL与局部模型的连续正弦闭环仿真，并输出轨迹和指标。`--out`应使用尚不存在的目录。仿真生成物统一保存在`build/`，可重新生成。

`fpga/src/`为最终硬件实现；各控制目录中的Python/C++文件用于理解定点计算、模型与设计过程。生成器输出属于开发过程的中间形式，重新生成后需同步后续实现优化，不是编译现有工程的前置步骤。

## 实现结果

既有完整Gowin实现：Logic42740/54720，寄存器17547/42000，BSRAM125/140，DSP14/20；50MHz下Setup裕量0.785ns、Hold裕量0.076ns，均无违例。具体实现会随工具与布局布线结果变化，重新编译以该次报告为准。

仓库保留上位机运行所需的`source_manifest.json`、`host/qualification.json`和`host/sta_review.json`。它们随本次精简工程配套；直接保留即可，无需登记电脑、盘符或位流存放位置。
