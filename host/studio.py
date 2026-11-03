#!/usr/bin/env python3
"""R40 Studio, 4A070257 only; qualification gate never arms on connect."""
import argparse
import bisect
import hashlib
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

from PySide6 import QtCore, QtGui, QtWidgets as W
import numpy as np
import pyqtgraph as pg
from serial.tools import list_ports

from studio_data import (APP_VERSION, DEFAULT_PROFILE, STATE_CN, FAULT_CN, archive_run,
    elapsed_ms, load_run, resource_dir, storage_dir, summarize, summary_text, validate_profile, output_issue)
from studio_session import SerialSession, permitted
from protocol import D03_BUILD, CONTROL_BUILD
from motion_config import pack as pack_motion_config,unpack as unpack_motion_config,PERIODS,target_bound_deg
from coordinates import SCALE,TOKEN,internal_row
from r01_gate import qualification,check,board_ok
from d02_data import load_captures, PHASES, REASONS, describe
from d03_plan import ExperimentPlan

STYLE='''
QMainWindow,QWidget{background:#0c121b;color:#e6edf5;font-size:12px;}
QLabel{background:transparent;} QLabel#title{font-size:25px;font-weight:700;letter-spacing:1px;}
QLabel#muted{color:#8c9bae;} QLabel#mode{color:#d9f391;font-weight:600;}
QFrame#card{background:#141e2b;border:1px solid #28364a;border-radius:10px;}
QLabel#number{font-size:28px;font-weight:600;color:#d9f391;}
QGroupBox{border:1px solid #29374a;border-radius:7px;margin-top:12px;padding:12px 8px 8px;}
QGroupBox::title{subcontrol-origin:margin;left:10px;color:#b2c2d4;}
QPushButton{background:#1b293a;border:1px solid #35455b;border-radius:5px;padding:7px 10px;}
QPushButton:hover{background:#2a3d53;} QPushButton:pressed{background:#3c5069;}
QPushButton:disabled{color:#5d6a7a;background:#121b27;border-color:#263140;}
QPushButton#primary{background:#d9f391;color:#14201b;font-weight:600;}
QPushButton#primary:disabled{background:#293630;color:#677569;}
QPushButton#stop{background:#823836;border:1px solid #d56a60;color:white;font-weight:700;font-size:15px;}
QPushButton#stop:disabled{background:#332327;color:#87696b;border-color:#554148;}
QComboBox,QLineEdit,QDoubleSpinBox,QSpinBox,QPlainTextEdit{background:#111c2a;border:1px solid #344459;border-radius:4px;padding:5px;selection-background-color:#385568;}
QComboBox QAbstractItemView{background:#182538;color:#e6edf5;selection-background-color:#345266;}
QTabWidget::pane{border:1px solid #29374a;border-radius:6px;}
QTabBar::tab{background:#141f2e;color:#8f9fb2;padding:9px 13px;}
QTabBar::tab:selected{color:#d9f391;border-bottom:2px solid #d9f391;}
QSlider::groove:horizontal{height:5px;background:#2b394c;border-radius:2px;}
QSlider::handle:horizontal{background:#d9f391;width:12px;margin:-5px 0;border-radius:5px;}
QScrollArea{border:0;} QScrollBar:vertical{width:10px;background:#121b27;}
QScrollBar::handle:vertical{background:#394a60;border-radius:4px;min-height:25px;}
QToolTip{color:#eef4fb;background:#253448;border:1px solid #52647b;}
'''

def label(text,muted=False):
    q=W.QLabel(text);q.setWordWrap(True)
    if muted:q.setObjectName('muted')
    return q

def button(text,callback,primary=False):
    q=W.QPushButton(text);q.clicked.connect(callback)
    if primary:q.setObjectName('primary')
    return q

def group(title):
    q=W.QGroupBox(title);layout=W.QVBoxLayout(q);layout.setSpacing(7)
    return q,layout

class Studio(W.QMainWindow):
    def __init__(self):
        super().__init__();self.setWindowTitle(f'J280 Studio {APP_VERSION} · FPGA 姿态控制');self.resize(1360,900)
        self.ext2_selected=False
        self.qualification_note=qualification()[1]
        self.session=None;self.replay=None;self.baseline=None;self.playing=False;self.replay_index=0
        self.times=[];self.plot_rows=[];self.tick_count=0;self.freeze=False;self.closing=False
        self.finalized=[];self.export_requested=False;self.firmware_info={};self.last_fresh=False
        self.profile_dirty=True;self.last_snapshot={};self.last_render_ms=None;self.last_result=0;self.focused=False
        self.setStyleSheet(STYLE)
        pg.setConfigOptions(antialias=True,background='#101925',foreground='#9fafc2')
        root=W.QWidget();self.setCentralWidget(root);outer=W.QVBoxLayout(root);outer.setContentsMargins(18,12,18,8);outer.setSpacing(8)
        head=W.QHBoxLayout();title=label('J280  /  CONTROL STUDIO');title.setWordWrap(False);title.setObjectName('title');head.addWidget(title)
        head.addStretch();self.mode=label('未连接 · 可离线回放');self.mode.setWordWrap(False);self.mode.setObjectName('mode');head.addWidget(self.mode)
        self.stop_button=button('■  停止输出  /  Esc',self.stop);self.stop_button.setObjectName('stop');head.addWidget(self.stop_button);outer.addLayout(head)
        connection=W.QHBoxLayout();self.ports=W.QComboBox();self.ports.setMinimumWidth(230);self.ports.setMaximumWidth(380)
        connection.addWidget(self.ports);self.refresh_button=button('刷新',self.refresh_ports);connection.addWidget(self.refresh_button)
        self.connect_button=button('连接设备',self.toggle_connection,True);connection.addWidget(self.connect_button)
        self.open_button=button('打开日志 / 实验包',self.choose_replay);connection.addWidget(self.open_button)
        self.example_button=button('示例回放',self.open_example);connection.addWidget(self.example_button)
        self.display_mode=W.QCheckBox('演示布局');self.display_mode.toggled.connect(self.demo_layout);connection.addWidget(self.display_mode);connection.addStretch()
        outer.addLayout(connection)
        split=W.QSplitter();outer.addWidget(split,1)
        self.sidebar=W.QTabWidget();self.sidebar.setMinimumWidth(305);self.sidebar.setMaximumWidth(370);split.addWidget(self.sidebar)
        self.build_operations();self.build_parameters();self.build_experiment();self.build_d02()
        main=W.QWidget();right=W.QVBoxLayout(main);right.setContentsMargins(10,0,0,0);right.setSpacing(8);split.addWidget(main);split.setStretchFactor(1,1)
        cards=W.QHBoxLayout();self.cards={};self.card_titles={}
        for key,name in [('theta','摆杆倾角（板端测量）'),('phi','摆臂位置'),('u','输出指令'),('state','控制状态')]:
            card=W.QFrame();card.setObjectName('card');box=W.QVBoxLayout(card);box.setContentsMargins(12,7,12,7);box.setSpacing(2)
            self.card_titles[key]=label(name,True);box.addWidget(self.card_titles[key]);value=label('—');value.setObjectName('number');box.addWidget(value)
            sub=label('等待数据',True);box.addWidget(sub);cards.addWidget(card);self.cards[key]=(value,sub)
        right.addLayout(cards)
        self.health=label('先选择串口连接，或打开一份 CSV / ZIP。连接不会自动使能。',True);right.addWidget(self.health)
        self.filter_note=label('角度/速度直接来自板端；上位机不平滑、不把小速度强制清零。',True);right.addWidget(self.filter_note)
        self.chart_tabs=W.QTabWidget();right.addWidget(self.chart_tabs,1)
        self.plots={};self.curves={};self.comparison_curves={};self.event_lines=[]
        for tab_name,specs in [('控制曲线',[
            ('theta','摆杆倾角','°',[('theta','摆杆角度','#d9f391')]),
            ('phi','摆臂位置','°',[('phi','实际','#70d8cf'),('reference','参考','#e7b979')]),
            ('u','输出与限幅','%',[('u','逻辑输出','#ee9b83'),('limit','上限','#64748b'),('negative_limit','下限','#64748b')])]),
            ('传感与速度',[
            ('rates','角速度估计','°/s',[('dtheta','摆杆','#d9f391'),('dphi','摆臂','#70d8cf')]),
            ('adc','ADC 原始码','code',[('adc','ADC','#e7b979'),('zero','校准零点','#64748b')]),
            ('encoder','编码器累计值','count',[('encoder','编码器','#70d8cf')])])]:
            pane=W.QWidget();layout=W.QVBoxLayout(pane);layout.setContentsMargins(4,4,4,4);layout.setSpacing(3)
            for key,title,unit,series in specs:
                plot=pg.PlotWidget();plot.setMinimumHeight(80)
                short={'theta':'θ','phi':'φ','u':'输出','rates':'角速度','adc':'ADC','encoder':'计数'}[key]
                plot.setLabel('left',short,units=unit);plot.getAxis('left').setWidth(58);plot.showGrid(x=True,y=True,alpha=.13)
                plot.addLegend(offset=(8,0),labelTextSize='9pt',colCount=3);plot.setLabel('bottom','相对捕获时间 / s')
                if key not in ('u','encoder'):plot.hideAxis('bottom')
                for field,legend,color in series:self.curves[field]=plot.plot(name=legend,pen=pg.mkPen(color,width=1.8 if field not in ('limit','negative_limit','zero') else 1))
                if key in ('theta','phi','u'):
                    self.comparison_curves[key]=plot.plot(name='对照（虚线）',pen=pg.mkPen('#8995ba',width=1.4,style=QtCore.Qt.PenStyle.DashLine))
                    plot.getPlotItem().legend.removeItem(self.comparison_curves[key])
                if key=='theta':
                    for value in (-5,5):plot.addItem(pg.InfiniteLine(value,angle=0,pen=pg.mkPen('#354d39',style=QtCore.Qt.PenStyle.DotLine)))
                self.plots[key]=plot;layout.addWidget(plot)
            self.chart_tabs.addTab(pane,tab_name)
        controls=W.QHBoxLayout();self.pause_button=button('暂停绘图',self.toggle_pause);controls.addWidget(self.pause_button)
        self.seek=W.QSlider(QtCore.Qt.Orientation.Horizontal);self.seek.setRange(0,0);self.seek.valueChanged.connect(self.seek_changed);controls.addWidget(self.seek,1)
        self.speed=W.QComboBox();self.speed.addItems(['0.5×','1×','2×','5×','10×']);self.speed.setCurrentIndex(1);controls.addWidget(self.speed)
        self.focus_button=button('定位捕获',self.focus_capture);controls.addWidget(self.focus_button)
        self.compare_button=button('添加对照',self.choose_baseline);controls.addWidget(self.compare_button)
        self.clear_compare_button=button('移除对照',self.clear_baseline);controls.addWidget(self.clear_compare_button)
        right.addLayout(controls)
        self.bottom_tabs=W.QTabWidget();self.bottom_tabs.setMaximumHeight(150);right.addWidget(self.bottom_tabs)
        self.results=W.QPlainTextEdit();self.results.setReadOnly(True);self.results.setMaximumBlockCount(500);self.bottom_tabs.addTab(self.results,'实验结果')
        self.event_log=W.QPlainTextEdit();self.event_log.setReadOnly(True);self.event_log.setMaximumBlockCount(500);self.bottom_tabs.addTab(self.event_log,'操作记录')
        foot=W.QHBoxLayout();self.path_label=label('记录目录：'+str(storage_dir()/'logs'),True);self.path_label.setWordWrap(False);self.path_label.setSizePolicy(W.QSizePolicy.Policy.Ignored,W.QSizePolicy.Policy.Preferred);foot.addWidget(self.path_label,1)
        foot.addWidget(button('打开记录目录',self.open_log_dir));self.export_button=button('结束并导出实验包',self.finish_export,True);foot.addWidget(self.export_button)
        self.png_button=button('保存曲线图片',self.save_plot);foot.addWidget(self.png_button);outer.addLayout(foot)
        self.statusBar().showMessage('默认手动STOP结束，可自行定时；通信心跳中断3秒板端撤使能。SW4立即停止。')
        self.escape=QtGui.QShortcut(QtGui.QKeySequence('Escape'),self);self.escape.activated.connect(self.stop)
        self.sidebar.setTabVisible(1,False);self.sidebar.setTabVisible(3,False)
        frozen=W.QWidget();layout=W.QVBoxLayout(frozen)
        layout.addWidget(label('R40 六功能控制配置',True))
        layout.addWidget(label('R43 起摆／保持／定位／正弦连续切换\n手动捕获保留原观测器；自动起摆使用能量与状态反馈。\n正常上限15%，外偏恢复最高35%，恢复后自动回落。\n自动起摆会话允许失衡后重新起摆；手动捕获仍保留原角度保护。\n自动恢复保留回位/定位目标；往返受扰后先回中心，再恢复往返。\n本版参数编入 FPGA，禁止旧 gain/profile 导入。'))
        layout.addWidget(label('位置单位为校准后的编码器角；先捕获再运动。\n参考按位移和所选速度平滑过渡，最短2秒。\n电脑负责指令、曲线和日志；闭环在 FPGA。'))
        layout.addWidget(label('运行资格及板端身份以连接区实时核对结果为准。',True));layout.addStretch()
        self.add_side_tab('冻结配置',frozen)
        for b in (self.jog_positive,self.jog_negative,self.jog_value):b.setVisible(False)
        self.jog_positive.parentWidget().setVisible(False)
        self.target.setRange(-350*SCALE,350*SCALE);self.trajectory_on.setText('普通往返（使用准备区幅度／周期）')
        self.trajectory_off.setText('结束往返 → 保持当前位置')
        self.run_seconds.setValue(0);self.run_seconds.setVisible(True);self.start_button.setToolTip('先向上校准；下垂静止后使能并启动。起摆超时或越界会停止。');self.capture_seconds.setValue(0)
        self.refresh_ports();self.update_controls({})
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(40)

    def add_side_tab(self,title,widget):
        scroll=W.QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(widget);self.sidebar.addTab(scroll,title)

    def build_operations(self):
        pane=W.QWidget();box=W.QVBoxLayout(pane);box.setContentsMargins(10,10,10,10)
        box.addWidget(label('01  准备与校准',True))
        self.profile_status=label('参数未知：请在「参数」页应用配置。');box.addWidget(self.profile_status)
        self.cal_button=button('扶杆向上静止 → 校准',lambda:self.command('cal'));box.addWidget(self.cal_button)
        self.cal_hint=label('零点未确认',True);box.addWidget(self.cal_hint)
        self.clear_button=button('清除故障（未使能时）',lambda:self.command('clear'));box.addWidget(self.clear_button)
        self.ready_check=W.QCheckBox('本次现场已确认');self.ready_check.setVisible(False);box.addWidget(self.ready_check)
        self.arm_button=button('使能会话（不启动运动）',lambda:self.command('arm'));box.addWidget(self.arm_button)
        g,l=group('02  局部捕获');box.addWidget(g)
        self.capture_seconds=W.QDoubleSpinBox();self.capture_seconds.setRange(0,86400);self.capture_seconds.setSpecialValueText('手动STOP结束（无定时停止）');self.capture_seconds.setValue(0);self.capture_seconds.setSuffix(' 秒后软件停止')
        l.addWidget(self.capture_seconds);self.capture_button=button('捕获并开始计时',lambda:self.command('capture',seconds=self.capture_seconds.value()),True);l.addWidget(self.capture_button)
        self.capture_hint=label('校准后保持接近直立，满足门限后捕获并松手。',True);l.addWidget(self.capture_hint)
        g,l=group('03  自动起摆、定位与往返');box.addWidget(g)
        self.run_seconds=W.QSpinBox();self.run_seconds.setRange(0,86400);self.run_seconds.setSpecialValueText('手动STOP结束（无定时停止）');self.run_seconds.setValue(0);self.run_seconds.setSuffix(' 秒后软件停止');l.addWidget(self.run_seconds)
        self.start_button=button('从下垂启动起摆',lambda:self.command('start',seconds=self.run_seconds.value()));l.addWidget(self.start_button)
        cfg=W.QFormLayout();self.motion_workspace=W.QDoubleSpinBox();self.motion_workspace.setDecimals(3);self.motion_workspace.setRange(20*SCALE,360*SCALE);self.motion_workspace.setSingleStep(SCALE);self.motion_workspace.setValue(360*SCALE);self.motion_workspace.setSuffix(' °')
        self.motion_amplitude=W.QDoubleSpinBox();self.motion_amplitude.setDecimals(3);self.motion_amplitude.setRange(.1*SCALE,180*SCALE);self.motion_amplitude.setSingleStep(.1*SCALE);self.motion_amplitude.setValue(15*SCALE);self.motion_amplitude.setSuffix(' °')
        from motion_config import DEFAULT_SPEED_DEG_S
        self.motion_speed=W.QDoubleSpinBox();self.motion_speed.setDecimals(3);self.motion_speed.setRange(1,60);self.motion_speed.setValue(DEFAULT_SPEED_DEG_S);self.motion_speed.setSuffix(' °/s')
        self.motion_period=W.QComboBox();self.motion_period.addItems([str(t)+' 秒' for t in PERIODS]);self.motion_period.setCurrentIndex(PERIODS.index(20))
        cfg.addRow('已核实机械半行程',self.motion_workspace);cfg.addRow('往返幅度',self.motion_amplitude);cfg.addRow('定位最高速度',self.motion_speed);cfg.addRow('往返周期',self.motion_period);l.addLayout(cfg)
        self.motion_range_check=W.QCheckBox('本次现场已确认');self.motion_range_check.setVisible(False);l.addWidget(self.motion_range_check)
        self.full_turn_recovery=W.QCheckBox(f'使用本队已核实机构配置：恢复范围约±{360*SCALE:.2f}°');self.full_turn_recovery.setChecked(True);self.full_turn_recovery.setVisible(False);l.addWidget(self.full_turn_recovery)
        l.addWidget(label(f'沿用本队同一机构：物理硬行程约±{360*SCALE:.2f}°。若线缆或机构布局改变，先停止并减小上方行程。',True))
        self.full_turn_recovery.toggled.connect(self.publish_preparation)
        self.motion_apply=button('确认现场畅通并准备本次运行',self.apply_motion_config);l.addWidget(self.motion_apply)
        self.motion_readback=W.QLabel('等待板端运动配置');self.motion_readback.setFixedHeight(120);l.addWidget(self.motion_readback)
        row=W.QHBoxLayout();self.target=W.QDoubleSpinBox();self.target.setRange(-350*SCALE,350*SCALE);self.target.setDecimals(3);self.target.setSuffix(' °');row.addWidget(self.target)
        self.target_button=button('设置目标',lambda:self.command('target',round(math.radians(self.target.value())*65536)));row.addWidget(self.target_button);l.addLayout(row)
        row=W.QHBoxLayout();self.trajectory_on=button('按板端配置往返',lambda:self.command('trajectory',1));self.trajectory_off=button('关闭往返',lambda:self.command('trajectory',0));row.addWidget(self.trajectory_on);row.addWidget(self.trajectory_off);l.addLayout(row)
        l.addWidget(label('先准备、使能并起摆，再在可达行程内输入任意目标角（角度为物理值，可正可负）。定位速度1～60°/s可直接输入，参考仍保留加速度限制；实际到位看测量值。默认手动STOP结束，也可自行设定停止时间。往返前先定位0°并等待参考静止；结束往返继续平衡。',True))
        g,l=group('04  正弦运动');box.addWidget(g)
        self.ext2_prepare=button('进入拓展2',self.prepare_ext2);self.ext2_prepare.setVisible(False);l.addWidget(self.ext2_prepare)
        self.ext2_hold=button('退出拓展2，继续保持',lambda:self.send_request('ext2_exit'));self.ext2_hold.setVisible(True);l.addWidget(self.ext2_hold)
        self.ext2_begin=button('进入拓展2正弦（无需重新捕获）',lambda:self.send_request('ext2_begin'),True);l.addWidget(self.ext2_begin)
        self.ext2_status=label('半幅和周期可编辑，以本轮定位目标为中心，未设置目标时使用首次进入参考。无需等待摆杆静稳。',True);l.addWidget(self.ext2_status)
        l.addWidget(label('统一准备一次即可使用六项功能。拓展2含2秒平滑渐入；退出时平滑缩小幅度并回到本轮中心，然后可直接定位或再次进入。进入过程中提前退出，先完成渐入再渐退；STOP始终立即结束运行。',True))
        g,l=group('短点动诊断 · 约 100 ms');box.addWidget(g)
        self.jog_value=W.QDoubleSpinBox();self.jog_value.setRange(.1,25);self.jog_value.setValue(10);self.jog_value.setSuffix(' %');l.addWidget(self.jog_value)
        row=W.QHBoxLayout();self.jog_negative=button('− 反向',lambda:self.command('jog',-round(self.jog_value.value()*10)));self.jog_positive=button('+ 正向',lambda:self.command('jog',round(self.jog_value.value()*10)))
        row.addWidget(self.jog_negative);row.addWidget(self.jog_positive);l.addLayout(row)
        l.addWidget(label('受实际限幅约束；只单次触发，不长按连发。',True));box.addStretch();self.add_side_tab('操作',pane)

    def build_parameters(self):
        pane=W.QWidget();box=W.QVBoxLayout(pane);box.setContentsMargins(10,10,10,10)
        box.addWidget(label('参数配置',True));self.profile_name=W.QLineEdit(DEFAULT_PROFILE['name']);box.addWidget(self.profile_name)
        box.addWidget(label('u = −K·x；此只读页系数属于保留的旧内部坐标。\n此配置是调试基线，不是已调稳参数。',True))
        form=W.QFormLayout();self.gains={}
        for key,text in [('theta','摆杆角 θ'),('dtheta','摆杆速度 dθ'),('phi','摆臂位置 φ'),('dphi','摆臂速度 dφ')]:
            spin=W.QDoubleSpinBox();spin.setRange(-15258,15258);spin.setDecimals(4);spin.setValue(DEFAULT_PROFILE[key]);spin.valueChanged.connect(self.mark_dirty)
            self.gains[key]=spin;form.addRow(text,spin)
        self.limit=W.QDoubleSpinBox();self.limit.setRange(2,60);self.limit.setDecimals(1);self.limit.setSingleStep(.5);self.limit.setValue(15);self.limit.setSuffix(' %');self.limit.valueChanged.connect(self.mark_dirty);form.addRow('输出上限',self.limit)
        self.swing=W.QSpinBox();self.swing.setRange(0,2000);self.swing.setValue(800);self.swing.valueChanged.connect(self.mark_dirty);form.addRow('起摆增益',self.swing);box.addLayout(form)
        self.apply_button=button('应用完整配置（不使能）',self.apply_profile,True);box.addWidget(self.apply_button)
        box.addWidget(label('依次执行停止、清故障、关轨迹、目标归零和参数设置；任何失败立即中止。请先停止并解除使能。',True))
        row=W.QHBoxLayout();row.addWidget(button('导入配置',self.import_profile));row.addWidget(button('保存配置',self.save_profile));box.addLayout(row)
        self.parameter_note=label('编辑值尚未应用。现有 FPGA 不支持增益读回。',True);box.addWidget(self.parameter_note)
        box.addStretch();self.add_side_tab('参数',pane)

    def build_experiment(self):
        pane=W.QWidget();box=W.QVBoxLayout(pane);box.setContentsMargins(10,10,10,10)
        box.addWidget(label('实验卡',True));self.experiment_name=W.QLineEdit('局部捕获对照');box.addWidget(self.experiment_name)
        self.question=W.QLineEdit();self.question.setPlaceholderText('本轮验证什么？例如：新估计器是否减少换向');box.addWidget(self.question)
        self.change=W.QLineEdit();self.change.setPlaceholderText('唯一主要变化 / 基准编号');box.addWidget(self.change)
        self.notes=W.QPlainTextEdit();self.notes.setPlaceholderText('机械状态、释放方式、现场观察……');self.notes.setMaximumHeight(120);box.addWidget(self.notes)
        self.note_button=button('记录实验卡 / 现场标记',self.add_note);box.addWidget(self.note_button)
        self.release_button=button('标记：已松手 / 已施加扰动',lambda:self.send_request('note','现场标记：已松手/施加扰动'));box.addWidget(self.release_button)
        box.addWidget(label('捕获命令另有自动事件标记。人工标记不是精确测力/释放传感器。',True))
        self.firmware_label=label('无需本地位流文件；板端版本等待新鲜回读',True);box.addWidget(self.firmware_label)
        box.addWidget(button('可选：归档位流文件信息',self.pick_firmware))
        box.addWidget(label('无需本地.fs或位流登记。板端版本、方向与配置由新鲜回读核对。',True))
        box.addStretch();self.add_side_tab('实验',pane)

    def build_d02(self):
        self.d03_plan=ExperimentPlan();self.d03_ready_since=None;self.d03_session=None
        pane=W.QWidget();box=W.QVBoxLayout(pane);box.setContentsMargins(10,10,10,10)
        box.addWidget(label('D03 · 10% / 15% 幅值对照'))
        self.d02_status=label('等待配套固件 4A030101；旧固件只可普通连接。',True);box.addWidget(self.d02_status)
        self.d02_apply=button('应用 D03 配置 · 限幅 15%',self.apply_d02,True);box.addWidget(self.d02_apply)
        box.addWidget(label('扶杆向上校准一次，再放下垂。每轮摆臂回到 0°±1°、撤手并停稳至少3秒；不要固定摆臂。',True))
        self.d02_check=W.QCheckBox('本轮两者自由运动；手已撤开，范围内无障碍');box.addWidget(self.d02_check)
        self.d02_trial=W.QLineEdit(self.d03_plan.label);self.d02_trial.setReadOnly(True);box.addWidget(self.d02_trial)
        self.d02_execute=button('执行本轮：无驱动检查',self.run_d03,True);box.addWidget(self.d02_execute)
        self.d02_retry=button('重新读取上次数据 · 不驱动',lambda:self.send_request('d02_retry'));box.addWidget(self.d02_retry)
        self.d02_progress=W.QProgressBar();self.d02_progress.setRange(0,1024);box.addWidget(self.d02_progress)
        self.d02_last_path='';self.d02_result=label('1024点 / 1kHz；驱动100ms，随后自动停止输出。接收约8–12秒。',True);box.addWidget(self.d02_result)
        box.addWidget(button('查看最近一次高速波形',self.show_latest_d02))
        box.addWidget(button('打开高速文件 / 实验包（兼容 D02）',self.open_d02))
        box.addWidget(label('顺序：无驱动 → +10 → +15 → −10 → −15 → +15 → −15。前四轮中任一15%位移不足8计数，将停止推进。异常时停止并导出，不自行追加。',True))
        box.addStretch();self.add_side_tab('D03 诊断',pane)

    def apply_d02(self):
        self.profile_name.setText('D03 幅值对照 · 15% 上限')
        self.limit.setValue(15);self.apply_profile()

    def run_d03(self):
        value=self.d03_plan.value
        if value is None:return
        if not self.d02_execute.isEnabled():return
        if self.replay or not self.d02_check.isChecked():self.alert('请先确认撤手及活动范围');return
        try:self.d03_plan.begin(value)
        except ValueError as e:self.alert(str(e));return
        self.experiment_name.setText('D03 驱动幅值对照')
        self.question.setText('同起点10%与15%的响应、方向差异与独立验证')
        self.change.setText(f'{self.d03_plan.label} / {value/10:+.0f}% / 100 ms / 1 kHz')
        self.add_note();self.send_request('d02_start',value)
        self.d02_check.setChecked(False);self.d02_execute.setEnabled(False);self.d03_ready_since=None

    def show_latest_d02(self):
        if self.d02_last_path:self.show_d02(self.d02_last_path)
        else:self.alert('尚无已保存的高速记录')

    def open_d02(self):
        path,_=W.QFileDialog.getOpenFileName(self,'打开 D03 / D02 高速记录',str(storage_dir()),'高速记录 (*.json *.zip)')
        if path:self.show_d02(path)

    def show_d02(self,path):
        try:
            records=load_captures(path)
            labels=[f"#{r['info']['capture_id']}  {r['info']['amplitude']/10:+.0f}%  {r['received']} 点" for r in records]
            index=0
            if len(records)>1:
                choice,ok=W.QInputDialog.getItem(self,'选择试次','高速记录',labels,0,False)
                if not ok:return
                index=labels.index(choice)
            r=records[index];rows=r['rows']
            if not rows:raise ValueError('此记录没有样本；请查看中止原因')
            dialog=W.QDialog(self);dialog.setWindowTitle('高速波形 · '+labels[index]);dialog.resize(1000,760)
            layout=W.QVBoxLayout(dialog)
            layout.addWidget(label(describe(r)))
            ts=[((v['ms']-rows[0]['ms'])&0xffffffff)/1000 for v in rows]
            for field,title in [('encoder','摆臂编码器（计数）'),('adc','摆杆 ADC（避免 ±180° 跳变）'),('u','输出（‰，非实测电流）')]:
                plot=pg.PlotWidget();plot.setLabel('left',title);plot.setLabel('bottom','相对记录起点','s');plot.showGrid(x=True,y=True,alpha=.2)
                plot.plot(ts,[v[field] for v in rows],pen=pg.mkPen('#a7d873',width=2))
                if field=='u':plot.plot(ts,[v['applied']*r['info']['motor_sign'] for v in rows],pen=pg.mkPen('#edb96a',width=1,style=QtCore.Qt.PenStyle.DashLine))
                layout.addWidget(plot)
            layout.addWidget(label('绿色：逻辑 command；橙色虚线：applied 换回逻辑方向。驱动记录与实时状态独立。',True))
            dialog.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose);dialog.show();self.d02_dialog=dialog
        except Exception as e:self.alert(f'D02 打开失败：{e}')

    def alert(self,text):
        self.statusBar().showMessage(text,15000);self.event_log.appendPlainText(text)

    def refresh_ports(self):
        selected=self.ports.currentData();self.ports.clear()
        for p in sorted(list_ports.comports(),key=lambda p:p.device):self.ports.addItem(f'{p.device} · {p.description}',p.device)
        if self.ports.count()==0:self.ports.addItem('未检测到串口；可先打开示例回放',None)
        idx=self.ports.findData(selected)
        if idx>=0:self.ports.setCurrentIndex(idx)

    def toggle_connection(self):
        if self.session and self.session.is_alive():self.session.request('close');self.alert('正在停止并关闭连接…');return
        port=self.ports.currentData()
        self.plots['adc'].setVisible(True);self.plots['encoder'].setVisible(True);self.chart_tabs.setTabText(1,'传感与速度');self.health.setStyleSheet('')
        if not port:self.alert('未检测到串口，请连接板卡、检查驱动后刷新。');return
        self.replay=None;self.playing=False;self.baseline=None;self.clear_baseline();self.plot_rows=[];self.ready_check.setChecked(False)
        self.session=SerialSession(port,storage_dir()/'logs',self.metadata());self.session.start();self.profile_dirty=True
        self.mode.setText('连接中…');self.connect_button.setEnabled(False);self.timer.start(40)

    def metadata(self):
        return dict(experiment_name=self.experiment_name.text(),question=self.question.text(),change=self.change.text(),
                    operator_notes=self.notes.toPlainText(),selected_bitstream=self.firmware_info)

    def send_request(self,kind,payload=None):
        if not self.session or not self.session.is_alive():self.alert('未连接设备');return
        try:self.session.request(kind,payload)
        except Exception as e:self.alert(str(e))

    def apply_motion_config(self):
        self.ext2_selected=False
        from session_preparation import automatic_bitstream
        from motion_config import speed_q16
        selected,why=automatic_bitstream(resource_dir().parent)
        if why:self.alert(why);return
        try:
            value=pack_motion_config(self.motion_workspace.value(),self.motion_amplitude.value(),2,PERIODS[self.motion_period.currentIndex()],recovery=self.full_turn_recovery.isChecked())
            velocity=speed_q16(self.motion_speed.value())
        except ValueError as e:self.alert(str(e));return
        self.firmware_info=selected;self.motion_range_check.setChecked(True);self.ready_check.setChecked(True);self.publish_preparation()
        self.send_request('prepare_session',dict(motion_config=value,motion_speed=velocity))

    def prepare_ext2(self):
        self.send_request('ext2_begin')

    def preparation_selection(self):
        from motion_config import speed_internal_q16
        try:cfg=pack_motion_config(self.motion_workspace.value(),self.motion_amplitude.value(),2,PERIODS[self.motion_period.currentIndex()],recovery=self.full_turn_recovery.isChecked())
        except ValueError:cfg=None
        return dict(selected_bitstream=dict(self.firmware_info),motion_config=cfg,range_confirmed=self.motion_range_check.isChecked(),motion_speed_internal_q16=speed_internal_q16(self.motion_speed.value()))

    def publish_preparation(self,*_):
        if self.session and hasattr(self.session,'set_preparation'):
            self.session.set_preparation(self.preparation_selection())

    def preparation_issue(self,snapshot=None):
        from session_preparation import reason
        snapshot=snapshot or getattr(self,'_preparation_snapshot',{})
        return reason(self.preparation_selection(),snapshot.get('expected_bitstreams',[]),snapshot.get('d02_info',{}),snapshot.get('d02_fresh',False))

    def command(self,name,value=0,seconds=0):
        if self.replay:self.alert('回放模式不发送命令');return
        if name in ('arm','capture','start','jog') and not self.ready_check.isChecked():
            self.alert('请先点击“确认现场畅通并准备本次运行”');return
        from session_preparation import guarded
        if guarded(name,value):
            self.publish_preparation()
            why=self.preparation_issue()
            if why:self.alert('未发送：'+why);return
        if name=='start':
            from session_preparation import swing_reason
            why=swing_reason(self.preparation_selection(),getattr(self,'_preparation_snapshot',{}).get('d02_info',{}))
            if why:self.alert('未发送：'+why);return
        if name=='arm':value=TOKEN
        expected=None
        if name in ('arm','start','capture','target') or (name=='trajectory' and value==1):
            try:expected=self.preparation_selection()['motion_config'] if self.ext2_selected else pack_motion_config(self.motion_workspace.value(),self.motion_amplitude.value(),2,PERIODS[self.motion_period.currentIndex()],recovery=self.full_turn_recovery.isChecked())
            except ValueError as e:self.alert(str(e));return
        payload=dict(name=name,value=value,seconds=seconds)
        if expected is not None:payload['expected_motion_config']=expected
        self.send_request('command',payload)

    def stop(self):
        if self.session and self.session.is_alive():
            self.send_request('stop');self.alert('已请求停止，等待板端确认；异常时使用 SW4。')

    def current_profile(self):
        return validate_profile(dict(name=self.profile_name.text(),**{k:v.value() for k,v in self.gains.items()},limit=round(self.limit.value()*10),swing=self.swing.value()))

    def mark_dirty(self,*args):
        self.profile_dirty=True
        if hasattr(self,'parameter_note'):self.parameter_note.setText('编辑值有改动，尚未应用。')

    def apply_profile(self):
        self.alert('R01 参数已固化在 FPGA，不发送旧参数配置。')

    def import_profile(self):
        self.alert('R01 禁止导入旧控制参数。');return
        path,_=W.QFileDialog.getOpenFileName(self,'导入配置','','JSON (*.json)')
        if not path:return
        try:
            p=validate_profile(json.loads(Path(path).read_text(encoding='utf-8-sig')))
            self.profile_name.setText(p['name'])
            for k,spin in self.gains.items():spin.setValue(p[k])
            self.limit.setValue(p['limit']/10);self.swing.setValue(p['swing']);self.mark_dirty()
        except Exception as e:self.alert(f'配置未导入：{e}')

    def save_profile(self):
        path,_=W.QFileDialog.getSaveFileName(self,'保存配置',str(storage_dir()/'profile.json'),'JSON (*.json)')
        if path:
            try:Path(path).write_text(json.dumps(self.current_profile(),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            except Exception as e:self.alert(str(e))

    def add_note(self):self.send_request('note',json.dumps(self.metadata(),ensure_ascii=False))

    def pick_firmware(self):
        path,_=W.QFileDialog.getOpenFileName(self,'登记已使用的位流','','Gowin bitstream (*.fs);;All files (*)')
        if path:
            try:
                self.firmware_info=dict(filename=Path(path).name,sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),board_readback=False)
                if hasattr(self,'publish_preparation'):self.publish_preparation()
                self.firmware_label.setText(f'选择文件：{Path(path).name}\nSHA-256：{self.firmware_info["sha256"][:16]}…\n板端版本仍未知')
                if self.session and self.session.is_alive():self.add_note()
            except Exception as e:self.alert(str(e))

    def update_controls(self,snapshot):
        self._preparation_snapshot=snapshot
        self.publish_preparation()
        snapshot['_preparation_issue']=self.preparation_issue(snapshot)
        row=snapshot.get('latest',{});connected=snapshot.get('connected',False)
        fresh=connected and time.monotonic()-snapshot.get('last_rx',0)<.5;busy=snapshot.get('busy',False)
        ack=snapshot.get('profile_confirmed',False);checked=self.ready_check.isChecked();diag_busy=snapshot.get('d02_busy',False)
        busy=busy or diag_busy
        if ack:self.profile_dirty=self.current_profile()!=snapshot.get('acknowledged_profile')
        offline=bool(self.replay);alive=self.session and self.session.is_alive()
        for name,btn in [('cal',self.cal_button),('clear',self.clear_button),('arm',self.arm_button),('capture',self.capture_button),
            ('start',self.start_button),('target',self.target_button),('trajectory',self.trajectory_on),('trajectory',self.trajectory_off),('jog',self.jog_positive),('jog',self.jog_negative)]:
            enable=not offline and not busy and permitted(name,row,fresh)
            if name in ('arm','capture','start','jog'):enable=enable and ack and checked and not self.profile_dirty
            btn.setEnabled(bool(enable))
        self.apply_button.setEnabled(bool(not offline and fresh and not busy and row.get('state') in (0,5) and not(int(row.get('flags',1))&1)))
        self.stop_button.setEnabled(bool(alive));self.connect_button.setEnabled(True);self.connect_button.setText('停止并断开' if alive else '连接设备')
        self.ports.setEnabled(not alive);self.refresh_button.setEnabled(not alive)
        self.open_button.setEnabled(not alive);self.example_button.setEnabled(not alive);self.compare_button.setEnabled(bool(offline))
        self.seek.setEnabled(offline);self.speed.setEnabled(offline);self.focus_button.setEnabled(offline)
        self.note_button.setEnabled(bool(connected and not offline));self.release_button.setEnabled(bool(connected and not offline))
        self.export_button.setEnabled(bool((connected and snapshot.get('active_path')) or self.replay or self.finalized))
        self.export_button.setText('导出回放摘要 / 实验包' if offline else '结束并导出实验包')
        if not offline:
            self.profile_status.setText('完整配置已 ACK（非板端读回）' if ack and not self.profile_dirty else '参数未知或编辑未应用：请应用完整配置。')
            if not ack:self.parameter_note.setText('板端参数未知；应用成功后显示确认。')
        info=snapshot.get('d02_info',{});capable=(info.get('build')==D03_BUILD and info.get('channel')==1 and info.get('motor_sign')==-1 and info.get('limit')==150 and snapshot.get('d02_fresh',False) and fresh and not offline)
        if not offline:
            file_note=('所选文件 SHA-256：'+self.firmware_info['sha256'][:16]+'…') if self.firmware_info else '未选择位流文件'
            control_known=(info.get('build')==CONTROL_BUILD and snapshot.get('d02_fresh',False) and fresh)
            board_note='板端确认 C02 · 4A050101 · 条件式直立反馈优先' if control_known else ('板端确认 D03 · 4A030101' if capable else '板端版本未确认')
            self.firmware_label.setText('无需本地.fs或位流登记。\n板端版本 '+(f'{CONTROL_BUILD:08X} 新鲜回读已确认' if control_known else '等待新鲜回读'))
        diag_idle=capable and not busy and not info.get('dumping',False)
        self.d02_apply.setEnabled(bool(diag_idle and row.get('state')==0 and not(int(row.get('flags',1))&1)))
        stable=bool(diag_idle and not diag_busy and ack and not self.profile_dirty and info.get('ready') and row.get('limit')==150 and abs(row.get('phi',99))<=math.radians(1))
        if stable:
            if self.d03_ready_since is None:self.d03_ready_since=time.monotonic()
        else:self.d03_ready_since=None
        waited=time.monotonic()-self.d03_ready_since if self.d03_ready_since is not None else 0
        enabled=stable and waited>=3 and self.d02_check.isChecked() and not self.d03_plan.pending and not self.d03_plan.halted and self.d03_plan.value is not None
        self.d02_execute.setEnabled(bool(enabled));self.d02_check.setEnabled(not diag_busy)
        self.d02_trial.setText(self.d03_plan.label)
        value=self.d03_plan.value
        self.d02_execute.setText('执行本轮：无驱动检查' if value==0 else f'执行本轮：{value/10:+.0f}% · 100 ms' if value is not None else '本组已完成')
        self.d02_retry.setEnabled(bool(capable and not snapshot.get('busy') and not snapshot.get('d02_busy') and not self.d02_pending_ui(snapshot) and info.get('phase') in (4,5) and info.get('count')))
        self.d02_progress.setValue(snapshot.get('d02_progress',0))
        if capable:self.d02_status.setText(f"固件 {info['build']:08X} · {PHASES.get(info['phase'],'未知')}\n"+('准备条件满足' if info.get('ready') else '需校准、限幅15%、下垂静止且摆臂在±1°内')+f"\n缓存 #{info['capture_id']} · {info['count']} 点")
        elif info.get('build')==CONTROL_BUILD:self.d02_status.setText('当前为 C02 控制候选，请使用「参数／操作／实验」页。D03 点动流程已结束，此页不执行新点动。')
        else:self.d02_status.setText('D03 固件/通道未确认或数据过期；需要 4A030101、XH1、方向−1、诊断限幅150。')
        if capable:self.d02_status.setText(self.d02_status.text()+f'\n摆臂 {math.degrees(row.get("phi",0)):+.2f}° · 就绪等待 {min(waited,3):.1f}/3秒')
        if self.d03_plan.halted:self.d02_status.setText(self.d03_plan.halted)
        if snapshot.get('d02_path'):self.d02_last_path=snapshot['d02_path']
        if diag_busy:self.export_button.setEnabled(False)
        flags=int(row.get('flags',0))
        if row:
            self.cal_hint.setText(f"{'已校准' if flags&2 else '未校准'} · {'静止就绪' if flags&8 else '尚未就绪'} · {'测量有效' if flags&4 else '测量无效'}\nADC {row['adc']:.0f} / 零点 {row['zero']:.0f}")
            angle=abs(math.degrees(row['theta']));phi=math.degrees(row['phi'])
            self.capture_hint.setText(f'当前 |θ|={angle:.2f}°，φ={phi:+.2f}°\n'+('小角度对照建议 1–2°；摆臂尽量居中。' if angle<3 else '当前偏角偏大；先减小倾角。'))

        # R01 final gate after inherited replay/render controls; worker repeats the gate.
        qualified=snapshot.get('qualification_ok',False)
        info=snapshot.get('d02_info',{})
        valid_board=board_ok(info,snapshot.get('d02_fresh',False))
        self.profile_status.setText(snapshot.get('qualification_note',self.qualification_note))
        for name,btn,value in [('cal',self.cal_button,0),('clear',self.clear_button,0),('arm',self.arm_button,0),('start',self.start_button,0),('capture',self.capture_button,0),('target',self.target_button,round(math.radians(self.target.value())*65536)),('trajectory',self.trajectory_on,1),('trajectory',self.trajectory_off,0)]:
            ok,_=check(name,value,row,info,snapshot.get('d02_fresh',False),qualified)
            enabled=not offline and not busy and ok and permitted(name,row,fresh)
            if name in ('arm','capture','start'):enabled=enabled and checked
            if name in ('arm','capture','start','target') or (name=='trajectory' and value==1):enabled=enabled and not snapshot['_preparation_issue']
            if name=='target' or (name=='trajectory' and value==1):enabled=enabled and bool(int(row.get('flags',0))&128)
            if name=='trajectory' and value==1:enabled=enabled and abs(internal_row(row).get('reference',99))<=1167/65536
            if name=='trajectory' and value==0:enabled=enabled and bool(int(row.get('flags',0))&256)
            btn.setEnabled(bool(enabled))
        cfg=info.get('motion_config')
        try:selected=pack_motion_config(self.motion_workspace.value(),self.motion_amplitude.value(),2,PERIODS[self.motion_period.currentIndex()],recovery=self.full_turn_recovery.isChecked())
        except ValueError:selected=None
        if selected is None or selected!=cfg:
            self.target_button.setEnabled(False);self.trajectory_on.setEnabled(False)
        if cfg is not None:
            mc=unpack_motion_config(cfg);bound=target_bound_deg(cfg);self.target.setRange(-bound,bound)
            speed_q=snapshot.get('d02_info',{}).get('motion_speed_internal_q16')
            speed_text=f'{math.degrees(speed_q/65536*SCALE):.3f}°/s' if isinstance(speed_q,int) and speed_q>0 else '等待速度回读'
            self.motion_readback.setText(f"板端：行程±{mc['workspace_deg']:.3f}° / 可达目标±{bound:.3f}°\n往返±{mc['amplitude_deg']:g}° / {mc['period_s']}秒 / 定位上限{speed_text}")
        if cfg is not None:
            from motion_config import reference_seconds
            from motion_config import reference_seconds_numeric
            duration=reference_seconds_numeric(math.radians(self.target.value())-row.get('reference',0),self.motion_speed.value())
            self.motion_readback.setText(self.motion_readback.text()+f"\n自动恢复行程：{f'约±{360*SCALE:.2f}°（已启用）' if mc['full_turn_recovery'] else '按原保护（未启用）'}\n当前目标参考运动约{duration:g}秒；到位另看实测"+('\n所选配置未应用：停止→应用运动配置→等回读' if selected!=cfg else '\n所选配置与板端一致'))
        self.motion_apply.setEnabled(bool(not offline and not busy and qualified and valid_board and permitted('motioncfg',row,fresh)))
        ext=snapshot.get('ext2',{});special_active=ext.get('state') in ('ORDINARY_STOP_READY','ORDINARY_STOP_ACK','CONFIG_AMPLITUDE','CONFIG_PERIOD','CONFIG_READBACK','CONFIG_ACK','WAIT_HOLD','WAIT_ACK','WAIT_FLAG','RUNNING','EXIT_READY','EXIT_ACK','EXITING')
        from ext2_sequence import capture_row
        self.ext2_prepare.setEnabled(False)
        self.ext2_begin.setEnabled(bool(not offline and not busy and fresh and qualified and valid_board and ext.get('state','READY') in ('READY','RUNNING') and capture_row(row) and not snapshot['_preparation_issue']))
        self.ext2_hold.setEnabled(bool(not offline and fresh and qualified and valid_board and ext.get('state') in ('ORDINARY_STOP_READY','WAIT_HOLD','WAIT_FLAG','RUNNING')))
        self.ext2_status.setText(ext.get('message','起摆/捕获后可直接进入拓展2，退出后继续定位。'))
        self.capture_seconds.setEnabled(not special_active)
        for control in (self.motion_workspace,self.motion_amplitude,self.motion_speed,self.motion_period):control.setEnabled(not special_active)
        if special_active or info.get('ext2_prepared'):
            for b in (self.start_button,self.target_button,self.trajectory_on,self.trajectory_off,self.cal_button,self.clear_button,self.arm_button,self.capture_button,self.motion_apply):b.setEnabled(False)
        self.apply_button.setEnabled(False)
        self.jog_positive.setEnabled(False);self.jog_negative.setEnabled(False)
        self.firmware_label.setText(('板端 R43 · 4A070257 · XH1 / −1' if valid_board else '板端标识未核对/过期')+'\n'+self.profile_status.text())
        if row:
            if int(row['state'])==7:
                self.capture_hint.setText('测量重获等待：电机输出关闭，会话仍使能。\n最多600 ms等真实回读，重获后自动续起摆；12秒起摆预算继续计时。\n这不是保持成功；可随时点击STOP或用SW4停止。')
            else:
                self.capture_hint.setText(f'捕获门限：|θ|≤1°、|摆杆速度|≤0.2 rad/s、|摆臂实际速度|≤{SCALE:.3f} rad/s\n当前 θ={math.degrees(row["theta"]):+.2f}°；手动捕获后可定位/往返，正常15%、恢复最高35%；自动起摆每次有12秒时限；仅自动起摆会话允许失衡后重新起摆，手动捕获保留原保护。')

    def d02_pending_ui(self,snapshot):
        return snapshot.get('d02_info',{}).get('phase') in (1,2,3)

    def choose_replay(self):
        path,_=W.QFileDialog.getOpenFileName(self,'打开一份实验日志',str(storage_dir()),'实验文件 (*.csv *.zip)')
        if path:self.open_replay(path)

    def open_example(self):self.open_replay(resource_dir()/'examples/r10_repeat_demo.csv')

    def open_replay(self,path):
        if self.session and self.session.is_alive():self.alert('请先停止并断开，再进入离线回放');return
        try:run=load_run(path)
        except Exception as e:self.alert(f'无法打开日志：{e}');return
        self.replay=run;self.times,_=elapsed_ms(run['rows']);self.playing=False;self.freeze=False;self.baseline=None;self.clear_baseline();self.focused=True
        self.seek.blockSignals(True);self.seek.setRange(0,len(run['rows'])-1);self.seek.blockSignals(False)
        captures=run['summary']['captures'];self.replay_index=min(len(run['rows'])-1,captures[0]['end_index']) if captures else len(run['rows'])-1
        self.seek.blockSignals(True);self.seek.setValue(self.replay_index);self.seek.blockSignals(False);self.mode.setText('RTL 非线性仿真 · 非实物' if run['meta'].get('is_simulation') else '离线回放 · 不发送命令');self.pause_button.setText('▶ 播放')
        self.profile_status.setText('离线数据：配置以原始命令/元数据为准');self.path_label.setText(str(path))
        self.results.setPlainText(summary_text(run['summary']));self.event_log.clear()
        simulated=bool(run['meta'].get('is_simulation'))
        self.card_titles['theta'].setText('摆杆倾角（模型真值）' if simulated else '摆杆倾角（板端遥测）')
        self.filter_note.setText('仿真回放：角度/速度为模型真值；滤波器效果请查看随包对照曲线。' if simulated else '离线回放：保留记录中的板端角度/速度；滤波版本和零点见元数据。')
        self.plots['adc'].setVisible(not simulated);self.plots['encoder'].setVisible(not simulated)
        self.chart_tabs.setTabText(1,'模型速度（非实测）' if simulated else '传感与速度')
        for event in run['commands'][-100:]:self.event_log.appendPlainText(json.dumps(event,ensure_ascii=False))
        self.render_replay();self.update_controls({})

    def choose_baseline(self):
        path,_=W.QFileDialog.getOpenFileName(self,'选择对照实验','','实验文件 (*.csv *.zip)')
        if path:
            try:
                self.baseline=load_run(path);self.render_replay()
                for k,curve in self.comparison_curves.items():
                    self.plots[k].getPlotItem().legend.removeItem(curve);self.plots[k].getPlotItem().legend.addItem(curve,'对照（虚线）')
                self.results.setPlainText('本轮：\n'+summary_text(self.replay['summary'])+'\n\n对照：\n'+summary_text(self.baseline['summary']))
            except Exception as e:self.alert(str(e))

    def clear_baseline(self):
        self.baseline=None
        for k,curve in self.comparison_curves.items():
            curve.setData([],[]);self.plots[k].getPlotItem().legend.removeItem(curve)
        if self.replay:self.results.setPlainText(summary_text(self.replay['summary']))

    def toggle_pause(self):
        if self.replay:
            self.focused=False
            if self.replay_index>=len(self.replay['rows'])-1:self.replay_index=0
            self.playing=not self.playing;self.play_wall=time.monotonic();self.play_ms=self.times[self.replay_index]
            self.pause_button.setText('暂停回放' if self.playing else '▶ 播放')
        else:
            self.freeze=not self.freeze;self.pause_button.setText('继续绘图' if self.freeze else '暂停绘图')
            self.alert('仅暂停绘图；串口采集和日志仍持续。' if self.freeze else '已恢复绘图')

    def seek_changed(self,value):
        if self.replay:
            self.focused=False;self.replay_index=value;self.play_ms=self.times[value];self.play_wall=time.monotonic();self.render_replay()

    def focus_capture(self):
        if self.replay and self.replay['summary']['captures']:
            self.playing=False;self.pause_button.setText('▶ 播放')
            self.replay_index=min(len(self.replay['rows'])-1,self.replay['summary']['captures'][0]['end_index'])
            self.seek.blockSignals(True);self.seek.setValue(self.replay_index);self.seek.blockSignals(False);self.focused=True;self.render_replay()

    def render_replay(self):
        if not self.replay:return
        rows=self.replay['rows'];i=self.replay_index
        caps=self.replay['summary']['captures'];origin=caps[0]['start_index'] if caps else 0
        left=self.times[origin]-250 if self.focused and caps else self.times[i]-10000
        start=max(0,bisect.bisect_left(self.times,left));segment=rows[start:i+1]
        t=[(v-self.times[origin])/1000 for v in self.times[start:i+1]]
        self.render(segment,t);self.show_row(rows[i])
        if self.replay['meta'].get('is_simulation'):
            self.health.setText(f"RTL 非线性仿真，非实物 · {self.times[i]/1000:.2f} / {self.times[-1]/1000:.2f} s · 角度/速度为模型真值，输出/参考来自RTL；非串口采集")
            self.health.setStyleSheet('color:#ffd083;font-weight:600;')
        else:
            self.health.setText(f"离线回放 · {self.times[i]/1000:.2f} / {self.times[-1]/1000:.2f} s · 第 {i+1}/{len(rows)} 帧 · 固件版本以元数据为准")
            self.health.setStyleSheet('')
        self.draw_events(segment,t)
        if self.baseline:
            rr=self.baseline['rows'];tt,_=elapsed_ms(rr);cc=self.baseline['summary']['captures'];o=cc[0]['start_index'] if cc else 0
            relative=np.array(tt)/1000-tt[o]/1000
            left=t[0] if t else -1;right=t[-1] if t else 1
            a=bisect.bisect_left(relative,left);b=bisect.bisect_right(relative,right)
            for k,c in self.comparison_curves.items():c.setData(relative[a:b],[math.degrees(r[k]) if k!='u' else r[k]/10 for r in rr[a:b]])

    def show_row(self,row):
        for key,sub in [('theta','dtheta'),('phi','dphi')]:
            self.cards[key][0].setText(f"{math.degrees(row[key]):+.2f}°")
            self.cards[key][1].setText(f"速度 {math.degrees(row[sub]):+.1f} °/s · {row[sub]:+.3f} rad/s")
        issue=output_issue(row)
        self.cards['u'][0].setText('输出异常' if issue else f"{row['u']/10:+.1f}%")
        self.cards['u'][0].setStyleSheet('color:#ff7777;' if issue else '')
        self.cards['u'][1].setText(f"{issue}；原始 u={row['u']:.0f}（曲线留空）" if issue else f"限幅 ±{row['limit']/10:.1f}% · 非实测电流")
        self.cards['state'][0].setText(STATE_CN.get(int(row['state']),'未知'))
        self.cards['state'][1].setText(FAULT_CN.get(int(row['fault']),'未知故障')+(' · 已使能' if int(row['flags'])&1 else ' · 未使能')+(' · 直立优先介入' if int(row['flags'])&32 else '')+(' · 角度短时估计' if int(row['flags'])&512 else '')+(' · 恢复升档中' if int(row['flags'])&1024 else (' · 本轮恢复曾触发' if int(row['flags'])&2048 else ''))+(' · 实测稳定到位' if int(row['flags'])&4096 else (' · 到位超时（仅提示）' if int(row['flags'])&8192 else ' · 到位指标未达（仅显示，不阻止正弦）')))
        if row['fault']:
            self.cards['state'][1].setText(f"故障{int(row['fault'])}：{FAULT_CN.get(int(row['fault']),'未知故障')} · {'已使能' if int(row['flags'])&1 else '未使能'}")
        if int(row['state'])==7:
            flags=int(row['flags'])
            self.cards['state'][1].setText(('会话使能' if flags&1 else '会话未使能')+' · '+('电机输出状态异常' if flags&16 or row['u'] else '电机输出关闭')+' · 等真实回读（最长600 ms）')

    def render(self,rows,t):
        if not rows:return
        self.plot_rows=rows
        for key,curve in self.curves.items():
            if key=='negative_limit':values=[-r['limit']/10 for r in rows]
            elif key in ('theta','phi','dtheta','dphi','reference'):values=[math.degrees(r[key]) for r in rows]
            elif key=='u':values=[float('nan') if output_issue(r) else r[key]/10 for r in rows]
            elif key=='limit':values=[r[key]/10 for r in rows]
            else:values=[r[key] for r in rows]
            curve.setData(t,values)
        if len(t)>1:
            for plot in self.plots.values():plot.setXRange(t[0],t[-1],padding=.01)

    def draw_events(self,rows,t):
        for plot,line in self.event_lines:plot.removeItem(line)
        self.event_lines=[]
        for i,r in enumerate(rows):
            if i and r['state']!=rows[i-1]['state'] and int(r['state']) in (2,5,7):
                for key in ('theta','phi','u'):
                    line=pg.InfiniteLine(t[i],pen=pg.mkPen('#b47a7a' if r['state']==5 else '#b18a46' if r['state']==7 else '#6b8762',style=QtCore.Qt.PenStyle.DotLine))
                    self.plots[key].addItem(line);self.event_lines.append((self.plots[key],line))

    def tick(self):
        self.tick_count+=1
        if self.replay:
            if self.playing:
                speed=float(self.speed.currentText().replace('×',''));position=self.play_ms+(time.monotonic()-self.play_wall)*1000*speed
                self.replay_index=min(len(self.times)-1,bisect.bisect_right(self.times,position)-1)
                self.seek.blockSignals(True);self.seek.setValue(self.replay_index);self.seek.blockSignals(False);self.render_replay()
                if self.replay_index>=len(self.times)-1:self.playing=False;self.pause_button.setText('▶ 播放')
            self.update_controls({});return
        if not self.session:self.update_controls({});return
        if self.d03_session is not self.session:
            self.d03_plan=ExperimentPlan();self.d03_ready_since=None;self.d03_session=self.session;self.d02_check.setChecked(False)
        snap=self.session.snapshot();self.last_snapshot=snap
        for event in snap['events']:self.handle_event(event)
        self.update_controls(snap)
        self.card_titles['theta'].setText('摆杆倾角（板端测量）')
        info=snap.get('d02_info',{})
        if info.get('filter_format') and snap.get('d02_fresh'):
            self.filter_note.setText(f"板端512点亚毫秒均值 · 当前均值 {info['adc_q4']/16:.4f} code · 精确零点 {info['zero_q4']/16:.4f} code（状态快照） · {'小数通道有效' if info['fractional_valid'] else '当前块回退原始通道'}")
        else:self.filter_note.setText('角度/速度直接来自板端；ADC曲线为原始诊断码。未收到新鲜小数滤波信息。')
        row=snap['latest'];fresh=snap['connected'] and time.monotonic()-snap['last_rx']<.5
        if row:
            self.show_row(row)
            age=max(0,time.monotonic()-snap['last_rx'])
            self.health.setText(f"{'实时遥测' if fresh else '数据已过期，请检查连接'} · 最近数据 {age:.2f} s 前 · ADC {row['adc']:.0f} / 编码器 {row['encoder']:.0f} · 错帧 {row['bad_commands']:.0f} / 非法跳变 {row['encoder_illegal']:.0f} / 丢帧 {row['dropped']:.0f}")
        if snap['active_path'] and snap['rows'] and time.monotonic()-self.last_result>1:
            self.last_result=time.monotonic();self.results.setPlainText('最近 60 秒窗口（完整结果见结束导出）：\n'+summary_text(summarize(snap['rows'])))
        self.mode.setText('实时连接 · 50 Hz 遥测' if fresh else ('串口已打开 · 等待有效遥测' if snap['connected'] else '已断开 · 保留最后数据'))
        if snap['rows'] and not self.freeze and self.tick_count%2==0:
            rows=snap['rows'][-500:];tt,_=elapsed_ms(rows);tt=[(v-tt[-1])/1000 for v in tt]
            self.render(rows,tt);self.draw_events(rows,tt)
        if snap['active_path']:self.path_label.setText('正在记录：'+snap['active_path'])
        if self.closing and not self.session.is_alive():self.close()

    def handle_event(self,event):
        kind=event['event']
        if kind in ('error','request_rejected','timeout','batch_failed') and self.d03_plan.pending:
            self.d03_plan.halted='诊断请求异常；请结束并导出'
        if kind in ('d02_complete','d02_failed'):
            text=event.get('reason','D02')
            if event.get('path'):self.d02_last_path=event['path']
            if event.get('path'):
                try:self.d02_result.setText(describe(load_captures(event['path'])[0]))
                except Exception:self.d02_result.setText(text)
            else:self.d02_result.setText(text)
            if event.get('path'):
                try:self.d03_plan.consume(load_captures(event['path'])[0])
                except Exception as e:self.d03_plan.halted='记录解析失败：'+str(e)
            if kind=='d02_failed' and not event.get('path'):
                self.d03_plan.halted='诊断未完成；请结束并导出';self.d03_plan.pending=False
        elif kind=='connected':
            self.d03_plan=ExperimentPlan();self.d03_ready_since=None;text='串口已连接；先核对板端版本，再应用本轮配置'
        elif kind=='send':text=f"发送 #{event['seq']} {event['name']} = {event['value_i32']}"
        elif kind=='ack':
            text=f"确认 #{event['seq']} {event['name']}："+('成功' if event['status']==0 else f"拒绝 ({event['status']})")
            if event['name']=='d02start' and event['status']:
                self.d03_plan.halted='板端拒绝诊断；请结束并导出';self.d03_plan.pending=False
            if event['name']=='cal' and event['status']==0:
                self.d03_plan=ExperimentPlan();self.d03_ready_since=None
        elif kind=='batch_done':
            text=f"操作完成：{event['label']}"
            if event['label']=='profile':
                self.profile_dirty=self.current_profile()!=self.session.profile
                self.parameter_note.setText('全部配置已获得 ACK；不是板端读回。' if not self.profile_dirty else '旧编辑值已 ACK；当前编辑值仍未应用。')
        elif kind=='finalize_blocked':
            self.export_requested=False;self.alert(event['reason']);text=event['reason']
        elif kind=='run_finalized':
            complete=not event.get('ext2_tail_required') or event.get('reason')=='confirmed_stop'
            if complete:self.finalized.append(event['path'])
            text=('日志已封存：' if complete else '拓展2日志保留为不完整记录：')+event['path']
            requested=self.export_requested;self.export_requested=False
            if requested and complete:QtCore.QTimer.singleShot(0,lambda p=event['path']:self.export_path(p))
        else:text=str(event.get('reason',event.get('text',event.get('path',kind))))
        self.event_log.appendPlainText(time.strftime('%H:%M:%S')+'  '+text)
        if kind in ('error','stale','timeout','batch_failed','request_rejected','unsafe_stop'):self.alert(text)
        if kind in ('reset','disconnected'):
            self.profile_dirty=True;self.ready_check.setChecked(False);self.d02_check.setChecked(False)
            self.d03_ready_since=None
            self.d03_plan.halted='设备复位或断开；本组结束，请保存日志后重新连接'

    def finish_export(self):
        if self.last_snapshot.get('ext2_tail_required') and (self.export_requested or self.last_snapshot.get('finalizing')):
            self.alert('正在等待STOP确认及完整5秒尾；请等待保存结果。');return
        if self.replay:
            self.export_replay();return
        if self.session and self.session.is_alive() and self.last_snapshot.get('active_path'):
            self.add_note();self.export_requested=True;self.send_request('finalize');self.alert('正在停止并封存日志，随后导出…')
        elif self.finalized:self.export_path(self.finalized[-1])

    def export_path(self,path):
        try:
            image_path=Path(path).with_suffix('.curves.png');self.chart_tabs.grab().save(str(image_path))
            output,summary=archive_run(path,[image_path]);self.results.setPlainText(summary_text(summary));self.bottom_tabs.setCurrentIndex(0)
            self.path_label.setText('实验包：'+str(output));self.alert('已导出：'+str(output))
            self.last_export=str(output)
        except Exception as e:self.alert(f'导出失败，原始日志仍保留：{e}')

    def export_replay(self):
        if not self.replay:return
        destination,_=W.QFileDialog.getSaveFileName(self,'导出回放分析',str(storage_dir()/'replay_review.zip'),'ZIP (*.zip)')
        if not destination:return
        import zipfile
        try:
            source=Path(self.replay['path'])
            if Path(destination).resolve()==source.resolve():raise ValueError('请另选文件名，不能覆盖原实验文件')
            png=storage_dir()/'replay_curves.png';png.parent.mkdir(parents=True,exist_ok=True);self.chart_tabs.grab().save(str(png))
            with zipfile.ZipFile(destination,'w',zipfile.ZIP_DEFLATED) as z:
                if source.suffix.lower()=='.csv':
                    for ext in ('.csv','.commands.jsonl','.meta.json'):
                        p=source.with_suffix(ext)
                        if p.exists():z.write(p,p.name)
                    for p in source.parent.glob(source.stem+'.d02_*.json'):z.write(p,p.name)
                else:
                    with zipfile.ZipFile(source) as original:
                        for info in original.infolist():
                            if info.filename.endswith(('.csv','.commands.jsonl','.meta.json')) or ('.d02_' in info.filename and info.filename.endswith('.json')):z.writestr(Path(info.filename).name,original.read(info))
                z.writestr('analysis.json',json.dumps(self.replay['summary'],ensure_ascii=False,indent=2));z.writestr('analysis.txt',summary_text(self.replay['summary']));z.write(png,'curves.png')
            self.alert('回放分析已导出：'+destination)
        except Exception as e:self.alert(f'导出失败：{e}')

    def save_plot(self):
        path,_=W.QFileDialog.getSaveFileName(self,'保存当前曲线',str(storage_dir()/'curves.png'),'PNG (*.png)')
        if path:
            if self.chart_tabs.grab().save(path):self.alert('已保存：'+path)
            else:self.alert('保存失败，请检查目录权限')

    def open_log_dir(self):
        path=storage_dir()/'logs';path.mkdir(parents=True,exist_ok=True);QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))

    def demo_layout(self,checked):self.sidebar.setVisible(not checked);self.bottom_tabs.setVisible(not checked)

    def closeEvent(self,event):
        if self.session and self.session.is_alive():
            if not(self.last_snapshot.get('ext2_tail_required') and self.closing):
                self.closing=True;self.playing=False;self.session.request('close');self.alert('正在停止并关闭；失联时请使用 SW4。')
            event.ignore()
        else:event.accept()

def main():
    parser=argparse.ArgumentParser(description='J280 图形上位机')
    parser.add_argument('--replay',type=Path);parser.add_argument('--screenshot',type=Path)
    args=parser.parse_args()
    app=W.QApplication(sys.argv[:1]);app.setApplicationName('J280 Studio')
    app.setFont(QtGui.QFont('Microsoft YaHei' if sys.platform=='win32' else 'PingFang SC',10))
    window=Studio();window.show()
    if args.replay:window.open_replay(args.replay)
    if args.screenshot:
        def capture():
            args.screenshot.parent.mkdir(parents=True,exist_ok=True);window.grab().save(str(args.screenshot));app.quit()
        QtCore.QTimer.singleShot(700,capture)
    def excepthook(t,v,tb):
        details=''.join(traceback.format_exception(t,v,tb));storage_dir().mkdir(parents=True,exist_ok=True)
        (storage_dir()/'last_error.txt').write_text(details,encoding='utf-8')
        window.stop();W.QMessageBox.critical(window,'上位机异常',str(v)+'\n已尝试停止，请确认 SW4。错误保存在 last_error.txt。')
    sys.excepthook=excepthook
    return app.exec()

if __name__=='__main__':raise SystemExit(main())
