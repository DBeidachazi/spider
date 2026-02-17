# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指引。

## 项目概述

Spider 是一个基于 Qt6 C++17 的桌面小部件，渲染一只沿屏幕边缘爬行的动画蜘蛛。使用逆运动学（IK）驱动腿部动画，弹簧物理模拟身体运动，贝塞尔插值实现迈步弧线。

## 构建命令

```bash
# CMake（主要）
cmake -B build && cmake --build build

# XMake（备选）
xmake build
```

系统需预装 Qt6 Widgets。

## 架构

整个应用由两个 Widget 类加一个入口点组成：

- **MainWindow** (`mainwindow.cpp/h`) — 动画控制器。持有一个约 60 FPS 触发的 QTimer，驱动完整的更新循环：边缘行走、弹簧物理身体跟踪、迈步触发、IK 求解、控件定位。所有调参常量位于文件顶部的 `SpiderTuning` 和 `SpringTuning` 命名空间中。
- **SpiderFeet** (`spiderfeet.cpp/h`) — 单个腿段控件，每条腿有 12 个。使用 `Qt::ToolTip` 窗口标志实现跨平台无边框渲染（兼容 Wayland/XCB）。
- **main.cpp** — 标准 Qt 应用启动入口。

### 动画循环 (`updateFeetPositions`)

1. `updateAutoWalk()` 沿屏幕边缘推进参数化位置，并对身体施加弹簧物理。
2. 对每条腿（共 4 条）：计算理想落脚点，检查迈步触发距离，若正在迈步则通过贝塞尔弧线插值。
3. `solveIK()` 执行 2 次迭代的类 FABRIK 正反向传递，计算每条腿全部 12 个段的位置。
4. 将每个 SpiderFeet 控件移动到计算所得的屏幕位置。

### 步态系统

腿分为交替的两组：(0,3) 和 (1,2)。同一时刻只有一组可以迈步，以保持稳定站姿。当脚到理想位置的距离超过 `kStepTriggerDist` 时触发迈步。

### 关键调参命名空间

所有物理和动画常量位于 `mainwindow.cpp` 顶部的 `SpiderTuning` 和 `SpringTuning` 命名空间中。调整这些参数可改变行走速度、腿部伸展距离、迈步高度、弹簧刚度等。

## 平台说明

- macOS：构建为 `.app` 包（MACOSX_BUNDLE）。
- Linux：SpiderFeet 自动处理 Wayland 与 XCB 的窗口标志差异。
- Windows：CMake 中设置了 WIN32_EXECUTABLE。
