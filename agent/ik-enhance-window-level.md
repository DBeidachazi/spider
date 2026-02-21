# IK 增强 + 窗口层级 + 拐角预判

## 变更概要

在 heading-driven leg system 基础上，增强 IK 求解、窗口层级控制和拐角行为。

## 1. Body Squish（身体压扁）

IK 求解时，将根部沿 根→脚 方向推进 `kBodySquish`（60%）比例，缩短有效距离，迫使 FABRIK 中间关节向外弯折，产生蛛腿自然弯曲。

```
ikRoot = rootPos + (footPos - rootPos) * kBodySquish
```

IK 求解完成后，`joints[0]` 被覆盖回 `rootPos`（身体锚点），并做一次正向传递重新约束段长，确保根段视觉上贴在身体上。

## 2. FABRIK 弯曲方向偏置（Pole Target）

用圆角矩形锚点处的**外法线**作为期望弯曲方向。在每轮 FABRIK 迭代的正向传递后，给中间关节施加一个朝外法线方向的小偏移（`kBendBias = 3px`）。

持续偏置让求解器自然收敛到正确的弯曲方向，避免事后镜像翻转导致的逐帧抖动。

## 3. 边缘距离过滤（理想落脚点重写）

`getIdealFootPos` 完全重写。不再用外法线 + 角度钳制，改为：

1. 计算锚点到四条屏幕边的距离
2. 过滤掉距离超过 `kEdgeReachFraction × kLegLengthTotal` 的边
3. 在合格边中选最近的，垂直投影锚点到边上
4. 用 bodyCenter→anchor 的切向分量沿边展开 `kFootReach`
5. 无合格边时回退到最近边

## 4. 拐角提前伸腿

`getIdealFootPos` 末尾新增拐角检测：当脚尖目标接近屏幕拐角（距下一条边 < `kCornerLookahead` 像素）时，将目标沿下一条边滑出。

只影响个别腿的脚尖位置，不改变身体朝向（heading），避免旋转整个坐标系导致另一侧腿被挤压。

## 5. 窗口层级（macOS）

SpiderFeet 使用 `Qt::ToolTip` 在 macOS 上层级高于普通 QMainWindow。新增 `windowlevel.mm`，通过 Objective-C 的 `NSWindow.level` 将 SpiderFeet 降到普通层级（0），使 MainWindow 覆盖在蛛腿上方。

非 macOS 平台仍使用 `foot->stackUnder(this)`。

## 6. 关节角度约束（已实现，未启用）

`constrainJointAngles()` 静态函数已实现，用于限制左腿只能右弯、右腿只能左弯，并钳制最大弯曲角度。由于弯曲方向已通过 pole target 偏置解决，此函数暂未调用。

## 7. Debug 工具

- `paintEvent` 绘制红色矩形显示 ikRoot 位置、蓝色虚线显示 80% 身体矩形
- 每 5 秒输出窗口坐标、各腿 rootPos/ikRoot/footPos

## 新增调参常量

```
kBodySquish          = 0.60   // IK根部压扁比例
kEdgeReachFraction   = 0.40   // 边缘伸腿距离比例
kMinJointAngleDeg    = 120.0  // 最小关节弯曲角度
kCornerLookahead     = 150.0  // 拐角提前伸腿距离(px)
kLegLengthTotal      = 400.0  // 总腿长（从220调增）
kRootLerpSpeed       = 0.25   // 根部跟随速度（从0.12调增）
```

## 修改的文件

- `mainwindow.cpp` — IK 重写、getIdealFootPos 重写、paintEvent、debug 输出、窗口层级调用
- `mainwindow.h` — `m_debugIkRoot[4]`、`paintEvent` 声明、`projectToScreenEdge` 重载
- `spiderfeet.cpp` — XCB 平台加 `FramelessWindowHint`
- `xmake.lua` — macOS 条件编译 `windowlevel.mm` + AppKit
- `windowlevel.mm` — 新增，macOS 原生窗口层级控制
