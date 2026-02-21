# macOS 摄像头采集集成

## 变更概要

在 MainWindow 中嵌入系统摄像头实时画面，作为蜘蛛身体内容（替代白色背景）。

## 核心架构

```
QCamera → QMediaCaptureSession → QVideoWidget (centralWidget 子控件)
```

Qt6 的 QCamera 内部使用 AVFoundation 后端。

## macOS 权限系统（踩坑记录）

macOS 摄像头权限涉及三层机制，缺一不可：

### 1. Info.plist 中的 NSCameraUsageDescription

macOS 强制要求 app bundle 的 Info.plist 包含此 key，否则系统直接拒绝摄像头访问。

**xmake 的问题**：`qt.widgetapp` 规则在 `after_build` 中重新生成 Info.plist，会覆盖任何自定义 key。无法通过 `after_build` hook 注入（target 的 hook 先于 rule 的 hook 执行）。

**解决方案**：在 `on_run` 中用 PlistBuddy 写入：
```lua
os.execv("/usr/libexec/PlistBuddy", {"-c",
    "Add :NSCameraUsageDescription string ...", plist})
```

CMake 则通过自定义 `Info.plist.in` 模板解决（`MACOSX_BUNDLE_INFO_PLIST` 属性）。

### 2. Qt 权限插件（静态库）

Qt6 (Homebrew) 的权限插件是**静态库** `.a`，不是动态库：
```
/opt/homebrew/share/qt/plugins/permissions/libqdarwincamerapermission.a
```

`QCamera::start()` 内部通过 `QCoreApplication::checkPermission(QCameraPermission{})` 检查权限。如果插件未链接，检查失败，摄像头**不会启动**（黑屏）。

**解决方案**：
- `main.cpp` 中 `Q_IMPORT_PLUGIN(QDarwinCameraPermissionPlugin)` 注册插件
- `xmake.lua` 中链接静态库：
  ```lua
  add_linkdirs("/opt/homebrew/share/qt/plugins/permissions")
  add_links("qdarwincamerapermission")
  ```
- 依赖框架：`AVFoundation`（插件的 .prl 文件声明的依赖）

### 3. 原生 AVFoundation 权限请求

Qt 的 `QCameraPermission` 插件依赖 Info.plist 中有 `NSCameraUsageDescription`。如果 app 不是通过 Launch Services 启动（如直接运行二进制），系统可能读不到 plist。

**解决方案**：在 `windowlevel.mm` 中用原生 Objective-C 直接调用 AVFoundation 权限 API：
```objc
[AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo
    completionHandler:^(BOOL granted) { ... }];
```

回调通过 `dispatch_async(dispatch_get_main_queue(), ...)` 确保在主线程执行。

### 4. 代码签名

修改 Info.plist 会使 ad-hoc 签名失效。必须在 plist 写入后重签名：
```bash
codesign --force --deep --sign - spider.app
```

必须通过 `open -W spider.app` 启动（而非直接运行二进制），Launch Services 才会正确读取 Info.plist。

## xmake on_run 完整流程

```lua
on_run(function (target)
    -- 1. PlistBuddy 写入 NSCameraUsageDescription
    -- 2. codesign --force --deep --sign - spider.app
    -- 3. open -W spider.app
end)
```

## 修改的文件

| 文件 | 变更 |
|------|------|
| `mainwindow.h` | 添加 QCamera/QMediaCaptureSession/QVideoWidget 成员，initCamera/startCamera/resizeEvent 声明 |
| `mainwindow.cpp` | initCamera()（原生权限请求）、startCamera()（QCamera 初始化）、resizeEvent（同步 videoWidget 尺寸） |
| `main.cpp` | `Q_IMPORT_PLUGIN(QDarwinCameraPermissionPlugin)` |
| `windowlevel.mm` | 新增 `requestCameraAccess()` 原生权限请求函数 |
| `xmake.lua` | QtMultimedia 框架、权限插件链接、on_run plist/codesign/open 流程 |
| `CMakeLists.txt` | Multimedia/MultimediaWidgets 依赖、OBJCXX、AppKit、Info.plist.in |
| `Info.plist.in` | 新增，CMake 用的 plist 模板（含 NSCameraUsageDescription） |
