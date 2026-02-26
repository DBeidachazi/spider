add_rules("mode.debug", "mode.release")

add_requires("opencv")

target("spider")
    set_kind("binary")
    set_languages("cxx17")

    add_rules("qt.widgetapp")
    add_frameworks("QtWidgets", "QtMultimedia")

    add_packages("opencv")

    add_files("main.cpp", "mainwindow.cpp", "spiderfeet.cpp", "facedetector.cpp", "faceworker.cpp")
    add_files("mainwindow.h", "spiderfeet.h", "faceworker.h")
    add_files("mainwindow.ui", "spiderfeet.ui")

    if is_plat("macosx") then
        add_files("windowlevel.mm")
        add_frameworks("AppKit", "AVFoundation")

        -- 静态链接 Qt 摄像头权限插件，让 QCamera::start() 内部的权限检查通过
        add_linkdirs("/opt/homebrew/share/qt/plugins/permissions")
        add_links("qdarwincamerapermission")

        on_run(function (target)
            local appdir = path.join(target:targetdir(), target:basename() .. ".app")
            local plist = path.join(appdir, "Contents/Info.plist")

            -- 将模型权重复制到 .app 包的 Resources 目录
            local resdir = path.join(appdir, "Contents/Resources/weights")
            os.mkdir(resdir)
            if os.isfile("weights/yolov8n-face.onnx") then
                os.cp("weights/yolov8n-face.onnx", resdir)
            end

            -- 写入 NSCameraUsageDescription（qt.widgetapp 每次重建都会覆盖 plist）
            if os.isfile(plist) then
                try { function() os.execv("/usr/libexec/PlistBuddy",
                    {"-c", "Delete :NSCameraUsageDescription", plist}) end }
                os.execv("/usr/libexec/PlistBuddy", {"-c",
                    "Add :NSCameraUsageDescription string Spider needs camera access to display your face as the spider body.",
                    plist})
            end

            -- 重签名 .app（修改 plist + 资源后签名失效，必须重签）
            os.execv("codesign", {"--force", "--deep", "--sign", "-", appdir})

            -- 通过 open 启动 .app 包，macOS 才会读取 Info.plist 中的权限声明
            os.execv("open", {"-W", appdir})
        end)
    end
