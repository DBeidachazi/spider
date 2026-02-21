cmake_minimum_required(VERSION 3.16)

project(spider VERSION 0.1 LANGUAGES CXX OBJCXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==========================================
# 1. 强制查找 Qt6 (删掉了 NAMES Qt6 Qt5)
# ==========================================
find_package(Qt6 REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        spiderfeet.h
        spiderfeet.cpp
        spiderfeet.ui
)

# ==========================================
# 2. 直接使用 Qt6 的构建命令
# ==========================================
if(APPLE)
    list(APPEND PROJECT_SOURCES windowlevel.mm)
    enable_language(OBJCXX)
endif()

qt_add_executable(spider
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
)

if(APPLE)
    find_library(APPKIT_LIBRARY AppKit)
    target_link_libraries(spider PRIVATE ${APPKIT_LIBRARY})
endif()

# ==========================================
# 3. 强制链接 Qt6 库
# ==========================================
target_link_libraries(spider PRIVATE Qt6::Widgets Qt6::Multimedia Qt6::MultimediaWidgets)

set_target_properties(spider PROPERTIES
        MACOSX_BUNDLE_GUI_IDENTIFIER com.example.spider
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        MACOSX_BUNDLE TRUE
        WIN32_EXECUTABLE TRUE
)

# macOS: add camera permission to Info.plist
if(APPLE)
    set_target_properties(spider PROPERTIES
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_SOURCE_DIR}/Info.plist.in"
    )
endif()

include(GNUInstallDirs)
install(TARGETS spider
        BUNDLE DESTINATION .
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# ==========================================
# 4. Qt6 必须的收尾步骤
# ==========================================
qt_finalize_executable(spider)