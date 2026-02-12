cmake_minimum_required(VERSION 3.16)

project(spider VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==========================================
# 1. 强制查找 Qt6 (删掉了 NAMES Qt6 Qt5)
# ==========================================
find_package(Qt6 REQUIRED COMPONENTS Widgets)

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
qt_add_executable(spider
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
)

# ==========================================
# 3. 强制链接 Qt6 库
# ==========================================
target_link_libraries(spider PRIVATE Qt6::Widgets)

set_target_properties(spider PROPERTIES
        MACOSX_BUNDLE_GUI_IDENTIFIER com.example.spider
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        MACOSX_BUNDLE TRUE
        WIN32_EXECUTABLE TRUE
)

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