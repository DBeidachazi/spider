add_rules("mode.debug", "mode.release")

-- Use system Qt via pkg-config (works well on Linux).
-- If you want Qt6, change to: pkgconfig::Qt6Widgets (and ensure Qt6 moc/uic are in PATH).
add_requires("pkgconfig::Qt6Widgets")

target("spider")
    set_kind("binary")
    set_languages("cxx17")

    add_rules("qt.widgetapp")
    add_packages("pkgconfig::Qt6Widgets")

    add_files("main.cpp", "mainwindow.cpp", "spiderfeet.cpp")
    add_files("mainwindow.h", "spiderfeet.h")
    add_files("mainwindow.ui", "spiderfeet.ui")
