add_rules("mode.debug", "mode.release")

target("spider")
    set_kind("binary")
    set_languages("cxx17")

    add_rules("qt.widgetapp")
    add_frameworks("QtWidgets")

    add_files("main.cpp", "mainwindow.cpp", "spiderfeet.cpp")
    add_files("mainwindow.h", "spiderfeet.h")
    add_files("mainwindow.ui", "spiderfeet.ui")
