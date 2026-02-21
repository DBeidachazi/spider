#include <AppKit/AppKit.h>
#include <QWidget>

// 设置 QWidget 对应的 NSWindow 层级
void setNativeWindowLevel(QWidget *widget, int level) {
    if (!widget || !widget->windowHandle()) return;
    NSView *view = (__bridge NSView *)reinterpret_cast<void *>(widget->winId());
    if (view && view.window) {
        view.window.level = level;
    }
}
