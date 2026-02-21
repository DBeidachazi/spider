#include <AppKit/AppKit.h>
#include <AVFoundation/AVFoundation.h>
#include <QWidget>
#include <functional>

// 设置 QWidget 对应的 NSWindow 层级
void setNativeWindowLevel(QWidget *widget, int level) {
    if (!widget || !widget->windowHandle()) return;
    NSView *view = (__bridge NSView *)reinterpret_cast<void *>(widget->winId());
    if (view && view.window) {
        view.window.level = level;
    }
}

// 请求 macOS 摄像头权限，回调在主线程执行
void requestCameraAccess(std::function<void(bool)> callback) {
    AVAuthorizationStatus status = [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
    if (status == AVAuthorizationStatusAuthorized) {
        callback(true);
    } else if (status == AVAuthorizationStatusNotDetermined) {
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted) {
            dispatch_async(dispatch_get_main_queue(), ^{
                callback(granted);
            });
        }];
    } else {
        callback(false);
    }
}
