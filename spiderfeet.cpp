#include "spiderfeet.h"
#include "ui_spiderfeet.h"

#include <QGuiApplication>

SpiderFeet::SpiderFeet(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SpiderFeet)
{
    ui->setupUi(this);

    // On Wayland, regular top-level windows cannot be positioned arbitrarily.
    // Use ToolTip for positioning; on XCB (X11/XWayland) we can show a normal tool window title bar.
    // TODO XWayland BUG!!!
    if (QGuiApplication::platformName() == "xcb") {
        setWindowFlags(Qt::Tool);
    } else {
        setWindowFlags(Qt::ToolTip | Qt::FramelessWindowHint | Qt::WindowStaysOnBottomHint);
    }

    setAttribute(Qt::WA_ShowWithoutActivating);
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setWindowTitle("SpiderFeet");
    setStyleSheet("background-color: #fff; border: 1px solid #333;");
}

SpiderFeet::~SpiderFeet()
{
    delete ui;
}

void SpiderFeet::setOffset(const QPoint &offset)
{
    m_offset = offset;
}

QPoint SpiderFeet::offset() const
{
    return m_offset;
}

void SpiderFeet::setCurrentPos(const QVector2D &pos)
{
    m_currentPos = pos;
}

QVector2D SpiderFeet::currentPos() const
{
    return m_currentPos;
}
