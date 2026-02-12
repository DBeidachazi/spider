#ifndef SPIDERFEET_H
#define SPIDERFEET_H

#include <QWidget>
#include <QPoint>
#include <QVector2D>

namespace Ui {
class SpiderFeet;
}

class SpiderFeet : public QWidget
{
    Q_OBJECT

public:
    explicit SpiderFeet(QWidget *parent = nullptr);
    ~SpiderFeet();

    void setOffset(const QPoint &offset);
    QPoint offset() const;

    void setCurrentPos(const QVector2D &pos);
    QVector2D currentPos() const;

private:
    Ui::SpiderFeet *ui;
    QPoint m_offset;
    QVector2D m_currentPos;
};

#endif
