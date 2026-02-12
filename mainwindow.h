#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QTimer>
#include <QVector2D>
#include "spiderfeet.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

struct LegState {
    QVector2D currentFootPos;
    QVector2D targetFootPos;
    QVector2D startStepPos;
    bool isStepping = false;
    float stepProgress = 0.0f;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void initSpiderFeet();
    void showSpiderFeet();

private:
    Ui::MainWindow *ui;
    QMap<QString, SpiderFeet*> spiderFeet;
    QTimer *m_animTimer;


    float m_perimeterPos = 0.0f;
    float m_perimeterDir = 1.0f;
    int m_frame = 0;


    QVector2D m_bodyVelocity;
    QVector2D m_bodyBob;
    QPoint m_lastBodyPos;

    LegState m_legStates[4];


    QVector2D projectToScreenEdge(const QVector2D &point) const;
    QVector2D perimeterToBodyPos(float t) const;
    float perimeterLength() const;

    void updateAutoWalk();
    QVector2D getIdealFootPos(int legIndex, const QVector2D &bodyPos,
                              const QVector2D &dirVelocity) const;
    void solveIK(int legIndex, const QVector2D &bodyPos, const QVector2D &footPos);

private slots:
    void manualActionCheckboxSlot();
    void autoActionCheckboxSlot();
    void updateFeetPositions();
};
#endif
