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

struct LegLocalConfig {
    QString prefix;
    float perimeterFraction;  // position on body rounded rect, 0=front center, +=clockwise
};

struct LegState {
    QVector2D currentFootPos;
    QVector2D targetFootPos;
    QVector2D startStepPos;
    QVector2D currentRootPos;   // smoothly follows targetRootPos
    QVector2D targetRootPos;    // rotated anchor position
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

    float m_heading = 90.0f;       // current heading (degrees)
    float m_targetHeading = 90.0f; // target heading from edge normal


    QVector2D m_bodyVelocity;
    QVector2D m_bodyBob;
    QPoint m_lastBodyPos;

    LegState m_legStates[4];
    QVector2D m_debugIkRoot[4];   // debug: ikRoot positions for red overlay

protected:
    void paintEvent(QPaintEvent *event) override;


    QVector2D projectToScreenEdge(const QVector2D &point) const;
    QVector2D projectToScreenEdge(const QVector2D &point,
                                   const QVector2D &anchor,
                                   float maxAnchorDist) const;
    QVector2D perimeterToBodyPos(float t) const;
    float perimeterLength() const;
    float computeHeadingFromPerimeter(float t) const;

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
