#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QTimer>
#include <QVector2D>
#include <QCamera>
#include <QMediaCaptureSession>
#include <QVideoSink>
#include <QVideoFrame>
#include <QImage>
#include "spiderfeet.h"

class FaceWorker;
class QThread;

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
    void initCamera();
    void startCamera();

private:
    Ui::MainWindow *ui;
    QMap<QString, SpiderFeet*> spiderFeet;
    QTimer *m_animTimer;


    float m_perimeterPos = 0.0f;
    float m_perimeterDir = 1.0f;
    int m_frame = 0;

    float m_heading = 90.0f;       // current heading (degrees)
    float m_targetHeading = 90.0f; // target heading from edge normal
    float m_faceHeading = 90.0f;   // face heading derived from leg positions (smooth)


    QVector2D m_bodyVelocity;
    QVector2D m_bodyBob;
    QPoint m_lastBodyPos;

    LegState m_legStates[4];
    QVector2D m_debugIkRoot[4];   // debug: ikRoot positions for red overlay

    QCamera *m_camera = nullptr;
    QMediaCaptureSession *m_captureSession = nullptr;

    // Face detection pipeline
    QVideoSink *m_videoSink = nullptr;
    QImage m_latestFrame;
    QRectF m_faceRect;
    bool m_faceDetected = false;
    int m_noFaceFrames = 0;

    QThread *m_faceThread = nullptr;
    FaceWorker *m_faceWorker = nullptr;

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;


    QVector2D projectToScreenEdge(const QVector2D &point) const;
    QVector2D projectToScreenEdge(const QVector2D &point,
                                   const QVector2D &anchor,
                                   float maxAnchorDist) const;
    QVector2D perimeterToBodyPos(float t) const;
    float perimeterLength() const;
    float computeHeadingFromPerimeter(float t) const;

    void updateAutoWalk();
    float computeHeadingFromLegs() const;
    QVector2D getIdealFootPos(int legIndex, const QVector2D &bodyPos,
                              const QVector2D &dirVelocity) const;
    void solveIK(int legIndex, const QVector2D &bodyPos, const QVector2D &footPos);

private slots:
    void manualActionCheckboxSlot();
    void autoActionCheckboxSlot();
    void updateFeetPositions();
    void onVideoFrame(const QVideoFrame &frame);
    void onFaceDetected(QRect bbox, float confidence);
    void onNoFaceDetected();
};
#endif
