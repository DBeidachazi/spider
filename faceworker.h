#ifndef FACEWORKER_H
#define FACEWORKER_H

#include <QObject>
#include <QImage>
#include <QRect>
#include <atomic>
#include "facedetector.h"

class FaceWorker : public QObject
{
    Q_OBJECT

public:
    explicit FaceWorker(const std::string &modelPath, QObject *parent = nullptr);
    bool isReady() const;
    bool isBusy() const;

public slots:
    void processFrame(const QImage &frame);

signals:
    void faceDetected(QRect bbox, float confidence);
    void noFaceDetected();

private:
    FaceDetector m_detector;
    std::atomic<bool> m_busy{false};
};

#endif // FACEWORKER_H
