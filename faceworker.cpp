#include "faceworker.h"
#include <opencv2/imgproc.hpp>

FaceWorker::FaceWorker(const std::string &modelPath, QObject *parent)
    : QObject(parent)
    , m_detector(modelPath)
{
}

bool FaceWorker::isReady() const
{
    return m_detector.isLoaded();
}

bool FaceWorker::isBusy() const
{
    return m_busy.load();
}

void FaceWorker::processFrame(const QImage &frame)
{
    m_busy.store(true);

    QImage rgb = frame.convertedTo(QImage::Format_RGB888);
    cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                const_cast<uchar *>(rgb.bits()), rgb.bytesPerLine());
    cv::Mat bgr;
    cv::cvtColor(mat, bgr, cv::COLOR_RGB2BGR);

    auto results = m_detector.detect(bgr);

    if (!results.empty()) {
        // Find the largest face (most likely the primary subject)
        int bestIdx = 0;
        int bestArea = 0;
        for (size_t i = 0; i < results.size(); ++i) {
            int area = results[i].bbox.width() * results[i].bbox.height();
            if (area > bestArea) {
                bestArea = area;
                bestIdx = static_cast<int>(i);
            }
        }
        emit faceDetected(results[bestIdx].bbox, results[bestIdx].confidence);
    } else {
        emit noFaceDetected();
    }

    m_busy.store(false);
}
