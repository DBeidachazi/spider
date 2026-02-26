#ifndef FACEDETECTOR_H
#define FACEDETECTOR_H

#include <QRect>
#include <QPointF>
#include <vector>
#include <string>
#include <opencv2/dnn.hpp>

struct FaceResult {
    QRect bbox;
    float confidence = 0.0f;
    QPointF landmarks[5];
};

class FaceDetector
{
public:
    FaceDetector(const std::string &modelPath,
                 float confThreshold = 0.45f,
                 float nmsThreshold = 0.5f);
    bool isLoaded() const { return m_loaded; }
    std::vector<FaceResult> detect(const cv::Mat &frame);

private:
    cv::Mat resizeImage(const cv::Mat &srcimg, int &newh, int &neww, int &padh, int &padw);
    void softmax(const float *x, float *y, int length);
    void generateProposal(const cv::Mat &out,
                          std::vector<cv::Rect> &boxes,
                          std::vector<float> &confidences,
                          std::vector<std::vector<cv::Point>> &landmarks,
                          int imgh, int imgw,
                          float ratioh, float ratiow,
                          int padh, int padw);

    cv::dnn::Net m_net;
    bool m_loaded = false;
    float m_confThreshold;
    float m_nmsThreshold;

    static constexpr int kInpWidth = 640;
    static constexpr int kInpHeight = 640;
    static constexpr int kNumClass = 1;
    static constexpr int kRegMax = 16;
    static constexpr bool kKeepRatio = true;
};

#endif // FACEDETECTOR_H
