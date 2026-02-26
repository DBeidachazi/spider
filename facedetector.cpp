#include "facedetector.h"
#include <opencv2/imgproc.hpp>
#include <QDebug>
#include <cmath>

static inline float sigmoid(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

FaceDetector::FaceDetector(const std::string &modelPath,
                           float confThreshold,
                           float nmsThreshold)
    : m_confThreshold(confThreshold)
    , m_nmsThreshold(nmsThreshold)
{
    try {
        m_net = cv::dnn::readNet(modelPath);
        m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        m_loaded = true;
    } catch (const cv::Exception &e) {
        qWarning() << "Failed to load face detection model:" << e.what();
        m_loaded = false;
    }
}

cv::Mat FaceDetector::resizeImage(const cv::Mat &srcimg, int &newh, int &neww, int &padh, int &padw)
{
    int srch = srcimg.rows, srcw = srcimg.cols;
    newh = kInpHeight;
    neww = kInpWidth;
    padh = 0;
    padw = 0;
    cv::Mat dstimg;

    if (kKeepRatio && srch != srcw) {
        float hw_scale = static_cast<float>(srch) / srcw;
        if (hw_scale > 1) {
            newh = kInpHeight;
            neww = static_cast<int>(kInpWidth / hw_scale);
            cv::resize(srcimg, dstimg, cv::Size(neww, newh), 0, 0, cv::INTER_AREA);
            padw = (kInpWidth - neww) / 2;
            cv::copyMakeBorder(dstimg, dstimg, 0, 0, padw, kInpWidth - neww - padw,
                               cv::BORDER_CONSTANT, 0);
        } else {
            newh = static_cast<int>(kInpHeight * hw_scale);
            neww = kInpWidth;
            cv::resize(srcimg, dstimg, cv::Size(neww, newh), 0, 0, cv::INTER_AREA);
            padh = (kInpHeight - newh) / 2;
            cv::copyMakeBorder(dstimg, dstimg, padh, kInpHeight - newh - padh, 0, 0,
                               cv::BORDER_CONSTANT, 0);
        }
    } else {
        cv::resize(srcimg, dstimg, cv::Size(neww, newh), 0, 0, cv::INTER_AREA);
    }
    return dstimg;
}

void FaceDetector::softmax(const float *x, float *y, int length)
{
    float sum = 0.0f;
    for (int i = 0; i < length; ++i) {
        y[i] = std::exp(x[i]);
        sum += y[i];
    }
    for (int i = 0; i < length; ++i) {
        y[i] /= sum;
    }
}

void FaceDetector::generateProposal(const cv::Mat &out,
                                     std::vector<cv::Rect> &boxes,
                                     std::vector<float> &confidences,
                                     std::vector<std::vector<cv::Point>> &landmarks,
                                     int imgh, int imgw,
                                     float ratioh, float ratiow,
                                     int padh, int padw)
{
    const int feat_h = out.size[2];
    const int feat_w = out.size[3];
    const int stride = static_cast<int>(std::ceil(static_cast<float>(kInpHeight) / feat_h));
    const int area = feat_h * feat_w;
    const float *ptr = reinterpret_cast<const float *>(out.data);
    const float *ptr_cls = ptr + area * kRegMax * 4;
    const float *ptr_kp = ptr + area * (kRegMax * 4 + kNumClass);

    // Pre-allocate DFL buffers outside the loop (fixes memory leak in reference code)
    std::vector<float> dfl_value(kRegMax);
    std::vector<float> dfl_softmax_buf(kRegMax);

    for (int i = 0; i < feat_h; ++i) {
        for (int j = 0; j < feat_w; ++j) {
            const int index = i * feat_w + j;
            float max_conf = -10000.0f;
            for (int k = 0; k < kNumClass; ++k) {
                float conf = ptr_cls[k * area + index];
                if (conf > max_conf) max_conf = conf;
            }

            float box_prob = sigmoid(max_conf);
            if (box_prob > m_confThreshold) {
                float pred_ltrb[4];
                for (int k = 0; k < 4; ++k) {
                    for (int n = 0; n < kRegMax; ++n) {
                        dfl_value[n] = ptr[(k * kRegMax + n) * area + index];
                    }
                    softmax(dfl_value.data(), dfl_softmax_buf.data(), kRegMax);
                    float dis = 0.0f;
                    for (int n = 0; n < kRegMax; ++n) {
                        dis += n * dfl_softmax_buf[n];
                    }
                    pred_ltrb[k] = dis * stride;
                }

                float cx = (j + 0.5f) * stride;
                float cy = (i + 0.5f) * stride;
                float xmin = std::max((cx - pred_ltrb[0] - padw) * ratiow, 0.0f);
                float ymin = std::max((cy - pred_ltrb[1] - padh) * ratioh, 0.0f);
                float xmax = std::min((cx + pred_ltrb[2] - padw) * ratiow, static_cast<float>(imgw - 1));
                float ymax = std::min((cy + pred_ltrb[3] - padh) * ratioh, static_cast<float>(imgh - 1));

                boxes.emplace_back(static_cast<int>(xmin), static_cast<int>(ymin),
                                   static_cast<int>(xmax - xmin), static_cast<int>(ymax - ymin));
                confidences.push_back(box_prob);

                std::vector<cv::Point> kpts(5);
                for (int k = 0; k < 5; ++k) {
                    float x = ((ptr_kp[(k * 3) * area + index] * 2 + j) * stride - padw) * ratiow;
                    float y = ((ptr_kp[(k * 3 + 1) * area + index] * 2 + i) * stride - padh) * ratioh;
                    kpts[k] = cv::Point(static_cast<int>(x), static_cast<int>(y));
                }
                landmarks.push_back(std::move(kpts));
            }
        }
    }
}

std::vector<FaceResult> FaceDetector::detect(const cv::Mat &srcimg)
{
    if (!m_loaded || srcimg.empty()) return {};

    int newh = 0, neww = 0, padh = 0, padw = 0;
    cv::Mat dst = resizeImage(srcimg, newh, neww, padh, padw);

    cv::Mat blob;
    cv::dnn::blobFromImage(dst, blob, 1.0 / 255.0,
                           cv::Size(kInpWidth, kInpHeight),
                           cv::Scalar(0, 0, 0), true, false);
    m_net.setInput(blob);

    std::vector<cv::Mat> outs;
    m_net.forward(outs, m_net.getUnconnectedOutLayersNames());

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<std::vector<cv::Point>> cvLandmarks;
    float ratioh = static_cast<float>(srcimg.rows) / newh;
    float ratiow = static_cast<float>(srcimg.cols) / neww;

    for (auto &out : outs) {
        generateProposal(out, boxes, confidences, cvLandmarks,
                         srcimg.rows, srcimg.cols, ratioh, ratiow, padh, padw);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, m_confThreshold, m_nmsThreshold, indices);

    std::vector<FaceResult> results;
    results.reserve(indices.size());
    for (int idx : indices) {
        FaceResult r;
        r.bbox = QRect(boxes[idx].x, boxes[idx].y, boxes[idx].width, boxes[idx].height);
        r.confidence = confidences[idx];
        for (int k = 0; k < 5; ++k) {
            r.landmarks[k] = QPointF(cvLandmarks[idx][k].x, cvLandmarks[idx][k].y);
        }
        results.push_back(r);
    }
    return results;
}
