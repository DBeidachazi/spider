#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QtMath>
#include <QVector2D>
#include <QScreen>
#include <QGuiApplication>
#include <QRandomGenerator>
#include <QPainter>
#include <QMediaDevices>
#include <QCameraDevice>
#include <functional>

#ifdef Q_OS_MACOS
extern void setNativeWindowLevel(QWidget *widget, int level);
extern void requestCameraAccess(std::function<void(bool)> callback);
#endif

namespace SpiderTuning {
constexpr int kFeetPerLeg = 12;          // 每条腿多少节
constexpr int kFootW = 80;
constexpr int kFootH = 50;
constexpr int kAnimIntervalMs = 16;

// IK
constexpr float kLegLengthTotal = 400.0f;
constexpr float kStepTriggerDist = 75.0f;  // 触发距离
constexpr float kStepSpeed = 0.15f;        // 快速迈步
constexpr float kStepHeight = 22.0f;       // 低弧线
constexpr float kFootReach = 130.0f;       // 脚伸向边缘的距离
constexpr float kVelocityPredict = 10.0f;
constexpr int kIkIterations = 2;

constexpr float kWalkSpeed = 1.8f;         // px/frame

// 弹簧身体物理
constexpr float kSpringStiffness = 0.10f;  // 身体追踪目标的刚度
constexpr float kSpringDamping = 0.78f;    // 速度衰减
constexpr float kBobStrength = 1.5f;       // 脚落地时身体的冲量

// 朝向驱动系统
constexpr float kHeadingLerpSpeed = 0.08f;   // 朝向平滑速度
constexpr float kRootLerpSpeed = 0.25f;      // 根部跟随速度
constexpr float kLegAngleThreshold = 150.0f; // 腿最大偏转角度
constexpr float kBodyRectScale = 0.80f;      // 圆角矩形为窗口的80%
constexpr float kCornerRadiusFraction = 0.35f; // 圆角半径占短边比例
constexpr float kBodySquish = 0.60f;          // IK根部向脚靠近60%，压扁身体使腿弯曲
constexpr float kEdgeReachFraction = 0.40f;   // 锚点距边缘 ≤ 腿长*此值 才伸腿
constexpr float kMinJointAngleDeg = 120.0f;   // 最小关节弯曲角度（度）
constexpr float kCornerLookahead = 150.0f;    // 接近拐角时提前多少像素开始转向
} // namespace SpiderTuning

// 腿锚点在身体圆角矩形上的位置
// perimeterFraction: 0=前方中心, +=顺时针, -=逆时针
//        前方 (heading)
//          ↑
//   FL ----●---- FR
//          |
//   BL ----●---- BR
static const LegLocalConfig LEG_CONFIGS[] = {
    {"leftFrontFeet_",  -0.20f},   // FL: 20% 逆时针
    {"leftBackFeet_",   -0.25f},   // BL: 25% 逆时针
    {"rightFrontFeet_",  0.20f},   // FR: 20% 顺时针
    {"rightBackFeet_",   0.25f},   // BR: 25% 顺时针
};

// ─── 圆角矩形路径辅助 ──────────────────────────────────────

struct LocalPoint {
    float right, forward;               // 本地坐标系位置
    float normalRight, normalForward;   // 外法线
};

// 在身体圆角矩形上采样：fraction ∈ [−0.5, 0.5), 0 = 前方中心, + = 顺时针
// 本地坐标系: right = 右, forward = 前 (heading方向)
static LocalPoint bodyRoundedRectPoint(float fraction, float halfW, float halfH) {
    float r = qMin(halfW, halfH) * SpiderTuning::kCornerRadiusFraction;
    r = qBound(0.0f, r, qMin(halfW, halfH));

    fraction = fmodf(fraction, 1.0f);
    if (fraction < 0.0f) fraction += 1.0f;

    const float halfFront = halfW - r;
    const float arcLen = static_cast<float>(M_PI) * r * 0.5f;
    const float sideLen = 2.0f * (halfH - r);
    const float fullBack = 2.0f * (halfW - r);
    const float totalPeri = 4.0f * (halfW - r) + 4.0f * (halfH - r)
                          + 2.0f * static_cast<float>(M_PI) * r;

    float dist = fraction * totalPeri;

    // 9 段: 前右半→前右角→右边→后右角→后边→后左角→左边→前左角→前左半
    const float segLens[] = {
        halfFront, arcLen, sideLen, arcLen, fullBack, arcLen, sideLen, arcLen, halfFront
    };

    int seg = 0;
    while (seg < 8 && dist > segLens[seg] + 1e-6f) {
        dist -= segLens[seg];
        seg++;
    }
    float t = (segLens[seg] > 1e-6f)
            ? qBound(0.0f, dist / segLens[seg], 1.0f)
            : 0.0f;

    LocalPoint lp{};
    switch (seg) {
    case 0: // 前边右半
        lp.right = t * halfFront;
        lp.forward = halfH;
        lp.normalRight = 0; lp.normalForward = 1;
        break;
    case 1: { // 前右圆角
        float a = static_cast<float>(M_PI) * 0.5f * (1.0f - t);
        lp.right = (halfW - r) + r * qCos(a);
        lp.forward = (halfH - r) + r * qSin(a);
        lp.normalRight = qCos(a); lp.normalForward = qSin(a);
        break;
    }
    case 2: // 右边
        lp.right = halfW;
        lp.forward = (halfH - r) - t * sideLen;
        lp.normalRight = 1; lp.normalForward = 0;
        break;
    case 3: { // 后右圆角
        float a = -static_cast<float>(M_PI) * 0.5f * t;
        lp.right = (halfW - r) + r * qCos(a);
        lp.forward = -(halfH - r) + r * qSin(a);
        lp.normalRight = qCos(a); lp.normalForward = qSin(a);
        break;
    }
    case 4: // 后边
        lp.right = (halfW - r) - t * fullBack;
        lp.forward = -halfH;
        lp.normalRight = 0; lp.normalForward = -1;
        break;
    case 5: { // 后左圆角
        float a = -static_cast<float>(M_PI) * 0.5f
                  - static_cast<float>(M_PI) * 0.5f * t;
        lp.right = -(halfW - r) + r * qCos(a);
        lp.forward = -(halfH - r) + r * qSin(a);
        lp.normalRight = qCos(a); lp.normalForward = qSin(a);
        break;
    }
    case 6: // 左边
        lp.right = -halfW;
        lp.forward = -(halfH - r) + t * sideLen;
        lp.normalRight = -1; lp.normalForward = 0;
        break;
    case 7: { // 前左圆角
        float a = static_cast<float>(M_PI)
                  - static_cast<float>(M_PI) * 0.5f * t;
        lp.right = -(halfW - r) + r * qCos(a);
        lp.forward = (halfH - r) + r * qSin(a);
        lp.normalRight = qCos(a); lp.normalForward = qSin(a);
        break;
    }
    case 8: // 前边左半
        lp.right = -halfFront + t * halfFront;
        lp.forward = halfH;
        lp.normalRight = 0; lp.normalForward = 1;
        break;
    }
    return lp;
}

// 本地坐标 (right, forward) → 屏幕偏移
// heading 方向 = (cos(h), −sin(h))，右方向 = (sin(h), cos(h))
static inline QVector2D localToScreen(float localRight, float localForward,
                                       float headingRad) {
    float sinH = qSin(headingRad);
    float cosH = qCos(headingRad);
    return QVector2D(localRight * sinH + localForward * cosH,
                     localRight * cosH - localForward * sinH);
}

// 关节角度约束：左腿只能向右弯，右腿只能向左弯
// allowedBendDir: 允许弯曲的方向（屏幕空间）
// TODO 这里调用导致蜘蛛的腿抽搐，目前左边两只腿与右边两只腿弯曲朝向一致，后面看一下从腿的定义去修改弯曲朝向
static void constrainJointAngles(QVector<QVector2D> &joints, float segLen,
                                  const QVector2D &allowedBendDir) {
    const float maxBendRad = qDegreesToRadians(180.0f - SpiderTuning::kMinJointAngleDeg);
    const float minDot = qCos(maxBendRad);  // cos(60°) = 0.5 for 120°

    for (int i = 1; i < joints.size() - 1; ++i) {
        QVector2D d1 = joints[i] - joints[i - 1];
        if (d1.lengthSquared() < 1e-6f) continue;
        d1.normalize();

        QVector2D d2 = joints[i + 1] - joints[i];
        if (d2.lengthSquared() < 1e-6f) continue;
        d2.normalize();

        float dot = QVector2D::dotProduct(d1, d2);
        QVector2D perp = d2 - d1 * dot;
        float perpLen = perp.length();

        if (perpLen < 1e-6f) continue;  // 几乎笔直，无需约束

        QVector2D perpDir = perp / perpLen;
        float bendAlign = QVector2D::dotProduct(perpDir, allowedBendDir);

        QVector2D newDir = d2;
        bool corrected = false;

        if (bendAlign < 0) {
            // 弯曲方向错误 → 拉直
            newDir = d1;
            corrected = true;
        } else if (dot < minDot) {
            // 弯曲方向正确但过度 → 钳制到最大弯曲角
            newDir = (d1 * minDot + perpDir * qSin(maxBendRad)).normalized();
            corrected = true;
        }

        if (corrected) {
            joints[i + 1] = joints[i] + newDir * segLen;
        }
    }
}

static QVector2D bezierInterp(const QVector2D &p0, const QVector2D &p1,
                               float t, float height) {
    QVector2D flat = p0 + (p1 - p0) * t;
    flat.setY(flat.y() - height * qSin(t * static_cast<float>(M_PI)));
    return flat;
}

// ─── Constructor ──────────────────────────────────────────────

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_animTimer(new QTimer(this))
{
    ui->setupUi(this);
    this->setWindowTitle("这是你自己");
    this->setStyleSheet("QMainWindow { background-color: #FFFFFF; }");
    ui->menubar->setStyleSheet(
        "QMenuBar { color: #000; }"
        "QMenuBar::item {  color: #000; }"
        "QMenu { background-color: #fff; color: #000; }"
        "QMenu::item {  color: #000; }"
        );
    connect(ui->autoActionCheckbox, &QAction::triggered,
            this, &MainWindow::autoActionCheckboxSlot);
    connect(ui->manualActionCheckbox, &QAction::triggered,
            this, &MainWindow::manualActionCheckboxSlot);
    connect(m_animTimer, &QTimer::timeout,
            this, &MainWindow::updateFeetPositions);
    ui->autoActionCheckbox->setChecked(true);
    ui->manualActionCheckbox->setChecked(false);
    autoActionCheckboxSlot();

    // 出生点：屏幕底部居中
    QScreen *screen = QGuiApplication::primaryScreen();
    if (screen) {
        QRect geo = screen->availableGeometry();
        this->move(geo.left() + (geo.width() - this->width()) / 2,
                   geo.bottom() - this->height());
        float usableW = qMax(1.0f, float(geo.width() - this->width()));
        m_perimeterPos = usableW / 2.0f;
    }
    m_lastBodyPos = this->pos();

    QTimer::singleShot(0, this, [this]() {
        this->initSpiderFeet();
        this->showSpiderFeet();
    });

    initCamera();
}

// ─── Camera ──────────────────────────────────────────────────

void MainWindow::initCamera() {
#ifdef Q_OS_MACOS
    // 用原生 AVFoundation API 请求摄像头权限，绕过 Qt 的权限插件系统
    requestCameraAccess([this](bool granted) {
        if (granted) {
            startCamera();
        } else {
            qDebug() << "Camera permission denied, falling back to white background.";
        }
    });
#else
    startCamera();
#endif
}

void MainWindow::startCamera() {
    QCameraDevice defaultCam = QMediaDevices::defaultVideoInput();
    if (defaultCam.isNull()) {
        qDebug() << "No camera available, falling back to white background.";
        return;
    }

    m_videoWidget = new QVideoWidget(centralWidget());
    m_videoWidget->setGeometry(0, 0, centralWidget()->width(), centralWidget()->height());
    m_videoWidget->lower();
    m_videoWidget->show();

    m_camera = new QCamera(defaultCam, this);
    m_captureSession = new QMediaCaptureSession(this);
    m_captureSession->setCamera(m_camera);
    m_captureSession->setVideoOutput(m_videoWidget);

    connect(m_camera, &QCamera::errorOccurred, this, [this](QCamera::Error error, const QString &desc) {
        qDebug() << "Camera error:" << error << desc;
        if (m_videoWidget) {
            m_videoWidget->hide();
        }
    });

    m_camera->start();
}

// ─── Perimeter helpers ────────────────────────────────────────

float MainWindow::perimeterLength() const {
    QScreen *screen = QGuiApplication::primaryScreen();
    if (!screen) return 100.0f;
    QRect geo = screen->availableGeometry();
    float W = qMax(1.0f, float(geo.width()  - this->width()));
    float H = qMax(1.0f, float(geo.height() - this->height()));
    return 2.0f * (W + H);
}

// 把周长参数 t 映射到身体（左上角）屏幕坐标
// 顺时针：底边(左→右) → 右边(下→上) → 顶边(右→左) → 左边(上→下)
QVector2D MainWindow::perimeterToBodyPos(float t) const {
    QScreen *screen = QGuiApplication::primaryScreen();
    if (!screen) return QVector2D(0, 0);
    QRect geo = screen->availableGeometry();
    float minX = geo.left();
    float maxX = geo.right()  - this->width();
    float minY = geo.top();
    float maxY = geo.bottom() - this->height();
    float W = qMax(1.0f, maxX - minX);
    float H = qMax(1.0f, maxY - minY);
    float peri = 2.0f * (W + H);

    t = fmod(t, peri);
    if (t < 0) t += peri;

    if (t < W)           return QVector2D(minX + t,     maxY);         // 底边
    t -= W;
    if (t < H)           return QVector2D(maxX,          maxY - t);    // 右边
    t -= H;
    if (t < W)           return QVector2D(maxX - t,      minY);        // 顶边
    t -= W;
    return QVector2D(minX,          minY + t);                          // 左边
}

// 把任意点投影到最近的屏幕边框上 TODO 脚会不朝一个方向
QVector2D MainWindow::projectToScreenEdge(const QVector2D &point) const {
    QScreen *screen = QGuiApplication::primaryScreen();
    if (!screen) return point;
    QRect geo = screen->geometry();
    float left   = geo.left();
    float right  = geo.right();
    float top    = geo.top();
    float bottom = geo.bottom();

    float dLeft   = qAbs(point.x() - left);
    float dRight  = qAbs(point.x() - right);
    float dTop    = qAbs(point.y() - top);
    float dBottom = qAbs(point.y() - bottom);
    float dMin    = qMin(qMin(dLeft, dRight), qMin(dTop, dBottom));

    QVector2D result = point;
    if      (dMin == dBottom) result.setY(bottom);
    else if (dMin == dTop)    result.setY(top);
    else if (dMin == dLeft)   result.setX(left);
    else                      result.setX(right);

    // 钳制到屏幕范围内 TODO 可以略微超出屏幕
    result.setX(qBound(left, result.x(), right));
    result.setY(qBound(top,  result.y(), bottom));
    return result;
}

// 带锚点距离过滤的投影：只投射到锚点足够近的边
QVector2D MainWindow::projectToScreenEdge(const QVector2D &point,
                                           const QVector2D &anchor,
                                           float maxAnchorDist) const {
    QScreen *screen = QGuiApplication::primaryScreen();
    if (!screen) return point;
    QRect geo = screen->geometry();
    float left   = geo.left();
    float right  = geo.right();
    float top    = geo.top();
    float bottom = geo.bottom();

    // 锚点到各边的距离
    float aDists[] = {
        qAbs(anchor.y() - bottom),  // 0: 底边
        qAbs(anchor.y() - top),     // 1: 顶边
        qAbs(anchor.x() - left),    // 2: 左边
        qAbs(anchor.x() - right),   // 3: 右边
    };
    // 点到各边的距离
    float pDists[] = {
        qAbs(point.y() - bottom),
        qAbs(point.y() - top),
        qAbs(point.x() - left),
        qAbs(point.x() - right),
    };

    // 只考虑锚点足够近的边，在其中找点最近的
    float bestDist = 1e9f;
    int bestEdge = -1;
    for (int i = 0; i < 4; ++i) {
        if (aDists[i] <= maxAnchorDist && pDists[i] < bestDist) {
            bestDist = pDists[i];
            bestEdge = i;
        }
    }

    // 无合格边时回退到最近边
    if (bestEdge < 0) {
        float dMin = qMin(qMin(pDists[0], pDists[1]), qMin(pDists[2], pDists[3]));
        for (int i = 0; i < 4; ++i) {
            if (pDists[i] == dMin) { bestEdge = i; break; }
        }
    }

    QVector2D result = point;
    switch (bestEdge) {
    case 0: result.setY(bottom); break;
    case 1: result.setY(top);    break;
    case 2: result.setX(left);   break;
    case 3: result.setX(right);  break;
    }

    result.setX(qBound(left, result.x(), right));
    result.setY(qBound(top,  result.y(), bottom));
    return result;
}

// 根据周长位置返回内法线角度（朝向用户/屏幕内部）
// 底边 → 90°（朝上）, 右边 → 180°（朝左）, 顶边 → 270°（朝下）, 左边 → 0°/360°（朝右）
float MainWindow::computeHeadingFromPerimeter(float t) const {
    QScreen *screen = QGuiApplication::primaryScreen();
    if (!screen) return 90.0f;
    QRect geo = screen->availableGeometry();
    float W = qMax(1.0f, float(geo.width()  - this->width()));
    float H = qMax(1.0f, float(geo.height() - this->height()));
    float peri = 2.0f * (W + H);

    t = fmod(t, peri);
    if (t < 0) t += peri;

    if (t < W)           return 90.0f;    // 底边：朝上
    t -= W;
    if (t < H)           return 180.0f;   // 右边：朝左
    t -= H;
    if (t < W)           return 270.0f;   // 顶边：朝下
    return 0.0f;                           // 左边：朝右
}

// ─── Leg segment init (structure unchanged) ───────────────────

void MainWindow::initSpiderFeet() {
    for (const auto &cfg : LEG_CONFIGS) {
        for (int i = 0; i < SpiderTuning::kFeetPerLeg; ++i) {
            auto *foot = new SpiderFeet(this);
            foot->setFixedSize(SpiderTuning::kFootW, SpiderTuning::kFootH);
            foot->setOffset(QPoint(0, 0));
            this->spiderFeet.insert(cfg.prefix + QString::number(i), foot);
        }
    }
}

void MainWindow::showSpiderFeet() {
    QPoint origin = this->mapToGlobal(QPoint(0, 0));
    for (auto it = spiderFeet.begin(); it != spiderFeet.end(); ++it) {
        SpiderFeet *foot = it.value();
        QPoint screenPos = origin + foot->offset();
        foot->setCurrentPos(QVector2D(screenPos.x() + SpiderTuning::kFootW / 2.0f,
                                       screenPos.y() + SpiderTuning::kFootH / 2.0f));
        foot->move(screenPos);
        foot->show();
#ifdef Q_OS_MACOS
        // macOS: 将蛛腿窗口层级设为普通级别(0)，低于 MainWindow 的 ToolTip 级别
        setNativeWindowLevel(foot, 0);
#else
        foot->stackUnder(this);
#endif
    }

    // 初始化朝向
    m_heading = computeHeadingFromPerimeter(m_perimeterPos);
    m_targetHeading = m_heading;

    // 初始化脚和根部位置
    QVector2D bodyPos(this->pos().x(), this->pos().y());
    QVector2D bodyCenter = bodyPos + QVector2D(this->width() * 0.5f, this->height() * 0.5f);
    float headingRad = qDegreesToRadians(m_heading);
    float halfW = this->width() * SpiderTuning::kBodyRectScale * 0.5f;
    float halfH = this->height() * SpiderTuning::kBodyRectScale * 0.5f;

    for (int leg = 0; leg < 4; ++leg) {
        const LegLocalConfig &cfg = LEG_CONFIGS[leg];
        LocalPoint lp = bodyRoundedRectPoint(cfg.perimeterFraction, halfW, halfH);
        QVector2D rootOffset = localToScreen(lp.right, lp.forward, headingRad);
        QVector2D rootPos = bodyCenter + rootOffset;

        m_legStates[leg].currentRootPos = rootPos;
        m_legStates[leg].targetRootPos  = rootPos;

        QVector2D ideal = getIdealFootPos(leg, bodyPos, QVector2D(0, 0));
        m_legStates[leg].currentFootPos = ideal;
        m_legStates[leg].targetFootPos  = ideal;
        m_legStates[leg].startStepPos   = ideal;
        m_legStates[leg].isStepping     = false;
        m_legStates[leg].stepProgress   = 0.0f;
    }
    m_animTimer->start(SpiderTuning::kAnimIntervalMs);
}

// ─── Edge-adhesion foot placement ─────────────────────────────

QVector2D MainWindow::getIdealFootPos(int legIndex,
                                       const QVector2D &bodyPos,
                                       const QVector2D &dirVelocity) const {
    const LegLocalConfig &cfg = LEG_CONFIGS[legIndex];
    const float headingRad = qDegreesToRadians(m_heading);

    // 身体中心
    const QVector2D bodyCenter = bodyPos + QVector2D(this->width() * 0.5f,
                                                      this->height() * 0.5f);

    // 圆角矩形上的锚点
    const float halfW = this->width() * SpiderTuning::kBodyRectScale * 0.5f;
    const float halfH = this->height() * SpiderTuning::kBodyRectScale * 0.5f;
    LocalPoint lp = bodyRoundedRectPoint(cfg.perimeterFraction, halfW, halfH);
    const QVector2D anchor = bodyCenter + localToScreen(lp.right, lp.forward, headingRad);

    QScreen *screen = QGuiApplication::primaryScreen();
    if (!screen) return anchor;
    QRect geo = screen->geometry();
    float left   = geo.left();
    float right  = geo.right();
    float top    = geo.top();
    float bottom = geo.bottom();

    const float edgeThresh = SpiderTuning::kLegLengthTotal * SpiderTuning::kEdgeReachFraction;

    // 锚点到各边距离: 底/顶/左/右
    float aDists[] = {
        qAbs(anchor.y() - bottom),
        qAbs(anchor.y() - top),
        qAbs(anchor.x() - left),
        qAbs(anchor.x() - right),
    };
    QVector2D edgeNormals[] = {
        {0, 1}, {0, -1}, {-1, 0}, {1, 0},
    };

    // 找最近的合格边（锚点距边 ≤ 阈值）
    int bestEdge = -1;
    float bestDist = 1e9f;
    for (int i = 0; i < 4; ++i) {
        if (aDists[i] <= edgeThresh && aDists[i] < bestDist) {
            bestDist = aDists[i];
            bestEdge = i;
        }
    }
    if (bestEdge < 0) {
        bestDist = 1e9f;
        for (int i = 0; i < 4; ++i) {
            if (aDists[i] < bestDist) { bestDist = aDists[i]; bestEdge = i; }
        }
    }

    // 锚点垂直投影到边
    QVector2D edgePoint;
    switch (bestEdge) {
    case 0: edgePoint = QVector2D(anchor.x(), bottom); break;
    case 1: edgePoint = QVector2D(anchor.x(), top);    break;
    case 2: edgePoint = QVector2D(left, anchor.y());   break;
    case 3: edgePoint = QVector2D(right, anchor.y());  break;
    }

    // 沿边展开：用 bodyCenter→anchor 向量在边上的切向分量
    // 左腿向左展开，右腿向右展开，自然形成蜘蛛腿的张开姿态
    QVector2D bodyToAnchor = anchor - bodyCenter;
    QVector2D eN = edgeNormals[bestEdge];
    QVector2D tangent = bodyToAnchor - eN * QVector2D::dotProduct(bodyToAnchor, eN);
    float tangentLen = tangent.length();

    QVector2D idealPos = edgePoint;
    if (tangentLen > 1e-3f) {
        idealPos += (tangent / tangentLen) * SpiderTuning::kFootReach;
    }
    idealPos += dirVelocity * SpiderTuning::kVelocityPredict;

    // 固定到边 + 钳制到屏幕
    switch (bestEdge) {
    case 0: idealPos.setY(bottom); break;
    case 1: idealPos.setY(top);    break;
    case 2: idealPos.setX(left);   break;
    case 3: idealPos.setX(right);  break;
    }
    idealPos.setX(qBound(left, idealPos.x(), right));
    idealPos.setY(qBound(top, idealPos.y(), bottom));

    // ── 拐角提前伸腿 ──
    // 当脚尖目标已经接近屏幕拐角（距两条边都很近），说明当前边快到头了
    // 将脚尖目标沿下一条边滑出，让前方的腿提前抓住下一条边
    const float L = SpiderTuning::kCornerLookahead;
    float dLeft   = idealPos.x() - left;
    float dRight  = right - idealPos.x();
    float dTop    = idealPos.y() - top;
    float dBottom = bottom - idealPos.y();

    // 检查是否卡在某个角落（一个轴贴着边，另一个轴也接近边）
    if (bestEdge == 0 || bestEdge == 1) {
        // 脚在上/下边，检查是否接近左右角
        if (dRight < L) {
            // 接近右边缘 → 让脚滑到右边上
            idealPos.setX(right);
            float slide = L - dRight;
            if (bestEdge == 0) idealPos.setY(bottom - slide);  // 右下角→右边往上
            else               idealPos.setY(top + slide);     // 右上角→右边往下
        } else if (dLeft < L) {
            // 接近左边缘 → 让脚滑到左边上
            idealPos.setX(left);
            float slide = L - dLeft;
            if (bestEdge == 0) idealPos.setY(bottom - slide);  // 左下角→左边往上
            else               idealPos.setY(top + slide);     // 左上角→左边往下
        }
    } else {
        // 脚在左/右边，检查是否接近上下角
        if (dBottom < L) {
            idealPos.setY(bottom);
            float slide = L - dBottom;
            if (bestEdge == 3) idealPos.setX(right - slide);   // 右下角→下边往左
            else               idealPos.setX(left + slide);    // 左下角→下边往右
        } else if (dTop < L) {
            idealPos.setY(top);
            float slide = L - dTop;
            if (bestEdge == 3) idealPos.setX(right - slide);   // 右上角→上边往左
            else               idealPos.setX(left + slide);    // 左上角→上边往右
        }
    }

    idealPos.setX(qBound(left, idealPos.x(), right));
    idealPos.setY(qBound(top, idealPos.y(), bottom));

    return idealPos;
}

// ─── IK solver ───────────────────────────────────────────────
//
// 对单条腿执行完整的逆运动学求解，流程：
//   1. 收集该腿的所有段 widget，读取当前关节位置
//   2. 根据身体朝向计算该腿在圆角矩形身体上的锚点（根部目标位置）
//   3. 根部位置 lerp 平滑跟随目标，避免瞬移
//   4. 对根部施加 squish 压扁，缩短根→脚有效距离以产生腿弯曲效果
//   5. 用类 FABRIK 算法迭代求解各关节位置
//   6. 将求解结果写回各段 widget 的屏幕位置
//
// 参数：
//   legIndex — 腿索引 (0=FL, 1=BL, 2=FR, 3=BR)
//   bodyPos  — MainWindow 左上角的屏幕坐标
//   footPos  — 该腿脚尖的目标屏幕坐标（由步态系统决定）

void MainWindow::solveIK(int legIndex, const QVector2D &bodyPos,
                          const QVector2D &footPos) {
    const LegLocalConfig &cfg = LEG_CONFIGS[legIndex];
    LegState &state = m_legStates[legIndex];

    // ── 第1步：收集该腿的所有段 widget ──
    // 每条腿有 kFeetPerLeg 个 SpiderFeet widget，按 "前缀+编号" 从哈希表中查找
    // segments[0] = 根部（靠近身体），segments[last] = 脚尖
    QVector<SpiderFeet*> segments;
    segments.reserve(SpiderTuning::kFeetPerLeg);
    for (int i = 0; i < SpiderTuning::kFeetPerLeg; ++i) {
        SpiderFeet *seg = spiderFeet.value(cfg.prefix + QString::number(i), nullptr);
        if (seg) segments.append(seg);
    }
    if (segments.isEmpty()) return;

    // ── 第2步：读取各关节当前屏幕位置作为 FABRIK 初始值 ──
    // 若某段尚未初始化（pos 为零向量），则回退到根部位置
    QVector<QVector2D> joints;
    joints.reserve(segments.size());
    for (SpiderFeet *seg : segments) {
        QVector2D p = seg->currentPos();
        if (p.isNull()) {
            p = state.currentRootPos;
        }
        joints.append(p);
    }

    // ── 第3步：计算该腿在身体圆角矩形上的锚点 ──
    // 将蜘蛛朝向角转为弧度，算出身体中心的屏幕坐标
    const float headingRad = qDegreesToRadians(m_heading);
    const QVector2D bodyCenter = bodyPos + QVector2D(this->width() * 0.5f,
                                                      this->height() * 0.5f);
    // 圆角矩形的半宽/半高 = 窗口尺寸 × kBodyRectScale 的一半
    const float halfW = this->width() * SpiderTuning::kBodyRectScale * 0.5f;
    const float halfH = this->height() * SpiderTuning::kBodyRectScale * 0.5f;
    // 在圆角矩形路径上按 perimeterFraction 采样，得到本地坐标和外法线
    LocalPoint lp = bodyRoundedRectPoint(cfg.perimeterFraction, halfW, halfH);
    // 将本地坐标旋转到屏幕空间，加上身体中心得到目标根部位置
    state.targetRootPos = bodyCenter + localToScreen(lp.right, lp.forward, headingRad);

    // ── 第4步：根部位置平滑跟随 ──
    // currentRootPos 以 kRootLerpSpeed 的比例向 targetRootPos 线性插值
    // 这使得转弯时根部不会瞬间跳到新锚点，而是平滑过渡
    state.currentRootPos = state.currentRootPos
        + (state.targetRootPos - state.currentRootPos) * SpiderTuning::kRootLerpSpeed;

    const QVector2D rootPos = state.currentRootPos;

    // ── 第5步：身体压扁（squish） ──
    // 将 IK 求解用的根部沿 根→脚 方向推进 kBodySquish 比例
    // 效果：缩短了根到脚的有效距离，FABRIK 为了让每段保持 segLen
    //       只能让中间关节向外弯折，从而产生蛛腿弯曲的自然效果
    const QVector2D ikRoot = rootPos + (footPos - rootPos) * SpiderTuning::kBodySquish;
    m_debugIkRoot[legIndex] = ikRoot;

    // 每段长度 = 总腿长 / (段数 - 1)，总腿长 = 所有段间距之和
    const float segLen = SpiderTuning::kLegLengthTotal / qMax(1, segments.size() - 1);
    const float totalLen = segLen * qMax(1, segments.size() - 1);

    // 计算锚点外法线方向（屏幕空间），用于 FABRIK 内部弯曲方向偏置
    QVector2D outwardNormal = localToScreen(lp.normalRight, lp.normalForward, headingRad);

    // ── 第6步：FABRIK 逆运动学求解 ──
    if ((footPos - ikRoot).length() >= totalLen) {
        // 情况A：脚尖超出最大伸展距离（根到脚距离 ≥ 总腿长）
        // 无法弯曲，所有关节沿根→脚方向拉直排列
        QVector2D dir = (footPos - ikRoot).normalized();
        if (dir.lengthSquared() < 1e-6f) dir = QVector2D(1.0f, 0.0f);
        joints[0] = ikRoot;
        for (int i = 1; i < joints.size(); ++i)
            joints[i] = joints[i - 1] + dir * segLen;
    } else {
        // 情况B：脚尖在可达范围内，执行 FABRIK 正反向迭代
        for (int it = 0; it < SpiderTuning::kIkIterations; ++it) {
            // ── 反向传递 ──
            joints.last() = footPos;
            for (int i = joints.size() - 2; i >= 0; --i) {
                QVector2D dir = (joints[i] - joints[i + 1]).normalized();
                if (dir.lengthSquared() < 1e-6f) dir = QVector2D(1.0f, 0.0f);
                joints[i] = joints[i + 1] + dir * segLen;
            }

            // ── 正向传递 ──
            joints.first() = ikRoot;
            for (int i = 1; i < joints.size(); ++i) {
                QVector2D dir = (joints[i] - joints[i - 1]).normalized();
                if (dir.lengthSquared() < 1e-6f) dir = QVector2D(1.0f, 0.0f);
                joints[i] = joints[i - 1] + dir * segLen;
            }

            // ── 弯曲方向偏置（pole target） ──
            // 每轮迭代后给中间关节施加一个朝外法线方向的小偏移
            // 持续引导求解器收敛到正确的弯曲方向，不会产生逐帧翻转抖动
            constexpr float kBendBias = 3.0f;  // 偏置强度(px)
            for (int i = 1; i < joints.size() - 1; ++i) {
                joints[i] += outwardNormal * kBendBias;
            }
        }
        // 经过 kIkIterations 轮迭代后，关节链满足：
        //   - 根部固定在 ikRoot
        //   - 相邻关节间距均为 segLen
        //   - 末端尽可能接近 footPos
    }

    // ── IK 后修正：将根关节拉回身体锚点并重新约束段长 ──
    // ikRoot 仅用于 IK 内部求解以产生弯曲，求解完成后把整条链锚定回 rootPos
    // 从根部做一次正向传递，保证每段长度仍为 segLen
    joints[0] = rootPos;
    for (int i = 1; i < joints.size(); ++i) {
        QVector2D dir = (joints[i] - joints[i - 1]).normalized();
        if (dir.lengthSquared() < 1e-6f) dir = QVector2D(1.0f, 0.0f);
        joints[i] = joints[i - 1] + dir * segLen;
    }

    // ── 第8步：将求解结果写回各段 widget ──
    // 更新每个 SpiderFeet 的逻辑位置和屏幕位置
    // move() 时减去半宽/半高，因为 joints 存的是段中心坐标
    for (int i = 0; i < segments.size(); ++i) {
        segments[i]->setCurrentPos(joints[i]);
        segments[i]->move(static_cast<int>(joints[i].x() - SpiderTuning::kFootW / 2.0f),
                          static_cast<int>(joints[i].y() - SpiderTuning::kFootH / 2.0f));
    }
}

// ─── Main animation loop ─────────────────────────────────────

void MainWindow::updateFeetPositions() {
    updateAutoWalk();

    const QVector2D bodyPos(this->pos().x(), this->pos().y());
    const QVector2D velocity = bodyPos - QVector2D(m_lastBodyPos);
    m_lastBodyPos = this->pos();

    ++m_frame;

    for (int i = 0; i < 4; ++i) {
        LegState &state = m_legStates[i];
        if (state.currentFootPos.isNull()) {
            state.currentFootPos = getIdealFootPos(i, bodyPos, velocity);
            state.targetFootPos  = state.currentFootPos;
            state.startStepPos   = state.currentFootPos;
        }

        const QVector2D idealPos = getIdealFootPos(i, bodyPos, velocity);
        const float dist = (state.currentFootPos - idealPos).length();

        // 步态：左前0+右后3, 左后1+右前2
        int group = (i == 0 || i == 3) ? 0 : 1;
        bool gaitGate = group == ((m_frame / 15) % 2);

        // 确保另一组腿已着地，才能迈步
        int otherA = (group == 0) ? 1 : 0;
        int otherB = (group == 0) ? 2 : 3;
        bool otherGrounded = !m_legStates[otherA].isStepping
                          && !m_legStates[otherB].isStepping;

        if (!state.isStepping
            && dist > SpiderTuning::kStepTriggerDist
            && gaitGate && otherGrounded) {
            state.isStepping    = true;
            state.startStepPos  = state.currentFootPos;
            state.targetFootPos = idealPos;
            state.stepProgress  = 0.0f;
        }

        if (state.isStepping) {
            state.stepProgress += SpiderTuning::kStepSpeed;
            if (state.stepProgress >= 1.0f) {
                state.stepProgress   = 1.0f;
                state.isStepping     = false;
                state.currentFootPos = state.targetFootPos;

                // 脚落地时给身体一个冲量
                QVector2D bodyCenter = bodyPos
                    + QVector2D(this->width() * 0.5f, this->height() * 0.5f);
                QVector2D toFoot = (state.targetFootPos - bodyCenter).normalized();
                m_bodyBob += toFoot * SpiderTuning::kBobStrength;
            } else {
                state.currentFootPos = bezierInterp(
                    state.startStepPos, state.targetFootPos,
                    state.stepProgress, SpiderTuning::kStepHeight);
            }
        }

        solveIK(i, bodyPos, state.currentFootPos);
    }

    update();  // 触发 paintEvent 重绘 debug 红框

    // debug: 每 ~5秒 输出一次坐标
    if (m_frame % 300 == 0) {
        qDebug() << "=== frame" << m_frame << "===";
        qDebug() << "  window pos:" << this->pos()
                 << " size:" << this->size()
                 << " heading:" << m_heading;
        for (int i = 0; i < 4; ++i) {
            qDebug().nospace()
                << "  leg[" << i << "]"
                << " rootPos=(" << m_legStates[i].currentRootPos.x()
                << "," << m_legStates[i].currentRootPos.y() << ")"
                << " ikRoot=(" << m_debugIkRoot[i].x()
                << "," << m_debugIkRoot[i].y() << ")"
                << " footPos=(" << m_legStates[i].currentFootPos.x()
                << "," << m_legStates[i].currentFootPos.y() << ")";
        }
    }
}

// ─── Perimeter walk + spring body ─────────────────────────────

void MainWindow::updateAutoWalk() {
    if (!ui->autoActionCheckbox->isChecked()) return;

    // 走走停停
    // float phase = qSin(m_frame * 0.012f);
    float phase = qSin(m_frame * 0.022f);
    float burst = qMax(0.0f, phase);
    float micro = 1.0f + 0.15f * qSin(m_frame * 0.07f);
    float speed = SpiderTuning::kWalkSpeed * burst * micro;

    // 沿周长推进
    float peri = perimeterLength();
    m_perimeterPos += speed * m_perimeterDir;
    if (m_perimeterPos >= peri) m_perimeterPos -= peri;
    if (m_perimeterPos < 0)    m_perimeterPos += peri;

    // 偶尔掉头
    if (QRandomGenerator::global()->bounded(2000) == 0) {
        m_perimeterDir *= -1.0f;
    }

    // 更新朝向：角度 lerp，处理 0°/360° 跨越
    m_targetHeading = computeHeadingFromPerimeter(m_perimeterPos);
    float diff = m_targetHeading - m_heading;
    // normalize to [-180, 180]
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    m_heading += diff * SpiderTuning::kHeadingLerpSpeed;
    // normalize heading to [0, 360)
    m_heading = fmod(m_heading, 360.0f);
    if (m_heading < 0) m_heading += 360.0f;

    QVector2D target = perimeterToBodyPos(m_perimeterPos);

    // 弹簧物理
    QVector2D current(this->pos().x(), this->pos().y());
    QVector2D springForce = (target - current) * SpiderTuning::kSpringStiffness;
    m_bodyVelocity = (m_bodyVelocity + springForce + m_bodyBob)
                   * SpiderTuning::kSpringDamping;
    m_bodyBob *= 0.85f;  // bob 冲量衰减

    QVector2D newPos = current + m_bodyVelocity;
    this->move(static_cast<int>(newPos.x()), static_cast<int>(newPos.y()));
}

// ─── Checkbox slots ───────────────────────────────────────────

void MainWindow::autoActionCheckboxSlot() {
    if (ui->autoActionCheckbox->isChecked()) {
        ui->manualActionCheckbox->setChecked(false);
    }
}

void MainWindow::manualActionCheckboxSlot() {
    if (ui->manualActionCheckbox->isChecked()) {
        ui->autoActionCheckbox->setChecked(false);
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QMainWindow::paintEvent(event);

    QPainter painter(this);
    painter.setPen(QPen(Qt::red, 2));
    painter.setBrush(Qt::NoBrush);

    const QPoint origin = this->mapToGlobal(QPoint(0, 0));
    const int hw = SpiderTuning::kFootW / 2;
    const int hh = SpiderTuning::kFootH / 2;

    for (int i = 0; i < 4; ++i) {
        // ikRoot 是屏幕坐标，转换为 MainWindow 本地坐标
        int lx = static_cast<int>(m_debugIkRoot[i].x()) - origin.x() - hw;
        int ly = static_cast<int>(m_debugIkRoot[i].y()) - origin.y() - hh;
        painter.drawRect(lx, ly, SpiderTuning::kFootW, SpiderTuning::kFootH);
    }

    // ideal root foot pos
    int winWidth = this->width();
    int winHeight = this->height();

    int rectW = static_cast<int>(winWidth * 0.8);
    int rectH = static_cast<int>(winHeight * 0.8);

    int rectX = (winWidth - rectW) / 2;
    int rectY = (winHeight - rectH) / 2;

    painter.setPen(QPen(Qt::blue, 2, Qt::DashLine));
    painter.drawRect(rectX, rectY, rectW, rectH);
}

void MainWindow::resizeEvent(QResizeEvent *event) {
    QMainWindow::resizeEvent(event);
    if (m_videoWidget && centralWidget()) {
        m_videoWidget->setGeometry(0, 0, centralWidget()->width(), centralWidget()->height());
    }
}
