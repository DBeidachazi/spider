#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QtMath>
#include <QVector2D>
#include <QScreen>
#include <QGuiApplication>
#include <QRandomGenerator>

namespace SpiderTuning {
constexpr int kFeetPerLeg = 12;          // 每条腿多少节
constexpr int kFootW = 80;
constexpr int kFootH = 50;
constexpr int kAnimIntervalMs = 16;

// IK
constexpr float kLegLengthTotal = 220.0f;
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
constexpr float kRootLerpSpeed = 0.12f;      // 根部跟随速度
constexpr float kLegAngleThreshold = 150.0f; // 腿最大偏转角度
constexpr float kBodyRectScale = 0.80f;      // 圆角矩形为窗口的80%
constexpr float kCornerRadiusFraction = 0.35f; // 圆角半径占短边比例
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
        foot->stackUnder(this);
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

    // 圆角矩形上的锚点 + 外法线
    const float halfW = this->width() * SpiderTuning::kBodyRectScale * 0.5f;
    const float halfH = this->height() * SpiderTuning::kBodyRectScale * 0.5f;
    LocalPoint lp = bodyRoundedRectPoint(cfg.perimeterFraction, halfW, halfH);

    // 旋转到屏幕空间
    const QVector2D anchor = bodyCenter + localToScreen(lp.right, lp.forward, headingRad);
    const QVector2D reachDir = localToScreen(lp.normalRight, lp.normalForward, headingRad);

    const QVector2D prediction = dirVelocity * SpiderTuning::kVelocityPredict;
    QVector2D rawPos = anchor + reachDir * SpiderTuning::kFootReach + prediction;

    // 投射到最近的屏幕边框
    QVector2D projected = projectToScreenEdge(rawPos);

    // 角度阈值钳制
    QVector2D toFoot = projected - anchor;
    float footDist = toFoot.length();
    if (footDist > 1e-3f) {
        float footAngle = qRadiansToDegrees(qAtan2(-toFoot.y(), toFoot.x()));
        float reachAngle = qRadiansToDegrees(qAtan2(-reachDir.y(), reachDir.x()));
        float angleDiff = footAngle - reachAngle;
        while (angleDiff > 180.0f) angleDiff -= 360.0f;
        while (angleDiff < -180.0f) angleDiff += 360.0f;

        if (qAbs(angleDiff) > SpiderTuning::kLegAngleThreshold) {
            float clampedAngle = reachAngle
                + qBound(-SpiderTuning::kLegAngleThreshold,
                         angleDiff,
                         SpiderTuning::kLegAngleThreshold);
            float clampedRad = qDegreesToRadians(clampedAngle);
            QVector2D clampedDir(qCos(clampedRad), -qSin(clampedRad));
            projected = anchor + clampedDir * footDist;
            projected = projectToScreenEdge(projected);
        }
    }

    return projected;
}

// ─── IK solver (unchanged) ────────────────────────────────────

void MainWindow::solveIK(int legIndex, const QVector2D &bodyPos,
                          const QVector2D &footPos) {
    const LegLocalConfig &cfg = LEG_CONFIGS[legIndex];
    LegState &state = m_legStates[legIndex];

    QVector<SpiderFeet*> segments;
    segments.reserve(SpiderTuning::kFeetPerLeg);
    for (int i = 0; i < SpiderTuning::kFeetPerLeg; ++i) {
        SpiderFeet *seg = spiderFeet.value(cfg.prefix + QString::number(i), nullptr);
        if (seg) segments.append(seg);
    }
    if (segments.isEmpty()) return;

    QVector<QVector2D> joints;
    joints.reserve(segments.size());
    for (SpiderFeet *seg : segments) {
        QVector2D p = seg->currentPos();
        if (p.isNull()) {
            p = state.currentRootPos;
        }
        joints.append(p);
    }

    // 圆角矩形锚点 → 目标根部
    const float headingRad = qDegreesToRadians(m_heading);
    const QVector2D bodyCenter = bodyPos + QVector2D(this->width() * 0.5f,
                                                      this->height() * 0.5f);
    const float halfW = this->width() * SpiderTuning::kBodyRectScale * 0.5f;
    const float halfH = this->height() * SpiderTuning::kBodyRectScale * 0.5f;
    LocalPoint lp = bodyRoundedRectPoint(cfg.perimeterFraction, halfW, halfH);
    state.targetRootPos = bodyCenter + localToScreen(lp.right, lp.forward, headingRad);

    // 平滑跟随
    state.currentRootPos = state.currentRootPos
        + (state.targetRootPos - state.currentRootPos) * SpiderTuning::kRootLerpSpeed;

    const QVector2D rootPos = state.currentRootPos;
    const float segLen = SpiderTuning::kLegLengthTotal / qMax(1, segments.size() - 1);
    const float totalLen = segLen * qMax(1, segments.size() - 1);

    if ((footPos - rootPos).length() >= totalLen) {
        QVector2D dir = (footPos - rootPos).normalized();
        if (dir.lengthSquared() < 1e-6f) dir = QVector2D(1.0f, 0.0f);
        joints[0] = rootPos;
        for (int i = 1; i < joints.size(); ++i)
            joints[i] = joints[i - 1] + dir * segLen;
    } else {
        for (int it = 0; it < SpiderTuning::kIkIterations; ++it) {
            joints.last() = footPos;
            for (int i = joints.size() - 2; i >= 0; --i) {
                QVector2D dir = (joints[i] - joints[i + 1]).normalized();
                if (dir.lengthSquared() < 1e-6f) dir = QVector2D(1.0f, 0.0f);
                joints[i] = joints[i + 1] + dir * segLen;
            }
            joints.first() = rootPos;
            for (int i = 1; i < joints.size(); ++i) {
                QVector2D dir = (joints[i] - joints[i - 1]).normalized();
                if (dir.lengthSquared() < 1e-6f) dir = QVector2D(1.0f, 0.0f);
                joints[i] = joints[i - 1] + dir * segLen;
            }
        }
    }

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

    this->raise();
}

// ─── Perimeter walk + spring body ─────────────────────────────

void MainWindow::updateAutoWalk() {
    if (!ui->autoActionCheckbox->isChecked()) return;

    // 走走停停
    float phase = qSin(m_frame * 0.012f);
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
