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
} // namespace SpiderTuning

// Leg configuration: anchor ratios + arc scan angles (degrees)
struct LegConfig {
    QString prefix;
    double anchorXRatio;  // proportional x on mainwindow
    double anchorYRatio;  // proportional y on mainwindow
    double startAngleDeg;
    double endAngleDeg;
};

static const LegConfig LEG_CONFIGS[] = {
    {"leftFrontFeet_",  0.0, 0.70, 175.0, 245.0},
    {"leftBackFeet_",   0.0, 0.85, 180.0, 250.0},
    {"rightFrontFeet_", 1.0, 0.70, 5.0, -65.0},
    {"rightBackFeet_",  1.0, 0.85, 0.0, -70.0},
};

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

// ─── Leg segment init (structure unchanged) ───────────────────

void MainWindow::initSpiderFeet() {
    int W = this->width();
    int H = this->height();
    const double step = SpiderTuning::kFootW * 0.22;

    for (const auto &cfg : LEG_CONFIGS) {
        const QPointF attachPoint(W * cfg.anchorXRatio, H * cfg.anchorYRatio);
        double cx = 0.0, cy = 0.0;

        for (int i = 0; i < SpiderTuning::kFeetPerLeg; ++i) {
            auto *foot = new SpiderFeet(this);
            foot->setFixedSize(SpiderTuning::kFootW, SpiderTuning::kFootH);

            int ox = static_cast<int>(attachPoint.x() + cx - SpiderTuning::kFootW / 2.0);
            int oy = static_cast<int>(attachPoint.y() + cy - SpiderTuning::kFootH / 2.0);
            foot->setOffset(QPoint(ox, oy));
            this->spiderFeet.insert(cfg.prefix + QString::number(i), foot);

            if (i < SpiderTuning::kFeetPerLeg - 1) {
                const double t = (SpiderTuning::kFeetPerLeg > 1)
                                     ? double(i) / (SpiderTuning::kFeetPerLeg - 1)
                                     : 0.0;
                const double angleDeg = cfg.startAngleDeg
                                      + (cfg.endAngleDeg - cfg.startAngleDeg) * t;
                const double angleRad = qDegreesToRadians(angleDeg);
                cx += step * qCos(angleRad);
                cy -= step * qSin(angleRad);
            }
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

    // 初始化脚的位置
    QVector2D bodyPos(this->pos().x(), this->pos().y());
    for (int leg = 0; leg < 4; ++leg) {
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
    const LegConfig &cfg = LEG_CONFIGS[legIndex];

    // 肩膀位置
    const float shoulderX = bodyPos.x() + float(this->width()  * cfg.anchorXRatio);
    const float shoulderY = bodyPos.y() + float(this->height() * cfg.anchorYRatio);
    const QVector2D shoulder(shoulderX, shoulderY);

    // 原始偏移方向
    const float angleDeg = float((cfg.startAngleDeg + cfg.endAngleDeg) * 0.5);
    const float angleRad = qDegreesToRadians(angleDeg);
    const QVector2D offset(qCos(angleRad) * SpiderTuning::kFootReach,
                           -qSin(angleRad) * SpiderTuning::kFootReach);
    const QVector2D prediction = dirVelocity * SpiderTuning::kVelocityPredict;

    QVector2D rawPos = shoulder + offset + prediction;

    // 投射到最近的屏幕边框
    return projectToScreenEdge(rawPos);
}

// ─── IK solver (unchanged) ────────────────────────────────────

void MainWindow::solveIK(int legIndex, const QVector2D &bodyPos,
                          const QVector2D &footPos) {
    const LegConfig &cfg = LEG_CONFIGS[legIndex];
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
            p = QVector2D(bodyPos.x() + this->width() * cfg.anchorXRatio,
                          bodyPos.y() + this->height() * cfg.anchorYRatio);
        }
        joints.append(p);
    }

    const QVector2D rootPos(bodyPos.x() + this->width() * cfg.anchorXRatio,
                            bodyPos.y() + this->height() * cfg.anchorYRatio);
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
