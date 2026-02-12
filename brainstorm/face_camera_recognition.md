# 摄像头人脸识别 + MainWindow 内切换人脸视频（设计草案）

> 约束：当前不改代码；这里只做实现思路/技术选型/模块拆分，后续再按此落地。

## 目标
1. 从摄像头实时获取画面。
2. 在画面中检测人脸（可多脸）。
3. 对每张脸做“识别/身份确认”（不是仅检测）。
4. 根据识别结果，把对应的“人脸视频/素材”切换显示到 MainWindow 中（例如识别到 A 就播放 A 的视频）。

## 推荐总体架构（Pipeline）
1. **采集**：Qt Multimedia 获取摄像头帧（QCamera + QVideoSink）。
2. **检测**：人脸检测模型（输出 bbox/landmarks）。
3. **跟踪**：多目标跟踪（减少每帧都跑检测/避免ID跳变）。
4. **对齐裁剪**：根据 landmarks 做仿射对齐，得到规范人脸 patch。
5. **特征提取**：人脸识别模型（embedding，典型 128/512 维）。
6. **比对/判决**：embedding 与本地人脸库做相似度（cosine）匹配 + 阈值。
7. **UI 切换**：把“识别到的身份 -> 对应视频源”映射到 MainWindow 播放控件；加入去抖/稳定逻辑。

## 是否用 YOLO？（结论：YOLO 可用但只负责“检测”）
- **YOLO（如 YOLOv8-face / YOLOv5-face）适合做人脸检测**：快、部署简单。
- 但“识别身份”通常需要 **ArcFace/FaceNet 这类识别网络** 来产出 embedding。
- 所以更常见组合是：
  - **YOLO/RetinaFace/MediaPipe**（检测+关键点） + **ArcFace**（识别）。

### 备选方案对比（Linux/Wayland 桌面）
- **MediaPipe Face Detection + Face Mesh**：关键点稳，工程集成稍重，但效果好。
- **RetinaFace**：检测强、关键点质量好，推理一般比 YOLO 稍重。
- **OpenCV Haar/LBP**：集成最简单但效果弱（光照/角度/遮挡容易崩）。

## 模型与推理后端建议
- 推理后端优先级：
  1. **ONNX Runtime**（CPU/GPU 都可，工程化友好）
  2. **OpenCV DNN**（少依赖但性能/算子支持可能受限）
  3. **TensorRT**（NVIDIA GPU 最强，但部署成本高）

### 推荐模型组合（实用路线）
- 检测：YOLOv8-face（ONNX）或 RetinaFace（ONNX）
- 识别：ArcFace (ResNet100/50)（ONNX，输出 512-d embedding）

## 人脸库（Identity DB）怎么做
- 给每个人：存 3~10 张不同角度的“注册照片”。
- 对每张注册照跑识别模型，得到 embeddings，存本地：
  - SQLite：`person_id, embedding_vector, meta`
- 识别时：对每张脸 embedding，与库做 cosine similarity：
  - `sim = dot(e, db_e) / (||e||*||db_e||)`
  - 取最大 sim，对比阈值（例如 0.35~0.55 需现场标定）。

## 跟踪与稳定（避免疯狂切视频）
必须做，否则多脸/抖动会导致 UI 频繁切换。
- **Tracker**：KCF/CSRT/ByteTrack/DeepSORT（按复杂度选择）。
- **识别稳定**：
  - 同一 track 连续 N 帧识别为同一人，才确认身份。
  - 身份确认后，设置“保持时间”T（例如 1~2s）再允许切换。
  - 多个人同时出现：定义优先级规则：
    - 最大脸（bbox 面积最大）
    - 或最靠近屏幕中心
    - 或相似度最高

## MainWindow 内“切人脸视频”的实现思路（UI 层）
可选三种：
1. **QMediaPlayer + QVideoWidget**：最省事，播放本地 mp4/网络流。
2. **QStackedWidget**：为每个人准备一个播放器页，切换 currentIndex。
3. **自绘渲染**：用 QOpenGLWidget/QGraphicsView 把视频帧当纹理贴图（更自由）。

### 关键：把“识别到的人”映射到“视频源”
- 配置表（json/yaml/sqlite都行）：
  - `person_id -> video_path_or_url`
- UI 逻辑：
  - `active_person_id` 变化时，停止旧视频、启动新视频（或预加载无缝切换）。

## 性能与线程
- 不要在 UI 线程跑推理。
- 推荐：
  - 采集线程/Qt 信号把帧送到 **Worker(QThread)**
  - Worker 负责检测/识别/跟踪
  - 只把“结果（bbox + person_id + 置信度）”发回 UI。
- 帧率策略：
  - 摄像头 30fps，但检测可降频到 10~15fps，tracker 负责补间。

## 误识别与隐私
- 增加“未知”类别：低于阈值不切换。
- 明确告知用户摄像头使用、是否存储图片、如何删除人脸库。

## 里程碑（落地时的建议顺序）
1. 仅采集并在 MainWindow 显示摄像头画面（不推理）。
2. 加人脸检测框（无识别）。
3. 加识别（embedding + 本地库）并输出 person_id。
4. 加稳定/跟踪（解决抖动切换）。
5. 最后接入“person_id -> 视频源”并做切换动画/预加载。
