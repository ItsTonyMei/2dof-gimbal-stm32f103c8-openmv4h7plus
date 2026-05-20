# CrowdHuman YOLOv11n → STM32N6 NPU 真 int8 量化导出指南 (Quantization Export Guide)

> **已废弃 (Deprecated)。** 当前项目使用 ROM 内置的 BlazeFace (186KB/128×128)，不再使用 YOLO。
> 此文档保留作为量化导出参考。

## 背景 (Background)

模型 (Model): YOLOv11n, CrowdHuman head+person 双类 (2 Classes), 320×320, int8 量化 (Quantization)。
后处理 (Postprocessing) 兼容 `ml.postprocessing.ultralytics.YoloV8`（YOLOv11 输出格式与 YOLOv8 一致）。

当前 `crowdhuman_head_person_int8.tflite` 文件**名**为 int8，实际是 float32 模型。ST Edge AI Core 编译结果为 pseudoint8，NPU 无法使用。

### 证据 (Evidence) — 来自 ST Edge AI Core 编译输出

```text
model_fmt    :   float                         ← 编译器识别为 float
input        :   f32(1x320x320x3), 1200 KB     ← int8 输入应为 300 KB
macc         :   0                              ← 无法分析计算量（int8 才能统计）
WARNING      :   conv2d_1 is not quantized     ← 全部 150+ 个算子未量化
WARNING      :   nl_2 is not quantized
...
epochs       :   337 total / 44 NPU / 274 SW    ← 仅 13% 跑在 NPU 上
```

TFLite 文件内部：输入输出层标为 `uint8`，但中间所有 Conv、Sigmoid、Mul 算子全是 float32 — 这是 **hybrid quantization（混合量化）**，NPU 无法加速。

### 对比：正常工作的 int8 量化模型 (Working int8 Model)

真正全 int8 量化 (Full-Integer Quantization) 的模型，ST 编译器会识别为 int8，几乎所有算子都跑在 NPU 上。

---

## 导出步骤 (Export Steps)

项目定位：OpenMV N6 云台 (Gimbal)，YOLOv11n，CrowdHuman 数据集 (Dataset)，head + person 双类，320×320 输入。

### 前提条件 (Prerequisites)

在训练机上需要：

- 训练好的 `.pt` 权重文件 (Weights)
- CrowdHuman 数据集的 `dataset.yaml`（用于量化校准 / Quantization Calibration）
- `ultralytics >= 8.0.0`（YOLOv11n 导出，ultralytics 8.x 支持 v11）

### 步骤 1 (Step 1)：准备量化校准数据 (Prepare Calibration Data)

确保 `dataset.yaml` 中包含 `val` 路径，指向至少 200-300 张有代表性的 CrowdHuman 图片（用于 int8 量化校准的激活值统计 / Activation Statistics）：

```yaml
# dataset.yaml 示例
path: /path/to/crowdhuman
train: images/train
val: images/val          # ← 必须有，用于 int8 校准 (Calibration)
nc: 2
names: ['head', 'person']
```

### 步骤 2 (Step 2)：导出真 int8 TFLite (Export True int8)

```python
from ultralytics import YOLO

# 加载训练好的权重 (Load Weights)
model = YOLO("crowdhuman_head_person.pt")

# 关键：int8=True + data= 数据集 yaml（校准必须）
model.export(
    format="tflite",
    int8=True,                              # 全整数量化 (Full-Integer Quantization)
    data="dataset.yaml",                    # 校准数据集（必须！否则只量化 I/O）
    imgsz=320,
    nms=False,                              # NMS 在 OpenMV 侧做
    batch=1,
)
```

**`data` 参数是核心 (Key)**：没有它，ultralytics 无法收集激活值统计 (Activation Statistics)，只能做 hybrid quantization（I/O 量化 + 内部 float32），这正是当前模型的问题。

### 步骤 3 (Step 3)：验证导出结果 (Verify Export)

导出完成后，用 ST Edge AI Core 快速验证（无需实际烧录到 N6）：

```bash
# 路径按你的 OpenMV IDE 安装位置调整
stedgeai.exe generate \
  --model crowdhuman_head_person_int8.tflite \
  --target stm32n6 \
  --st-neural-art default@neuralart.json \
  --relocatable
```

**验证通过的标准 (Pass Criteria)：**

| 指标 (Metric) | ❌ 失败 (Fail, 当前) | ✅ 成功 (Pass) |
| ------ | -------------- | -------- |
| `model_fmt` | `float` | `int8` |
| input | `f32`, 1200 KB | `s8/u8`, ~300 KB |
| macc | `0` | > 0 (有实际值) |
| WARNING "not quantized" | 150+ 行 | **0 行**（无此警告） |
| NPU 利用率 (Utilization) | ~13% | > 80% |

### 步骤 4 (Step 4)：复制到 OpenMV 项目 (Copy to Project)

将验证通过的 `.tflite` 文件和对应的 `.txt` 标签文件 (Label File) 复制到：

```text
openmv/crowdhuman_head_person_int8.tflite
openmv/crowdhuman_head_person_int8.txt     # 内容: head\nperson
```

---

## 常见问题 (FAQ)

### Q: 已经用了 `int8=True`，为什么还是 float？

`int8=True` 不加 `data=xxx.yaml` 时，ultralytics 只做 I/O 量化（input/output 标 uint8，内部仍是 float32）。**必须同时传 `data` 参数**才能触发全量激活值校准 (Full-Integer Quantization)。

### Q: 校准需要多少图片 (How Many Images)？

200-300 张足够。图片应覆盖典型场景（不同光照、距离、人群密度）。

### Q: 可以用 COCO 数据集做校准吗？

可以，但 CrowdHuman 的图片分布不同（更密集的人群），用 CrowdHuman 的 val 集校准效果更好。

### Q: 目标设备和当前 yolo 版本？

- 目标 NPU：STM32N6 Neural-ART (ST Edge AI Core v3.0.0)
- OpenMV N6 firmware：最新版
- 后处理 (Postprocessing)：`ml.postprocessing.ultralytics.YoloV8`（兼容 YOLOv11n 输出格式）
- 模型结构 (Model Structure)：YOLOv11n, 320×320×3 输入, [1,6,2100] 输出 (2 类)
