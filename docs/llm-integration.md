# 大模型集成

## 概述

项目集成了 3 种大语言模型后端，用于 OCR 文本理解、食材识别、BMI 计算等任务。

## 后端对比

| 后端 | 类 | 模型 | API | 配置文件 |
|------|---|------|-----|---------|
| 百度文心 | `ErnieBotWrap` | ernie-3.5 | erniebot SDK | — |
| DeepSeek | `OpenAiWrap` | deepseek-chat | OpenAI 兼容 | `answer.py` |
| 阿里通义 | `OpenAiWrap` | qwen-plus | OpenAI 兼容 | `gpt_bot_wrap.py` |

## API 密钥（⚠️ 硬编码在源码中）

| 后端 | 位置 | 密钥 |
|------|------|------|
| 百度 AI Studio | `ernie_bot_wrap.py:272` | `0feeac9b...` |
| DeepSeek | `answer.py:12` | `sk-628347019c...` |
| 阿里 DashScope | `gpt_bot_wrap.py` | `sk-62acf89ce...` |
| 百度 OCR | `car_wrap.py:950-951` | `js7RZ6BHSIK...` |
| 高德天气 | `weather_api.py:5` | `b49609e522...` |

**建议：** 迁移到 `.env` 文件。

## Prompt 工程系统

### PromptJson 基类

所有 Prompt 继承自 `PromptJson`，构造结构化请求：

```python
class PromptJson:
    def __init__(self):
        self.rulers = "系统指令（中文）"
        self.schema = "JSON Schema（输出格式）"
        self.examples = "Few-shot 示例"

    def get_prompt(self, user_input):
        return f"""
{self.rulers}

## 输出格式
```json
{self.schema}
```

## 示例
{self.examples}

## 用户输入
{user_input}
"""
```

### 现有 Prompt 类

| 类 | 功能 | 输入 | 输出 Schema |
|----|------|------|------------|
| `ActionPrompt` | 自然语言→机器人动作 | 中文指令 | `[{func, x, y, angle, ...}]` |
| `FoodGetPrompt` | 描述→食材名称 | OCR 文本 | `{food: "egg"}` |
| `FoodPutPrompt` | 两道食材→菜名 | 食材名 × 2 | `{dish: "番茄炒蛋"}` |
| `BMIAnaPrompt` | BMI 数据→分类 | 身高体重 | `{category: 1~4}` |
| `EduCounselerPrompt` | K12 选择题 | 题目文本 | `{answer: "A", analysis: "..."}` |
| `HumAttrPrompt` | 人物属性总结 | 属性文本 | `{hat, glasses, sleeve, ...}` |

### JSON 提取

LLM 返回的 JSON 通常包裹在 markdown 代码块中：

```
```json
{"food": "egg"}
```
```

`get_json_str()` 用正则提取三引号之间的内容。

## 竞赛用 AI 接口

### answer.py (DeepSeek) / answer_wenxin.py (文心)

提供统一的竞赛 AI 接口：

```python
def ask1(ocr_text):
    """根据 OCR 描述识别食材 → 返回食材名称"""

def ask2(food1, food2, dish_text):
    """根据两道食材和菜描述 → 返回菜名"""

def ask3(height, weight, bmi_value):
    """根据 BMI 数据 → 返回分类 (1~4)"""
```

**⚠️ `answer.py` 中使用 `eval(answer)` 解析 LLM 返回值，有代码执行风险！**

## 天气 API

`ernie_bot/base/weather_api.py` 使用高德地图 API 获取实时天气：

```python
url = f"https://restapi.amap.com/v3/weather/weatherInfo?key={API_KEY}&city=110000"
```

返回天气编码，映射到舵机位置显示。
