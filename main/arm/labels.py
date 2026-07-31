"""业务目标类别 catalog —— 对齐 task backend 模型输出 (20 项)。

设计：
  - `Label` 是 str 子类 Enum，可直接当 str 传给 runtime
  - `LABELS` 是 (id, name, desc) 元组列表，按 id 升序（用户给定格式）
  - `LABEL_GROUPS` 是自然分组（animal / ball / cylinder / vegetable / water）
"""
from dataclasses import dataclass
from enum import Enum
from typing import Tuple, Dict


@dataclass(frozen=True)
class LabelInfo:
    id: int
    name: str
    desc: str

    def __str__(self) -> str: return f"Label({self.name})"


class Label(str, Enum):
    ANIMAL        = "animal"
    BALL_BLUE     = "ball_blue"
    BALL_YELLOW   = "ball_yellow"
    CYLINDER_1    = "cylinder_1"
    CYLINDER_2    = "cylinder_2"
    CYLINDER_3    = "cylinder_3"
    CYLINDER_SET  = "cylinder_set"
    H_DOU_JIAO    = "h_dou_jiao"
    H_FAN_QIE     = "h_fan_qie"
    H_JIN_ZHEN_GU = "h_jin_zhen_gu"
    H_MO_GU       = "h_mo_gu"
    H_QIN_CAI     = "h_qin_cai"
    H_QING_JIAO   = "h_qing_jiao"
    H_TU_DOU      = "h_tu_dou"
    H_XI_LAN_HUA  = "h_xi_lan_hua"
    H_YOU_CAI     = "h_you_cai"
    WATER         = "water"
    WATER_L1      = "water_l1"
    WATER_L2      = "water_l2"
    WATER_L3      = "water_l3"


LABELS: Tuple[LabelInfo, ...] = (
    LabelInfo(1,  "animal",        "动物"),
    LabelInfo(2,  "ball_blue",     "蓝色球"),
    LabelInfo(3,  "ball_yellow",   "黄色球"),
    LabelInfo(4,  "cylinder_1",    "圆柱体（1号）"),
    LabelInfo(5,  "cylinder_2",    "圆柱体（2号）"),
    LabelInfo(6,  "cylinder_3",    "圆柱体（3号）"),
    LabelInfo(7,  "cylinder_set",  "圆柱体组合"),
    LabelInfo(8,  "h_dou_jiao",    "豆角"),
    LabelInfo(9,  "h_fan_qie",     "番茄"),
    LabelInfo(10, "h_jin_zhen_gu", "金针菇"),
    LabelInfo(11, "h_mo_gu",       "蘑菇"),
    LabelInfo(12, "h_qin_cai",     "芹菜"),
    LabelInfo(13, "h_qing_jiao",   "青椒"),
    LabelInfo(14, "h_tu_dou",      "土豆"),
    LabelInfo(15, "h_xi_lan_hua",  "西兰花"),
    LabelInfo(16, "h_you_cai",     "油菜"),
    LabelInfo(17, "water",         "水容器"),
    LabelInfo(18, "water_l1",      "水容器（等级1）"),
    LabelInfo(19, "water_l2",      "水容器（等级2）"),
    LabelInfo(20, "water_l3",      "水容器（等级3）"),
)


LABEL_GROUPS: Dict[str, Tuple[Label, ...]] = {
    "animal":    (Label.ANIMAL,),
    "ball":      (Label.BALL_BLUE, Label.BALL_YELLOW),
    "cylinder":  (Label.CYLINDER_1, Label.CYLINDER_2, Label.CYLINDER_3),
    "cylinder_meta": (Label.CYLINDER_SET,),
    "vegetable": (Label.H_DOU_JIAO, Label.H_FAN_QIE, Label.H_JIN_ZHEN_GU,
                  Label.H_MO_GU, Label.H_QIN_CAI, Label.H_QING_JIAO,
                  Label.H_TU_DOU, Label.H_XI_LAN_HUA, Label.H_YOU_CAI),
    "water":     (Label.WATER, Label.WATER_L1, Label.WATER_L2, Label.WATER_L3),
}


def get_label_info(name: str) -> LabelInfo:
    """label 名 → LabelInfo；不在表里抛 ValueError"""
    for info in LABELS:
        if info.name == name:
            return info
    raise ValueError(f"未知 label: {name!r}（共 20 项，参考 LABELS）")


def is_in_group(name: str, group: str) -> bool:
    """name 是否在 group 内；name 不在 20 项内返回 False 而非 raise"""
    try:
        return Label(name) in LABEL_GROUPS.get(group, ())
    except ValueError:
        return False