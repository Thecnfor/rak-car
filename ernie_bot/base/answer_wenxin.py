import re
from ernie_bot import ErnieBotWrap, HumAttrPrompt, ActionPrompt,EduCounselerPrompt,FoodGetPrompt,FoodPutPrompt,BMIAnaPrompt

from openai import OpenAI
import json
#openai.api_key = "sk-XOszJEbvpM2tgsOK09B1EaBf6b034fD39379Cc84AcB3DfBa"
# all client options can be configured just like the `OpenAI` instantiation counterpart
#openai.base_url = "https://free.v36.cm/v1/"
#openai.default_headers = {"x-foo": "true"}





def ask1(text):
  ernie = ErnieBotWrap()
  # print(str(FoodGetPrompt()))
  ernie.set_promt(str(FoodGetPrompt()))
  # ernie.set_promt(str(EduCounselerPrompt()))
  json_res = ernie.get_res_json(text)
  return json_res


# str_input = '''是一种圆形或椭圆形的浆果，成熟时多为红色，可以直接生食，烹饪'''
# answer=ask1(str_input)
# print(answer)





def ask2(text):
  ernie = ErnieBotWrap()
  ernie.set_promt(str(FoodPutPrompt()))
  # ernie.set_promt(str(EduCounselerPrompt()))
  json_res = ernie.get_res_json(text)
  return json_res

# text="tomato,egg  选项 1.表面油亮，整体呈现鲜亮的绿色，口感清脆，带有淡淡的蔬菜清甜，味道清淡爽口。2.深绿色的切片与浅棕色的肉块混合，肉片略带焦香，带有明显的辛辣味，咸香嫩滑整体口感鲜辣开胃间的色彩。3.表面裹着红亮的酱汁呈深褐色，表面略带光泽，酸甜味突出外酥里嫩，酱汁浓郁，口感层次丰富。4.金黄色与鲜红色混合食材软烂出汁，整体呈现红黄相间的色彩。"
# a=ask2(text)
# print(a)



def ask3(text):
  ernie = ErnieBotWrap()
  ernie.set_promt(str(BMIAnaPrompt()))
  # ernie.set_promt(str(EduCounselerPrompt()))
  json_res = ernie.get_res_json(text)
  return json_res

# text="身高是1.75m 体重是65kg"
# answer=ask3(text)
# print(answer)