import re
from ernie_bot import OpenAiWrap,ActionPrompt,EduCounselerPrompt,HumAttrPrompt,FoodGetPrompt,FoodPutPrompt,BMIAnaPrompt
from openai import OpenAI
import json
#openai.api_key = "sk-XOszJEbvpM2tgsOK09B1EaBf6b034fD39379Cc84AcB3DfBa"
# all client options can be configured just like the `OpenAI` instantiation counterpart
#openai.base_url = "https://free.v36.cm/v1/"
#openai.default_headers = {"x-foo": "true"}

client = OpenAI(
# 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx",
api_key="sk-628347019c98415cab239b5e31394ac0",
base_url="https://api.deepseek.com/beta",
)




def get_json_str(json_str: str):
  try:
    index_s = json_str.find("```json")
    if index_s == -1:
      index_s = json_str.find("```")
      if index_s == -1:
        return None
      else:
        index_s += 3

    else:
      index_s += 7
    # print(json_str[index_s:])
    index_e = json_str[index_s:].find("```") + index_s
    if index_e == -1:
      return None
    # json_str = json_str[index_s:index_e]
    # print(json_str[index_s:index_e])
    # print(index_s, index_e)
    json_str = json_str[index_s:index_e]
    # 找到注释内容并删除
    json_str.replace("\n", "")
    # print(json_str)
    msg_json = json.loads(json_str)
    return msg_json
  # print(index_s)
  # return json_str
  except Exception as e:
    # print(e)
    return json_str


def ask(text):
	prompt = "回答问题，输出问题的答案，目标是水果，只输出答案名字，无需解释"
	completion = client.chat.completions.create(
	model="deepseek-chat",
	messages=[
	 {"role": "system", "content": prompt},
	 {"role": "user", "content":text },
	],
	
	# Qwen3模型通过enable_thinking参数控制思考过程（开源版默认True，商业版默认False）
	# 使用Qwen3开源版模型时，若未启用流式输出，请将下行取消注释，否则会报错
	# extra_body={"enable_thinking": False},
	)
	answer=completion.choices[0].message.content
	
	#gpt = OpenAiWrap()
	# 设置prompt
	#gpt.set_promt(str(FoodGetPrompt()))
	#json_res = get_res_json(answer)
	
	return answer



def ask1(text):
  promt =str(FoodGetPrompt())
  completion = client.chat.completions.create(
    # 模型列表：https://help.aliyun.com/zh/model-studio/getting-started/models
    model="deepseek-chat",
    messages=[
      {"role": "system", "content": promt},
      {"role": "user", "content":text },
    ],
    # Qwen3模型通过enable_thinking参数控制思考过程（开源版默认True，商业版默认False）
    # 使用Qwen3开源版模型时，若未启用流式输出，请将下行取消注释，否则会报错
    # extra_body={"enable_thinking": False},
  )
  answer=completion.choices[0].message.content

  #gpt = OpenAiWrap()
  # 设置prompt
  #gpt.set_promt(str(FoodGetPrompt()))
  #json_res = get_res_json(answer)
  
  return eval(answer)


#str_input = '''是一种圆形或椭圆形的浆果，成熟时多为红色，可以直接生食，烹饪'''
#answer=ask1(str_input)
#print(answer)


def ask2(text):
  promt = str(FoodPutPrompt())
  completion = client.chat.completions.create(
    # 模型列表：https://help.aliyun.com/zh/model-studio/getting-started/models
    model="deepseek-chat",
    messages=[
      {"role": "system", "content": promt},
      {"role": "user", "content": text},
    ],
    # Qwen3模型通过enable_thinking参数控制思考过程（开源版默认True，商业版默认False）
    # 使用Qwen3开源版模型时，若未启用流式输出，请将下行取消注释，否则会报错
    # extra_body={"enable_thinking": False},
  )
  answer = completion.choices[0].message.content

  # gpt = OpenAiWrap()
  # 设置prompt
  # gpt.set_promt(str(FoodGetPrompt()))
  # json_res = get_res_json(answer)
  return eval(answer)


def ask3(text):
  promt = str(BMIAnaPrompt())
  completion = client.chat.completions.create(
    # 模型列表：https://help.aliyun.com/zh/model-studio/getting-started/models
    model="deepseek-chat",
    messages=[
      {"role": "system", "content": promt},
      {"role": "user", "content": text},
    ],
    # Qwen3模型通过enable_thinking参数控制思考过程（开源版默认True，商业版默认False）
    # 使用Qwen3开源版模型时，若未启用流式输出，请将下行取消注释，否则会报错
    # extra_body={"enable_thinking": False},
  )
  answer = completion.choices[0].message.content

  # gpt = OpenAiWrap()
  # 设置prompt
  # gpt.set_promt(str(FoodGetPrompt()))
  # json_res = get_res_json(answer)
  return eval(answer)

text="身高是1.75m 体重是65kg"
# print("开始运行\n")
answer=ask3(text)
# print(answer)





