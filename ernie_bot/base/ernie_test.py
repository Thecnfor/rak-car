import json
import re
import erniebot
erniebot.api_type = 'aistudio'
erniebot.access_token = '0feeac9b5c673b6b23a48e27836785c6c25341e3'
messages=[{'role': 'user', 'content': '''“这个水果是苹果,id记录为123456, 价格是5人民币每500克”, 请根据我的描述生成一段json结果,json数据参考如下的jsonschema的描述。
      ```{ "title": "Product",
      "description": "一个商品的目录",
      "type": "object",
      "properties": {"productId": { "description": "商品唯一识别码", "type": "integer"},
        "productName": {"description": "商品名称","type": "string"},
        "price": {"description": "商品的价格","type": "number","exclusiveMinimum": 0}
      },
      "required": [ "productId", "productName", "price" ]
      }```'''}]
#response= erniebot.ChatCompletion.create(model='ernie-3.5', messages=messages)
#first_response = response.get_result()
#print(first_response)
def answer(text):
    messages2 = [{
        'role': 'user',
        'content': f'''{text}+
{{
    "title": "food",
    "description": "符合食物的描述,只返回json数据格式的内容,只返回一种食物",
    "option": "tofu,tomato,chili,chicken,meat,celery,egg,mushroom,green_beans,potato,cauliflower,greens",
    "type": "object",
    "required": [ "foodname"]
}}
'''
    }]
    response = erniebot.ChatCompletion.create(model='ernie-3.5', messages=messages2)
    first_response = response.get_result()

    match = re.search(r'"foodname"\s*:\s*"([^"]+)"', first_response)
    if match:
        return match.group(1)
    else:
        return None