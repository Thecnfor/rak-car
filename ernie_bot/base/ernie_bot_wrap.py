# -*- coding: utf-8 -*-
import erniebot, json
from jsonschema import validate


class PromptJson:
	def __init__(self, rulers) -> None:
		self.rulers_str = '请根据下面的schema描述生成给定格式json,只返回json数据,不要其他内容。'
		self.schema_str = ''
		self.example_str = ''

		self.set_rulers(rulers)
		self.set_scheame(self.json_obj())
		self.set_example(self.example())

	def json_obj(self):
		return '''```{'type':'string'}```'''

	def example(self):
		return '正确的示例如下：'
	
	def __call__(self, *args, **kwargs):
		pass
    
	def set_scheame(self, json_obj):
		# json转字符串去空格,换行，制表符
		json_str = str(json_obj).replace(' ', '').replace('\n', '').replace('\t', '')
		# 加上三个引号
		json_str = '```' + json_str + '```'
		self.schema_str = json_str

	def set_example(self, example_str:str):
		# 去空格,换行，制表符
		example_str = example_str.replace(' ', '').replace('\n', '').replace('\t', '')
		self.example_str = example_str

	def set_rulers(self, rulers):
		self.rulers_str = rulers.replace(' ', '').replace('\n', '').replace('\t', '')

	def __str__(self) -> str:
		return self.__repr__()
	
	def __repr__(self) -> str:
		return self.rulers_str + self.schema_str + self.example_str
	
class ActionPrompt(PromptJson):
	def __init__(self) -> None:
		
		rulers = '''你是一个机器人动作规划者，需要把我的话翻译成机器人动作规划并生成对应的json结果，机器人工作空间参考右手坐标系。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)
		# self.set_rulers(rulers)
		# self.set_scheame(self.json_obj())
		# self.set_example(self.example())

	def json_obj(self)->dict:
		schema_move = {'type': 'object', 'required': ['func', 'x', 'y', 'angle'],
               'porperties':{
                                'func': {'description': '移动', 'const': 'move'},
                                'x': {'description': 'x坐标, 前后移动, 向前移动正值，向后移动负值', 'type': 'number'},
                                'y': {'description': 'y坐标, 左右移动, 向左移动正值，向右移动负值', 'type': 'number'}, 
                                'angle': {'description': '旋转或者转弯角度，右转顺时针负值，左转逆时针正值', 'type': 'number'}
                            }
            }
		schema_beep = { 'type': 'object', 'required': ['func', 'time_dur'],
				'properties': {'func': {'description': '蜂鸣器,需要发声时', 'const': 'beep'}, 
                   'time_dur': {'description': '蜂鸣器发声持续时间', 'type': 'number'}}
		}

		schema_light = { 'type': 'object', 'required': ['func', 'time_dur'],
						'properties': {'func': {'description': '发光,需要照明时', 'const': 'light'}, 
						'time_dur': {'description': '照亮持续时间', 'type': 'number'}}
		}
		schema_actions = {'type': 'array', 'required': ['items'],
                  'items': {'anyOf': [schema_move, schema_beep, schema_light],
                        'minItems': 1
                    }
		}
		return schema_actions
	
	def example(self)->str:
		example = '''正确的示例如下：
					向左移0.1m, 向左转弯85度: ```[{'func': 'move', 'x': 0, 'y': 0.1, 'angle': 85}]```,
					向右移0.2m, 向前0.1m。 ```[{'func': 'move', 'x': 0.1, 'y': -0.2, 'angle': 0}]```,
					向右转弯85度, 向右移0.1m,。 ```[{'func': 'move', 'x': 0, 'y': -0.1, 'angle': -85}]```,
					蜂鸣器发声5秒。 ```[{'func': 'beep', 'time_dur': 5}]```,
					发光5秒。 ```[{'func': 'light', 'time_dur': 5}]```。
				'''
		return example
	
class HumAttrPrompt(PromptJson):
	def __init__(self) -> None:
		rulers = '''你是一个人特征总结程序，需要根据描述把人的特征生成对应的json结果，如果有对应的描述就写入对应位置。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)

	def json_obj(self)->dict:
		'''
		0 = Hat - 帽子:0无1有
		1 = Glasses - 眼镜:0无1有
		2 = ShortSleeve - 短袖
		3 = LongSleeve - 长袖
		4 = UpperStride - 有条纹
		5 = UpperLogo - 印有logo/图案
		6 = UpperPlaid - 撞色衣服(多种颜色)
		7 = UpperSplice - 格子衫
		8 = LowerStripe - 有条纹
		9 = LowerPattern - 印有图像
		10 = LongCoat - 长款大衣
		11 = Trousers - 长裤
		12 = Shorts - 短裤
		13 = Skirt&Dress - 裙子/连衣裙
		14 = boots - 鞋子
		15 = HandBag - 手提包
		16 = ShoulderBag - 单肩包
		17 = Backpack - 背包
		18 = HoldObjectsInFront - 手持物品
		19 = AgeOver60 - 大于60
		20 = Age18-60 - =18~60
		21 = AgeLess18 - 小于18
		22 = Female - 0:男性; 1:女性
		23 = Front - 人体朝前
		24 = Side - 人体朝侧
		25 = Back - 人体朝后
		'''
		schema_attr = {'type': 'object', 
                'properties':{
                    'hat':{'type': 'boolean', 'description': '戴帽子真,没戴帽子假'},
					'glasses': {"type": 'boolean', 'description': '戴眼镜真,没戴眼镜假', 'threshold':0.15},
					'sleeve':{'enum': ['Short', 'Long'], 'description': '衣袖长短'},
					# 'UpperStride', 'UpperLogo', 'UpperPlaid', 'UpperSplice'	有条纹		印有logo/图案	撞色衣服(多种颜色) 格子衫
					'color_upper':{'enum':['Stride', 'Logo', 'Plaid', 'Splice'], 'description': '上衣衣服颜色'},
					# 'LowerStripe', 'LowerPattern'		有条纹		印有图像
					'color_lower':{'enum':['Stripe', 'Pattern'], 'description': '下衣衣服长短'},
					# 'LongCoat', 长款大衣
					'clothes_upper':{'enum':['LongCoat'], 'description': '上衣衣服类型', 'threshold':0.8},
					# 'Trousers', 'Shorts', 'Skirt&Dress'  长裤		短裤 	裙子/连衣裙
					'clothes_lower':{'enum':['Trousers', 'Shorts', 'Skirt_dress'], 'description': '下衣衣服类型'},
					'boots':{'type': 'boolean', 'description': '穿着鞋子真,没穿鞋子假'},
					'bag':{'enum': ['HandBag', 'ShoulderBag', 'Backpack'], 'description': '带着包的类型'},
					'holding':{'type': 'boolean', 'description': '持有物品为真', 'threshold':0.5},
					'age':{'enum': ['Old', 'Middle', 'Young'], 'description': '年龄,小于18岁为young, 18到60为middle, 大于60为old'},
					'sex':{'enum': ['Female', 'Male'], 'threshold':0.6},
					'direction':{'enum': ['Front', 'Side', 'Back'], 'description': '人体朝向'},
					},
                "additionalProperties": False
            }
		return schema_attr
	
	def example(self)->str:
		example = '''正确的示例如下：
					一个带着眼镜的老人: ```{'glasses': True, 'age': 'old'}```,
					一个带着帽子的中年人: ```{'hat': True, 'age': 'middle'}``` ,
					穿着短袖的带着眼镜的人: ```{'glasses': True, 'clothes': 'short'}``` 。
				'''
		return example




class EduCounselerPrompt(PromptJson):
	def __init__(self) -> None:
		rulers = '''你是一个人中小学指导程序，需要根据描述的题目，逐步进行推理，根据给出的选项选择出正确答案的json结果。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)
	
	def json_obj(self)->dict:
		schema_edu = {
						"type": "object", "required": ['answer', 'analysis'],
						"properties": {
							"analysis":{'type':"string", "description":"题目分析的具体过程,分析的过程少于20字"},
							"answer": {'enum': ['A', 'B', 'C', "D"], "description": "答案选项中的一个"}
						},
						"additionalProperties": False
            		 }
		return schema_edu
	
	def example(self)->str:
		example = '''正确的示例如下：
					题目: 1+1=？, 答案选项有: A.4 B.44 C.7 D.2 ```{"description":"1+1的结果是2,其中选项D和答案一致,所以选D",'answer': 'D'}``` ,
					题目: 1+2=？, 答案选项有: A.3 B.44 C.6 D.2: ```{"description":"1+2的结果是3,其中选项A和答案一致,所以选A",'answer': 'A'}``` 。'''
		return example




class FoodGetPrompt(PromptJson):
	def __init__(self) -> None:
		rulers = '''你是一个通过食物描述推断食物的程序，需要根据描述的题目，逐步进行推理，根据给出的选项选择出正确答案的json结果。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)

	def json_obj(self) -> dict:
		schema_food = {
			"type": "object", "required": ['answer', 'analysis'],
						"properties": {
							"analysis":{'type':"string", "description":"题目分析的具体过程,分析的过程少于20字"},
							"answer": {'enum': ["chicken", "tofu", "celery", "meat", "chili", "tomato", "potato", "cauliflower", "egg", "mushroom", "greens", "green_beans","wax_gourd","eggplant","sparerib","cabbage"], "description": "答案选项中的一个"}
						},
						"additionalProperties": False
					   }
		return schema_food

	def example(self) -> str:
		example = '''正确的示例如下：
						题目:  是一种圆形或椭圆形的浆果，成熟时多为红色，可以直接生食，烹饪```{"description":",'answer': 'tomato'}``` ,
						题目:  有一层坚硬的外壳，中心部分呈黄色或橙色，富含蛋白质，可以水煮，煎，炒，烘培食用```{"description":",'answer': 'egg'}```
						'''
		return example


class BMIAnaPrompt(PromptJson):
    def __init__(self) -> None:
        rulers = '''你是一个BMI计算的程序，需要根据描述的文字，计算出正确的BMI值，返回正确答案的json结果。
						严格按照下面的scheame描述生成给定格式json，只返回json数据:
					'''
        super().__init__(rulers)

    def json_obj(self) -> dict:
        schema_food = {
            "type": "object", "required": ['answer', 'analysis'],
            "properties": {
                "analysis": {'type': "string", "description": "题目分析的具体过程,分析的过程少于20字"},
                "answer": {
                    'enum': [1,2,3,4], "description": "1. BMI<18.5  2. 18.5≤BMI≤24  3. 24<BMI≤28  4. BMI>28  选择其中的一个并将其代表的序号返回"}
            },
            "additionalProperties": False
        }
        return schema_food
    def example(self) -> str:
        example = '''正确的示例如下：
					题目: 性别:男 年龄：30岁 身高：180cm 体重：65kg ```{"description":"计算得到bim值大约为20.06",'answer': '2'}``` ,
					'''
        return example





class FoodPutPrompt(PromptJson):
	def __init__(self) -> None:
		rulers = '''你是一个通过两种食材推断菜品（菜品选项是菜品的描述）的程序，需要根据描述的题目，逐步进行推理，根据给出的选项选择出正确答案的json结果。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)

	def json_obj(self) -> dict:
		schema_food = {
			"type": "object", "required": ['answer', 'analysis'],
						"properties": {
							"analysis":{'type':"string", "description":"题目分析的具体过程,分析的过程少于20字"},
							"answer": { "description": "答案选项中的一个"}
						},
						"additionalProperties": False
					   }
		return schema_food

	def example(self) -> str:
		example = '''正确的示例如下：
					题目: tomato,egg  选项 1.表面油亮，整体呈现鲜亮的绿色，口感清脆，带有淡淡的蔬菜清甜，味道清淡爽口。2.深绿色的切片与浅棕色的肉块混合，肉片略带焦香，带有明显的辛辣味，咸香嫩滑整体口感鲜辣开胃间的色彩。3.表面裹着红亮的酱汁呈深褐色，表面略带光泽，酸甜味突出外酥里嫩，酱汁浓郁，口感层次丰富。4.金黄色与鲜红色混合食材软烂出汁，整体呈现红黄相间的色彩。```{"description":"通过描述可知该菜品为番茄炒蛋",'answer':4 }``` ,
					题目: chili,meat  选项 1.表面油亮，整体呈现鲜亮的绿色，口感清脆，带有淡淡的蔬菜清甜，味道清淡爽口。2.深绿色的切片与浅棕色的肉块混合，肉片略带焦香，带有明显的辛辣味，咸香嫩滑整体口感鲜辣开胃间的色彩。3.表面裹着红亮的酱汁呈深褐色，表面略带光泽，酸甜味突出外酥里嫩，酱汁浓郁，口感层次丰富。4.金黄色与鲜红色混合食材软烂出汁，整体呈现红黄相间的色彩。```{"description":"通过描述可知该菜品为辣椒炒肉",'answer':2 }``` ,
					'''
		return example
class ErnieBotWrap():

	def __init__(self):
		erniebot.api_type = 'aistudio'
		erniebot.access_token = '0feeac9b5c673b6b23a48e27836785c6c25341e3'
		models = erniebot.Model.list()
		print(models)
		
		self.msgs = []
		# self.model = 'ernie-4.0'
		self.model = 'ernie-3.5'
		# self.model = 'ernie-turbo'
		# self.model = "ernie-text-embedding"
		# self.model = "ernie-vilg-v2"
		self.prompt_str = '请根据下面的描述生成给定格式json'

	@staticmethod
	def get_mes(role, dilog):
		data = {}
		if role == 0:
			data['role'] = 'user'
		elif role ==1:
			data['role'] = 'assistant'
		data['content'] = dilog	
		return data

	def set_promt(self, prompt_str):
		# str_input = prompt_str
		# self.msgs.append(self.get_mes(0, str_input))
		# response = erniebot.ChatCompletion.create(model=self.model, messages=self.msgs, system=prompt_str)
		# str_res = response.get_result()
		# self.msgs.append(self.get_mes(1, str_res))
		# print(str_res)
		# print("设置成功")
		self.prompt_str = prompt_str
		# print(self.prompt_str)


	def get_res(self, str_input, record=False, request_timeout=5):
		if len(str_input)<1:
			return False, None
		start_str = " ```"
		end_str = " ```, 根据这段描述生成给定格式json"
		str_input = start_str + str_input + end_str
		msg_tmp = self.get_mes(0, str_input)
		if record:
			self.msgs.append(msg_tmp)
			msgs = self.msgs
		else:
			msgs = [msg_tmp]
		# Create a chat completion
		try:
			response = erniebot.ChatCompletion.create(model=self.model, messages=msgs, system=self.prompt_str, top_p=0.1,
											_config_=dict(api_type="AISTUDIO",), request_timeout=request_timeout)
			# print(response)
		except Exception as e:
			# print(e)
			return False, None
		# _config_=dict(api_type="QIANFAN",)
		# _config_=dict(api_type="AISTUDIO",)
		# print(response)
		str_res = response.get_result()
		if record:
			self.msgs.append(self.get_mes(1, str_res))
		return True, str_res
	
	@staticmethod
	def get_json_str(json_str:str):
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
			'''
			try:
				index_s = json_str.find("```json") + 7
				# index_s = json_str.find("```json") + 7
			except Exception as e:
				index_s = 0
			try:
				index_e = json_str[index_s:].find("```") + index_s
			except Exception as e:
				index_e = len(json_str)
			import json
			msg_json = json.loads(json_str[index_s:index_e])
			return msg_json
			'''
	
	def get_res_json(self, str_input, record=False, request_timeout=10):
		state, str_res = self.get_res(str_input, record, request_timeout)
		if state:
			# print(str_res)
			obj_json = self.get_json_str(str_res)
			return obj_json
		else:
			return None

# def test():
# 	res = '''```json\n[\n  {\n    "func": "my_light",\n    "count": 3\n  },\n  {\n    "func": "beep",\n    "time_dur": 3  // 假设蜂鸣器持续发声3秒作为紧急警示，具体时长可根据实际情况调整\n  }\n]\n```'''
# 	json_test = ErnieBotWrap.get_json_str(res)
# 	print(json_test)



# if __name__ == "__main__":
# 	# test()
# 	str_input = ''' 如果买满200元可优惠40元。购买了3件商品,价格分别为85元、130元和115元。最终支付多少钱?	选项有: A.300元 B.180元 C.130元 D.290元'''
# 	erniebot.api_type = 'aistudio'
# 	erniebot.access_token = '0feeac9b5c673b6b23a48e27836785c6c25341e3'
# 	# ernie_edu = EduCounselerPrompt()
#
# 	ernie = ErnieBotWrap()
# 	# 设置prompt
# 	ernie.set_promt(str(HumAttrPrompt()))
# 	#ernie.set_promt(str(EduCounselerPrompt()))
# 	json_res = ernie.get_res_json(str_input)
# 	print(json_res)
# 	# while True:
# 	# 	print("用户")
# 	# 	str_tmp = input("输入:")
# 	# 	if len(str_tmp)<1:
# 	# 		continue
# 	# 	# Create a chat completion
# 	# 	print("文心一言")
# 	# 	# _, str_res = ernie.get_res(str_tmp)
# 	# 	json_res = ernie.get_res_json(str_tmp)
# 	# 	print("输出:",json_res)
	
