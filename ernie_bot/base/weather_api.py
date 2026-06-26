import requests
import logging


WEATHER_API_KEY = "b49609e52282ed4c5ba1e546cf7f7e39"

class Weather:
    def __init__(self, api_key = WEATHER_API_KEY):
        self.api_key = api_key
        self.base_url = "https://restapi.amap.com/v3"
    
    def get_weather_by_city(self, city):
        url = f"{self.base_url}/weather/weatherInfo"
        params = {
            'key': self.api_key,
            'city': city,
            'extensions': 'base'  # base: 实况天气
        }
        
        try:
            response = requests.get(url, params=params)
            response.raise_for_status()
            data = response.json()
            
            if data.get('status') == '1' and data.get('lives'):
                return data['lives'][0]
            else:
                return {'error': data.get('info', '获取天气信息失败')}
                
        except requests.exceptions.RequestException as e:
            return {'error': f'网络请求失败: {str(e)}'}
        except json.JSONDecodeError:
            return {'error': '解析响应数据失败'}


    def get_weather_num(self, weather):
        if weather == '晴':
            return 1
        elif weather == '阴' or weather == '多云':
            return 2
        elif weather == '小雨' or weather == '中雨':
            return 3
        else:
            return 4