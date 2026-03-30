
# import json
# import os
# import time
# import threading
# import requests
# import traceback

# #  Описание доступных функций для LLM
# FUNCTIONS_DESCRIPTION = """
# Доступные функции (вызывай ТОЛЬКО их):

# 1. silero_tts.say(text, voice="eugene")
#    Озвучить текст. Голоса: "aidar"(муж), "baya"(жен), "kseniya"(жен), "eugene"(муж), "xenia"(жен).
#    Поддерживает SSML: <speak><prosody rate="slow">медленно</prosody><break time="500ms"/></speak>
#    Ударение: символ '+' перед гласной: "з+амок" (крепость), "зам+ок" (дверной).

# 2. ears_driver.ears_set_angle(left=0, right=0, duration=1.0)
#    Установить углы ушей. left и right: от -90 до 90 градусов. duration: секунды.
#    Положительный угол поворачивает уши вперед, отрицательный угол поворачивает уши назад.

# 3. neck_driver.neck_set_angle(vertical=0, horizontal=0, duration=1.0)
#    Установить углы шеи. vertical: от -30 до 30 (наклон). horizontal: от -30 до 30 (поворот). duration: секунды.
#    vertical > 0 поднимает голову вверх, vertical < 0 опускает голову вниз.
#    horizontal > 0 поворачивает голову вправо, horizontal < 0 поворачивает голову влево.

# 4. media_driver.play_audio(path, loop=false)
#    Воспроизвести аудиофайл или URL. loop: зациклить.

# 5. media_driver.play_display(path, loop=false)
#    Показать изображение/видео на дисплее. loop: зациклить.

# 6. media_driver.set_volume(volume)
#    Установить громкость. volume: от 0 до 100.

# 7. sleep(duration)
#    Пауза. duration: секунды.
# """


# SYSTEM_PROMPT = f"""
# Ты — робот «Робоголов+а. Уверенный, ироничный, умный. Разработан в НИИ Механики МГУ.

# Твоя задача — преобразовать команду пользователя в последовательность вызовов функций.
# Отвечай JSON-объектом с полем "actions" — массивом вызовов.

# {FUNCTIONS_DESCRIPTION}

# Правила:
# - Отвечай JSON: {{"actions":[{{"function":"имя","args":{{...}}}}]}}
# - Если нужно что-то сказать — используй функцию say().
# - Все числа в тексте say() пиши словами.
# - Не используй смайлики и спецсимволы в тексте say().
# - Не выдумывай функции, которых нет в списке.
# - Если команда непонятна — вызови say() с объяснением.
# - Делай действия выразительными: комбинируй движения ушей, шеи и речь.

# Пример для "Привет":
# {{"actions":[
#   {{"function":"neck_set_angle","args":{{"vertical":15,"horizontal":0,"duration":0.5}}}},
#   {{"function":"ears_set_angle","args":{{"left":20,"right":20,"duration":0.5}}}},
#   {{"function":"say","args":{{"text":"Привет! Рад тебя видеть!"}}}}
# ]}}
# """

# FORMAT_SCHEMA = {
#     "type": "object",
#     "properties": {
#         "actions": {
#             "type": "array",
#             "minItems": 1,
#             "maxItems": 10,
#             "items": {
#                 "type": "object",
#                 "properties": {
#                     "function": {
#                         "type": "string",
#                         "enum": [
#                             "silero_tts.say", "ears_driver.ears_set_angle", "neck_driver.neck_set_angle",
#                             "media_driver.play_audio", "media_driver.play_display", "media_driver.set_volume",
#                             "sleep",
#                         ]
#                     },
#                     "args": {
#                         "type": "object"
#                     }
#                 },
#                 "required": ["function", "args"]
#             }
#         }
#     },
#     "required": ["actions"]
# }

# MODEL_URL = "http://robohead0llm.local:11434/api/generate"
# MODEL_NAME = "qwen3:4b-instruct"

# prompt = "Привет, расскажи о себе"

# payload = {
#     'model': MODEL_NAME,
#     'prompt': prompt,
#     'system': SYSTEM_PROMPT,
#     'stream': False,
#     'format': FORMAT_SCHEMA,         
#     'think': False,             
#     'options': {
#         'temperature': 0.3,
#         'num_predict': 1024,
#     }
# }


# # response = requests.post(MODEL_URL, json=payload, timeout=120)
# # response.raise_for_status()

# # llm_answer = response.json().get('response', '')
# llm_answer = """{"actions":[
#   {"function":"neck_driver.neck_set_angle","args":{"vertical":15,"horizontal":0,"duration":0.5}},
#   {"function":"ears_driver.ears_set_angle","args":{"left":20,"right":20,"duration":0.5}},
#   {"function":"silero_tts.say","args":{"text":"Привет! Я — Робоголов+а, разработанный в НИИ Механики МГУ. Я умею говорить, слушать, двигаться и вести диалог. Могу озвучивать текст, управлять движением головы и ушей, воспроизводить аудио и видео. Если хочешь, я могу рассказать о себе подробнее или помочь с чем-то интересным! Как я могу тебе помочь сегодня?"}}
# ]}"""
# print(f"Raw answer: {llm_answer}")

# # Теперь гарантированно валидный JSON
# result = json.loads(llm_answer)
# actions = result["actions"]

# class MediaDriver:
#     def set_volume(self, volume):
#         print(volume)
#     def play_display(self, path, loop=False):
#         print(path, loop)
#     def play_audio(self, path, loop=False):
#         print(path, loop)

# class SileroTTS:
#     def say(self, text, voice="eugene"):
#         print(text, voice)
# class EarsDriver:
#     def ears_set_angle( self,left=0, right=0, duration=1.0):
#         print(left, right, duration)
# class NeckDriver:
#     def neck_set_angle(self, vertical=0, horizontal=0, duration=1.0):
#         print(vertical, horizontal, duration)

# class ControllerAI:
#     def __init__(self):
#         self.media_driver = MediaDriver()
#         self.silero_tts = SileroTTS()
#         self.ears_driver = EarsDriver()
#         self.neck_driver = NeckDriver()
#     def sleep(self, duration):
#         print(duration)

# def get_nested_attr(obj, name):
#     for part in name.split('.'):
#         obj = getattr(obj, part)
#     return obj

# controller = ControllerAI()

# for action in actions:
#     func_name = action["function"]
#     args = action["args"]
#     print(f"  → {func_name}({args})")

#     method = get_nested_attr(controller, func_name)
#     if callable(method):
#         method(**args)
#     else:
#         print("Метод не найден")


import json

raw =  """{"actions":[{"function":"silero_tts.say","args":{"text":"Извини, я не понял твою команду. Пожалуйста, повтори или скажи, что тебе нужно.","block":true}}]}"""

d = json.loads(raw)
print(d)