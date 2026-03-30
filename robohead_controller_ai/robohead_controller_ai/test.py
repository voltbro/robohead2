
import json
import os
import time
import threading
import requests
import traceback

#  Описание доступных функций для LLM
FUNCTIONS_DESCRIPTION = """
Доступные функции (вызывай ТОЛЬКО их):

1. say(text, voice="eugene")
   Озвучить текст. Голоса: "aidar"(муж), "baya"(жен), "kseniya"(жен), "eugene"(муж), "xenia"(жен).
   Поддерживает SSML: <speak><prosody rate="slow">медленно</prosody><break time="500ms"/></speak>
   Ударение: символ '+' перед гласной: "з+амок" (крепость), "зам+ок" (дверной).

2. ears_set_angle(left=0, right=0, duration=1.0)
   Установить углы ушей. left и right: от -90 до 90 градусов. duration: секунды.
   Положительный угол поворачивает уши вперед, отрицательный угол поворачивает уши назад.

3. neck_set_angle(vertical=0, horizontal=0, duration=1.0)
   Установить углы шеи. vertical: от -30 до 30 (наклон). horizontal: от -30 до 30 (поворот). duration: секунды.
   vertical > 0 поднимает голову вверх, vertical < 0 опускает голову вниз.
   horizontal > 0 поворачивает голову вправо, horizontal < 0 поворачивает голову влево.

4. play_audio(path, loop=false)
   Воспроизвести аудиофайл или URL. loop: зациклить.

5. play_display(path, loop=false)
   Показать изображение/видео на дисплее. loop: зациклить.

6. set_volume(volume)
   Установить громкость. volume: от 0 до 100.

7. sleep(duration)
   Пауза. duration: секунды.

8. led_set_color(red, green, blue)
   Установить цвет всех светодиодов. red, green, blue: 0-255.

9. led_set_mode(mode)
   Режим светодиодов. 0=выкл, 1=trace/doa, 2=listen/rainbow, 3=wait/single, 4=speak/breathe, 5=spin/ring.

10. led_set_brightness(brightness)
    Яркость светодиодов. 0-31 или 0-255 (зависит от модели).
"""


SYSTEM_PROMPT = f"""
Ты — робот «Робоголов+а. Уверенный, ироничный, умный. Разработан в НИИ Механики МГУ.

Твоя задача — преобразовать команду пользователя в последовательность вызовов функций.
Отвечай JSON-объектом с полем "actions" — массивом вызовов.

{FUNCTIONS_DESCRIPTION}

Правила:
- Отвечай JSON: {{"actions":[{{"function":"имя","args":{{...}}}}]}}
- Если нужно что-то сказать — используй функцию say().
- Все числа в тексте say() пиши словами.
- Не используй смайлики и спецсимволы в тексте say().
- Не выдумывай функции, которых нет в списке.
- Если команда непонятна — вызови say() с объяснением.
- Делай действия выразительными: комбинируй движения ушей, шеи и речь.

Пример для "Привет":
{{"actions":[
  {{"function":"neck_set_angle","args":{{"vertical":15,"horizontal":0,"duration":0.5}}}},
  {{"function":"ears_set_angle","args":{{"left":20,"right":20,"duration":0.5}}}},
  {{"function":"say","args":{{"text":"Привет! Рад тебя видеть!"}}}}
]}}
"""

FORMAT_SCHEMA = {
    "type": "object",
    "properties": {
        "actions": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "function": {
                        "type": "string",
                        "enum": [
                            "say", "ears_set_angle", "neck_set_angle",
                            "play_audio", "play_display", "set_volume",
                            "sleep", "led_set_color", "led_set_mode",
                            "led_set_brightness"
                        ]
                    },
                    "args": {
                        "type": "object"
                    }
                },
                "required": ["function", "args"]
            }
        }
    },
    "required": ["actions"]
}

MODEL_URL = "http://robohead0llm-2.local:11434/api/generate"
MODEL_NAME = "qwen3:4b-instruct"

prompt = "Привет, расскажи о себе"

payload = {
    'model': MODEL_NAME,
    'prompt': prompt,
    'system': SYSTEM_PROMPT,
    'stream': False,
    'format': FORMAT_SCHEMA,         
    'think': False,             
    'options': {
        'temperature': 0.3,
        'num_predict': 1024,
    }
}


response = requests.post(MODEL_URL, json=payload, timeout=120)
response.raise_for_status()

llm_answer = response.json().get('response', '')
print(f"Raw answer: {llm_answer}")

# Теперь гарантированно валидный JSON
result = json.loads(llm_answer)
actions = result["actions"]

for action in actions:
    func_name = action["function"]
    args = action["args"]
    print(f"  -> {func_name}({args})")