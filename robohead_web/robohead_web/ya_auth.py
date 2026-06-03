#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import threading
from pathlib import Path
from yandex_music import Client

class YandexMusicAuth:
    def __init__(self):
        self.env_file = Path.home() / '.env'
        self.auth_sessions = {}
        self._lock = threading.Lock()

    def get_token(self) -> str:
        """Читает токен из ~/.env"""
        if not self.env_file.exists():
            return ""
        try:
            with open(self.env_file, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('YANDEX_MUSIC_TOKEN='):
                        return line.split('=', 1)[1].strip()
        except Exception as e:
            print(f"[ya_auth] Ошибка чтения .env: {e}")
        return ""

    def save_token(self, token: str) -> bool:
        """Сохраняет или обновляет токен в ~/.env"""
        token = token.strip()
        if not token:
            return False
        try:
            lines = []
            token_updated = False
            
            if self.env_file.exists():
                with open(self.env_file, 'r', encoding='utf-8') as f:
                    for line in f:
                        if line.startswith('YANDEX_MUSIC_TOKEN='):
                            lines.append(f'YANDEX_MUSIC_TOKEN={token}\n')
                            token_updated = True
                        else:
                            lines.append(line)
                            
            if not token_updated:
                if lines and not lines[-1].endswith('\n'):
                    lines[-1] += '\n'
                lines.append(f'YANDEX_MUSIC_TOKEN={token}\n')
                
            with open(self.env_file, 'w', encoding='utf-8') as f:
                f.writelines(lines)
                
            os.chmod(self.env_file, 0o600)
            return True
        except Exception as e:
            print(f"[ya_auth] Ошибка сохранения .env: {e}")
            return False

    def init_auth(self) -> dict:
        """Запускает Device Auth flow в фоне"""
        poll_id = str(int(time.time() * 1000))
        
        def on_code(code_obj):
            """Библиотека передаёт объект с атрибутами user_code и verification_url"""
            with self._lock:
                self.auth_sessions[poll_id] = {
                    'user_code': code_obj.user_code,
                    'verification_url': code_obj.verification_url,
                    'status': 'waiting'
                }

        def run_auth():
            try:
                client = Client()
                token_obj = client.device_auth(on_code=on_code)
                
                if token_obj and token_obj.access_token:
                    self.save_token(token_obj.access_token)
                    with self._lock:
                        if poll_id in self.auth_sessions:
                            self.auth_sessions[poll_id]['status'] = 'ready'
                            self.auth_sessions[poll_id]['token'] = token_obj.access_token
                else:
                    with self._lock:
                        if poll_id in self.auth_sessions:
                            self.auth_sessions[poll_id]['status'] = 'error'
                            self.auth_sessions[poll_id]['error'] = 'Не удалось получить токен'
            except Exception as e:
                with self._lock:
                    if poll_id in self.auth_sessions:
                        self.auth_sessions[poll_id]['status'] = 'error'
                        self.auth_sessions[poll_id]['error'] = str(e)

        thread = threading.Thread(target=run_auth, daemon=True)
        thread.start()
        
        # Ждём до 30 секунд, пока библиотека сгенерирует код и вызовет on_code
        for _ in range(30):
            with self._lock:
                if poll_id in self.auth_sessions:
                    return {
                        'poll_id': poll_id,
                        'user_code': self.auth_sessions[poll_id]['user_code'],
                        'verification_url': self.auth_sessions[poll_id]['verification_url']
                    }
            time.sleep(1)
            
        return {'error': 'Таймаут ожидания кода авторизации'}

    def poll_auth(self, poll_id: str) -> dict:
        """Проверяет статус конкретной сессии авторизации"""
        with self._lock:
            if poll_id not in self.auth_sessions:
                return {'ready': False, 'error': 'Неверный poll_id'}
            
            session = self.auth_sessions[poll_id]
            if session['status'] == 'ready':
                return {'ready': True, 'token': session.get('token')}
            elif session['status'] == 'error':
                return {'ready': False, 'error': session.get('error')}
            else:
                return {'ready': False}

# Глобальный экземпляр
ya_auth = YandexMusicAuth()