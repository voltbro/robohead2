#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import logging
from pathlib import Path
from ruamel.yaml import YAML

logger = logging.getLogger(__name__)

class ConfigManager:
    def __init__(self, config_dir: str):
        self.config_dir = Path(config_dir)
        self.yaml = YAML()
        self.yaml.preserve_quotes = True
        self.yaml.indent(mapping=2, sequence=4, offset=2)
        # Отключаем сортировку ключей при дампе
        self.yaml.sort_base_mapping_type_on_output = False

    def get_param(self, file_name: str, param_path: str):
        yaml_path = self._resolve_path(file_name)
        if not yaml_path.exists():
            logger.error(f"Файл конфига не найден: {yaml_path}")
            return None
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = self.yaml.load(f)
            return self._navigate(data or {}, param_path)
        except Exception as e:
            logger.error(f"Ошибка чтения {file_name}: {e}")
            return None

    def update_param(self, file_name: str, param_path: str, new_value) -> bool:
        yaml_path = self._resolve_path(file_name)
        if not yaml_path.exists():
            logger.error(f"Файл конфига не найден: {yaml_path}")
            return False
            
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = self.yaml.load(f)
            
            # Защита от пустого файла
            if data is None:
                data = {}

            keys = param_path.split('.')
            if keys and keys[0] == 'ros__parameters':
                keys = keys[1:]
            if not keys:
                return False

            node_mask = '/**' if '/**' in data else (list(data.keys())[0] if data else None)
            if not node_mask:
                logger.error("Не удалось определить корневую ноду")
                return False
                
            data.setdefault(node_mask, {}).setdefault('ros__parameters', {})
            params = data[node_mask]['ros__parameters']

            current = params
            for key in keys[:-1]:
                if not isinstance(current.get(key), dict):
                    current[key] = {}
                current = current[key]

            target_key = keys[-1]
            old_val = current.get(target_key)
            
            # === БЕЗОПАСНАЯ КОНВЕРТАЦИЯ ТИПА ===
            try:
                num_val = float(new_value)
                # Если в конфиге было целое число (int), сохраняем int
                if isinstance(old_val, int) and not isinstance(old_val, bool):
                    num_val = int(num_val)
                new_value = num_val
            except (ValueError, TypeError):
                pass  # Оставляем как строку/другой тип

            current[target_key] = new_value

            # Записываем обратно
            with open(yaml_path, 'w', encoding='utf-8') as f:
                self.yaml.dump(data, f)
                
            logger.info(f"✅ Параметр {param_path} = {new_value} сохранён в {file_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Ошибка записи YAML {file_name}: {type(e).__name__}: {e}", exc_info=True)
            return False

    def _resolve_path(self, file_name: str) -> Path:
        if not file_name.endswith('.yaml'):
            file_name += '.yaml'
        return self.config_dir / file_name

    def _navigate(self, data: dict, param_path: str):
        keys = param_path.split('.')
        if keys and keys[0] == 'ros__parameters':
            keys = keys[1:]
        node_mask = '/**' if '/**' in data else (list(data.keys())[0] if data else None)
        if not node_mask:
            return None
        params = data.get(node_mask, {}).get('ros__parameters', {})
        current = params
        for key in keys:
            if not isinstance(current, dict) or key not in current:
                return None
            current = current[key]
        return current