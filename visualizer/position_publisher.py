#!/usr/bin/env python3
"""
Модуль для публикации координат дронов в визуализатор.

Скрипты симуляции (leader_forward_one_move.py, drone_following*.py и т.д.)
импортируют этот модуль и вызывают publish_positions() в цикле обмена координатами.

Формат: positions = {drone_id: {'x': float, 'y': float, 'z': float}, ...}
"""
import json
import socket
import sys

# Порт, на котором слушает визуализатор (должен совпадать с drone_position_visualizer.py)
DEFAULT_VISUALIZER_PORT = 15551


def publish_positions(positions: dict, host: str = "127.0.0.1", port: int = DEFAULT_VISUALIZER_PORT,
                      rates: dict = None) -> None:
    """
    Отправить текущие позиции дронов на визуализатор по UDP.
    
    Args:
        positions: словарь {drone_id: {'x': float, 'y': float, 'z': float}, ...}
        host: адрес визуализатора (по умолчанию localhost)
        port: порт визуализатора (по умолчанию 15551)
        rates: опционально {"follower_hz": float|None, "exchange_hz": float|None, "webots_step_hz": float|None}
               для отображения частот контуров в визуализаторе.
    
    Если визуализатор не запущен, пакеты теряются — ошибок не возникает.
    """
    if not positions and not rates:
        return
    try:
        payload_dict = {}
        if positions:
            payload_dict["positions"] = {
                str(k): {"x": v.get("x", 0), "y": v.get("y", 0), "z": v.get("z", 0)}
                for k, v in positions.items()
            }
        if rates is not None:
            payload_dict["rates"] = {
                "follower_hz": rates.get("follower_hz"),
                "exchange_hz": rates.get("exchange_hz"),
                "webots_step_hz": rates.get("webots_step_hz"),
            }
        payload = json.dumps(payload_dict).encode("utf-8")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(payload, (host, port))
        sock.close()
    except Exception:
        # Тихо игнорируем — визуализатор может быть не запущен
        pass
