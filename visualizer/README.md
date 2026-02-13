# Визуализатор позиций дронов

Отдельная программа для отображения координат двух дронов в реальном времени. Координаты приходят из MAVLink (LOCAL_POSITION_NED) через CoordsMonitor и публикуются по UDP.

## Запуск

### Вариант 1: Сначала визуализатор

```bash
# Терминал 1
cd /path/to/drone_swarm_simulation
python visualizer/drone_position_visualizer.py
```

Затем в другом терминале:

```bash
# Терминал 2
python run_sitl_only.py
python new_iteration_RC_OVERRIDE/experiment_stability/leader_forward_one_move.py
# или run_sim.py + ваш сценарий
```

### Вариант 2: Сначала симуляция

```bash
# Терминал 1
python run_sitl_only.py
python new_iteration_RC_OVERRIDE/experiment_stability/leader_forward_one_move.py
```

```bash
# Терминал 2 — визуализатор можно запустить в любой момент
python visualizer/drone_position_visualizer.py
```

Порядок запуска не важен — график будет обновляться, как только визуализатор получит данные.

## Параметры

| Параметр     | По умолчанию | Описание                          |
|-------------|--------------|-----------------------------------|
| `--port -p` | 15551        | UDP-порт для приёма координат     |
| `--trail`   | 300          | Длина следа (точек) на графике    |
| `--interval`| 0.05         | Интервал обновления графика (с)   |

## Интеграция в свои скрипты

Чтобы скрипт симуляции отправлял координаты в визуализатор, добавьте:

1. Импорт в начале файла:

```python
try:
    import sys, os
    _proj_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if _proj_root not in sys.path:
        sys.path.insert(0, _proj_root)
    from visualizer.position_publisher import publish_positions
    HAS_VISUALIZER_PUBLISHER = True
except ImportError:
    HAS_VISUALIZER_PUBLISHER = False
```

2. В цикле обмена координатами (coordinate_exchange_loop), после сбора `positions`:

```python
if HAS_VISUALIZER_PUBLISHER:
    publish_positions(positions)
```

Уже интегрировано в: `new_iteration_RC_OVERRIDE/experiment_stability/leader_forward_one_move.py`.

## Зависимости

- `matplotlib` — для отображения графика
- `python3` — стандартная библиотека (socket, json)
