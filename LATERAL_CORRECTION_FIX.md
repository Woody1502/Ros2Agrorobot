# Исправление: латеральная коррекция курса (visual_multi_crop_row_navigation)

## Проблема

Синяя линия в визуализации (`vs_nav/graphic`) оставалась прямой, даже когда робот уходил в сторону от рядов.

**Причина:** управление рулением использовало **только угол (`ang`)** — наклон обнаруженной линии ряда относительно вертикали. Когда робот смещается латерально (вбок), линии рядов в кадре остаются параллельными — угол не меняется, `ang ≈ 0`, коррекции нет.

Параметр `P[0]` — **боковое смещение линии от центра кадра в пикселях** — вычислялся, но полностью игнорировался.

```python
# Было: только угол
position_command.data = [-self.imageProcessor.ang]
```

## Исправление

### 1. Добавлена латеральная коррекция (`visual_servoing_node.py`)

```python
# Стало: угол + пропорциональная коррекция бокового смещения
lateral_correction = self.lateralGain * self.imageProcessor.P[0]
position_command.data = [-self.imageProcessor.ang - lateral_correction]
```

`P[0]` — смещение в пикселях от центра изображения (~[-640, 640]).
При смещении вправо `P[0] > 0` → руль поворачивает обратно влево.

### 2. Добавлен параметр `lateralGain` (`configs/params.yaml`)

```yaml
lateralGain: 0.0005
```

Подбор: если реакция слабая — увеличить (0.001, 0.002). Если робот дёргается — уменьшить (0.0003).

### 3. Исправлен краш при первом вызове (`imageProc.py`)

`P` и `ang` не инициализировались в `reset()`. При первом вызове `trackCropLane()`, если трекер сразу терял линию ("Lost at least one line"), обращение к `P[0]` давало `AttributeError`.

```python
# Добавлено в reset():
self.P = None
self.ang = 0.0
```

И защитная проверка в ноде:
```python
if self.imageProcessor.P is not None:
    lateral_correction = self.lateralGain * self.imageProcessor.P[0]
else:
    lateral_correction = 0.0
```

## Файлы изменены

- `src/visual_multi_crop_row_navigation/visual_multi_crop_row_navigation/visual_servoing_node.py`
- `src/visual_multi_crop_row_navigation/visual_multi_crop_row_navigation/imageProc.py`
- `src/visual_multi_crop_row_navigation/configs/params.yaml`
