# Конфигурация AI Модуля

При связи модуля с одноплатным компьютером (в нашем случае Orange PI) используется USB LAN через Type C порт на плате LicheeRV Nano.

Плата выдаёт статический IP адрес `10.160.209.100` одноплатнику, а сама доступна по IP адресу `10.160.209.1`.

Для настройки модуля с одноплатника можно использовать скрипт [manage.py](https://github.com/ret7020/ROS2_Robot_Hackathon/blob/master/ai_module_sources/inference/manage.py)

Поддерживаемые команды:

Настройка камеры:
```bash
board_camera_prepare
```

Запуск стрима детекции YOLO в MJPEG
```bash
start_yolo
```

Получить ID процесса с YOLO детектором на AI модуле:
```bash
get_yolo_pid
```

Корректная остановка YOLO инференса на модуле:

```bash
stop_yolo
```

После запуска YOLO инференса данные о детекции (только массив данных с позициями и классами объектов) можно через ноду `lichee_http_read`

### lichee_http_read

```bash
ros2 run lichee_http_read lichee_http_read
```

Нода запускает http сервер на Flask, который принимает POST запросы от LicheeRV Nano и публикует их в топик `/detector` в виде 2D массива:
```
[
  [Int32: object_class, Int32: object_x, Int32: object_y],
  [Int32: object_class, Int32: object_x, Int32: object_y]
]
```

![image](https://github.com/user-attachments/assets/8d94c106-6d03-42fc-84ff-17c61a23fc90)


### lichee_camera

```bash
ros2 run lichee_camera lichee_camera
```

Эта нода принимает MJPEG стрим с LicheeRV Nano камеры и конвертирует в ROS2 Image, который публикуется в топик `camera/image`. Частота публикации - 10 Hz (ограничена на стороне таймера в ROS2). Этот топик можно добавить в RVIZ для визуализации.

### lichee_manage_service

Это ROS2 Service, который позволяет через инфраструктуру ROS конвигурировать детектор на LicheeRV Nano.
