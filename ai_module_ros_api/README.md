# AI Module ROS API

В данной директории находится набор ROS2 пакетов, которые позволяют взаимодействоввать с AI модулем через ROS2: ноды для получения данных, сервис для управления модулем.

## Запуск и использование
Прошивка платы описана в отдельном разделе документации. Настройка ROS2 драйвера:

```bash
git clone https://github.com/ret7020/ROS2_Robot_Hackathon
cd ROS2_Robot_Hackathon/ai_module_ros_api
colcon build
source install/setup.bash
pip3 install flask opencv-python
ros2 launch bringup ai_driver.launch.py
```

После запуска 'драйвера' у вас будет следующий набор топиков:
```
/ai_module/detector
/ai_module/image
```

Описание топиков-ножд:
### detector (нода camera) 

Возвращает данные детекции объектов в виде `MultiArrayDimension`, внутри которого трёхэлементные массивы, каждый такой массив - одна отдельная детекция:

```
[
  [Int32: object_class, Int32: object_x, Int32: object_y],
  [Int32: object_class, Int32: object_x, Int32: object_y]
]
```


### image (нода http_api)

Эта нода принимает MJPEG стрим с камеры `LicheeRV Nano` (AI модуля) и конвертирует в `ROS2 Image`, который публикуется в топик `ai_module/image`. Частота публикации - `10 Hz` (ограничена на стороне таймера в ROS2). Этот топик можно добавить в `RVIZ` для визуализации.

Сервис для конфигурации:

```
/ai_module/camera_prepare
/ai_module/update_model
/ai_module/yolo_get_pid
/ai_module/yolo_start
/ai_module/yolo_stop
```

Используемые типы сообщений для конфигурации:

```bash
# ConfigArg.srv
string ai_module_ip
int32 process_id
---
bool success
string info

# ConfigBase.srv
string ai_module_ip
---
bool success
string info

# ConfigModel.srv
string ai_module_ip
string model_local_path
---
bool success
string info

```

### camera_prepare

Принимает сообщение `ConfigBase`.

Необходимо вызывать после включения AI модуля. Достаточно один раз после каждой перезагрузки.

Пример вызова:

```bash
ros2 service call /ai_module/camera_prepare config_service_interface/srv/ConfigBase "{ai_module_ip: '10.160.209.1'}"
```

### update_model

Принимает сообщение `ConfigModel`.

Используется для удобного обновления весов модели YOLO. В сообщении нужно передать `model_local_path` - путь к новым весам на хостовой системе.

### yolo_start

Принимает сообщение `ConfigBase` (только IP адрес платы).

Используется для запуска софта детектора YOLO на плате.

### yolo_get_pid

Принимает сообщение `ConfigBase`.

Возвращает PID процесса с детектором YOLO. Далее этот PID передаётся в `yolo_stop`, чтобы остановить детектор.

### yolo_stop

Принимает сообщение `ConfigArg` (нужно передать PID процесса детектора).

Останавливает детектор по PID.

## Общий пайплайн использования сервисов конфига:

```
camera_prepare - подготовить камеру

*(optional) update_model - если надо обновить веса модели на плате
yolo_start - запуск YOLO

- Фаза активной работы -

yolo_get_pid - получаем PID процесса с YOLO
   |
   |
   V
yolo_stop - завершаем работу детектора
```


## Настройка

//TODO
