# AI Module Camera RX

Нода для получения стрима камеры с AI модуля (LicheeRV Nano). ПО на AI модуле должно быть запущено с включенным MJPEG стримом (подробнее в разделе про конфигурацию AI модуля).

Параметры:

| Название             | Дефолтное значение | Описание                                       |
|----------------------|--------------------|------------------------------------------------|
| **ai_module_ip**         | 10.160.209.1       | IP адрес платы в USB LAN (обычно всегда такой) |
| **ai_module_mjpeg_port** | 7777               | Порт с MJPEG стримом                           |
| **receive_fps**          | 10                 | FPS чтение кадров стрима                       |


Отдельный запуск (с заменой значения параметра):

```bash
ros2 run camera ai_module_camera --ros-args -p ai_module_ip:="10.10.10.1"
```