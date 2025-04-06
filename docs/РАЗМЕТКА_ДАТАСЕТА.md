# Разметка датасета в Roboflow
После того, как вы записали датасет, необходимо загрузить его в Roboflow (предварительно зарегистрируйтесь на платформе).

Далее необходимо создать новый проект:

![image](https://github.com/user-attachments/assets/64c48bc8-67a5-416a-a4aa-527d0fe9b39d)

В появившемся окне пишем название проекта, выбираем задачу “Object detection” и нажимаем на кнопку “Create Public Project”:

![image](https://github.com/user-attachments/assets/c304b5e7-b770-4900-a783-dbaba64fbfc2)

Появится окно, в которое можно загрузить материалы для датасета. Это могут быть видео (в туториале мы рассмотрим вариант) или обычные изображения.

![image](https://github.com/user-attachments/assets/4a2a5ca6-1efc-4ab6-b855-b835c13bc7d3)

Если вы выбрали видео, то будет откроется окно, в котором необходимо настроить как датасет будет разбит на кадры:

![image](https://github.com/user-attachments/assets/7f64d0eb-0143-4a3a-9bc2-9bcece4bb390)

Я рекомендую выбирать примерно 5 кадров в секунду. После нажатия на кнопку “Choose Frame Rate” пойдёт процесс нарезки видео на кадры с заданным шагом:

![image](https://github.com/user-attachments/assets/0b1aea65-0fa7-43d7-b32e-a6793a23c970)

В итоге загружаем файлы (кнопка “Save and Continue”):

![image](https://github.com/user-attachments/assets/36e9cb0b-08b0-437a-b870-c438a520dca5)

![image](https://github.com/user-attachments/assets/239e2fe1-f164-4619-83a7-05a99934cc80)

Далее будет предложено выбрать режим разметки, мы рассмотрим классический, ручной способ разметки:

![image](https://github.com/user-attachments/assets/eb4bc4d1-82d3-4721-ab62-76aaaeb03661)

Откроется окно, в котором необходимо распределить задачи на разметку. Если вы работаете в команде, то вы можете разделить разметку между несколькими людьми. Если вы работаете один, то все изображения выбирайте для своего пользователя:

![image](https://github.com/user-attachments/assets/9c8c0640-e9eb-4320-a7bc-0be70d828803)

Приступаем к разметке датасета:

![image](https://github.com/user-attachments/assets/cd831a77-a384-48b9-9b8c-e1a036b07be2)

Справа находится панель инструментов, мы рассмотрим только основные:

![image](https://github.com/user-attachments/assets/21130843-38db-4973-b538-d7f0ab6dd632)

При обучении детектора нам достаточно обвести нужный объект просто прямоугольной рамкой (вторая кнопка в панели инструментов):

![image](https://github.com/user-attachments/assets/62715291-ea6a-4a71-beb0-09db1ee74900)

Далее необходимо написать название класса, т.е. тип объекта, который мы хотим детектировать. После того как вы напишите название класса, оно сохранится и его не придется заново писать на новых изображениях в датасете.

В некоторых случаях удобно обвести объект не рамкой, а многоугольником (по точкам). Это можно сделать инструментом “3” из панели:
![image](https://github.com/user-attachments/assets/4749fb62-77bd-4863-ac79-51634c1a921c)

Если объект достаточно сильно отличается от окружающего его фона, то его контур, можно построить автоматическим инструментом: “Smart Polygon” (4 кнопка на панели):
![image](https://github.com/user-attachments/assets/bff054d9-d8e6-4050-b2d2-cf7b79ea2899)

В открывшемся окне выбираем “Standard” (он полностью бесплатный). Теперь необходимо кликнуть левой кнопкой мыши примерно посередине объекта:

Можно выбрать сложность, контура, которая показывает количество точек, по которым будет обведен объект. В целом, это значение можно оставить по-умолчанию. Далее необходимо выбрать класс объекта.

Теперь размечаем все оставшиеся изображения. Когда вы закончите этот процесс, то можно будет перейти к экспорту датасета. Для этого переходим в зачу по разметке и нажимаем на кнопку “Add images to dataset”:

![image](https://github.com/user-attachments/assets/d850424e-9400-4fac-bff2-e3439ef14301)

Выбираем метод автоматического распределения на Train/Valid/Test:

![image](https://github.com/user-attachments/assets/7b4cccfa-d86d-4adc-afa5-3ac2ccc0c9c0)

Теперь нажимаем “New Dataset Version”

![image](https://github.com/user-attachments/assets/12a0bae3-f2fb-459f-b430-bea0a9ff309a)

Бесплатная версия Roboflow позволяет провести аугментацию датасета, с увеличением общего количества изображений в 3 раза.

![image](https://github.com/user-attachments/assets/d2337e0a-0a32-40da-b7f8-522dae810bb9)

Можно выбрать различные виды аргументации и настроить их:

![image](https://github.com/user-attachments/assets/0abd4b43-1fa9-43cd-9d05-682f140e660e)

![image](https://github.com/user-attachments/assets/ca89fc39-16a1-4cf3-a1d2-ad4c67ea7f6f)

После настройки нажимаем на “Create”. Теперь датасет можно скачать в виде zip архива, для этого надо нажать на кнопку “Download Dataset”:

![image](https://github.com/user-attachments/assets/c5ec4cd7-f4ce-422a-9ba3-e560043521b5)

Выбираем формат Yolo11:

![image](https://github.com/user-attachments/assets/7e5d2fc4-0c71-4e1a-9c44-306b4aa7b41b)

В итоге скачается zip архив с датасетом. Для дальнейшего обучения модели на Google Colab загрузите этот архив на ваш гугл диск.
