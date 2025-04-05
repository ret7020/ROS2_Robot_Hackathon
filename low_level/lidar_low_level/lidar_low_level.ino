#include <Arduino.h>

/********************************************************
 *  КОНСТАНТЫ И ПАРАМЕТРЫ
 ********************************************************/

// Определение количества «секторов» (равномерно делим 360° на 12 частей).
#define NUM_SECTORS 12

// Параметры UART для лидара (ESP32-специфичный Serial1)
#define LIDAR_RX_PIN 19
#define LIDAR_TX_PIN 18
#define BAUDRATE 115200

// UART2 - передача данных в следующий микроконтроллер
HardwareSerial UART2(2);
// Инициализация UART
#define UART_TX_PIN 16
#define UART_RX_PIN 4  // Любой свободный, если Arduino не отправляет обратно данные обратно
const unsigned long STATUS_PRINT_INTERVAL = 100; // 100 мс = 10 раз в секунду

// Структура пакета лидара: 4 байта заголовка, затем 32 байта данных
static const uint8_t LIDAR_HEADER[] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t LIDAR_HEADER_LEN = 4;
static const uint8_t LIDAR_BODY_LEN = 32;

// Пороговые расстояния (в миллиметрах) и время залипания аварии
// ALARM_DIST  — красная зона, WARNING_DIST — жёлтая зона
#define ALARM_DIST 400     // Менее 400 мм -> сектор в красном цвете
#define WARNING_DIST 650   // Менее 650 мм (но >= 400 мм) -> жёлтый
#define ALARM_HOLD_MS 300  // Время (мс), которое сектор будет «залипать» в красном
#define SECTOR_OFFSET 1    // Cдвиг секторов (0..11)

/********************************************************
 *  ГЛОБАЛЬНЫЕ МАССИВЫ
 ********************************************************/
// Храним текущее измеренное расстояние по каждому из 12 секторов.
// При отсутствии данных в секторе значение будет NO_VALUE.
static float sectorDistances[NUM_SECTORS] = { 0.0f };

// Время последнего обновления данных сектора (в миллисекундах).
static uint32_t sectorUpdateTime[NUM_SECTORS] = { 0 };

// Время, до которого сектор должен находиться в состоянии «тревоги» (красный цвет).
static uint32_t sectorAlarmUntil[NUM_SECTORS] = { 0 };

// Константа, обозначающая «нет данных».
static const float NO_VALUE = 99999.0f;

/********************************************************
 *  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
 ********************************************************/

/**
 * @brief Блокирующее чтение заданного количества байт из Serial
 *        с учётом таймаута (timeout_ms).
 * 
 * @param ser       Ссылка на объект Serial (например, Serial1)
 * @param buffer    Указатель на буфер, куда записывать считанные байты
 * @param length    Количество байт, которое необходимо прочитать
 * @param timeout_ms Максимальное время ожидания в миллисекундах (по умолчанию 500 мс)
 * @return true, если все байты успешно прочитаны, иначе false
 */
bool readBytesWithTimeout(HardwareSerial &ser, uint8_t *buffer, size_t length, uint32_t timeout_ms = 500) {
  uint32_t start = millis();
  size_t count = 0;

  while (count < length) {
    if (ser.available()) {
      buffer[count++] = ser.read();
    }
    if (millis() - start > timeout_ms) {
      return false;  // Истёк таймаут, данные не успели прийти
    }
  }
  return true;
}

/**
 * @brief Ожидание появления в потоке UART специфического заголовка лидара.
 *        Заголовок определён в массиве LIDAR_HEADER.
 * 
 * @param ser Ссылка на Serial (обычно Serial1)
 * @return true, если заголовок найден, иначе false (по таймауту)
 */
bool waitForHeader(HardwareSerial &ser) {
  uint8_t matchPos = 0;
  uint32_t start = millis();

  // Пытаемся «выровнять» поток байт на заголовок (4 байта)
  while (true) {
    if (ser.available()) {
      uint8_t b = ser.read();
      if (b == LIDAR_HEADER[matchPos]) {
        // Совпало очередное ожидаемое значение заголовка
        matchPos++;
        if (matchPos == LIDAR_HEADER_LEN) {
          // Все байты заголовка совпали
          return true;
        }
      } else {
        // Сброс, если последовательность прервалась
        matchPos = 0;
      }
    }
    // Если долго не приходит корректный заголовок, выходим с false
    if (millis() - start > 100) {
      return false;
    }
  }
}

/**
 * @brief Преобразует «rawAngle» из пакета (двухбайтовое значение)
 *        в угол в градусах [0..360).
 * 
 * Формула для данного конкретного лидара: angle = (rawAngle - 0xA000) / 64
 * 
 * @param rawAngle Сырой угол (число 16 бит из пакета лидара)
 * @return Угол в градусах, нормализованный в диапазон [0..360)
 */
float decodeAngle(uint16_t rawAngle) {
  float angleDeg = (float)(rawAngle - 0xA000) / 64.0f;
  while (angleDeg < 0) angleDeg += 360.0f;
  while (angleDeg >= 360) angleDeg -= 360.0f;
  return angleDeg;
}

/**
 * @brief Определяет, к какому сектору [0..11] относится данный угол.
 *        Один сектор = 30°, но для удобства «нулевой» сектор
 *        смещён так, что его диапазон ~ 345..15 градусов.
 * 
 * @param angleDeg Угол в градусах [0..360)
 * @return Индекс сектора [0..11]
 */
int angleToSector(float angleDeg) {
  // Сдвигаем угол на +15°, чтобы 0-й сектор приходился примерно на зону 345..15
  float shifted = angleDeg + 15.0f;
  while (shifted < 0) shifted += 360.0f;
  while (shifted >= 360) shifted -= 360.0f;

  // Делим на 30°, получаем индекс сектора
  int sector = (int)(shifted / 30.0f) % NUM_SECTORS;
  sector = (sector + SECTOR_OFFSET) % NUM_SECTORS;
  return sector;
}

/********************************************************
 *  ОСНОВНАЯ ФУНКЦИЯ ПАРСИНГА И ОБРАБОТКИ ДАННЫХ ЛИДАРА
 ********************************************************/

/**
 * @brief Считывает один пакет данных из лидара, обновляет глобальные
 *        массивы расстояний, а также управляет цветом светодиодов.
 * 
 * @return true, если пакет обработан успешно, false — при ошибке чтения
 */
bool parseAndProcessPacket() {
  // 1) Дожидаемся заголовка лидара
  if (!waitForHeader(Serial1)) {
    return false;  // Заголовок не найден, пропускаем
  }

  // 2) Считываем тело пакета из 32 байт
  uint8_t buffer[LIDAR_BODY_LEN];
  if (!readBytesWithTimeout(Serial1, buffer, LIDAR_BODY_LEN, 500)) {
    Serial.println("Failed to read 32 bytes");
    return false;
  }

  // 3) Извлекаем общие данные пакета
  //    (Например, скорость вращения, углы начала и конца)
  uint16_t rotationSpeedTmp = buffer[0] | (buffer[1] << 8);
  float rpm = (float)rotationSpeedTmp / 64.0f;

  uint16_t startAngleTmp = buffer[2] | (buffer[3] << 8);
  float startAngleDeg = decodeAngle(startAngleTmp);

  // Данные о расстояниях и интенсивностях занимают 8 групп по 3 байта
  uint8_t offset = 4;
  uint16_t distances[8];
  uint8_t intensities[8];
  for (int i = 0; i < 8; i++) {
    distances[i] = (buffer[offset] | (buffer[offset + 1] << 8));
    intensities[i] = buffer[offset + 2];
    offset += 3;
  }

  // В самом конце пакета — «конечный угол» (2 байта)
  uint16_t endAngleTmp = buffer[offset] | (buffer[offset + 1] << 8);
  float endAngleDeg = decodeAngle(endAngleTmp);

  // Если конечный угол «меньше» начального, то добавим 360
  if (endAngleDeg < startAngleDeg) {
    endAngleDeg += 360.0f;
  }

  // 4) Вычисляем углы для всех 8 точек внутри пакета
  float angleRange = endAngleDeg - startAngleDeg;
  float angleInc = angleRange / 8.0f;
  float packetAngles[8];
  for (int i = 0; i < 8; i++) {
    float angle = startAngleDeg + i * angleInc;
    // Нормализуем в [0..360)
    while (angle < 0) angle += 360.0f;
    while (angle >= 360) angle -= 360.0f;
    packetAngles[i] = angle;
  }

  // 5) Создаём в ременный массив, где соберём минимальные расстояния по секторам
  //    (для 8 точек, что пришли в пакете).
  float tempSectorMin[NUM_SECTORS];
  for (int s = 0; s < NUM_SECTORS; s++) {
    tempSectorMin[s] = NO_VALUE;  // Изначально никаких данных
  }

  // 6) Проходим по всем 8 точкам пакета, фильтруя по интенсивности
  for (int i = 0; i < 8; i++) {
    if (intensities[i] > 15) {
      int sectorIndex = angleToSector(packetAngles[i]);
      float dist = (float)distances[i];
      // Запоминаем минимальное расстояние на сектор
      if (dist < tempSectorMin[sectorIndex]) {
        tempSectorMin[sectorIndex] = dist;
      }
    }
  }

  // 7) Обновляем глобальный массив расстояний и время их обновления
  uint32_t now = millis();
  for (int s = 0; s < NUM_SECTORS; s++) {
    if (tempSectorMin[s] != NO_VALUE) {
      sectorDistances[s] = tempSectorMin[s];
      sectorUpdateTime[s] = now;
    }
  }

  // 8) Если сектор не обновлялся более 500 мс, считаем, что данных по нему нет
  for (int s = 0; s < NUM_SECTORS; s++) {
    if ((now - sectorUpdateTime[s]) > 500) {
      sectorDistances[s] = NO_VALUE;
    }
  }

  // 9) Обработка «залипания» красного: если новое расстояние < ALARM_DIST,
  //    продлеваем время «AlarmUntil».
  for (int s = 0; s < NUM_SECTORS; s++) {
    float dist = sectorDistances[s];
    if (dist != NO_VALUE && dist < ALARM_DIST) {
      sectorAlarmUntil[s] = now + ALARM_HOLD_MS;
    }
  }

  //10) (от макса: это 12ый пункт изначально) (Опционально) выводим отладочную информацию через Serial
  Serial.print("RPM=");
  Serial.print(rpm);
  Serial.print("  sectorDistances: ");
  for (int s = 0; s < NUM_SECTORS; s++) {
    Serial.print(sectorDistances[s]);
    Serial.print(", ");
  }

  return true;  // Пакет успешно обработан
}

void setup() {
  // Инициализация Serial для вывода отладочных сообщений в консоль
  Serial.begin(BAUDRATE);

  // Инициализация Serial1 для чтения данных лидара (указать пины RX/TX)
  Serial1.begin(BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  // Инициализация Serial2 для передачи
  UART2.begin(BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.println("ESP32 Lidar parser.");
}

void loop() {
  // В основном цикле просто пытаемся считать и обработать пакет
  parseAndProcessPacket();
}
