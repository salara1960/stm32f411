#################################################################################################
#
# New BlackPill board - stm32f411 + ssd1306(spi) + bme280(i2c with DMA) +
#                       qmc5883l(i2c with DMA) + mpu6050(i2c with DMA)
#
#################################################################################################


## Состав рабочего оборудования:

```
* stm32f411 (BlackPill board) - плата микроконтроллера
* ssd1306 - OLED дисплей 0.96" 128x64 (интерфейы SPI)
* bme280 - датчик атмосферного давления, температуры и влажности воздуха(интерфейс I2C с DMA).
* qmc5883L - магнитометер - электронный компас (интерфейс I2C c DMA)
* audio mic - модуль аудиомикрофона, выполняющего функцию источника звукового сигнала (интерфейс GPIO с interrupt)
* mpu6050 - акселерометр + гироскоп + датчик температуры (интерфейс I2C c DMA)
* TL1883 - ик-приемник ((интерфейы GPIO))
```


# Средства разработки:

```
* STM32CubeMX - графический пакет для создание проектов (на языке Си) под микроконтроллеры семейства STM32
  (https://www.st.com/en/development-tools/stm32cubemx.html).
* System Workbench for STM32 - IDE среда разработки ПО для микроконтроллеров семейства STM32
  (https://www.st.com/en/development-tools/sw4stm32.html).
* stm32flash - утилита для записи firmware в flash-память микроконтроллеров семейства STM32
  через встроенный порт USART1 (https://sourceforge.net/projects/stm32flash/files/)
* STM32CubeProgrammer - утилита для записи firmware в flash-память микроконтроллеров семейства STM32
  (https://www.st.com/en/development-tools/stm32cubeprog.html).
* ST-LINK V2 - usb отладчик для микроконтроллеров семейства STM8/STM32.
* Saleae USB Logic 8ch - логический анализатор сигналов, 8 каналов , макс. частота дискретизации 24МГц
  (https://www.saleae.com/ru/downloads/)
```


# Функционал:

* ПО построено по модели BARE METAL (без использования ОС) с использованием самостоятельного буфера событий,
  построенного по принципу fifo. События обслуживаются в основном цикле программы. Формируются события в callBack-функциях
  по завершении прерываний от используемых модулей микроконтроллера.
* Устройство инициализирует некоторые интерфейсы микроконтроллера :
  - ADC1 : аналогово-цифровой преобразователь (измеряет напряжение питания устройства).
  - GPIO : подключены два сетодиода : PB5 - секундный тик, PB8 - индикатор ошибки на устройстве; user_key : пользовательская кнопка.
  - I2C1 : режим мастера с частотой 400Кгц (шина ослуживает bme280, qmc5883l, mpu6050).
  - USART1 : параметры порта 115200 8N1 - порт для логов и передачи AT команд утсройству, если подключен комп.
  - TIM2 : таймер-счетчик временных интервалов в 1 мс., реализован в callback-функции.
  - TIM4 : таймер-счетчик временных интервалов в 50 мкс., реализован в callback-функции для ИК-приемника.
  - RTC : часы реального времени, могут быть установлены с помощью команды epoch=XXXXXXXXXX:Y
  - SPI4 : обслуживает OLED SSD1306.
* Прием данных по последовательному порту (USART1) выполняется в callback-функции обработчика прерывания.
* Каждые 250 мс считываются данные с датчиков bme280 и qmc5883l, выполняется пересчет атмосферного
  давления в мм ртутного столба и температуры в градусы Цельсия и влажномть в проценты, полученные данные выдаются
  в USART1, например :

```
{
        "time": "18.01 18:41:33",
        "ms": 2258550,
        "fifo": [1, 3],
        "devError": 0,
        "volt": 3.27,
        "BME280": {
                "pres": 759.86,
                "temp": 24.59,
                "humi": 30.59
        },
        "QMC5883L": {
                "azimut": 287.90,
                "temp2": 26.69
        },
        "MPU6050": {
                "stat": 0,
                "temp3": 25.53,
                "accel": [0, 0, 0],
                "gyro": [0, 0, -1]
        }
}
```

  Часть этих данные (дата и время, код ошибки устройства, напряжение питания, атмосферное давление,
температура и влажность воздуха, азимут) отображаются на дисплей ssd1306.

* Через usart1 можно отправлять команды на устройство, например :

```
epoch=1608307455:2
    установить текущее время и временную зону
 Unix epoch time - 1608307455
 Time zone - 2
```

```
json
    установить формат выдаваемых данных в json виде
```

```
text
    установить формат выдаваемых данных в текстовом виде
```

```
rst
    рестарт устройства
```

```
period=1000
    установить период выдачи данных от устройства в 1000 мсек.
```


* Функционал проекта в процессе пополнения.

