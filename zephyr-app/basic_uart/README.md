Build and flash this project to a Olimex STM32E407:

```
mavgen.py --lang=C --wire-protocol=1.0 --output=generated/include/mavlink/v1.0 src/mavlink/message_definitions/v1.0/common.xml
source ../../zephyr/zephyr/zephyr-env.sh
west build -p auto -b olimex_stm32_e407 .
dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D build/zephyr/zephyr.bin
```

TODO:

```
mavgen.py --lang=C++11 --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 src/mavlink/message_definitions/v1.0/common.xml
```
