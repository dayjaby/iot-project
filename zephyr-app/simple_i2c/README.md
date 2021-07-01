Build and flash this project to a Olimex STM32E407:

```
mavgen.py --lang=C --wire-protocol=1.0 --output=generated/include/mavlink/v1.0 src/mavlink/message_definitions/v1.0/common.xml
source ../../zephyr/zephyr/zephyr-env.sh
west build -p auto -b nucleo_f446re .
west flash
```
