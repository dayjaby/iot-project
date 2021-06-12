```
mavgen.py --lang=C++11 --wire-protocol=2.0 --output=include/mavlink/v2.0 src/mavlink/message_definitions/v1.0/common.xml
dfu-util -a 0 -i 0 -s 0x08000000:leave -D nuttx.bin
```


