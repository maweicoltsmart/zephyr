gcc -o LoRaServerSample -g mqtt.c -std=c99 -I/usr/include/json -L/usr/lib/i386-linux-gnu/ -ljson -lpthread -lmosquitto