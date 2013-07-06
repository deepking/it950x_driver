# it950x_driver

## Structure
* src/dvb_api - User API for kernel space
* src/dvb_net - network device 
* src/ule - protocol encode/decode
* src/it950x-core - driver api
* tools 測量工具

## Develop

* install module:

```
    $ make clean
    $ make
    $ sudo make install
```
* enable network device:

```
    $ sudo ifconfig dvb0 IP netmask 255.255.255.0
```

* log 位置 /var/log/syslog 

