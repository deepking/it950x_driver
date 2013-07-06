# it950x_driver

## Structure
* src/dvb_api - User API for kernel space
* src/dvb_net - network device 
* src/ule - protocol encode/decode
* src/it950x-core - driver api
* tools 測量工具

## Develop

* install module:

    make 會將 src 複製到 project root，對此 source code 修改，無意義。

    ```
    $ make clean
    $ make
    $ sudo make install
    ```

* uninstall module:

    ```
    $ sudo make remove
    ```

* enable network device:

    ```
    $ sudo ifconfig dvb0 IP netmask 255.255.255.0
    ```

* log 位置 /var/log/syslog 

* 修改 Frequency/Bandwidth in src/dvb_net.c :

    ```c
    #define SEND_FREQ 666000
    #define RECV_FREQ 666000
    #define DEFAULT_BANDWIDTH 6000
    ```
