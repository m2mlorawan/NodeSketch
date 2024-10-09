Example Sketch for LoRaWAN Node. For Model A328, B1284 and Auduino Pro Mini LoRaWAN Kit.
You can find our product on Tindie.com  

LMIC-Arduino-AS923-upper-master.rar
oslmic.h
added static to fix error when compile.
static inline type table_get ## postfix(const type *table, size_t index) { \

Download Lowpowerlibrary from https://github.com/rocketscream/Low-Power

Searching Term "lora"

If you want to send only one channel such as 923.2Mhz, change \src\lmic\lorabase.h as following:

enum { AS923_F1 = 923200000,      // g1   SF7-12

       AS923_F2 = 923200000,      // g1   SF7-12 
       
       AS923_F3 = 923200000,      // g1   SF7-12
       
       AS923_F4 = 923200000,      // g2   SF7-12
       
       AS923_F5 = 923200000,      // g2   SF7-12
       
       AS923_F6 = 923200000,      // g3   SF7-12
       
       AS923_J4 = 923200000,      // g2   SF7-12  
       
       AS923_J5 = 923200000,      // g2   SF7-12   
       
       AS923_J6 = 923200000,      // g2   SF7      
};

m2mlorawan@gmail.com
