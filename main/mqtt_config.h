/*

*/
#ifndef _MQTT_CONFIG_H_
#define _MQTT_CONFIG_H_

#define	PUBLISH		100
#define	SUBSCRIBE	200
#define	STOP		900

#define MQTT_CONNECTED_BIT BIT2


void mqtt_task(void *pvParameters);

typedef struct {
    int topic_type;
    int topic_len;
    char topic[64];
    int data_len;
    char data[64];
} MQTT_t;


#endif

