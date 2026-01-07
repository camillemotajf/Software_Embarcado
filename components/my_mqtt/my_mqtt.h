#ifndef MY_MQTT_H
#define MY_MQTT_H

void mqtt_start();
void mqtt_publish(char *topic, char *msg);
void mqtt_sbscribe(char *topic);

#endif