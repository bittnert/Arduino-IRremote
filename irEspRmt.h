#ifndef IR_ESP_RMT_H
#define IR_ESP_RMT_H

/* Function to start rx task which will configure the
 * ESP IDF RMT driver and translate the data provided by 
 * the driver into the internal data structure
 * */
void start_rx_task();

#endif
