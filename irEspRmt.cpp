#include "IRremoteInt.h"
#include "IRremote.h"
/* This code is only usable within ESP32*/
#if defined(ESP32)

#include "driver/rmt.h"

#define RMT_CLK_DIV       100    /*!< RMT counter clock divider => 1.25 µs*/
#define APB_CLK           80000000 /*!< APB clock == RMT source clock == 80 MHz */
#define RMT_TICK_10_US    (APB_CLK/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */
#define RMT_TIMEBASE_NS   (RMT_CLK_DIV * 1000 /80)   /*!< time base of RMT core (1250 ns)*/
#define TARGET_TIMEBASE_NS  (50 * 1000) /*!< decoder expects 50 us time base in nano seconds*/

#define RMT_FILTER_THRESHOLD  100 /*!< amount of RMT ticks under which a signal will not be processed*/

#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */

#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */


/*
 * @brief RMT receiver initialization
 */
static bool rmt_rx_init()
{
  rmt_config_t rmt_rx;
  rmt_rx.channel = (rmt_channel_t) RMT_RX_CHANNEL;
  rmt_rx.gpio_num = (gpio_num_t) irparams.recvpin;
  rmt_rx.clk_div = RMT_CLK_DIV;
  rmt_rx.mem_block_num = 1;
  rmt_rx.rmt_mode = RMT_MODE_RX;
  rmt_rx.rx_config.filter_en = true;
  rmt_rx.rx_config.filter_ticks_thresh = RMT_FILTER_THRESHOLD;
  rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
  rmt_config(&rmt_rx);
  rmt_driver_install(rmt_rx.channel, 1000, 0);
}

/*
 * @brief function to store received data in IR library internal data structure
 *
 * @param item	pointer to data provided by default ISR 
 * @param size  size of array which contains the received data
 * */

static void store_data(rmt_item32_t* item, size_t size)
{
  irparams.rawlen = 1;
  /*workaround to not trigger sony or sayo repeat detection => sony and sayo fast repeats will not be filtered!*/
  irparams.rawbuf[0] = 80000;
  //every Item contains 2 values corresponding to one high and one low level on the line
  for(int i = 0; i < size;i++) {
	  //duration == 0 signals end of data
      if( item[i].duration0 != 0 && irparams.rawlen < RAWBUF)
      {
        irparams.rawbuf[irparams.rawlen++] = item[i].duration0/(TARGET_TIMEBASE_NS/RMT_TIMEBASE_NS);
      }
      else
      {
        break;
      }

	  //duration == 0 signals end of data
      if( item[i].duration1 != 0 && irparams.rawlen < RAWBUF)
      {
        irparams.rawbuf[irparams.rawlen++] = item[i].duration1/(TARGET_TIMEBASE_NS/RMT_TIMEBASE_NS);
      }
      else
      {
        break;
      }
  }

  //if rawlen was bigger or equal to RAWBUF, the loop before was cancled due to overflow
  if(irparams.rawlen >= RAWBUF)
  {
    irparams.overflow = true;
  }

  //mark state machine as done
  irparams.rcvstate = STATE_STOP;
}

/**
 * @brief this function will run as an  free RTOS task. It will first initialize and
 * start the default RMT driver (including its ISR). Afterwards it will wait for new
 * data from the ring buffer. If it is available, it will translate the data into 
 * the IR library internal representation and change the state machine to mark the
 * data as new.
 *
 * If the stored data is not read before the next data is received, the next data will
 * be discarded intentially.
 *
 * */
static void rmt_rx_task(void* param)
{
  rmt_rx_init();
  RingbufHandle_t rb = NULL;
  rmt_get_ringbuf_handle((rmt_channel_t)RMT_RX_CHANNEL, &rb);
  rmt_rx_start((rmt_channel_t)RMT_RX_CHANNEL, 1);
  
  while(rb) {
    size_t rx_size = 0;
    //get new data from ring buffer (default ISR stores received data in this ring buffer)
    rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
    //if item is null, xRingbufferReceive ran into timeout
    if(item)
    {
      if(irparams.rcvstate == STATE_IDLE)
      {
	//data can be stored in irparams so do it.
        store_data(item, rx_size);
      }
      else
      {
	//data in irparams was not read yet, discard received data.
        DBG_PRINT("RMT task: Discarding received data\n");
      }
      //after parsing the data, return spaces to ringbuffer.
      vRingbufferReturnItem(rb, (void*) item);
    }
    else
    {
      DBG_PRINT("RMT task: nothing received, retrying\n");
    }
  }
  vTaskDelete(NULL);
}

void start_rx_task()
{
  DBG_PRINT("Starting rx task");
  xTaskCreate(rmt_rx_task, "rmt_rx_task", 2048, NULL, 1, NULL);
}
#endif
