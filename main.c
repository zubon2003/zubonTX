/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 zubon2003 (zubon2003@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */



#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "bsp/board.h"
#include "tusb.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"


//--------------------------------------------------------------------+
// Definitions
//--------------------------------------------------------------------+

//HEADTRACKER CH SELECT PIN (ALWAYS PULLUP ->7-8CH INPUT  GND ->5-6CH)
#define HEADTRACKER_CH_SELECT_PIN 2

//THROTTLE CENTER THRESHOLD (FOR THROTTLE BEEPER)
#define THROTTLE_CENTER_THRESHOLD 8
#define BEEP_DURATION 100 // 500ms
#define BUZZER_PIN 15

//PPM SUM INPUT(FOR HEADTRACKER HEADTRACKER PUTPUT CHANNEL MUST BE 7-8ch)
#define PPM_IN_PIN 11

//PPM-SUM OUTPUT
#define PPM_OUT_PIN 19
#define PPM_FRAME_LENGTH 30500
#define PPM_ONESHOT_PULSE  300

//SBUS OUTPUT
#define SBUS_FRAME_SEND_INTERVAL_MS  14 //SEND SBUS FRAME INTER 
#define UART_ID uart1
#define BPS 100000

#define DATA_BITS 8
#define STOP_BITS 2
#define PARITY    UART_PARITY_EVEN
#define UART_TX_PIN 8
#define UART_RX_PIN 9

#define PPM_HIGH 2000
#define PPM_LOW  1000
#define PPM_MIDDLE 1500
#define SBUS_HIGH 2047
#define SBUS_LOW  0
#define SBUS_MIDDLE 1023

//--------------------------------------------------------------------+
// GLOBAL VALUABLES
//--------------------------------------------------------------------+
uint16_t channel[12] = {PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW};
uint16_t tmp_channel[12] = {PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW,PPM_LOW};
uint16_t frame_interval;

uint8_t SBUS_FRAME[25];
uint16_t channel_SBUS[16] = {SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW,SBUS_LOW};

uint8_t buttons_a = 0;
uint8_t buttons_b = 0;

uint8_t beep_on = 0;
uint32_t beep_end_millis = 0;

volatile uint16_t ppm_in[32] = {1500};


typedef struct TU_ATTR_PACKED
{
  union { //axes and hut switch
    uint32_t axes;
    struct {
      uint32_t x : 10;
      uint32_t y : 10;
      uint32_t hat : 4;
      uint32_t twist : 8;
    };
  };
  uint8_t buttons_a;
  uint8_t slider;
  uint8_t buttons_b;

} gamepad_report_t;




static bool gamepad_mounted = false;
static uint8_t gamepad_dev_addr = 0;
static uint8_t gamepad_instance = 0;

static semaphore_t semaphore;

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
void led_blinking_task(void);
static inline bool is_supported_gamepad(uint8_t dev_addr);
void process_gamepad(uint8_t const* report, uint16_t len);
void core1_generate_ppm(void);
void sbus_output_task(void);
uint8_t button_pushed(uint8_t button_no);
void ppm_in_callback(uint gpio,uint32_t emask);

/*------------- MAIN -------------*/
int main(void)
{
  board_init();
  sem_init(&semaphore,1,1);

  //ppm_input_interrupt
  gpio_init(PPM_IN_PIN);
  gpio_set_dir(PPM_IN_PIN,GPIO_IN);
  gpio_pull_down(PPM_IN_PIN);
  gpio_set_irq_enabled_with_callback(PPM_IN_PIN,GPIO_IRQ_EDGE_RISE+GPIO_IRQ_EDGE_FALL,true,&ppm_in_callback);

  gpio_init(HEADTRACKER_CH_SELECT_PIN);
  gpio_set_dir(HEADTRACKER_CH_SELECT_PIN,GPIO_IN);
  gpio_pull_up(HEADTRACKER_CH_SELECT_PIN);


  uint32_t next_sbus_output_ms = board_millis() + SBUS_FRAME_SEND_INTERVAL_MS;
  printf("TinyUSB Host HID Controller Example\r\n");
  printf("Note: Events only displayed for explicit supported controllers\r\n");

  // init host stack on configured roothub port
  tuh_init(BOARD_TUH_RHPORT);

  //core1 -> for ppm output
  multicore_launch_core1(core1_generate_ppm);

  //Serial1 init.(FOR SBUS OUTPUT)
  uart_init(UART_ID, BPS);
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  //gpio_set_outover(UART_TX_PIN,GPIO_OVERRIDE_INVERT);
  //gpio_set_inover(UART_RX_PIN,GPIO_OVERRIDE_INVERT);

  //PWM_INIT(BUZZER)
  //  gpio_set_function( BUZZER_PIN, GPIO_FUNC_PWM );
  //  slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
  //  pwm_set_clkdiv(slice_num, 138.71626);
  //  pwm_set_wrap(slice_num, 2047);
  //  pwm_set_chan_level(slice_num, PWM_CHAN_A, 1024);
  //  pwm_set_chan_level(slice_num, PWM_CHAN_B, 1024);
  gpio_init(BUZZER_PIN);
  gpio_set_dir(BUZZER_PIN,GPIO_OUT);
  gpio_put(BUZZER_PIN,false);

  //SBUS FRAME INIT
  SBUS_FRAME[0] = 0x0F;
  SBUS_FRAME[6] =  0;
  SBUS_FRAME[7] =  0;
  SBUS_FRAME[8] =  0;
  SBUS_FRAME[9] =  0;
  SBUS_FRAME[10] =  0;
  SBUS_FRAME[11] =  0;
  SBUS_FRAME[12] =  0;
  SBUS_FRAME[13] =  0;
  SBUS_FRAME[14] =  0;
  SBUS_FRAME[15] =  0;
  SBUS_FRAME[16] =  0;
  SBUS_FRAME[17] =  0;
  SBUS_FRAME[18] =  0;
  SBUS_FRAME[19] =  0;
  SBUS_FRAME[20] =  0;
  SBUS_FRAME[21] =  0;
  SBUS_FRAME[22] =  0;
  SBUS_FRAME[23] =  0;
  SBUS_FRAME[24] = 0x00;
  next_sbus_output_ms = board_millis() + SBUS_FRAME_SEND_INTERVAL_MS;

  while (1)
  {
    // tinyusb host task
    tuh_task();
    led_blinking_task();
    
    if (next_sbus_output_ms <= board_millis()) {
        next_sbus_output_ms = board_millis() + SBUS_FRAME_SEND_INTERVAL_MS;
        sbus_output_task();
    }

    //printf("%d      %d\r\n",ppm_in[6],ppm_in[7]);


  }
}

//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
  (void)desc_report;
  (void)desc_len;
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
  printf("VID = %04x, PID = %04x\r\n", vid, pid);

  // Supported Gamepad?
  if ( is_supported_gamepad(dev_addr) )
  {
    if (!gamepad_mounted)
    {
      gamepad_dev_addr = dev_addr;
      gamepad_instance = instance;
      gamepad_mounted = true;
    }
    // request to receive report
    // tuh_hid_report_received_cb() will be invoked when report is available
    if ( !tuh_hid_receive_report(dev_addr, instance) )
    {
      printf("Error: cannot request to receive report\r\n");
    }
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  if (gamepad_mounted && gamepad_dev_addr == dev_addr && gamepad_instance == instance)
  {
    gamepad_mounted = false;
  }
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  if ( is_supported_gamepad(dev_addr) ) process_gamepad(report, len);
  

  // continue to request to receive report
  if ( !tuh_hid_receive_report(dev_addr, instance) ) printf("Error: cannot request to receive report\r\n");
  
}

//--------------------------------------------------------------------+
// Blinking Task
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  const uint32_t interval_ms = 1000;
  static uint32_t start_ms = 0;

  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

//--------------------------------------------------------------------+
// SBUS OUTPUT Task
//--------------------------------------------------------------------+
void sbus_output_task(void)
{
    SBUS_FRAME[1] =   (channel_SBUS[0] & 0x00ff);
    SBUS_FRAME[2] =  ((channel_SBUS[1] & 0x001f) << 3) + (channel_SBUS[0] >> 8);
    SBUS_FRAME[3] =  ((channel_SBUS[2] & 0x0003) << 6) + (channel_SBUS[1] >> 5);
    SBUS_FRAME[4] =  ((channel_SBUS[2] & 0x03ff) >> 2);
    SBUS_FRAME[5] =  ((channel_SBUS[3] & 0x007f) << 1) + (channel_SBUS[2] >> 10);
    SBUS_FRAME[6] =  ((channel_SBUS[4] & 0x000f) << 4)  + (channel_SBUS[3] >> 7);
    SBUS_FRAME[7] =  ((channel_SBUS[5] & 0x0001) << 7) + (channel_SBUS[4] >> 4);
    SBUS_FRAME[8] =  ((channel_SBUS[5] & 0x0001) >> 1);
    SBUS_FRAME[9] =  ((channel_SBUS[6] & 0x003f) << 2) + (channel_SBUS[5] >> 9);
    SBUS_FRAME[10]=  ((channel_SBUS[7] & 0x0007) << 5) + (channel_SBUS[6] >> 6);
    SBUS_FRAME[11]=   (channel_SBUS[7] >> 3);
    SBUS_FRAME[12] =  (channel_SBUS[8] & 0x00ff);
    SBUS_FRAME[13] = ((channel_SBUS[9] & 0x001f) << 3) + (channel_SBUS[8] >> 8);
    SBUS_FRAME[14] = ((channel_SBUS[10] & 0x0003) << 6) + (channel_SBUS[9] >> 5);
    SBUS_FRAME[15] =  ((channel_SBUS[10] & 0x03ff) >> 2);
    SBUS_FRAME[16] =  ((channel_SBUS[11] & 0x007f) << 1) + (channel_SBUS[10] >> 10);
    SBUS_FRAME[17] =  (channel_SBUS[11] >> 7);

    uart_write_blocking(UART_ID,SBUS_FRAME,25);
}

// check if device is supported gamepad
static inline bool is_supported_gamepad(uint8_t dev_addr)
{
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  return ( (vid == 0x046d && pid == 0xc215) // logicool Extreme 3d pro
        || (vid == 0x046d && pid == 0xc215)                 // logicool Extreme 3d pro
          );
}

// read value
void process_gamepad(uint8_t const* report, uint16_t len)
{
    uint16_t tmp_frame_interval;
    gamepad_report_t gamepad_report;
    uint8_t is_throttle_center;
    memcpy(&gamepad_report, report, sizeof(gamepad_report));
    buttons_a = gamepad_report.buttons_a;
    buttons_b = gamepad_report.buttons_b;

    if (ppm_in[4] > 2000) ppm_in[4] = 2000;
    if (ppm_in[5] > 2000) ppm_in[5] = 2000;
    if (ppm_in[6] > 2000) ppm_in[6] = 2000;
    if (ppm_in[7] > 2000) ppm_in[7] = 2000;
    if (ppm_in[4] < 1000) ppm_in[4] = 1000;
    if (ppm_in[5] < 1000) ppm_in[5] = 1000;
    if (ppm_in[6] < 1000) ppm_in[6] = 1000;
    if (ppm_in[7] < 1000) ppm_in[7] = 1000;
    //THROTTLE CENTER = BEEP ON

    if ((gamepad_report.slider < (127 + THROTTLE_CENTER_THRESHOLD)) && (gamepad_report.slider > (127 - THROTTLE_CENTER_THRESHOLD))) {
      is_throttle_center = 1;
    } else {
      is_throttle_center = 0;
    }

    if ((is_throttle_center == 1) && (beep_on == 0)) {
      beep_on = 1;
      beep_end_millis = board_millis() + BEEP_DURATION;
    }
    if (is_throttle_center == 0){
      beep_on = 0;
      beep_end_millis = 0;
    }
    if ((beep_end_millis >= board_millis()) && (beep_on == 1)) {
        gpio_put(BUZZER_PIN,true);
    }
    if (beep_end_millis < board_millis()) {
        gpio_put(BUZZER_PIN,false);
    }
    if (beep_on == 0) {
        gpio_put(BUZZER_PIN,false);
    }
    
    channel_SBUS[0]=gamepad_report.x * 2;
    channel_SBUS[1]=gamepad_report.y * 2;
    channel_SBUS[2]=2047 - (gamepad_report.slider * 8);
    channel_SBUS[3]=gamepad_report.twist * 8;

    tmp_channel[0]  = 1000 + gamepad_report.x * 0.9765625;    //AILERON  0-1023 -> 1000-2000
    tmp_channel[1]  = 1000 + gamepad_report.y * 0.9765625;    //ELEVATOR 0-1023 -> 1000-2000
    tmp_channel[2]  = 2000 - gamepad_report.slider * 3.90625; //THROTTLE 0-255  -> 1000-2000
    tmp_channel[3]  = 1000 + gamepad_report.twist * 3.90625;  //RUDDER   0-255  -> 1000-2000

    if (gpio_get(HEADTRACKER_CH_SELECT_PIN) == false) {
    tmp_channel[6]  = ppm_in[6];
    tmp_channel[7]  = ppm_in[7];
    channel_SBUS[6]=(ppm_in[6]-1000) * 2.047;
    channel_SBUS[7]=(ppm_in[7]-1000) * 2.047;
    }
    else{
    tmp_channel[6]  = ppm_in[4];
    tmp_channel[7]  = ppm_in[5];
    channel_SBUS[6]=(ppm_in[4]-1000) * 2.047;
    channel_SBUS[7]=(ppm_in[5]-1000) * 2.047;
    }

    if((gamepad_report.hat == 0) && (button_pushed(1))){
      channel_SBUS[4] = SBUS_HIGH;
      tmp_channel[4]  = PPM_HIGH;
    }

    if((gamepad_report.hat == 4) && (button_pushed(1))){
      channel_SBUS[4] = SBUS_LOW;
      tmp_channel[4]  = PPM_LOW;
    }

    if(gamepad_report.hat == 2) {
      channel_SBUS[10] = SBUS_HIGH;
      tmp_channel[10]  = PPM_HIGH;
    }

    if(gamepad_report.hat == 6) {
      channel_SBUS[10] = SBUS_LOW;
      tmp_channel[10]  = PPM_LOW;
    }

    if(button_pushed(5)) {
      channel_SBUS[5] = SBUS_HIGH;
      tmp_channel[5]  = PPM_HIGH;
    }

    if(button_pushed(3)) {
      channel_SBUS[5] = SBUS_LOW;
      tmp_channel[5]  = PPM_LOW;
    }

    if(button_pushed(6)) {
      channel_SBUS[11] = SBUS_HIGH;
      tmp_channel[11]  = PPM_HIGH;
    }

    if(button_pushed(4)) {
      channel_SBUS[11] = SBUS_LOW;
      tmp_channel[11]  = PPM_LOW;
    }

    if(button_pushed(7)) {
      channel_SBUS[8] = SBUS_HIGH;
      tmp_channel[8]  = PPM_HIGH;
    }

    if(button_pushed(9)) {
      channel_SBUS[8] = SBUS_MIDDLE;
      tmp_channel[8]  = PPM_MIDDLE;
    }

    if(button_pushed(11)) {
      channel_SBUS[8] = SBUS_LOW;
      tmp_channel[8]  = PPM_LOW;
    }

    if(button_pushed(8)) {
      channel_SBUS[9] = SBUS_HIGH;
      tmp_channel[9]  = PPM_HIGH;
    }

    if(button_pushed(10)) {
      channel_SBUS[9] = SBUS_MIDDLE;
      tmp_channel[9]  = PPM_MIDDLE;
    }

    if(button_pushed(12)) {
      channel_SBUS[9] = SBUS_LOW;
      tmp_channel[9]  = PPM_LOW;
    }
    tmp_frame_interval = PPM_FRAME_LENGTH - PPM_ONESHOT_PULSE;
    for(uint8_t i = 0;i <= 11; i++) tmp_frame_interval -= tmp_channel[i];
    //for(uint8_t i = 0;i <= 11; i++) printf("%4d  ",tmp_channel[i]);
    //printf("%d\r\n",gpio_get(HEADTRACKER_CH_SELECT_PIN));
    sem_acquire_blocking(&semaphore);
    memcpy(channel,tmp_channel,sizeof(channel));
    frame_interval = tmp_frame_interval;
    sem_release(&semaphore);
}

//button pushed?
uint8_t button_pushed(uint8_t button_no) {
  uint8_t evaluate = 0;
  switch(button_no){
    case 1:
      evaluate = ((buttons_a & 0x01) == 0x01);
      break;
    case 2:
      evaluate = ((buttons_a & 0x02) == 0x02);
      break;
    case 3:
      evaluate = ((buttons_a & 0x04) == 0x04);
      break;
    case 4:
      evaluate = ((buttons_a & 0x08) == 0x08);
      break;
    case 5:
      evaluate = ((buttons_a & 0x10) == 0x10);
      break;
    case 6:
      evaluate = ((buttons_a & 0x20) == 0x20);
      break;
    case 7:
      evaluate = ((buttons_a & 0x40) == 0x40);
      break;
    case 8:
      evaluate = ((buttons_a & 0x80) == 0x80);
      break;
    case 9:
      evaluate = ((buttons_b & 0x01) == 0x01);
      break;
    case 10:
      evaluate = ((buttons_b & 0x02) == 0x02);
      break;
    case 11:
      evaluate = ((buttons_b & 0x04) == 0x04);
      break;
    case 12:
      evaluate = ((buttons_b & 0x08) == 0x08);
      break;

  }
  return(evaluate);
}

//core 1 task to generationg ppm
void core1_generate_ppm(void){

    gpio_init(PPM_OUT_PIN);
    gpio_set_dir(PPM_OUT_PIN,GPIO_OUT);

    gpio_put(PPM_OUT_PIN,true);

    uint16_t tmp_channel[12];
    uint16_t tmp_frame_interval;

    while(1){
      sem_acquire_blocking(&semaphore);
      memcpy(tmp_channel,channel,sizeof(channel));
      tmp_frame_interval = frame_interval;
      sem_release(&semaphore);



      //channel 0
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[0]-PPM_ONESHOT_PULSE);

      //channel 1
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[1]-PPM_ONESHOT_PULSE);

      //channel 2
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[2]-PPM_ONESHOT_PULSE);

      //channel 3
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[3]-PPM_ONESHOT_PULSE);

      //channel 4
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[4]-PPM_ONESHOT_PULSE);

      //channel 5
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[5]-PPM_ONESHOT_PULSE);

      //channel 6
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[6]-PPM_ONESHOT_PULSE);

      //channel 7
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[7]-PPM_ONESHOT_PULSE);

      //channel 8
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[8]-PPM_ONESHOT_PULSE);

      //channel 9
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[9]-PPM_ONESHOT_PULSE);

      //channel 10
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[10]-PPM_ONESHOT_PULSE);

      //channel 11
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_channel[11]-PPM_ONESHOT_PULSE);

      //frame interval
      gpio_put(PPM_OUT_PIN,false);
      sleep_us(PPM_ONESHOT_PULSE);
      gpio_put(PPM_OUT_PIN,true);
      sleep_us(tmp_frame_interval);

    }
}

void ppm_in_callback(uint gpio,uint32_t emask) {
  static uint32_t pulse;
  static uint32_t counter;
  static uint32_t lastmicros = 0;
  static uint8_t ppm_in_channel = 0;
  gpio_set_irq_enabled(gpio,(GPIO_IRQ_EDGE_RISE+GPIO_IRQ_EDGE_FALL),false);
  counter = time_us_32() - lastmicros;
  lastmicros = time_us_32();
  if (counter < 510){
    pulse = counter;
  }
  else if (counter > 1910) {
    ppm_in_channel = 0;
  }
  else {
    ppm_in[ppm_in_channel] = pulse + counter;
    ppm_in_channel++;
  }
  //printf("%d    %d\r\n",ppm_in[6],ppm_in[7]);
  gpio_set_irq_enabled(gpio,(GPIO_IRQ_EDGE_RISE+GPIO_IRQ_EDGE_FALL),true);
}

