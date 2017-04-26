#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "ili9341.h"


#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22

#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5


static const char* TAG = "ili9341";


/*
 Some info about the ILI9341: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

/*
 The ILI9341 needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} ili_init_cmd_t;

static const ili_init_cmd_t ili_init_cmds[]={
    {0xCF, {0x00, 0x83, 0X30}, 3},
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    {0xE8, {0x85, 0x01, 0x79}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x26}, 1},
    {0xC1, {0x11}, 1},
    {0xC5, {0x35, 0x3E}, 2},
    {0xC7, {0xBE}, 1},
    {0x36, {0xe8}, 1},
    {0x3A, {0x55}, 1},
    {0xB1, {0x00, 0x1B}, 2},
    {0xF2, {0x08}, 1},
    {0x26, {0x01}, 1},
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4}, 
    {0x2C, {0}, 0},
    {0xB7, {0x07}, 1},
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

typedef struct {
    uint8_t dc;
    uint8_t free_buffer;
    void * buffer;
} spi_transaction_user_data_t;

//Send a command to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
void ili_cmd(spi_device_handle_t spi, const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
    spi_transaction_user_data_t u;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    memset(&u, 0, sizeof(u));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=&u;
    u.dc=0;                         //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Send data to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
void ili_data(spi_device_handle_t spi, const uint8_t *data, int len) 
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=NULL;                    //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void ili_spi_pre_transfer_callback(spi_transaction_t *t) 
{
    if ( t->user == NULL || ((spi_transaction_user_data_t*)t->user)->dc == 1 ) gpio_set_level(PIN_NUM_DC, 1);
    else gpio_set_level(PIN_NUM_DC, 0);
}

//Initialize the display
void ili_init(spi_device_handle_t spi) 
{
    int cmd=0;
    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //Send all the commands
    while (ili_init_cmds[cmd].databytes!=0xff) {
        ili_cmd(spi, ili_init_cmds[cmd].cmd);
        ili_data(spi, ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
        if (ili_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    ///Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 0);
}


void ili_draw_bitmap(spi_device_handle_t spi, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t * bitmap, int free_buffer)
{
    esp_err_t ret;
    int i;
    int pixel = w * h;
    int remain = pixel % 1024;
    int chunks = (pixel - 1) / 1024 + 1;

    ESP_LOGD(TAG, "bitmap address: 0x%08x", (unsigned int)bitmap);
    ESP_LOGD(TAG, "pixel: %d, chunks: %d, remain: %d", pixel, chunks, remain);

    spi_transaction_t * trans = calloc(5 + chunks, sizeof(spi_transaction_t));
    ESP_LOGD(TAG, "calloc'ed trans: 0x%08x", (unsigned int)trans);
    assert(trans);

    spi_transaction_user_data_t * userdata = calloc(4, sizeof(spi_transaction_user_data_t));
    ESP_LOGD(TAG, "calloc'ed userdata: 0x%08x", (unsigned int)userdata);
    assert(userdata);

    trans[0].length=8;
    trans[0].user=(void*)&userdata[0];
    trans[0].flags=SPI_TRANS_USE_TXDATA;
    trans[0].tx_data[0]=0x2A;           //Column Address Set
    userdata[0].dc=0;

    trans[1].length=8*4;
    trans[1].user=NULL;
    trans[1].flags=SPI_TRANS_USE_TXDATA;
    trans[1].tx_data[0]=x>>8;           //Start Col High
    trans[1].tx_data[1]=x&0xff;         //Start Col Low
    trans[1].tx_data[2]=(x+w-1)>>8;     //End Col High
    trans[1].tx_data[3]=(x+w-1)&0xff;   //End Col Low

    trans[2].length=8;
    trans[2].user=(void*)&userdata[1];
    trans[2].flags=SPI_TRANS_USE_TXDATA;
    trans[2].tx_data[0]=0x2B;           //Page address set
    userdata[1].dc=0;

    trans[3].length=8*4;
    trans[3].user=NULL;
    trans[3].flags=SPI_TRANS_USE_TXDATA;
    trans[3].tx_data[0]=y>>8;           //Start page high
    trans[3].tx_data[1]=y&0xff;         //start page low
    trans[3].tx_data[2]=(y+h-1)>>8;     //end page high
    trans[3].tx_data[3]=(y+h-1)&0xff;   //end page low

    trans[4].length=8;
    trans[4].flags=SPI_TRANS_USE_TXDATA;
    trans[4].user=(void*)&userdata[2];
    trans[4].tx_data[0]=0x2C;           //memory write
    userdata[2].dc=0;

    trans[4+chunks].user=(void*)&userdata[3];
    userdata[3].dc=1;
    userdata[3].free_buffer=free_buffer;
    userdata[3].buffer=(void*)bitmap;

    for (i=0; i<5; i++)
    {
        ESP_LOGD(TAG, "transmitting command and address: %d", i);
        ret=spi_device_queue_trans(spi, &trans[i], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    for (i=0; i<chunks; i++)
    {
        trans[5+i].tx_buffer=bitmap+1024*i;
        if ( i*1024 < pixel ) trans[5+i].length=16384;
        else trans[5+i].length=16*remain;
        ret=spi_device_queue_trans(spi, &trans[5+i], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

}


void send_line_finish(spi_device_handle_t spi) 
{
    spi_transaction_t *rtrans;
    esp_err_t ret;

    for (;;) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);

        if ( rtrans->user && ((spi_transaction_user_data_t*)rtrans->user)->free_buffer )
        {
            ESP_LOGD(TAG, "free'ing buffer: 0x%08x", (unsigned int)((spi_transaction_user_data_t*)rtrans->user)->buffer);
            free(((spi_transaction_user_data_t*)rtrans->user)->buffer);
        }
    }
}
