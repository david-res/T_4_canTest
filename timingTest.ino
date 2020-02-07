#include "SPI.h"
#include <FlexCAN_T4.h>
#include <ILI9341_t3n.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
CAN_message_t msgTx, msgRx;

#define COLOR_RED   ILI9341_RED
#define COLOR_GREEN ILI9341_GREEN
#define COLOR_BLUE  ILI9341_BLUE
#define COLOR_YELLOW ILI9341_YELLOW
#define COLOR_BLACK ILI9341_BLACK

//
#include "defi_n_150.c" //the picture
// Normal Connections
//#define USE_SPI1
//#define USE_SPI2
//-----------------------------
// SPI1
//-----------------------------
#ifdef USE_SPI1
#ifdef __IMXRT1062__
#define TFT_DC       9
#define TFT_CS      10
#define TFT_RST     8  // 255 = unused, connect to 3.3V
#define TFT_MOSI    26
#define TFT_SCLK    27
#define TFT_MISO    1
#else
#define TFT_DC      31
#define TFT_CS      10
#define TFT_RST     8  // 255 = unused, connect to 3.3V
#define TFT_MOSI    0
#define TFT_SCLK    32
#define TFT_MISO    1
#endif
//-----------------------------
// SPI2
//-----------------------------
#elif defined(USE_SPI2)
#ifdef __IMXRT1062__
#define TFT_DC       9
#define TFT_CS      10
#define TFT_RST     8  // 255 = unused, connect to 3.3V
#define TFT_MOSI    35
#define TFT_SCLK    37
#define TFT_MISO    34
#else
#define TFT_DC      43
#define TFT_CS      10
#define TFT_RST     8  // 255 = unused, connect to 3.3V
#define TFT_MOSI    44
#define TFT_SCLK    46
#define TFT_MISO    45
#endif
//-----------------------------
// SPI - Default
//-----------------------------
#else
#define TFT_DC       9
#define TFT_CS      10
#define TFT_RST    255  // 255 = unused, connect to 3.3V
#define TFT_MOSI    11
#define TFT_SCLK    13
#define TFT_MISO    12
#endif

ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

enum {
    INTAKE_MANIFOLD_ABSOLUTE_PRESSURE = 0x0b,
    ABSOLULTE_BAROMETRIC_PRESSURE = 0x33,
   
};

//colors
#define background 0x2945



uint16_t g_center_x;
uint16_t g_center_y;
uint16_t g_offset_x;
uint16_t g_offset_y;

int16_t g_needle_rect_min_x;
int16_t g_needle_rect_min_y;
int16_t g_needle_rect_max_x;
int16_t g_needle_rect_max_y;

void setup() {
  while (!Serial && millis() < 5000) ;
  Serial.begin(115200);
  //Serial.printf("CS:%d, DC:%d RST:%d MOSI:%d, SCLK:%d MISO:%d\n", TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

 
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMBFilter(MB0,0x7E8);
  Can0.enableMBInterrupt(MB0);
  Can0.onReceive(canSniff);




  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(COLOR_RED);
  tft.fillScreen(COLOR_GREEN);
  tft.fillScreen(COLOR_BLUE);
  delay(100);
  tft.fillScreen(COLOR_BLACK);
  g_center_x = tft.width() / 4;
  g_center_y = tft.height() / 2;

  g_offset_x = 5; //(tft.width() - 150) / 2;
  g_offset_y = 45; //(tft.height() - 150) / 2;
  Serial.printf("center(%u, %u) offset(%u %u)\n", g_center_x, g_center_y, g_offset_x, g_offset_y);
  tft.writeRect(g_offset_x, g_offset_y, 150, 150, (const uint16_t*)defi);
  tft.useFrameBuffer(true);
  tft.fillScreen(COLOR_BLACK);
}


const long loopDelay1 = 10;
unsigned long timeNow1, timeNow2 = 0;
float canData [2] ; //place up to date CAN data into this array

int boost_percent;
float boost_pressuer, boost_final;

void loop(void) {
    if(millis() > timeNow1 + loopDelay1){
        timeNow1 = millis();
        elapsedMillis em = 0; 
        canSend(INTAKE_MANIFOLD_ABSOLUTE_PRESSURE);
        canSend(ABSOLULTE_BAROMETRIC_PRESSURE);
        Can0.events();
        Serial.printf("elapsed PID loop poll %u\n", (uint32_t)em);
        em = 0;
    }

        elapsedMillis em2 = 0; 
        boost_pressuer = (canData [0] - 102);
        boost_final = boost_pressuer * 0.145038;


    if (boost_pressuer < 0 ){
        boost_percent = map(boost_pressuer, -110, -0.99,0,249);
    }

    if (boost_pressuer >= 0.1) {
        boost_percent = map(boost_pressuer, 0.1, 200.0, 250, 750);
    }


    int16_t min_x = g_needle_rect_min_x;
    int16_t min_y = g_needle_rect_min_y;
    int16_t max_x = g_needle_rect_max_x;
    int16_t max_y = g_needle_rect_max_y;
    tft.writeRect(g_offset_x, g_offset_y, 150, 150, (const uint16_t*)defi);
    

    drawNeedle(boost_percent, COLOR_RED);
    //tft.setTextColor(ILI9341_WHITE);
    //tft.setTextSize(2);
    //tft.setCursor(90, 220);
    //tft.print (boost_final, 1);
    
    min_x = min(min_x, g_needle_rect_min_x);
    min_y = min(min_y, g_needle_rect_min_y);
    max_x = max(max_x, g_needle_rect_max_x);
    max_y = max(max_y, g_needle_rect_max_y);
    //tft.setClipRect(5, 45, 150, 150);
    tft.setClipRect(min_x, min_y, max_x - min_x + 1, max_y - min_y +1);
    tft.updateScreen();
    tft.setClipRect();
    Serial.printf("elapsed Screen Update %u\n", (uint32_t)em2);
    em2 = 0;

}


void canSend(uint16_t pid){  //Send the PID requests
  elapsedMillis em1 = 0; 

  
  msgTx.buf[0] = 0x02;  // Two bytes in the request
  msgTx.buf[1] = 0x01; // OBD mode 1
  msgTx.buf[2] = pid;  
  msgTx.buf[3] = 0;
  msgTx.buf[4] = 0;  
  msgTx.buf[5] = 0;
  msgTx.buf[6] = 0;  
  msgTx.buf[7] = 0;
  msgTx.len = 8; // number of bytes in request
  msgTx.flags.extended = 0; // 11 bit header, not 29 bit
  msgTx.flags.remote = 0;
  msgTx.id = 0x7E0; // request header for OBD

  Can0.write(msgTx);  

  Serial.printf("elapsed TX %u\n", (uint32_t)em1);
  em1 = 0;
}


void canSniff(const CAN_message_t &msg) { // global callback

    #define res_count msg.buf[0] // numper of bytes returned
    #define req_mode msg.buf[1] // mode 0x01 or 0x22
    #define a msg.buf[2] // PID
    #define b msg.buf[3] // res1 or PID (for mode 0x22)
    #define c msg.buf[4] // res 2 or 1
    #define d msg.buf[5] // res 3 or 2
    #define e msg.buf[6] // res 4 or 3
    #define f msg.buf[7] // res 5 or 4

    if(req_mode == 0x01){
          switch (a){
            case INTAKE_MANIFOLD_ABSOLUTE_PRESSURE:
            canData [0] = b;
            break;
            
            case ABSOLULTE_BAROMETRIC_PRESSURE:
            canData [1] = b;
            break;                  
        }
      }
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
    Serial.print("  EXT: "); Serial.print(msg.flags.extended );
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    for ( uint8_t i = 0; i < 8; i++ ) {
      Serial.print(msg.buf[i]); Serial.print(" ");
    }
}


const double NEEDLE_LEN = 60;
const double TRI_HALF_WIDTH = 2;
void drawNeedle(int percent, uint16_t color) {
  double rads = HALF_PI + TWO_PI * percent/ 1000.0;
  uint16_t x0 = cos(rads) * NEEDLE_LEN + g_center_x;
  uint16_t y0 = sin(rads) * NEEDLE_LEN + g_center_y;
  uint16_t x1 = cos(rads - HALF_PI) * TRI_HALF_WIDTH + g_center_x;
  uint16_t y1 = sin(rads - HALF_PI) * TRI_HALF_WIDTH + g_center_y;
  uint16_t x2 = cos(rads + HALF_PI) * TRI_HALF_WIDTH + g_center_x;
  uint16_t y2 = sin(rads + HALF_PI) * TRI_HALF_WIDTH + g_center_y;
  /*
    Serial.print("Percent: ");Serial.print(percent, 3);
    Serial.print("Rads: ");Serial.print(rads, 3);
    Serial.printf(" (%u, %u) (%u %u) (%u %u)\n", x0, y0, x1, y1, x2, y2);
  */
  tft.fillTriangle(x0, y0, x1, y1, x2, y2, color);
  g_needle_rect_min_x = min(min(x0, x1), x2);
  g_needle_rect_min_y = min(min(y0, y1), y2);
  g_needle_rect_max_x = max(max(x0, x1), x2);
  g_needle_rect_max_y = max(max(y0, y1), y2);
}
