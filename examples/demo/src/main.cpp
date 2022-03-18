#ifndef BOARD_HAS_PSRAM
static_assert(false,"PSRAM must be enabled to use this device");
#endif

#define EPD_HOST    VSPI
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_DC   2
#define PIN_NUM_RST  4
#define PIN_NUM_WAIT 15

#include <Arduino.h>
#include <tft_io.hpp>
#include <waveshare5in65f.hpp>
#include <gfx_cpp14.hpp>
#include "Maziro.h"
#include "image.h"
using namespace arduino;
using namespace gfx;
using epd_bus = tft_spi_ex<EPD_HOST,PIN_NUM_CS,PIN_NUM_MOSI,PIN_NUM_MISO,PIN_NUM_CLK,SPI_MODE0>;
waveshare4in2gsc<PIN_NUM_DC,PIN_NUM_RST,PIN_NUM_WAIT,epd_bus> drv;

// with this driver we must select a mode first.
// mode<>() takes a bit depth. if it's 0, the native grayscale is used
// if it's 1, monochrome is used
// if it's any other number it's a grayscale of that bit depth, 
// dithered to monochrome. Note that each mode has its own
// frame buffer, so it's a good idea to keep only one around
// at once.
auto epd = drv.mode<4>();
void setup() {
  Serial.begin(115200);

}

void loop() {
  // epd.dithering(false); // uncomment this to disable dithering for subsequent updates.
  using epd0_color = color<decltype(epd)>;
  draw::suspend(epd);
  draw::filled_rectangle(epd,(srect16)epd.bounds(),epd0_color::white);
  const char* text = "GFX DEMO";
  const open_font& f=Maziro_ttf;
  float scale = f.scale(60);
  srect16 text_rect = f.measure_text((ssize16)epd.dimensions(),{0,0},
                          text,scale).bounds();
  draw::text(epd,
          text_rect.center((srect16)epd.bounds()),
          {0,0},
          text,
          f,scale,
          epd0_color::blue,epd0_color::white,false,true /* anti-aliasing doesn't work great with this display due to no grays*/);
  for(int i = 1;i<100;i+=2) {
      // calculate our extents
      srect16 r(i*(epd.dimensions().width/100.0),
              i*(epd.dimensions().height/100.0),
              epd.dimensions().width-i*(epd.dimensions().width/100.0)-1,
              epd.dimensions().height-i*(epd.dimensions().height/100.0)-1);
      // draw the four lines
      draw::line(epd,srect16(0,r.y1,r.x1,epd.dimensions().height-1),epd0_color::green);
      draw::line(epd,srect16(r.x2,0,epd.dimensions().width-1,r.y2),epd0_color::red);
      draw::line(epd,srect16(0,r.y2,r.x1,0),epd0_color::yellow);
      draw::line(epd,srect16(epd.dimensions().width-1,r.y1,r.x2,epd.dimensions().height-1),epd0_color::orange);
      
  }
  /* // use indexed colors:
    for(int i = 1;i<100;i+=2) {
      // calculate our extents
      srect16 r(i*(epd.dimensions().width/100.0),
              i*(epd.dimensions().height/100.0),
              epd.dimensions().width-i*(epd.dimensions().width/100.0)-1,
              epd.dimensions().height-i*(epd.dimensions().height/100.0)-1);
      // draw the four lines
      draw::line(epd,srect16(0,r.y1,r.x1,epd.dimensions().height-1),epd0ipt(2));
      draw::line(epd,srect16(r.x2,0,epd.dimensions().width-1,r.y2),epd0ipt(3));
      draw::line(epd,srect16(0,r.y2,r.x1,0),epd0ipt(4));
      draw::line(epd,srect16(epd.dimensions().width-1,r.y1,r.x2,epd.dimensions().height-1),epd0ipt(6));
      
  }
  */
  draw::resume(epd);
  epd.sleep(); // saves power, but takes longer to draw the next frame
  delay(2000);
  draw::suspend(epd);
  draw::filled_rectangle(epd,(srect16)epd.bounds(),epd0_color::black);
  
  for(int y = 0;y<epd.dimensions().height;y+=16) {
      for(int x = 0;x<epd.dimensions().width;x+=16) {
          if(0!=((x+y)%32)) {
              draw::filled_rectangle(epd,
                                  srect16(
                                      spoint16(x,y),
                                      ssize16(16,16)),
                                  epd0_color::white);
          }
      }
  }
  
  draw::image(epd,srect16(0,0,335,255).center((srect16)epd.bounds()),&image_jpg_stream);
  draw::resume(epd);
  delay(2000);
}