#pragma once
#include <Arduino.h>
#ifdef ESP32
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif
#include <gfx_bitmap.hpp>
#include <tft_driver.hpp>
namespace arduino {
namespace waveshare4in2gsc_helpers {
struct arrays {
    static const uint8_t* lut_vcom0;
    static const uint8_t* lut_ww;
    static const uint8_t* lut_bw;
    static const uint8_t* lut_wb;
    static const uint8_t* lut_bb;
    static const uint8_t* gray4_lut_vcom;
    static const uint8_t* gray4_lut_ww;
    static const uint8_t* gray4_lut_bw;
    static const uint8_t* gray4_lut_wb;
    static const uint8_t* gray4_lut_bb;
};
template <typename Source, size_t DitherBitDepth>
struct get_pixel_impl {
    inline static void get_pixel(const Source& frame_buffer, gfx::point16 location,
                          gfx::gsc_pixel<DitherBitDepth>* out_pixel) {
        frame_buffer.point(location, out_pixel);
    }
    inline static void get_pixel(const Source& frame_buffer, gfx::point16 location,
                          gfx::gsc_pixel<1>* out_pixel) {}
};
template <typename Source>
struct get_pixel_impl<Source, 1> {
    inline static void get_pixel(const Source& frame_buffer, gfx::point16 location,
                          gfx::gsc_pixel<1>* out_pixel) {
        frame_buffer.point(location, out_pixel);
    }
};

template <typename Driver>
struct driver_state {
    using driver = Driver;
    using type = driver_state;
    using bus = typename driver::bus;
    using bus_driver = typename driver::bus_driver;
    int is_bw;
    void* (*allocator)(size_t);
    void (*deallocator)(void*);
    static void reset() {
        digitalWrite(driver::pin_rst, LOW);
        delay(2);
        digitalWrite(driver::pin_rst, HIGH);
        delay(20);
        digitalWrite(driver::pin_rst, LOW);
        delay(2);
        digitalWrite(driver::pin_rst, HIGH);
        delay(20);
        digitalWrite(driver::pin_rst, LOW);
        delay(2);
        digitalWrite(driver::pin_rst, HIGH);
        delay(20);
    }
    static void initialize() {
        pinMode(driver::pin_wait, INPUT);
        bus_driver::initialize();
        bus::set_speed_multiplier(driver::write_speed_multiplier);
    }
    static void wait_busy() {
        uint32_t start = millis();
        while (LOW == digitalRead(driver::pin_wait) &&
               millis() - start < driver::timeout) {
            delay(1);
        }
    }
    static void expand_rect(gfx::rect16& dst, const gfx::rect16& src) {
        if(dst.x1==uint16_t(-1)) {
            dst=src;
            return;
        }
        if(src.x1<dst.x1) {
            dst.x1 = src.x1;
        }
        if(src.y1<dst.y1) {
            dst.y1 = src.y1;
        }
        if(src.x2>dst.x2) {
            dst.x2 = src.x2;
        }
        if(src.y2>dst.y2) {
            dst.y2 = src.y2;
        }
    }
    static void set_lut() {
        unsigned int count;
        bus_driver::send_command(0x20);  // vcom
        for (count = 0; count < 36; count++) {
            bus_driver::send_data8(pgm_read_byte(arrays::lut_vcom0 + count));
        }

        bus_driver::send_command(0x21);  // ww --
        for (count = 0; count < 36; count++) {
            bus_driver::send_data8(pgm_read_byte(arrays::lut_ww + count));
        }

        bus_driver::send_command(0x22);  // bw r
        for (count = 0; count < 36; count++) {
            bus_driver::send_data8(pgm_read_byte(arrays::lut_bw + count));
        }

        bus_driver::send_command(0x23);  // wb w
        for (count = 0; count < 36; count++) {
            bus_driver::send_data8(pgm_read_byte(arrays::lut_bb + count));
        }

        bus_driver::send_command(0x24);  // bb b
        for (count = 0; count < 36; count++) {
            bus_driver::send_data8(pgm_read_byte(arrays::lut_wb + count));
        }
    }

    static void set_gray_lut() {
        unsigned int count;
        bus_driver::send_command(0x20);  // vcom
        for (count = 0; count < 42; count++) {
            bus_driver::send_data8(
                pgm_read_byte(arrays::gray4_lut_vcom + count));
        }

        bus_driver::send_command(0x21);  // red not used
        for (count = 0; count < 42; count++) {
            bus_driver::send_data8(pgm_read_byte(arrays::gray4_lut_ww + count));
        }

        bus_driver::send_command(0x22);  // bw r
        for (count = 0; count < 42; count++) {
            bus_driver::send_data8(pgm_read_byte(arrays::gray4_lut_bw + count));
        }

        bus_driver::send_command(0x23);  // wb w
        for (count = 0; count < 42; count++) {
            bus_driver::send_data8(pgm_read_byte(arrays::gray4_lut_wb + count));
        }

        bus_driver::send_command(0x24);  // bb b
        for (count = 0; count < 42; count++) {
            bus_driver::send_data8(pgm_read_byte(arrays::gray4_lut_bb + count));
        }

        bus_driver::send_command(0x25);  // vcom
        for (count = 0; count < 42; count++) {
            bus_driver::send_data8(pgm_read_byte(arrays::gray4_lut_ww + count));
        }
    }
    static void initialize_bw() {
        reset();
        bus_driver::send_command(0x01);
        bus_driver::send_data8(0x03);  // VDS_EN, VDG_EN
        bus_driver::send_data8(0x00);  // VCOM_HV, VGHL_LV[1], VGHL_LV[0]
        bus_driver::send_data8(0x2b);  // VDH
        bus_driver::send_data8(0x2b);  // VDL

        bus_driver::send_command(0x06);
        bus_driver::send_data8(0x17);
        bus_driver::send_data8(0x17);
        bus_driver::send_data8(0x17);  // 07 0f 17 1f 27 2F 37 2f
        bus_driver::send_command(0x04);
        wait_busy();
        bus_driver::send_command(0x00);
        bus_driver::send_data8(0xbf);  // KW-BF   KWR-AF  BWROTP 0f

        bus_driver::send_command(0x30);
        bus_driver::send_data8(0x3c);  // 3A 100HZ   29 150Hz 39 200HZ  31 171HZ

        bus_driver::send_command(0x61);  // resolution setting

        bus_driver::send_data8(0x01);
        bus_driver::send_data8(0x90);  // 128

        bus_driver::send_data8(0x01);  //
        bus_driver::send_data8(0x2c);

        bus_driver::send_command(0x82);  // vcom_DC setting
        bus_driver::send_data8(0x12);

        bus_driver::send_command(0X50);  // VCOM AND DATA INTERVAL SETTING
        bus_driver::send_data8(
            0x97);  // 97white border 77black border    VBDF 17|D7 VBDW 97 VBDB
                    // 57    VBDF F7 VBDW 77 VBDB 37  VBDR B7

        set_lut();
    }
    static void initialize_gray() {
        /* EPD hardware init start */
        reset();
        bus_driver::send_command(0x01);  // POWER SETTING
        bus_driver::send_data8(0x03);
        bus_driver::send_data8(0x00);  // VGH=20V,VGL=-20V
        bus_driver::send_data8(0x2b);  // VDH=15V
        bus_driver::send_data8(0x2b);  // VDL=-15V
        bus_driver::send_data8(0x13);

        bus_driver::send_command(0x06);  // booster soft start
        bus_driver::send_data8(0x17);    // A
        bus_driver::send_data8(0x17);    // B
        bus_driver::send_data8(0x17);    // C

        bus_driver::send_command(0x04);
        wait_busy();

        bus_driver::send_command(0x00);  // panel setting
        bus_driver::send_data8(
            0x3f);  // KW-3f   KWR-2F	BWROTP 0f	BWOTP 1f

        bus_driver::send_command(0x30);  // PLL setting
        bus_driver::send_data8(0x3c);    // 100hz

        bus_driver::send_command(0x61);  // resolution setting
        bus_driver::send_data8(0x01);    // 400
        bus_driver::send_data8(0x90);
        bus_driver::send_data8(0x01);  // 300
        bus_driver::send_data8(0x2c);

        bus_driver::send_command(0x82);  // vcom_DC setting
        bus_driver::send_data8(0x12);

        bus_driver::send_command(0X50);  // VCOM AND DATA INTERVAL SETTING
        bus_driver::send_data8(0x97);
    }
    template <typename FrameBufferType>
    static void update_display_gray(const FrameBufferType& frame_buffer) {
        bus_driver::send_command(0x61);
        bus_driver::send_data8(driver::width >> 8);
        bus_driver::send_data8(driver::width & 0xff);
        bus_driver::send_data8(driver::height >> 8);
        bus_driver::send_data8(driver::height & 0xff);
        bus_driver::send_command(0x82);
        bus_driver::send_data8(0x12);
        bus_driver::send_command(0x50);
        bus_driver::send_command(0x97);  // V
        bus_driver::send_command(0x10);  // writes old data to SRAM.
        for (int y = 0; y < driver::height; y++) {
            for (int x = 0; x < driver::width; x += 8) {
                uint8_t data = 0;
                for (int i = 0; i < 8; ++i) {
                    const int xx = x + i;
                    typename FrameBufferType::pixel_type px;
                    frame_buffer.point(gfx::point16(xx, y), &px);
                    gfx::gsc_pixel<2> cpx;
                    gfx::convert(px, &cpx);
                    auto ii = cpx.template channel<gfx::channel_name::L>();
                    data |= (1 << (7 - i)) * (ii == 0 || ii == 1);
                }
                bus_driver::send_data8(~data);
            }
        }
        for (int m = 0; m < driver::height; m++)
            for (int i = 0; i < driver::width / 8; i++) {
                uint8_t temp3 = 0;
                for (int j = 0; j < 2; j++) {
                    temp3<<=4;
                    typename FrameBufferType::pixel_type px;
                    frame_buffer.point(gfx::point16(i+j, m), &px);

                    int temp1 = px.template channel<0>();
                    for (int k = 0; k < 2; k++) {
                        int temp2 = temp1 & 0xC0;
                        if (temp2 == 0xC0)
                            temp3 |= 0x01;  // white
                        else if (temp2 == 0x00)
                            temp3 |= 0x00;  // black
                        else if (temp2 == 0x80)
                            temp3 |= 0x00;  // gray1
                        else                // 0x40
                            temp3 |= 0x01;  // gray2
                        temp3 <<= 1;

                        temp1 <<= 2;
                        temp2 = temp1 & 0xC0;
                        if (temp2 == 0xC0)  // white
                            temp3 |= 0x01;
                        else if (temp2 == 0x00)  // black
                            temp3 |= 0x00;
                        else if (temp2 == 0x80)
                            temp3 |= 0x00;  // gray1
                        else                // 0x40
                            temp3 |= 0x01;  // gray2
                        if (j != 1 || k != 1) temp3 <<= 1;

                        temp1 <<= 2;
                    }
                }
                bus_driver::send_data8(temp3);
            
            }
        bus_driver::send_command(0x13);  // writes New data to SRAM.
        for (int y = 0; y < driver::height; y++) {
            for (int x = 0; x < driver::width; x += 8) {
                uint8_t data = 0;
                for (int i = 0; i < 8; ++i) {
                    const int xx = x + i;
                    typename FrameBufferType::pixel_type px;
                    frame_buffer.point(gfx::point16(xx, y), &px);
                    gfx::gsc_pixel<2> cpx;
                    gfx::convert(px, &cpx);
                    auto ii = cpx.template channel<gfx::channel_name::L>();
                    data |= (1 << (7 - i)) * (ii == 3);
                }
                bus_driver::send_data8(data);
            }
        }
        set_gray_lut();

        bus_driver::send_command(0x12);
        delay(100);
        wait_busy();
    }
    template <typename FrameBufferType>
    static void update_display_bw(const FrameBufferType& frame_buffer,
                                  bool dithering = true,bool refresh=true) {
        constexpr static const bool dithered =
            FrameBufferType::pixel_type::bit_depth > 1;
        bus_driver::send_command(0x61);
        bus_driver::send_data8(driver::width >> 8);
        bus_driver::send_data8(driver::width & 0xff);
        bus_driver::send_data8(driver::height >> 8);
        bus_driver::send_data8(driver::height & 0xff);

        bus_driver::send_command(0x82);
        bus_driver::send_data8(0x12);

        bus_driver::send_command(0x50);
        bus_driver::send_command(0x97);  // VBDF 17|D7 VBDW 97  VBDB 57  VBDF F7
                                         // VBDW 77  VBDB 37  VBDR B7

        bus_driver::send_command(0x10);
        for (int i = 0; i < driver::width / 8 * driver::height; i++) {
            bus_driver::send_data8(0xFF);  // bit set: white, bit reset: black
        }
        delay(2);
        bus_driver::send_command(0x13);
        if (!dithered) {
            gfx::gsc_pixel<1> px;
            for (int y = 0; y < driver::height; ++y) {
                for (int x = 0; x < driver::width; x += 8) {
                    uint8_t b = 0;
                    for (int xx = x; xx < (x + 8); ++xx) {
                        waveshare4in2gsc_helpers::get_pixel_impl<
                            FrameBufferType,
                            FrameBufferType::pixel_type::bit_depth>::
                            get_pixel(frame_buffer, {uint16_t(xx), uint16_t(y)},
                                      &px);
                        b |= (1 << (7 - (xx - x))) * (!!px.native_value);
                    }
                    bus_driver::send_data8(b);
                }
            }
        } else {
            typename FrameBufferType::pixel_type px;
            if (dithering) {
                for (int y = 0; y < driver::height; ++y) {
                    int row = y & 15;
                    for (int x = 0; x < driver::width; x += 8) {
                        uint8_t b = 0;
                        for (int xx = x; xx < (x + 8); ++xx) {
                            int col = xx & 15;
                            waveshare4in2gsc_helpers::get_pixel_impl<
                                FrameBufferType,
                                FrameBufferType::pixel_type::bit_depth>::
                                get_pixel(frame_buffer,
                                          {uint16_t(xx), uint16_t(y)}, &px);
                            b |= (1 << (7 - (xx - x))) *
                                 !!(255.0 * px.template channelr<
                                                gfx::channel_name::L>() >
                                    gfx::helpers::dither::bayer_16[col +
                                                                   row * 16]);
                        }
                        bus_driver::send_data8(b);
                    }
                }
            } else {
                for (int y = 0; y < driver::height; ++y) {
                    for (int x = 0; x < driver::width; x += 8) {
                        uint8_t b = 0;
                        for (int xx = x; xx < (x + 8); ++xx) {
                            waveshare4in2gsc_helpers::get_pixel_impl<
                                FrameBufferType,
                                FrameBufferType::pixel_type::bit_depth>::
                                get_pixel(frame_buffer,
                                          {uint16_t(xx), uint16_t(y)}, &px);
                            b |= (1 << (7 - (xx - x))) *
                                 !!(px.template channelr<
                                        gfx::channel_name::L>() >= .5);
                        }
                        bus_driver::send_data8(b);
                    }
                }
            }
        }
        delay(2);
        if(refresh) {
            bus_driver::send_command(0x12);
            delay(100);
            wait_busy();
        }
    }
    template<typename FrameBufferType> 
    static void write_window_part(const FrameBufferType& frame_buffer, int x1,int x2,const gfx::rect16& bounds,bool dithering) {
        if(dithering) {
            for(int y = bounds.y1;y<=bounds.y2;++y) {
                int row = y&15;
                for(int x=x1;x<=x2;x+=8) {
                    uint8_t b = 0;
                    for(int bx=0;bx<8;++bx) {
                        int xx = (bx+x);
                        int col = xx & 15;
                        typename FrameBufferType::pixel_type px;
                        frame_buffer.point(gfx::point16(xx,y),&px);
                        b|=(1<<(7-bx))*
                                 !!(255.0 * px.template channelr<
                                                gfx::channel_name::L>() >
                                    gfx::helpers::dither::bayer_16[col +
                                                                   row * 16]);
                    }
                    bus_driver::send_data8(b);
                }
            }
        } else {
            // this is the one getting called
            // everything about this loop has been tested
            // in the update_display_bw() routine which contains 
            // the same code
            for(int y = bounds.y1;y<=bounds.y2;++y) {
                for(int x=x1;x<=x2;x+=8) {
                    uint8_t b = 0;
                    for(int bx=0;bx<8;++bx) {
                        typename FrameBufferType::pixel_type px;
                        frame_buffer.point(gfx::point16(bx+x,y),&px);
                        b|=(1<<(7-bx))*!!px.native_value;
                    }
                    bus_driver::send_data8(b);
                }
            }
        }
    }
    template<typename FrameBufferType> 
    static void update_display_bw_part(const FrameBufferType& frame_buffer,const gfx::rect16& bounds,bool dithering) {
        //Serial.printf("(%d,%d)-(%d,%d)\r\n",bounds.x1,bounds.y1,bounds.x2,bounds.y2);
        // i've checked these constants over and over again. they're right:
        bus_driver::send_command(0x91);
        bus_driver::send_command(0x90);
        int x1=bounds.x1&0xF8;
        int x2=bounds.x2|0x7;
        bus_driver::send_data8(x1 >> 8);
        bus_driver::send_data8(x1 & 0xff);     // x should be the multiple of 8, the last 3 bit will always be ignored
        bus_driver::send_data8(x2 >> 8);
        bus_driver::send_data8(x2 & 0xff);
        bus_driver::send_data8(bounds.y1 >> 8);        
        bus_driver::send_data8(bounds.y1 & 0xff);
        bus_driver::send_data8(bounds.y2 >> 8);        
        bus_driver::send_data8(bounds.y2 & 0xff);
        bus_driver::send_data8(0x01);         // Gates scan both inside and outside of the partial window. (default) 
        delay(2);
        
        bus_driver::send_command(0x13);
        
        write_window_part(frame_buffer,x1,x2,bounds,dithering);
        delay(2);

        bus_driver::send_command(0x92);  
       
    }
};
template <typename Driver>
struct driver_state_holder {
    static driver_state<Driver> instance;
};
template <typename Driver>
driver_state<Driver> driver_state_holder<Driver>::instance = {-1, ::malloc,
                                                              ::free};
}  // namespace waveshare4in2gsc_helpers
template <typename Driver, size_t DitherBitDepth = 0>
struct waveshare4in2gsc_mode final {
    using driver = Driver;
    using pixel_type = gfx::gsc_pixel<DitherBitDepth>;
    using caps = gfx::gfx_caps<false, false, false, false, true, true, false>;
    using frame_buffer_type = gfx::large_bitmap<pixel_type>;
    using type = waveshare4in2gsc_mode;
    constexpr static const bool dithered = DitherBitDepth != 1;

   private:
    using state_type = waveshare4in2gsc_helpers::driver_state<driver>;
    using state_holder = waveshare4in2gsc_helpers::driver_state_holder<driver>;
    frame_buffer_type m_frame_buffer;
    int m_suspend_count;
    gfx::rect16 m_suspend_bounds;
    bool m_dithering;
    waveshare4in2gsc_mode(const waveshare4in2gsc_mode& rhs) = delete;
    waveshare4in2gsc_mode& operator=(const waveshare4in2gsc_mode& rhs) = delete;
    void update() {
        if(m_suspend_count==0) {
            if(m_suspend_bounds.x1==-1 || m_suspend_bounds == bounds()) {
                m_suspend_bounds=bounds();
                state_type::update_display_bw(m_frame_buffer,dithering());
            } else {
                // TODO: partial update isn't working
                //state_type::update_display_bw_part(m_frame_buffer,m_suspend_bounds,dithering());
                state_type::update_display_bw(m_frame_buffer,dithering());
            }
            m_suspend_bounds.x1 = -1;
        }
    }
   public:
    waveshare4in2gsc_mode(waveshare4in2gsc_mode&& rhs)
        : m_frame_buffer(dimensions(), 1, nullptr,
                         state_holder::instance.allocator,
                         state_holder::instance.deallocator) {
        m_suspend_count = rhs.m_suspend_count;
        m_suspend_bounds = rhs.m_suspend_bounds;
        m_dithering = rhs.m_dithering;
    }
    waveshare4in2gsc_mode& operator=(waveshare4in2gsc_mode&& rhs) {
        m_suspend_count = rhs.m_suspend_count;
        m_suspend_bounds = rhs.m_suspend_bounds;
        m_dithering = rhs.m_dithering;
        return *this;
    }
    waveshare4in2gsc_mode()
        : m_frame_buffer(dimensions(), 1, nullptr,
                         state_holder::instance.allocator,
                         state_holder::instance.deallocator),
          m_suspend_count(0),
          m_suspend_bounds(-1,-1,-1,-1),
          m_dithering(dithered) {}
    inline bool initialized() const {
        return state_holder::instance.is_bw == 0;
    }
    gfx::gfx_result initialize() {
        if (0 != state_holder::instance.is_bw) {
            if (0 > state_holder::instance.is_bw) {
                state_type::initialize();
            }
            state_holder::instance.is_bw = 0;
            state_type::initialize_bw();
        }
        return gfx::gfx_result::success;
    }
    inline bool dithering() { return dithered && m_dithering; }
    inline void dithering(bool value) { m_dithering = value; }
    inline gfx::size16 dimensions() const {
        return {driver::width, driver::height};
    }
    inline gfx::rect16 bounds() const { return dimensions().bounds(); }
    inline gfx::gfx_result clear(const gfx::rect16& bounds) {
        pixel_type px;
        return fill(bounds, px);
    }
    inline gfx::gfx_result fill(const gfx::rect16& bounds, pixel_type color) {
        gfx::rect16 rb = bounds.normalize().crop(this->bounds());
        if(!rb.intersects(this->bounds())) {
            return gfx::gfx_result::success;
        }
        gfx::gfx_result rr = initialize();
        if (rr != gfx::gfx_result::success) {
            return rr;
        }
        rr = m_frame_buffer.fill(rb, color);
        if (rr != gfx::gfx_result::success) {
            return rr;
        }
        state_type::expand_rect(m_suspend_bounds,rb);
        update();
        return gfx::gfx_result::success;
    }
    gfx::gfx_result point(gfx::point16 location, pixel_type color) {
        if(!bounds().intersects(location)) {
            return gfx::gfx_result::success;
        }
        gfx::gfx_result r = initialize();
        if (r != gfx::gfx_result::success) {
            return r;
        }
        r = m_frame_buffer.point(location, color);
        if (r != gfx::gfx_result::success) {
            return r;
        }
        state_type::expand_rect(m_suspend_bounds,{location.x,location.y,location.x,location.y});
        update();
        return gfx::gfx_result::success;
    }
    gfx::gfx_result point(gfx::point16 location, pixel_type* out_color) const {
        if (!initialized()) {
            return gfx::gfx_result::invalid_state;
        }
        return m_frame_buffer.point(location, out_color);
    }
    inline gfx::gfx_result suspend() {
        ++m_suspend_count;
        return gfx::gfx_result::success;
    }
    gfx::gfx_result resume(bool force = false) {
        if (force || 0 == --m_suspend_count) {
            m_suspend_count = 0;
            gfx::gfx_result r = initialize();
            if (r != gfx::gfx_result::success) {
                return r;
            }
            update();
        }
        return gfx::gfx_result::success;
    }
};
template <typename Driver>
struct waveshare4in2gsc_mode<Driver, 0> final {
    using driver = Driver;
    using pixel_type = gfx::gsc_pixel<2>;
    using caps = gfx::gfx_caps<false, false, false, false, true, true, false>;
    using frame_buffer_type = gfx::large_bitmap<pixel_type>;
    using type = waveshare4in2gsc_mode;
    constexpr static const bool dithered = false;

   private:
    using state_type = waveshare4in2gsc_helpers::driver_state<driver>;
    using state_holder = waveshare4in2gsc_helpers::driver_state_holder<driver>;
    frame_buffer_type m_frame_buffer;
    int m_suspend_count;
    waveshare4in2gsc_mode(const waveshare4in2gsc_mode& rhs) = delete;
    waveshare4in2gsc_mode& operator=(const waveshare4in2gsc_mode& rhs) = delete;

   public:
    waveshare4in2gsc_mode(waveshare4in2gsc_mode&& rhs)
        : m_frame_buffer(dimensions(), 1, nullptr,
                         state_holder::instance.allocator,
                         state_holder::instance.deallocator) {
        m_suspend_count = rhs.m_suspend_count;
    }
    waveshare4in2gsc_mode& operator=(waveshare4in2gsc_mode&& rhs) {
        m_suspend_count = rhs.m_suspend_count;
        return *this;
    }
    waveshare4in2gsc_mode()
        : m_frame_buffer(dimensions(), 1, nullptr,
                         state_holder::instance.allocator,
                         state_holder::instance.deallocator),
          m_suspend_count(0) {}
    inline bool initialized() const {
        return state_holder::instance.is_bw == 1;
    }
    gfx::gfx_result initialize() {
        if (1 != state_holder::instance.is_bw) {
            if (0 > state_holder::instance.is_bw) {
                state_type::initialize();
            }
            state_holder::instance.is_bw = 1;
            state_type::initialize_gray();
        }
        return gfx::gfx_result::success;
    }
    inline gfx::size16 dimensions() const {
        return {driver::width, driver::height};
    }
    inline gfx::rect16 bounds() const { return dimensions().bounds(); }
    inline gfx::gfx_result clear(const gfx::rect16& bounds) {
        pixel_type px;
        return fill(bounds, px);
    }
    inline gfx::gfx_result fill(const gfx::rect16& bounds, pixel_type color) {
        gfx::gfx_result r = initialize();
        if (r != gfx::gfx_result::success) {
            return r;
        }
        r = m_frame_buffer.fill(bounds, color);
        if (r != gfx::gfx_result::success) {
            return r;
        }
        if (!m_suspend_count) {
            state_type::update_display_gray(m_frame_buffer);
        }
        return gfx::gfx_result::success;
    }
    gfx::gfx_result point(gfx::point16 location, pixel_type color) {
        gfx::gfx_result r = initialize();
        if (r != gfx::gfx_result::success) {
            return r;
        }
        r = m_frame_buffer.point(location, color);
        if (r != gfx::gfx_result::success) {
            return r;
        }
        if (!m_suspend_count) {
            state_type::update_display_gray(m_frame_buffer);
        }
        return gfx::gfx_result::success;
    }
    gfx::gfx_result point(gfx::point16 location, pixel_type* out_color) const {
        if (!initialized()) {
            return gfx::gfx_result::invalid_state;
        }
        return m_frame_buffer.point(location, out_color);
    }
    inline gfx::gfx_result suspend() {
        ++m_suspend_count;
        return gfx::gfx_result::success;
    }
    gfx::gfx_result resume(bool force = false) {
        if (force || 0 == --m_suspend_count) {
            m_suspend_count = 0;
            gfx::gfx_result r = initialize();
            if (r != gfx::gfx_result::success) {
                return r;
            }
            state_type::update_display_gray(m_frame_buffer);
        }
        return gfx::gfx_result::success;
    }
};
template <int8_t PinDC, int8_t PinRst, int8_t PinWait, typename Bus,
          unsigned int WriteSpeedPercent = 40>
struct waveshare4in2gsc final {
    constexpr static const uint16_t width = 400;
    constexpr static const uint16_t height = 300;
    constexpr static const int8_t pin_dc = PinDC;
    constexpr static const int8_t pin_rst = PinRst;
    constexpr static const int8_t pin_wait = PinWait;
    constexpr static const float write_speed_multiplier =
        (WriteSpeedPercent / 100.0);
    constexpr static const int timeout = 5000;
    using bus = Bus;
    using bus_driver = tft_driver<pin_dc, pin_rst, -1, bus>;

   private:
    using state_type = waveshare4in2gsc_helpers::driver_state<waveshare4in2gsc>;
    using state_holder =
        waveshare4in2gsc_helpers::driver_state_holder<waveshare4in2gsc>;
    waveshare4in2gsc(const waveshare4in2gsc& rhs) = delete;
    waveshare4in2gsc& operator=(const waveshare4in2gsc& rhs) = delete;

   public:
    waveshare4in2gsc(waveshare4in2gsc&& rhs) {}

    waveshare4in2gsc& operator=(waveshare4in2gsc&& rhs) { return *this; }

    template <size_t BitDepth = 0>
    inline waveshare4in2gsc_mode<waveshare4in2gsc, BitDepth> mode() {
        waveshare4in2gsc_mode<waveshare4in2gsc, BitDepth> result;
        return result;
    }

    waveshare4in2gsc(void*(allocator)(size_t) = ::malloc,
                     void(deallocator)(void*) = ::free) {
        state_holder::instance.allocator = allocator;
        state_holder::instance.deallocator = deallocator;
        state_holder::instance.is_bw = -1;
    }
};
}  // namespace arduino