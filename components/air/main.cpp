//#include <Arduino.h>
#include <algorithm>

#include "esp_camera.h"
//#include "EEPROM.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
//#include <driver/adc.h>
//#include "esp_adc_cal.h"
#include "esp_wifi_types.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
//#include "esp_wifi_internal.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_private/wifi.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_mac.h"
//#include "bt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include <unistd.h>
#include <fcntl.h>
#include "esp_random.h"

#include "fec_codec.h"
#include "packets.h"
#include "safe_printf.h"
#include "structures.h"
#include "crc.h"
#include "driver/gpio.h"
#include "main.h"
#include "queue.h"
#include "circular_buffer.h"

#include "ll_cam.h" // cam_obj_t defination, used in camera_data_available

#include "wifi.h"
#include "nvs_args.h"

#include "osd.h"
#include "msp.h"
#include "avi.h"

#include "vcd_profiler.h"

#include "jpeg_parser.h"
#include "util.h"

#include "hx_mavlink_parser.h"

#include "temperature_sensor.h"

static int s_stats_last_tp = -10000;
static int s_last_osd_packet_tp = -10000;
static int s_last_config_packet_tp = -10000;

#define MJPEG_PATTERN_SIZE 512 

/////////////////////////////////////////////////////////////////////////

static size_t s_video_frame_data_size = 0;
static uint32_t s_video_frame_index = 0;
static uint8_t s_video_part_index = 0;
static bool s_video_frame_started = false;
static size_t s_video_full_frame_size = 0;
static uint8_t s_osdUpdateCounter = 0;
static bool s_lastByte_ff = false;

static int s_actual_capture_fps = 0;
static int s_actual_capture_fps_expected = 0;

static int s_quality = 20;
static float s_quality_framesize_K1 = 0; //startup from minimum quality to decrease pressure
static float s_quality_framesize_K2 = 1;
static float s_quality_framesize_K3 = 1;
static int s_max_frame_size = 0;
static int s_sharpness = 20;

static int64_t s_video_last_sent_tp = esp_timer_get_time();
static int64_t s_video_target_frame_dt = 0;
static uint8_t s_max_wlan_outgoing_queue_usage = 0;
static uint8_t s_min_wlan_outgoing_queue_usage_seen = 0;

static int64_t s_last_seen_config_packet = esp_timer_get_time();;

static int64_t s_restart_time = 0;
static int64_t s_wifi_ovf_time = 0;

extern WIFI_Rate s_wlan_rate;

bool SDError = false;
uint16_t SDTotalSpaceGB16 = 0;
uint16_t SDFreeSpaceGB16 = 0;
static uint8_t cam_ovf_count = 0;
static float s_camera_temperature = 0;

int32_t s_dbg;
uint16_t s_framesCounter = 0;

static bool s_initialized = false;

static uint32_t s_last_rc_packet_tp = 0;

static uint16_t s_air_device_id;
static uint16_t s_connected_gs_device_id = 0;

static int64_t s_accept_connection_timeout_ms = 0;

#ifdef UART_MAVLINK

//constexpr size_t MAX_TELEMETRY_PAYLOAD_SIZE = AIR2GROUND_MTU - sizeof(Air2Ground_Data_Packet);
//constexpr size_t MAX_TELEMETRY_PAYLOAD_SIZE = 512;
constexpr size_t MAX_TELEMETRY_PAYLOAD_SIZE = 128;

static uint8_t s_mavlink_out_buffer[MAX_TELEMETRY_PAYLOAD_SIZE];
static int s_mavlinkOutBufferCount = 0;
#endif
/////////////////////////////////////////////////////////////////////////

static int s_uart_verbose = 1;

#define LOG(...) do { if (s_uart_verbose > 0) SAFE_PRINTF(__VA_ARGS__); } while (false) 

/////////////////////////////////////////////////////////////////////////

sdmmc_card_t* card = nullptr;

static bool s_air_record = false;

static bool s_camera_stopped = false;
static bool s_camera_stopped_requested = false;

static uint64_t s_shouldRestartRecording;

HXMavlinkParser mavlinkParserIn(true);

//=============================================================================================
//=============================================================================================
#ifdef GPIO_CONTROL_PIN
static bool s_gpio_control_state = false;
#endif

static bool s_i2c_initialized = false;

//=============================================================================================
//=============================================================================================
void initialize_i2c()
{
    if (s_i2c_initialized) return;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;  // Use default clock flags

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        LOG("Failed to configure I2C parameters: %d\n", err);
        return;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        LOG("Failed to install I2C driver: %d\n", err);
        return;
    }

    s_i2c_initialized = true;
    LOG("I2C initialized on SDA: %d, SCL: %d\n", I2C_SDA_PIN, I2C_SCL_PIN);
}

//=============================================================================================
//=============================================================================================
void send_i2c_command(uint8_t command)
{
    if (!s_i2c_initialized) {
        initialize_i2c();
        if (!s_i2c_initialized) {
            LOG("Failed to initialize I2C\n");
            return;
        }
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        LOG("I2C command %d sent successfully\n", command);
    } else {
        LOG("Failed to send I2C command %d, error: %d\n", command, ret);
    }
}

//=============================================================================================
//=============================================================================================
void initialize_gpio_control_pin()
{
#ifdef GPIO_CONTROL_PIN
    gpio_config_t io_conf;
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = 1ULL << GPIO_CONTROL_PIN;
    io_conf.mode = GPIO_MODE_OUTPUT;
    // io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    gpio_set_level(GPIO_CONTROL_PIN, 0); // Start with GPIO off
    s_gpio_control_state = false;
    LOG("GPIO control initialized on pin %d, state: OFF\n", GPIO_CONTROL_PIN);
#endif
}

//=============================================================================================
void set_gpio_control_pin(bool enabled)
{
#ifdef GPIO_CONTROL_PIN
    gpio_set_level(GPIO_CONTROL_PIN, enabled ? 1 : 0);
    s_gpio_control_state = enabled;
    LOG("GPIO control pin %d set to %s\n", GPIO_CONTROL_PIN, enabled ? "HIGH" : "LOW");
#endif
}

//=============================================================================================
//=============================================================================================
IRAM_ATTR uint64_t micros()
{
    return esp_timer_get_time();
}

//=============================================================================================
//=============================================================================================
IRAM_ATTR uint64_t millis()
{
    return esp_timer_get_time() / 1000ULL;
}

//=============================================================================================
//=============================================================================================
void set_status_led(bool enabled)
{
#ifdef STATUS_LED_PIN
    gpio_set_level(STATUS_LED_PIN, enabled ? STATUS_LED_ON : STATUS_LED_OFF);
#endif    

#ifdef ESP32CAM_FLASH_LED_PIN
    enable_esp32cam_flash_led_pin(enabled);
#endif    
}

//=============================================================================================
//=============================================================================================
bool getButtonState()
{
#ifdef REC_BUTTON_PIN
    return gpio_get_level((gpio_num_t)REC_BUTTON_PIN) == 0;
#endif


#ifdef ESP32CAM_FLASH_LED_PIN
    return read_esp32cam_flash_led_pin();
#endif

    return false;
}

//=============================================================================================
//=============================================================================================
void checkButton()
{
  static uint32_t debounceTime = millis() + 100;
  static bool lastButtonState = false;

  if ( debounceTime > millis() ) return;
  bool buttonState = getButtonState();
  debounceTime = millis() + 10;
  if ( buttonState != lastButtonState )
  {
    debounceTime = millis() + 100;
    lastButtonState = buttonState;

    if ( buttonState )
    {
        if ( s_restart_time == 0 )
        {
#if defined(ENABLE_PROFILER) && defined (START_PROFILER_WITH_BUTTON)
            if ( s_profiler.isActive())
            {
                LOG("Profiler stopped!\n");
                s_profiler.stop();
                s_profiler.save();
                s_profiler.clear();
            }
            else
            {
                LOG("Profiler started!\n");
                s_profiler.start(1000);
            }
#else
            s_air_record = !s_air_record;
#endif            
        }
        LOG("Button pressed!\n");
    }
    else
    {
        LOG("Button unpressed!\n");
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
static bool s_recv_ground2air_packet = false;

SemaphoreHandle_t s_serial_mux = xSemaphoreCreateBinary();

auto _init_result2 = []() -> bool
{
  xSemaphoreGive(s_serial_mux);
  return true;
}();

//=============================================================================================
//=============================================================================================
IRAM_ATTR void packet_received_cb(void* buf, wifi_promiscuous_pkt_type_t type)
{
    if (type == WIFI_PKT_DATA)
    {
        //LOG("data packet\n");
    }
    else if (type == WIFI_PKT_MGMT)
    {
        //LOG("management packet\n");
        s_stats.inRejectedPacketCounter++;
        return;
    }
    else if (type == WIFI_PKT_MISC)
    {
        //LOG("misc packet\n");
        s_stats.inRejectedPacketCounter++;
        return;
    }
    else if (type == WIFI_PKT_CTRL)
    {
        //LOG("misc packet\n");
        s_stats.inRejectedPacketCounter++;
        return;
    }

    wifi_promiscuous_pkt_t *pkt = reinterpret_cast<wifi_promiscuous_pkt_t*>(buf);

    if (pkt->rx_ctrl.channel != s_ground2air_config_packet.dataChannel.wifi_channel)
    {
        //LOG("Packet received on wrong channel: %d, expected: %d\n", pkt->rx_ctrl.channel, s_ground2air_config_packet.dataChannel.wifi_channel);
        s_stats.inRejectedPacketCounter++;
        return;
    }    

    uint16_t len = pkt->rx_ctrl.sig_len;
    //s_stats.wlan_data_received += len;
    //s_stats.wlan_data_sent += 1;

    if (len <= WLAN_IEEE_HEADER_SIZE)
    {
        LOG("WLAN receive header error");
        s_stats.wlan_error_count++;
        return;
    }

    //LOG("Recv %d bytes\n", len);
    //LOG("Channel: %d\n", (int)pkt->rx_ctrl.channel);

    //uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    //LOG("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    uint8_t *data = pkt->payload;
    if (memcmp(data + 10, WLAN_IEEE_HEADER_GROUND2AIR + 10, 6) != 0)
    {
        //here we filter also AIR2GROUND packets from other air devices on the same channel
        s_stats.inRejectedPacketCounter++;
        return;
    }

    data += WLAN_IEEE_HEADER_SIZE;
    len -= WLAN_IEEE_HEADER_SIZE; //skip the 802.11 header

    if (len >= 4) 
    {
        len -= 4;
    } 
    else 
    {
        // Packet is too small after removing header and potential FCS
        //LOG("WLAN payload error - packet too small after header removal (%d bytes)\n", len);
        s_stats.wlan_error_count++;
        return;
    }

    size_t size = std::min<size_t>(len, WLAN_MAX_PAYLOAD_SIZE);

    auto res = s_fec_decoder.packetFilter.filter_packet( data, size );
    if ( res != PacketFilter::PacketFilterResult::Pass)
    {
        s_stats.inRejectedPacketCounter++;
        return;
    }

    s_stats.inPacketCounter++;
    s_stats.rssiDbm = -pkt->rx_ctrl.rssi;
    s_stats.noiseFloorDbm = -pkt->rx_ctrl.noise_floor;

    s_fec_decoder.lock();
    if (!s_fec_decoder.decode_data(data, size, false))
    {
        s_stats.wlan_received_packets_dropped++;
    }
    s_fec_decoder.unlock();

    s_stats.wlan_data_received += len;
}

//=============================================================================================
//=============================================================================================
__attribute__((optimize("Os")))
IRAM_ATTR bool processSetting(const char* valueName, int fromValue, int toValue, const char *nvsName )
{
    if ( fromValue != toValue )
    {
        LOG("%s changed from %d to %d\n", valueName, fromValue, toValue );
        if ( !!nvsName ) 
        {
            nvs_args_set(nvsName, (uint32_t)toValue);
        }
        return true;
    }
    return false;
}

//=============================================================================================
//=============================================================================================
//process settings not related to camera sensor setup
static void handle_ground2air_config_packetEx1(Ground2Air_Config_Packet& src)
{
    s_recv_ground2air_packet = true;
    s_accept_connection_timeout_ms = 0;

    int64_t t = esp_timer_get_time();
    s_last_seen_config_packet = t;
    
    Ground2Air_Config_Packet& dst = s_ground2air_config_packet;

    if ( processSetting( "Wifi rate", (int)dst.dataChannel.wifi_rate, (int)src.dataChannel.wifi_rate, "rate") )
    {
        ESP_ERROR_CHECK(set_wifi_fixed_rate(src.dataChannel.wifi_rate));
    }
    
    if ( processSetting( "Wifi power", dst.dataChannel.wifi_power, src.dataChannel.wifi_power, NULL ) )
    {
        ESP_ERROR_CHECK(set_wlan_power_dBm(src.dataChannel.wifi_power));
    }

    if ( processSetting( "FEC codec N", dst.dataChannel.fec_codec_n, src.dataChannel.fec_codec_n, "fec_n") )
    {
        s_fec_encoder.switch_n( src.dataChannel.fec_codec_n );
    }
    
    if ( processSetting( "Wifi channel", dst.dataChannel.wifi_channel, src.dataChannel.wifi_channel, "channel" ) )
    {
        ESP_ERROR_CHECK(esp_wifi_set_channel((int)src.dataChannel.wifi_channel, WIFI_SECOND_CHAN_NONE));
    }

    if ( processSetting( "Target FPS", dst.camera.fps_limit, src.camera.fps_limit, NULL) )
    {
        if (src.camera.fps_limit == 0)
            s_video_target_frame_dt = 0;
        else
            s_video_target_frame_dt = 1000000 / src.camera.fps_limit;
    }

    processSetting( "AutostartRecord", dst.dataChannel.autostartRecord, src.dataChannel.autostartRecord, "autostartRecord" );

    processSetting( "CameraStopChannel", dst.dataChannel.cameraStopChannel, src.dataChannel.cameraStopChannel, "cameraStopCH" );

    processSetting( "mavlink2mspRC",  dst.dataChannel.mavlink2mspRC, src.dataChannel.mavlink2mspRC, "mavlink2mspRC" );

    if ( s_restart_time == 0 )
    {
        if ( dst.dataChannel.air_record_btn != src.dataChannel.air_record_btn )
        {
            s_air_record = !s_air_record;
            //dst.dataChannel.air_record_btn = src.dataChannel.air_record_btn;
        }




#if defined(ENABLE_PROFILER)
        if ( dst.dataChannel.profile1_btn != src.dataChannel.profile1_btn )
        {
            if ( s_profiler.isActive())
            {
                LOG("Profiler stopped!\n");
                s_profiler.stop();
                s_profiler.save();
                s_profiler.clear();
            }
            else
            {
                LOG("Profiler started!\n");
                s_profiler.start(500);
            }
            dst.dataChannel.profile1_btn = src.dataChannel.profile2_btn;
        }

        if ( dst.dataChannel.profile2_btn != src.dataChannel.profile2_btn )
        {
            if ( s_profiler.isActive())
            {
                LOG("Profiler stopped!\n");
                s_profiler.stop();
                s_profiler.save();
                s_profiler.clear();
            }
            else
            {
                LOG("Profiler started!\n");
                s_profiler.start(3000);
            }
            dst.dataChannel.profile2_btn = src.dataChannel.profile2_btn;
        }
#endif
    }

    dst = src;
}

//=============================================================================================
//=============================================================================================
//process settings related to camera sensor setup
void handle_ground2air_config_packetEx2(bool forceCameraSettings)
{
    Ground2Air_Config_Packet& src = s_ground2air_config_packet;
    Ground2Air_Config_Packet& dst = s_ground2air_config_packet2;

    sensor_t* s = esp_camera_sensor_get();

#ifdef SENSOR_OV5640
    //on ov5640, aec2 is not aec dsp but "night vision" mode which decimate framerate dynamically
    src.camera.aec2 = false;
#endif

    bool resolutionChanged = (dst.camera.resolution != src.camera.resolution);
    bool ov2640HighFPSChanged = (src.camera.ov2640HighFPS != dst.camera.ov2640HighFPS );
    bool ov5640HighFPSChanged = (src.camera.ov5640HighFPS != dst.camera.ov5640HighFPS );
    if ( forceCameraSettings || resolutionChanged || ov2640HighFPSChanged || ov5640HighFPSChanged )
    {
        s_shouldRestartRecording =  esp_timer_get_time() + 1000000;
        LOG("Camera resolution changed from %d to %d\n", (int)dst.camera.resolution, (int)src.camera.resolution);

#ifdef SENSOR_OV5640
        s->set_colorbar(s, src.camera.ov5640HighFPS?1:0);
#else
        if ( src.camera.ov2640HighFPS && 
            ((src.camera.resolution == Resolution::VGA) ||
            (src.camera.resolution == Resolution::VGA16) ||
            (src.camera.resolution == Resolution::SVGA16)) 
            )
        {
            s->set_xclk( s, LEDC_TIMER_0, 16 );
        }
        else
        {
            s->set_xclk( s, LEDC_TIMER_0, 12 );
        }
#endif

        switch (src.camera.resolution)
        {
            case Resolution::QVGA: s->set_framesize(s, FRAMESIZE_QVGA); break;
            case Resolution::CIF: s->set_framesize(s, FRAMESIZE_CIF); break;
            case Resolution::HVGA: s->set_framesize(s, FRAMESIZE_HVGA); break;
            case Resolution::VGA: s->set_framesize(s, FRAMESIZE_VGA); break;

            case Resolution::VGA16:
#ifdef SENSOR_OV5640
                s->set_framesize(s, FRAMESIZE_P_3MP); //640x360
#else
                s->set_res_raw(s, 1/*OV2640_MODE_SVGA*/,0,0,0, 0, 72, 800, 600-144, 800,600-144,false,false);   //800x456x29.5? fps
                
#endif
            break;

            case Resolution::SVGA: s->set_framesize(s, FRAMESIZE_SVGA); break;

            case Resolution::SVGA16:
#ifdef SENSOR_OV5640
                //s->set_res_raw(s, 0, 0, 2623, 1951, 32, 16, 2844, 1968, 800, 600, true, true);  //attempt for 800x600
                //s->set_res_raw(s, 0, 240, 2623, 1711, 32, 16, 2844, 1488, 800, 450, true, true); //attempt for 800x450

                //s->set_pll(s, false, 26, 1, 1, false, 3, true, 4);  - root2x and pre_div are swapped due to incompatible signatures!
                //s->set_pll(s, false, 25, 1, false, 1, 3, true, 4); 

                //waning: LOGxxx should be commented out in ov5640.c otherwise there will be stack overflow in camtask
                s->set_framesize(s, FRAMESIZE_P_HD); //800x456
#else
                //s->set_framesize(s, FRAMESIZE_P_HD);  800x448 13 fps
                s->set_res_raw(s, 1/*OV2640_MODE_SVGA*/,0,0,0, 0, 72, 800, 600-144, 800,600-144,false,false);   //800x456 13 fps
#endif
            break;

            case Resolution::XGA: s->set_framesize(s, FRAMESIZE_XGA); break; //1024x768

            case Resolution::XGA16:  //1024x576
#ifdef SENSOR_OV5640
                s->set_framesize(s, FRAMESIZE_P_FHD);
#else
                s->set_res_raw(s, 0/*OV2640_MODE_UXGA*/,0,0,0, 0, 150, 1600, 1200-300, 1024, 576, false, false);   //1024x576 13 fps
                
#endif
            break;
            case Resolution::SXGA: s->set_framesize(s, FRAMESIZE_SXGA); break;
            case Resolution::HD: s->set_framesize(s, FRAMESIZE_HD); break;
            case Resolution::UXGA: s->set_framesize(s, FRAMESIZE_UXGA); break;

            case Resolution::COUNT:
            break;

        }

        if ( resolutionChanged)
        {
            nvs_args_set("resolution", (uint32_t)src.camera.resolution);
        }
        if ( ov2640HighFPSChanged)
        {
            nvs_args_set( "ov2640hfps", src.camera.ov2640HighFPS?1:0 );
        }
        if ( ov5640HighFPSChanged)
        {
            nvs_args_set( "ov5640hfps", src.camera.ov5640HighFPS?1:0 );
        }
    }

#define APPLY(n1, n2, type) \
    if (forceCameraSettings || (dst.camera.n1 != src.camera.n1)) \
    { \
        LOG("Camera " #n1 " from %d to %d\n", (int)dst.camera.n1, (int)src.camera.n1); \
        s->set_##n2(s, (type)src.camera.n1); \
    }

#define SAVE(n) \
    if (dst.camera.n != src.camera.n) \
    { \
        nvs_args_set(#n, (uint32_t)src.camera.n); \
    }

    if ( src.camera.quality != 0 )
    {
        APPLY(quality, quality, int);
        s_quality = src.camera.quality;
    }
    else
    {
        if ( forceCameraSettings )
        {
            s->set_quality(s, 20); 
        }
    }

    if ( dst.camera.agc != src.camera.agc)
    {
        //reapply agc gain if agc changed
        forceCameraSettings = true;
    }

    if ( dst.camera.aec != src.camera.aec)
    {
        //reapply aec value if aec changed
        forceCameraSettings = true;
    }

    SAVE(brightness)
    APPLY(brightness, brightness, int)

    SAVE(contrast)
    APPLY(contrast, contrast, int)

    SAVE(saturation)
    APPLY(saturation, saturation, int)

    SAVE(sharpness)
    if ( s_quality < 50 )
    {
        APPLY(sharpness, sharpness, int);
        s_sharpness = src.camera.sharpness;
    }
    APPLY(denoise, denoise, int);

#ifdef SENSOR_OV5640
    //gainceiling for ov5640 is range 0...3ff
    if (forceCameraSettings || (dst.camera.gainceiling != src.camera.gainceiling)) 
    { 
        LOG("Camera gainceiling from %d to %d\n", (int)dst.camera.gainceiling, (int)src.camera.gainceiling); 
        //s->set_gainceiling(s, (gainceiling_t)(2 << src.camera.gainceiling));
        //do not limit gainceiling on OV5640. Contrary to OV2640, it does good images with large gain ceiling, without enormous noise in dark scenes.
        s->set_gainceiling(s, (gainceiling_t)(0x3ff));
    }
#else
    APPLY(gainceiling, gainceiling, gainceiling_t);
#endif

    APPLY(awb, whitebal, int);
    APPLY(awb_gain, awb_gain, int);
    APPLY(wb_mode, wb_mode, int);
    APPLY(agc, gain_ctrl, int);
    APPLY(agc_gain, agc_gain, int);
    APPLY(aec, exposure_ctrl, int);
    APPLY(aec_value, aec_value, int);
    APPLY(aec2, aec2, int);

    SAVE(ae_level);
    APPLY(ae_level, ae_level, int);

    APPLY(hmirror, hmirror, int);
    
    if ( dst.camera.vflip != src.camera.vflip )
    {
        nvs_args_set( "vflip", src.camera.vflip?1:0 );
    }
    APPLY(vflip, vflip, int);

    APPLY(special_effect, special_effect, int);
    APPLY(dcw, dcw, int);
    APPLY(bpc, bpc, int);
    APPLY(wpc, wpc, int);
    APPLY(raw_gma, raw_gma, int);
    APPLY(lenc, lenc, int);
#undef APPLY
#undef SAVE

    dst = src;
}

//===========================================================================================
//===========================================================================================
__attribute__((optimize("Os")))
IRAM_ATTR static void acceptConnectionWithGS( uint16_t gsDeviceId )
{
    s_connected_gs_device_id = gsDeviceId;
    s_fec_encoder.packetFilter.set_packet_header_data( s_air_device_id, s_connected_gs_device_id );
    s_fec_decoder.packetFilter.set_packet_filtering( s_connected_gs_device_id, s_air_device_id );

    LOG("Accepting connection to GS device ID: 0x%04x\n", s_connected_gs_device_id );

    s_accept_connection_timeout_ms = millis() + 3000;
}

//===========================================================================================
//===========================================================================================
static void unpairGS()
{
    s_connected_gs_device_id = 0;
    s_fec_encoder.packetFilter.set_packet_header_data( s_air_device_id, 0 );
    s_fec_decoder.packetFilter.set_packet_filtering( 0, 0 );

    LOG("Unpair from GS\n" );

    s_accept_connection_timeout_ms = 0;
}

//===========================================================================================
//===========================================================================================
static void handle_ground2air_config_packet(Ground2Air_Config_Packet& src)
{
    if ( s_connected_gs_device_id == 0 ) 
    {
        if ( src.airDeviceId == s_air_device_id )
        {
            //accept connection with GS. This GS was connected to this camera before. 
            //Camera rebooted?
            acceptConnectionWithGS( src.gsDeviceId );
        }
        else
        {
            return;
        }
    }

    //handle settings not related to camera sensor setup.
    //camera sensor settings are processed in camera_data_available() callback
    handle_ground2air_config_packetEx1(src);
}

//===========================================================================================
//===========================================================================================
IRAM_ATTR static void handle_ground2air_connect_packet(Ground2Air_Config_Packet& src)
{
    if ( s_connected_gs_device_id == 0 )
    {
        acceptConnectionWithGS( src.gsDeviceId );
    }
}

static void init_camera();

//===========================================================================================
//===========================================================================================
__attribute__((optimize("Os")))
IRAM_ATTR static void handle_ground2air_control_packet(Ground2Air_Control_Packet& src)
{
    // Handle flash command (previously gpio_control_btn)
    #ifdef GPIO_CONTROL_PIN
    if (s_restart_time == 0)
    {
        bool new_flash_state = (src.command == I2C_CMD_FLASH); // Command 5 is flash
        if (s_gpio_control_state != new_flash_state)
        {
            set_gpio_control_pin(new_flash_state);
            LOG("Flash state changed to: %s\n", new_flash_state ? "ON" : "OFF");
        }
    }
    #endif

    // Handle movement commands and retransmit via I2C
    if (s_restart_time == 0)
    {
        switch (src.command)
        {
            case I2C_CMD_FORWARD: // Forward
                send_i2c_command(I2C_CMD_FORWARD);
                LOG("Forward command sent via I2C\n");
                break;
            case I2C_CMD_BACKWARD: // Backward
                send_i2c_command(I2C_CMD_BACKWARD);
                LOG("Backward command sent via I2C\n");
                break;
            case I2C_CMD_RIGHT: // Right
                send_i2c_command(I2C_CMD_RIGHT);
                LOG("Right command sent via I2C\n");
                break;
            case I2C_CMD_LEFT: // Left
                send_i2c_command(I2C_CMD_LEFT);
                LOG("Left command sent via I2C\n");
                break;
            case I2C_CMD_FLASH: // Flash
                // Already handled above
                break;
            default:
                LOG("Unknown command received: %d\n", src.command);
                break;
        }
    }
}

//===========================================================================================
//===========================================================================================
__attribute__((optimize("Os")))
IRAM_ATTR static void handle_ground2air_data_packet(Ground2Air_Data_Packet& src)
{
#ifdef UART_MAVLINK
    if ( ( s_connected_gs_device_id == 0 ) || ( src.gsDeviceId != s_connected_gs_device_id ) ) return;

    xSemaphoreTake(s_serial_mux, portMAX_DELAY);

    int s = src.size - sizeof(Ground2Air_Header);
    s_stats.in_telemetry_data += s;

    uint8_t* dPtr = ((uint8_t*)&src) + sizeof(Ground2Air_Header);
    for ( int i = 0; i < s; i++ )
    {
        mavlinkParserIn.processByte(*dPtr++);
        if ( mavlinkParserIn.gotPacket())
        {
            if ( mavlinkParserIn.getMessageId() == HX_MAXLINK_RC_CHANNELS_OVERRIDE )
            {
                uint32_t t = (uint32_t)millis();
                int d = t - s_last_rc_packet_tp;
                s_last_rc_packet_tp = t;
                if ( d > s_stats.RCPeriodMaxMS ) 
                {
                    s_stats.RCPeriodMaxMS = d;
                }

                if ( s_ground2air_config_packet2.dataChannel.mavlink2mspRC != 0 )
                {
                    const HXMAVLinkRCChannelsOverride* msg = mavlinkParserIn.getMsg<HXMAVLinkRCChannelsOverride>();
                    //LOG("%d %d %d %d\n", msg->chan1_raw, msg->chan2_raw, msg->chan3_raw, msg->chan4_raw);
                    uint16_t ch[MSP_RC_CHANNELS_COUNT];
                    for ( int i = 0; i < MSP_RC_CHANNELS_COUNT; i++ )
                    {
                        ch[i] = msg->getChannelValue( i + 1 );
                    }
                    g_msp.setRCChannels(ch);
                }

                if ( s_ground2air_config_packet2.dataChannel.cameraStopChannel != 0 )
                {
                    const HXMAVLinkRCChannelsOverride* msg = mavlinkParserIn.getMsg<HXMAVLinkRCChannelsOverride>();
                    s_camera_stopped_requested = msg->getChannelValue( s_ground2air_config_packet2.dataChannel.cameraStopChannel ) > 1700;

                    //LOG("%d %d %d %d\n", msg->chan1_raw, msg->chan2_raw, msg->chan3_raw, msg->chan4_raw);
                    //LOG("%d %d %d %d %d\n", msg->chan9_raw, msg->chan10_raw, msg->chan11_raw, msg->chan12_raw, mavlinkParserIn.getPacketLength());
                }
                else
                {
                    s_camera_stopped_requested = false;
                }
            }
        }
    }

    size_t freeSize = 0;
    ESP_ERROR_CHECK( uart_get_tx_buffer_free_size(UART_MAVLINK, &freeSize) );

    if ( freeSize >= s )
    {
        uart_write_bytes(UART_MAVLINK, ((uint8_t*)&src) + sizeof(Ground2Air_Header), s);
    }

    xSemaphoreGive(s_serial_mux);
#endif
}

//=============================================================================================
//=============================================================================================
IRAM_ATTR void send_air2ground_video_packet(bool last)
{
    s_stats.video_data += s_video_frame_data_size;

    uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);

    if(!packet_data)
    {
        LOG("no data buf!\n");
    }

    Air2Ground_Video_Packet& packet = *(Air2Ground_Video_Packet*)packet_data;
    packet.type = Air2Ground_Header::Type::Video;
    packet.resolution = s_ground2air_config_packet.camera.resolution;
    packet.frame_index = s_video_frame_index;
    packet.part_index = s_video_part_index;
    packet.last_part = last ? 1 : 0;
    packet.size = s_video_frame_data_size + sizeof(Air2Ground_Video_Packet);
    packet.pong = s_ground2air_config_packet.ping;
    packet.version = PACKET_VERSION;
    packet.airDeviceId = s_air_device_id;
    packet.gsDeviceId = s_connected_gs_device_id;
    packet.crc = 0;
    packet.crc = crc8(0, &packet, sizeof(Air2Ground_Video_Packet));
    if (!s_fec_encoder.flush_encode_packet(true))
    {
        LOG("Fec codec busy\n");
        s_stats.wlan_error_count++;
#ifdef PROFILE_CAMERA_DATA    
        s_profiler.toggle(PF_CAMERA_FEC_OVF);
#endif
    }
}

#ifdef UART_MAVLINK
//=============================================================================================
//=============================================================================================
//this currently called every frame: 50...11 fps
//30 fps: 30 * 128 = 3840 bytes/sec or 38400 baud
//11 fps: 11 * 128 - 1408 = 14080 baud
//todo: increase max mavlink payload size to 1K. Packets are 1.4K anyway. With 128 bytes we can not push 115200 mavlink stream currently.
//also UART RX ring buffer of 512 can not handle 115200 at 11 fps
IRAM_ATTR void send_air2ground_data_packet()
{
    int avail = MAX_TELEMETRY_PAYLOAD_SIZE - s_mavlinkOutBufferCount;
    if ( avail > 0 )
    {
        size_t rs = 0;
        ESP_ERROR_CHECK( uart_get_buffered_data_len(UART_MAVLINK, &rs) );
        if ( rs > avail ) rs = avail;

        if ( rs > 0 )
        {
            int len = uart_read_bytes(UART_MAVLINK, &(s_mavlink_out_buffer[s_mavlinkOutBufferCount]),rs, 0);
            if ( len < 0 )
            {
                LOG("MAVLNK COM error\n");
            }
            else
            {
                s_mavlinkOutBufferCount += len;
            }
        }
    }

    if ( s_mavlinkOutBufferCount < MAX_TELEMETRY_PAYLOAD_SIZE ) return; //todo: or agregationtime

    uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);
    if(!packet_data)
    {
        LOG("no data buf!\n");
        return;
    }

    Air2Ground_Data_Packet& packet = *(Air2Ground_Data_Packet*)packet_data;
    packet.type = Air2Ground_Header::Type::Telemetry;
    packet.size = s_mavlinkOutBufferCount + sizeof(Air2Ground_Data_Packet);
    packet.pong = s_ground2air_config_packet.ping;
    packet.version = PACKET_VERSION;
    packet.airDeviceId = s_air_device_id;
    packet.gsDeviceId = s_connected_gs_device_id;
    packet.crc = 0;
    packet.crc = crc8(0, &packet, sizeof(Air2Ground_Data_Packet));

    memcpy( packet_data + sizeof(Air2Ground_Data_Packet), s_mavlink_out_buffer, s_mavlinkOutBufferCount );

    if (!s_fec_encoder.flush_encode_packet(true))
    {
        LOG("Fec codec busy\n");
        s_stats.wlan_error_count++;
#ifdef PROFILE_CAMERA_DATA
        s_profiler.toggle(PF_CAMERA_FEC_OVF);
#endif
    }
    else
    {
        s_stats.out_telemetry_data += s_mavlinkOutBufferCount;
        s_mavlinkOutBufferCount = 0;
    }
}
#endif

//=============================================================================================
//=============================================================================================
IRAM_ATTR void send_air2ground_osd_packet()
{
    uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);

    if(!packet_data)
    {
        LOG("no data buf!\n");
        return ;
    }

    Air2Ground_OSD_Packet& packet = *(Air2Ground_OSD_Packet*)packet_data;
    packet.type = Air2Ground_Header::Type::OSD;
    packet.size = sizeof(Air2Ground_OSD_Packet);
    packet.pong = s_ground2air_config_packet.ping;
    packet.version = PACKET_VERSION;
    packet.airDeviceId = s_air_device_id;
    packet.gsDeviceId = s_connected_gs_device_id;
    packet.crc = 0;

    packet.stats.curr_wifi_rate = (uint8_t)s_wlan_rate;

    packet.stats.wifi_queue_min = s_min_wlan_outgoing_queue_usage_seen;
    packet.stats.wifi_queue_max = s_max_wlan_outgoing_queue_usage;
    packet.stats.air_record_state = s_air_record ? 1 : 0;

    packet.stats.wifi_ovf = 0;
    if ( s_wifi_ovf_time > 0 )
    {
        int64_t t = esp_timer_get_time();
        t -= s_wifi_ovf_time;
        if ( t < 1000000 )
        {
            packet.stats.wifi_ovf = 1;
        }
        else
        {
            s_wifi_ovf_time = 0;
        }
    }
    
    packet.stats.SDFreeSpaceGB16 = SDFreeSpaceGB16;
    packet.stats.SDTotalSpaceGB16 = SDTotalSpaceGB16;
    packet.stats.curr_quality = s_quality;

    packet.stats.temperature = (uint8_t)(s_camera_temperature + 0.5f);
    packet.stats.overheatTrottling = 0;

    if ( s_last_stats.RCPeriodMaxMS < 0 )
    {
        packet.stats.RCPeriodMax = 0;
    }
    else if ( s_last_stats.RCPeriodMaxMS == 0 )
    {
        packet.stats.RCPeriodMax = 1;
    }
    else if ( s_last_stats.RCPeriodMaxMS <= 100 )
    {
        packet.stats.RCPeriodMax = s_last_stats.RCPeriodMaxMS;
    }
    else 
    {
        //s_last_stats.RCPeriodMaxMS can not be > 1000
        //1000/10 + 101 < 255
        packet.stats.RCPeriodMax = s_last_stats.RCPeriodMaxMS / 10 + 101;   
    }

#ifdef SENSOR_OV5640
    packet.stats.isOV5640 = 1;
#else
    packet.stats.isOV5640 = 0;
#endif    

    packet.stats.outPacketRate = s_last_stats.outPacketCounter;
    packet.stats.inPacketRate = s_last_stats.inPacketCounter;
    packet.stats.inRejectedPacketRate = s_last_stats.inRejectedPacketCounter;
    packet.stats.rssiDbm = s_last_stats.rssiDbm;
    packet.stats.noiseFloorDbm = s_last_stats.noiseFloorDbm;
    packet.stats.captureFPS = s_actual_capture_fps;
    packet.stats.cam_frame_size_min = s_last_stats.camera_frame_size_min;
    packet.stats.cam_frame_size_max = s_last_stats.camera_frame_size_max;
    packet.stats.cam_ovf_count = cam_ovf_count;
    packet.stats.inMavlinkRate = s_last_stats.in_telemetry_data;
    packet.stats.outMavlinkRate = s_last_stats.out_telemetry_data;

    packet.stats.suspended = s_camera_stopped != 0;

    memcpy( &packet.buffer, g_osd.getBuffer(), OSD_BUFFER_SIZE );
    
    packet.crc = crc8(0, &packet, sizeof(Air2Ground_OSD_Packet));

    if (!s_fec_encoder.flush_encode_packet(true))
    {
        LOG("Fec codec busy\n");
        s_stats.wlan_error_count++;
#ifdef PROFILE_CAMERA_DATA    
        s_profiler.toggle(PF_CAMERA_FEC_OVF);
#endif
    }
}

//=============================================================================================
//=============================================================================================
IRAM_ATTR void send_air2ground_config_packet()
{
    uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);
    if( !packet_data )
    {
        LOG("no data buf!\n");
        return;
    }

    Air2Ground_Config_Packet& packet = *(Air2Ground_Config_Packet*)packet_data;
    packet.type = Air2Ground_Header::Type::Config;
    packet.size = sizeof(Air2Ground_Config_Packet);
    packet.dataChannel = s_ground2air_config_packet.dataChannel;
    packet.camera = s_ground2air_config_packet.camera;
    packet.version = PACKET_VERSION;
    packet.airDeviceId = s_air_device_id;
    packet.gsDeviceId = s_connected_gs_device_id;

    packet.crc = 0;
    packet.crc = crc8(0, &packet, sizeof(Air2Ground_Config_Packet));

    if (!s_fec_encoder.flush_encode_packet(true))
    {
        LOG("Fec codec busy\n");
        s_stats.wlan_error_count++;
#ifdef PROFILE_CAMERA_DATA    
        s_profiler.toggle(PF_CAMERA_FEC_OVF);
#endif
    }
}

constexpr size_t MAX_VIDEO_DATA_PAYLOAD_SIZE = AIR2GROUND_MTU - sizeof(Air2Ground_Video_Packet);

static const int WifiRateBandwidth[] = 
{
    2*1024*125, // 0 - RATE_B_2M_CCK,
    2*1024*125, // 1 - RATE_B_2M_CCK_S,
    5*1024*125, // 2 - RATE_B_5_5M_CCK,
    5*1024*125, // 3 - RATE_B_5_5M_CCK_S,
    11*1024*125, // 4 - RATE_B_11M_CCK,
    11*1024*125, // 5 - RATE_B_11M_CCK_S,

    6*1024*125, // 6 - RATE_G_6M_ODFM,
    9*1024*125, // 7 - RATE_G_9M_ODFM,
    12*1024*125, // 8 - RATE_G_12M_ODFM,
    18*1024*125, // 9 - RATE_G_18M_ODFM,
    24*1024*125,  // 10 - RATE_G_24M_ODFM,
    36*1024*125,  // 11 - RATE_G_36M_ODFM,
    48*1024*125,  // 12 - RATE_G_48M_ODFM,
    54*1024*125,  // 13 - RATE_G_54M_ODFM,

    6*1024*125, // 14 - RATE_N_6_5M_MCS0,
    7*1024*125, // 15 - RATE_N_7_2M_MCS0_S,
    13*1024*125, // 16 - RATE_N_13M_MCS1,
    14*1024*125, // 17 - RATE_N_14_4M_MCS1_S,
    19*1024*125, // 18 - RATE_N_19_5M_MCS2,
    21*1024*125, // 19 - RATE_N_21_7M_MCS2_S,
    26*1024*125, // 20 - RATE_N_26M_MCS3,
    28*1024*125, // 21 - RATE_N_28_9M_MCS3_S,
    39*1024*125, // 22 - RATE_N_39M_MCS4,
    43*1024*125, // 23 - RATE_N_43_3M_MCS4_S,
    52*1024*125, // 24 - RATE_N_52M_MCS5,
    57*1024*125, // 25 - RATE_N_57_8M_MCS5_S,
    58*1024*125, // 26 - RATE_N_58M_MCS6,
    65*1024*125, // 27 - RATE_N_65M_MCS6_S,
    65*1024*125, // 28 - RATE_N_65M_MCS7,
    72*1024*125 // 29 - RATE_N_72M_MCS7_S,
};


IRAM_ATTR int getBandwidthForRate(WIFI_Rate rate)
{
    return WifiRateBandwidth[(int)rate];
}

//=============================================================================================
//=============================================================================================
IRAM_ATTR int calculateAdaptiveQualityValue()
{
    int quality1 = (int)(8 + (63-8) * ( 1 - s_quality_framesize_K1 * s_quality_framesize_K2 * s_quality_framesize_K3));
    if ( quality1 < 8) quality1 = 8;
    if ( quality1 > 63) quality1 = 63;

    //recode due to non-linear frame size changes depending on quality
    //from 8 to 19 frame size decreases by half, from 20 to 63 frame size decreases by half
    //y=(x-8)^2.3/185 + 8
    static const uint8_t recode[64-8] = { 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 16, 17, 18, 19, 20, 20, 21, 23, 24, 25, 26, 27, 29, 30, 31, 33, 34, 36, 37, 39, 41, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 63};

    return recode[quality1-8];
}

//=============================================================================================
//=============================================================================================
IRAM_ATTR bool isHQDVRMode()
{
    return s_ground2air_config_packet2.camera.resolution == Resolution::HD;
}

//=============================================================================================
//=============================================================================================
__attribute__((optimize("Os")))
IRAM_ATTR void recalculateFrameSizeQualityK(int video_full_frame_size)
{
    if ( video_full_frame_size > s_max_frame_size )
    {
        s_max_frame_size = video_full_frame_size;
    }
    if ( video_full_frame_size == 0 ) return;

    int fps = s_actual_capture_fps_expected > 1 ? s_actual_capture_fps_expected : 10;
    if (s_actual_capture_fps_expected < 15 ) s_actual_capture_fps_expected++;  //under low FPS mesaurements fluctuate. Enforce lower estimation.

    //K1 - wifi bandwidth
    //data rate available with current wifi rate
    int rateBandwidth = getBandwidthForRate(s_wlan_rate);

    rateBandwidth = rateBandwidth * 5 / 10;  //assume only  50% of theoretical maximum bandwidth is available in practice

// #ifdef BOARD_XIAOS3SENSE        
//     //2.9MB/sec is practical maximum limit which works
//     if ( rateBandwidth > 2900*1024 ) rateBandwidth = 2900*1024;
// #else
    //2.3MB/sec is practical maximum limit which works
    if ( rateBandwidth > 2300*1024 ) rateBandwidth = 2300*1024;
// #endif

    //decrease available data rate using FEC codec parameters
    int FECbandwidth = rateBandwidth * s_ground2air_config_packet.dataChannel.fec_codec_k / s_ground2air_config_packet.dataChannel.fec_codec_n;

    if ( s_air_record )
    {
// #ifdef BOARD_XIAOS3SENSE        
//         if ( FECbandwidth > MAX_SD_WRITE_SPEED_ESP32S3 ) FECbandwidth = MAX_SD_WRITE_SPEED_ESP32S3;
// #else
        if ( FECbandwidth > MAX_SD_WRITE_SPEED_ESP32 ) FECbandwidth = MAX_SD_WRITE_SPEED_ESP32;
// #endif
    }

    if ( isHQDVRMode() )
    {
// #ifdef BOARD_XIAOS3SENSE        
//         FECbandwidth  = MAX_SD_WRITE_SPEED_ESP32S3;
// #else
        FECbandwidth  = MAX_SD_WRITE_SPEED_ESP32;
// #endif
    }
    
    int frameSize = FECbandwidth / fps;
    if ( frameSize < 1 ) frameSize = 1;

    float k = frameSize * 1.0f / video_full_frame_size;
    if ( k > 1.1f ) k = 1.1f;

    s_quality_framesize_K1 = s_quality_framesize_K1 * k;
    if ( s_quality_framesize_K1 < 0.05f ) s_quality_framesize_K1 = 0.05f;
    if ( s_quality_framesize_K1 > 1.0f ) s_quality_framesize_K1 = 1.0f;


    //k2 - max frame size which do not overload wifi output queue
    //wifi output queue should have space to hold frame data and fec data
    int safe_frame_size = WLAN_OUTGOING_BUFFER_SIZE *  s_ground2air_config_packet.dataChannel.fec_codec_k / s_ground2air_config_packet.dataChannel.fec_codec_n;
    //queue is emptied by tx thread constantly so we can assume virtually "larger buffer"
    safe_frame_size += FECbandwidth / fps; 
    safe_frame_size = safe_frame_size * 7 / 10;  //assume next frame can suddenly increase is size by 30%
    if ( video_full_frame_size > safe_frame_size )
    {
        s_quality_framesize_K2 -= (video_full_frame_size - safe_frame_size) / 100000.0f;// * 30 / fps;
    }
    else
    {
        s_quality_framesize_K2 += ( safe_frame_size - video_full_frame_size ) / 1000000.0f;// * 30 / fps;
    }
    if ( s_quality_framesize_K2 < 0.05f ) s_quality_framesize_K2 = 0.05f;
    if ( s_quality_framesize_K2 > 1.0f ) s_quality_framesize_K2 = 1.0f;


    /*
    s_quality_framesize_K2 = s_quality_framesize_K2 * safe_frame_size * 1.0f / video_full_frame_size;
    if ( s_quality_framesize_K2 < 0.05f ) s_quality_framesize_K2 = 0.05f;
    if ( s_quality_framesize_K2 > 1.0f ) s_quality_framesize_K2 = 1.0f;
    */

    if ( isHQDVRMode() )
    {
        s_quality_framesize_K2 = 1;
    }

    //K3 - wifi queue
    if ( (s_stats.wlan_error_count > 0) || (s_fec_spin_count > 0) || (s_fec_wlan_error_count > 0))
    {
        s_quality_framesize_K3 = 0; //we overloaded wifi buffer. drop quality immediatelly, we do not want to loose more frames
    }

    int min_wlan_outgoing_queue_usage_frame = getMinWlanOutgoingQueueUsageFrame(); //get the minimum wifi queue usage while sending frame

    //keep max wifi out queue usage below 80%
    //keep min wifi out queue usage in range 0...10%
    int max_wlan_outgoing_queue_usage_frame = getMaxWlanOutgoingQueueUsageFrame(); //get the maximum wifi queue usage while sending frame 
    if ( min_wlan_outgoing_queue_usage_frame > 10 )
    {
        s_quality_framesize_K3 -= (min_wlan_outgoing_queue_usage_frame - 10) / 100.0f;
        //make sure quality is dropped at least by 1 point
        while ( s_quality_framesize_K3 > 0 )
        {
            int quality1 = calculateAdaptiveQualityValue();
            if ( quality1 > s_quality ) break;
            s_quality_framesize_K3 -= 0.01;
        }
    }
    else if ( max_wlan_outgoing_queue_usage_frame > 80 )
    {
        s_quality_framesize_K3 -= (max_wlan_outgoing_queue_usage_frame - 60) / 100.0f;
    }
    else 
    {
        s_quality_framesize_K3 += ( 40 - max_wlan_outgoing_queue_usage_frame ) / 10000.0f;
    }
    if ( s_quality_framesize_K3 < 0) s_quality_framesize_K3 = 0;
    else if ( s_quality_framesize_K3 > 1) s_quality_framesize_K3 = 1;

    if ( isHQDVRMode() )
    {
        s_quality_framesize_K3 = 1;
    }
}

//=============================================================================================
//=============================================================================================
IRAM_ATTR void applyAdaptiveQuality()
{
    if ( s_ground2air_config_packet.camera.quality != 0 ) return;

    int quality1 = calculateAdaptiveQualityValue();

    if (s_quality != quality1)
    {
        s_quality = quality1;
        sensor_t* s = esp_camera_sensor_get(); 
        s->set_quality(s, s_quality); 

        int sharpness1 = s_ground2air_config_packet.camera.sharpness;

        if ( s_quality > 50 ) sharpness1 = -2; 
        else if ( s_quality > 45 ) sharpness1 = -1;
        else if ( s_quality > 40 ) sharpness1 = 0;

        if ( s_ground2air_config_packet.camera.sharpness < sharpness1 ) sharpness1 = s_ground2air_config_packet.camera.sharpness;

        if (s_sharpness != sharpness1 )
        {
            s->set_sharpness(s, sharpness1);
            s_sharpness = sharpness1;
        }
    }
}

/* 
'bool last' does not mark last JPEG block reliably.

ov2640: if VSYNC interrupt occurs near the end of the last block, we receive another 1Kb block of garbage.
Note that ov2460 sends up zero bytes after FF D9 till 16 byte boundary, then a garbage to the end of the block, 
so if we receive extra block, the end of proper 'last' block is filled by zeros after FF D9

ov5640: we always receive up to 3 more zero 1kb blocks (uncertain: can we receive +7 blocks if VSYNK interrupt occurs at the end of 4th block?)
The end of proper 'last' block is filled by zeros after FF D9 (up to 1 kb)

There are two problems:
1) As we may receive garbage block, searching for the end marker in garbage may lead to incorrect JPEG size calculation and inclusion of garbage bytes 
  (possibly with misleading JPEG markers) in the stream.
2) with 0v5640 we waste bandwidth for 1..3 1kb blocks, it's 10% video bandwidth wasted!

Proper solution - jpeg chunks parsing - is possible but waste of CPU resources, because jpeg has to be processed byte-by-byte.

We apply very fast workaround instead.

We always zero first 16 bytes in the DMA buffer after processing.

ov2640: instead of receiving garbage block, we receive block with 16 zeros at start. This way we know that the whole buffer has to be skipped; 
jpeg is terminated already in the previous block. We waste only few bytes at the end of the block, which are filled by zeros.

ov5640: we are able to finish frame at the first completely zero block,
and even on the block wich has 16 zeros at end
*/

//=============================================================================================
//=============================================================================================
IRAM_ATTR size_t camera_data_available(void * cam_obj,const uint8_t* data, size_t count, bool last)
{
    if ( !s_initialized ) return count;

#ifdef PROFILE_CAMERA_DATA    
    s_profiler.set(PF_CAMERA_DATA, 1);
#endif

    size_t stride = ((cam_obj_t *)cam_obj)->dma_bytes_per_item;

    if ( getOVFFlagAndReset() )
    {
#ifdef PROFILE_CAMERA_DATA    
        s_profiler.toggle(PF_CAMERA_OVF);
#endif
        s_quality_framesize_K3 = 0.05;
        cam_ovf_count++;
        s_stats.video_frames_expected++;
        applyAdaptiveQuality();
    }

    if (data == nullptr) //start frame
    {
        if (  !s_video_frame_started )
        {
            //limit fps
            int64_t n = esp_timer_get_time();
            int64_t send_dt = n - s_video_last_sent_tp;
            if (send_dt >= s_video_target_frame_dt)
            {
                s_video_last_sent_tp = n;

                s_video_frame_started = true;

                s_video_frame_data_size = 0;
                s_video_part_index = 0;

                s_video_full_frame_size = 0;
                
                s_lastByte_ff = false;
            }

            if ( !isHQDVRMode() || (s_wlan_outgoing_queue_usage < 5))
            {
                s_encoder_output_ovf_flag = false;
            }
        }
    }
    else 
    {
        s_fec_encoder.lock();

#ifdef BOARD_ESP32CAM
        //ESP32 - sample offset: 2, stride: 4
        const uint8_t* src = (const uint8_t*)data + 2;  // jump to the sample1 of DMA element
#endif            
// #ifdef BOARD_XIAOS3SENSE
//         //ESP32S3 - sample offset: 0, stride: 1
//         const uint8_t* src = (const uint8_t*)data;  
// #endif
        uint8_t* clrSrc = (uint8_t*)src;

        if ( s_video_frame_started && (s_video_full_frame_size == 0) )
        {
            if (src[0] != 0xff || src[stride] != 0xd8)
            {
                //broken frame data, no start marker
                //we probably missed the start of the frame. Have to skip it.
                cam_ovf_count++; 
                s_stats.video_frames_expected++;
                s_video_frame_started = false;
            }
        }

        if (s_video_frame_started)
        {
            count /= stride;

            //check if we missed last block due to VSYNK near the end of prev (last) block
            // if block starts with 16 zero bytes - it is either not filled at all (ov2640)
            //or is fully filled with zeros(ov5640)
            const uint8_t* src1 = src;
            int i;
            for ( i = 0; i < 16; i++ )
            {
                if ( *src1 != 0 ) break;
                src1 += stride;
            }
            
            if ( i == 16 ) 
            {
                //we missed last block, this is next block after last
                //the whole block should be skipped
                last = true;
                count = 0;
            }
            else 
            {
                if (last) //find the end marker for JPEG. Data after that can be discarded
                {
                    //edge case - 0xFF at the end of prev block, 0xD9 on the start of this
                    if ( s_lastByte_ff && (*src == 0xD9))
                    {
                        count = 1;
                    }
                    else
                    {
                        //search from the start of the block
                        //tail of the block can contain end markers in garbage
                        const uint8_t* dptr = src;
                        const uint8_t* dptrEnd = src + (count - 2) * stride;
                        while (dptr <= dptrEnd)
                        {
                            if (dptr[0] == 0xFF && dptr[stride] == 0xD9)
                            {
                                count = (dptr - src) / stride + 2; //to include the 0xFFD9
                                if ((count & 0x1FF) == 0)
                                    count += 1; 
                                if ((count % 100) == 0)
                                    count += 1;
                                break;
                            }
                            dptr += stride;
                        }
                    }
                }
            }

            s_lastByte_ff = count > 0 ? src[count-1] == 0xFF : false;
            while (count > 0)
            {
                //fill the buffer
                uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);
                uint8_t* start_ptr = packet_data + sizeof(Air2Ground_Video_Packet) + s_video_frame_data_size;
                uint8_t* ptr = start_ptr;
                size_t c = std::min(MAX_VIDEO_DATA_PAYLOAD_SIZE - s_video_frame_data_size, count);

                count -= c;
                s_video_frame_data_size += c;
                s_video_full_frame_size += c;

#ifdef PROFILE_CAMERA_DATA    
                s_profiler.set(PF_CAMERA_DATA_SIZE, s_video_full_frame_size / 1024);
#endif

#ifdef BOARD_ESP32CAM
                //ESP32 - sample offset: 2, stride: 4
                size_t c8 = c >> 3;
                
                for (size_t i = c8; i > 0; i--)
                {
                    const uint8_t* src1  = src + 3*4;
                    uint32_t temp;

                    temp = *src1; 
                    src1 -= 4;

                    temp = (temp << 8) | *src1;
                    src1 -= 4;

                    temp = (temp << 8) | *src1;
                    src1 -= 4;

                    temp = (temp << 8) | *src1;

                    *(uint32_t*)ptr = temp;
                    ptr += 4;

                    src1  = src + 7*4;

                    temp = *src1;
                    src1 -= 4;

                    temp = (temp << 8) | *src1;
                    src1 -= 4;

                    temp = (temp << 8) | *src1;
                    src1 -= 4;

                    temp = (temp << 8) | *src1;
                    
                    src += 8*4;

                    *(uint32_t*)ptr = temp;
                    ptr+=4;

                    /*
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    */
                }
                for (size_t i = c - (c8 << 3); i > 0; i--)
                {
                    *ptr++ = *src; src += stride;
                }
#endif

                if (s_video_frame_data_size == MAX_VIDEO_DATA_PAYLOAD_SIZE) 
                {
                    //LOG("Flush: %d %d\n", s_video_frame_index, s_video_frame_data_size);
                    //if wifi send queue was overloaded, do not send frame data till the end of the frame. 
                    //Frame is lost anyway. 
                    //Let fec_encoder and wifi to send leftover and start with emtpy queues at the next camera frame.
                    if (!s_encoder_output_ovf_flag)
                    { 
                        send_air2ground_video_packet(false);
                    }
                    s_video_frame_data_size = 0;
                    s_video_part_index++;
                }

                //LOG("Add: %d %d %d %d\n", s_video_frame_index, s_video_part_index, count, s_video_frame_data_size);

            }  //while count>0
        }  //s_frame_started

        //////////////////

        if (last)  //note: can occur multiple times during frame
        {
            //end of frame - send leftover
            if ( s_video_frame_started )
            {
                s_video_frame_started = false;

                s_stats.video_frames++;
                s_stats.video_frames_expected++;

                //LOG("Finish: %d %d\n", s_video_frame_index, s_video_frame_data_size);
                if ((s_video_frame_data_size > 0) && !s_encoder_output_ovf_flag) //left over
                {
                    send_air2ground_video_packet(true);
                }

                if ( s_video_full_frame_size  < 2000)
                {
                    //probably broken frame - too small
                    cam_ovf_count++;
                    s_stats.video_frames_expected++;
                }

                //end of frame - stats, camera settings, osd, mavlink
                if ( s_video_full_frame_size > 0)
                {
                    if ( (s_stats.camera_frame_size_min == 0) || (s_stats.camera_frame_size_min > s_video_full_frame_size) )
                    {
                        s_stats.camera_frame_size_min = s_video_full_frame_size;
                    } 

                    if ( s_stats.camera_frame_size_max < s_video_full_frame_size )
                    {
                        s_stats.camera_frame_size_max = s_video_full_frame_size;
                    } 

                    recalculateFrameSizeQualityK(s_video_full_frame_size);
                    applyAdaptiveQuality();
#ifdef PROFILE_CAMERA_DATA    
                    s_profiler.set(PF_CAMERA_FRAME_QUALITY, s_quality);
#endif
                    s_stats.fec_spin_count += s_fec_spin_count;
                    s_fec_spin_count = 0;
#ifdef PROFILE_CAMERA_DATA    
                    s_profiler.set(PF_CAMERA_DATA_SIZE, 0);
#endif
                    handle_ground2air_config_packetEx2(false);

                    if ( !g_osd.isLocked() && (g_osd.isChanged() || (s_osdUpdateCounter == 15)) )
                    {
                        send_air2ground_osd_packet();
                        s_osdUpdateCounter = 0;
                    }
                    else
                    {
                        s_osdUpdateCounter++;
                    }

#ifdef UART_MAVLINK
                    send_air2ground_data_packet();
#endif

                    int64_t dt = millis() - s_last_config_packet_tp;
                    if ( dt > 500 ) 
                    {
                        s_last_config_packet_tp = millis();
                        send_air2ground_config_packet();
                    }

                }

                s_video_frame_index++;
            }
        }

        //zero start of the DMA block
        for ( int i = 0; i < 16; i++ )
        {
            *clrSrc = 0;
            clrSrc += stride;
        }

        s_fec_encoder.unlock();
    }

#ifdef PROFILE_CAMERA_DATA    
    s_profiler.set(PF_CAMERA_DATA, 0);
#endif

    return count;
}

//=============================================================================================
//=============================================================================================
static void init_camera()
{
    printf("Init camera...\n");

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
#ifdef SENSOR_OV5640    
    config.xclk_freq_hz = 20000000;
#else
    config.xclk_freq_hz = 12000000;  //real frequency will be 80Mhz/6 = 13,333Mhz and we use clk2x
#endif    
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 63;  //start from lowest quality to decrease pressure at startup
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.data_available_callback = camera_data_available;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        LOG("Camera init failed with error 0x%x", err);
        return;
    }
}

//#define SHOW_CPU_USAGE

//=============================================================================================
//=============================================================================================
static void print_cpu_usage()
{
#ifdef SHOW_CPU_USAGE
    TaskStatus_t* pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime, ulStatsAsPercentage;

    // Take a snapshot of the number of tasks in case it changes while this
    // function is executing.
    uxArraySize = uxTaskGetNumberOfTasks();
    //LOG("%u tasks\n", uxArraySize);

    // Allocate a TaskStatus_t structure for each task.  An array could be
    // allocated statically at compile time.
    pxTaskStatusArray = (TaskStatus_t*)heap_caps_malloc(uxArraySize * sizeof(TaskStatus_t), MALLOC_CAP_SPIRAM);

    if (pxTaskStatusArray != NULL)
    {
        // Generate raw status information about each task.
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
        //LOG("%u total usage\n", ulTotalRunTime);

        // For percentage calculations.
        ulTotalRunTime /= 100UL;

        // Avoid divide by zero errors.
        if (ulTotalRunTime > 0)
        {
            // For each populated position in the pxTaskStatusArray array,
            // format the raw data as human readable ASCII data
            for (x = 0; x < uxArraySize; x++)
            {
                // What percentage of the total run time has the task used?
                // This will always be rounded down to the nearest integer.
                // ulTotalRunTimeDiv100 has already been divided by 100.
                ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

                if (ulStatsAsPercentage > 0UL)
                {
                    LOG("%s\t\t%u\t\t%u%%\r\n", pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter, ulStatsAsPercentage);
                }
                else
                {
                    // If the percentage is zero here then the task has
                    // consumed less than 1% of the total run time.
                    LOG("%s\t\t%u\t\t<1%%\r\n", pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter);
                }
            }
        }

        // The array is no longer needed, free the memory it consumes.
        free(pxTaskStatusArray);
    }
#endif
}

//=============================================================================================
//=============================================================================================
uint16_t generateDeviceId() 
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA); // Read base MAC address
    uint16_t id = 0;

    // Fold MAC address into 16 bits
    id ^= (mac[0] << 8) | mac[1];
    id ^= (mac[2] << 8) | mac[3];
    id ^= (mac[4] << 8) | mac[5];

    return id;
}
//=============================================================================================
//=============================================================================================
void readConfig()
{
    s_air_device_id = (uint16_t)nvs_args_read( "deviceId", 0 );
    if( s_air_device_id == 0 )
    {
        s_air_device_id = generateDeviceId();
        nvs_args_set("deviceId", s_air_device_id);
    }
    LOG("Air Device ID: 0x%04x\n", (int)s_air_device_id);

    s_ground2air_config_packet.dataChannel.wifi_channel = (uint16_t)nvs_args_read( "channel", DEFAULT_WIFI_CHANNEL );
    if((s_ground2air_config_packet.dataChannel.wifi_channel < 1)  || (s_ground2air_config_packet.dataChannel.wifi_channel > 13))
    {
        s_ground2air_config_packet.dataChannel.wifi_channel = DEFAULT_WIFI_CHANNEL;
        nvs_args_set("channel", s_ground2air_config_packet.dataChannel.wifi_channel);
    }

    s_ground2air_config_packet.dataChannel.wifi_rate = (WIFI_Rate)nvs_args_read( "rate", -1 );
    if( s_ground2air_config_packet.dataChannel.wifi_rate > WIFI_Rate::RATE_N_72M_MCS7_S )
    {
        s_ground2air_config_packet.dataChannel.wifi_rate = DEFAULT_WIFI_RATE;
        nvs_args_set("rate", (uint32_t)s_ground2air_config_packet.dataChannel.wifi_rate);
    }

    s_ground2air_config_packet.dataChannel.fec_codec_k = (uint8_t)nvs_args_read( "fec_k", 0 );
    s_ground2air_config_packet.dataChannel.fec_codec_n = (uint8_t)nvs_args_read( "fec_n", 0 );

    if( 
        (s_ground2air_config_packet.dataChannel.fec_codec_k == 0) ||
        (s_ground2air_config_packet.dataChannel.fec_codec_k > 12) || 
        (s_ground2air_config_packet.dataChannel.fec_codec_n == 0) ||
        (s_ground2air_config_packet.dataChannel.fec_codec_n > 12) ||
        (s_ground2air_config_packet.dataChannel.fec_codec_k >= s_ground2air_config_packet.dataChannel.fec_codec_n) 
        )
    {
        s_ground2air_config_packet.dataChannel.fec_codec_k = 6;
        s_ground2air_config_packet.dataChannel.fec_codec_n = 8;
        nvs_args_set("fec_k", s_ground2air_config_packet.dataChannel.fec_codec_k);
        nvs_args_set("fec_n", s_ground2air_config_packet.dataChannel.fec_codec_n);
    }

    s_ground2air_config_packet.camera.resolution = (Resolution)nvs_args_read("resolution", (uint32_t)Resolution::SVGA);
    if ( s_ground2air_config_packet.camera.resolution > Resolution::HD  )
    {
        s_ground2air_config_packet.camera.resolution = Resolution::SVGA;
        nvs_args_set("resolution", (uint32_t)s_ground2air_config_packet.camera.resolution);
    }

    s_ground2air_config_packet.camera.brightness = (int8_t)nvs_args_read("brightness", 10);
    if ( ( s_ground2air_config_packet.camera.brightness < -2 ) || ( s_ground2air_config_packet.camera.brightness > 2 ) )
    {
        s_ground2air_config_packet.camera.brightness = 0;
        nvs_args_set("brightness", s_ground2air_config_packet.camera.brightness);
    }

    s_ground2air_config_packet.camera.contrast = (int8_t)nvs_args_read("contrast", 10);
    if ((s_ground2air_config_packet.camera.contrast < -2) || (s_ground2air_config_packet.camera.contrast > 2))
    {
        s_ground2air_config_packet.camera.contrast = 0; // Default contrast
        nvs_args_set("contrast", s_ground2air_config_packet.camera.contrast);
    }

    s_ground2air_config_packet.camera.saturation = (int8_t)nvs_args_read("saturation", 10);
    if ((s_ground2air_config_packet.camera.saturation < -2) || (s_ground2air_config_packet.camera.saturation > 2))
    {
        s_ground2air_config_packet.camera.saturation = 1; // Default saturation
        nvs_args_set("saturation", s_ground2air_config_packet.camera.saturation);
    }

    s_ground2air_config_packet.camera.sharpness = (int8_t)nvs_args_read("sharpness", 10);
    if ((s_ground2air_config_packet.camera.sharpness < -2) || (s_ground2air_config_packet.camera.sharpness > 3))
    {
        s_ground2air_config_packet.camera.sharpness = 0; // Default sharpness
        nvs_args_set("sharpness", s_ground2air_config_packet.camera.sharpness);
    }

    s_ground2air_config_packet.camera.ae_level = (int8_t)nvs_args_read("ae_level", 10);
    if ((s_ground2air_config_packet.camera.ae_level < -2) || (s_ground2air_config_packet.camera.ae_level > 2))
    {
        s_ground2air_config_packet.camera.ae_level = 1; // Default ae_level
        nvs_args_set("ae_level", s_ground2air_config_packet.camera.ae_level);
    }

    s_ground2air_config_packet.camera.vflip = nvs_args_read( "vflip", 0 ) == 1;
    s_ground2air_config_packet.camera.ov2640HighFPS = nvs_args_read( "ov2640hfps", 0 ) == 1;
    s_ground2air_config_packet.camera.ov5640HighFPS = nvs_args_read( "ov5640hfps", 0 ) == 1;

    s_ground2air_config_packet.dataChannel.autostartRecord = nvs_args_read( "autostartRecord", 1 );

    s_ground2air_config_packet.dataChannel.cameraStopChannel = nvs_args_read( "cameraStopCH", 0 );
    if ( s_ground2air_config_packet.dataChannel.cameraStopChannel > 18 )
    {
        s_ground2air_config_packet.dataChannel.cameraStopChannel = 0;
    }

    s_ground2air_config_packet.dataChannel.mavlink2mspRC = nvs_args_read( "mavlink2mspRC", 0 );

    s_ground2air_config_packet2 = s_ground2air_config_packet;
}

//=============================================================================================
//=============================================================================================
extern "C" void app_main()
{
    //esp_task_wdt_init();

    printf("Initializing...\n");

    s_ground2air_data_packet.type = Ground2Air_Header::Type::Telemetry;
    s_ground2air_data_packet.size = sizeof(s_ground2air_data_packet);

    s_ground2air_config_packet.type = Ground2Air_Header::Type::Config;
    s_ground2air_config_packet.size = sizeof(s_ground2air_config_packet);
    s_ground2air_config_packet.dataChannel.wifi_rate = DEFAULT_WIFI_RATE;

    printf("MEMORY at start: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);
    heap_caps_print_heap_info(MALLOC_CAP_EXEC);

    // initialize_status_led();
    // initialize_esp32cam_flash_led_pin(true);
    // initialize_rec_button();
    initialize_gpio_control_pin();

#ifdef ENABLE_PROFILER
    s_profiler.init();
#endif

#ifdef INIT_UART_0
    printf("Init UART0...\n");
    uart_config_t uart_config0 = 
    {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
        .flags = 0
    };  
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config0) );
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, TXD0_PIN, RXD0_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif

    //reinitialize rec button after configuring uarts
    // initialize_rec_button();

    nvs_args_init();

    readConfig();

    //allocates large continuous Wifi output bufer. Allocate ASAP until memory is not fragmented.
    setup_wifi(s_ground2air_config_packet.dataChannel.wifi_rate, s_ground2air_config_packet.dataChannel.wifi_channel, s_ground2air_config_packet.dataChannel.wifi_power, packet_received_cb);
    s_fec_encoder.packetFilter.set_packet_header_data( s_air_device_id, 0 );

    if ( !getButtonState() )
    {
        //allocates 16kb dma buffer. Allocate ASAP before memory is fragmented.
        init_camera();
    }

#ifdef INIT_UART_1
    printf("Init UART1...\n");

    uart_config_t uart_config1 = 
    {
        .baud_rate = UART1_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
        .flags = 0
    };  

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config1) );
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, UART1_RX_BUFFER_SIZE, UART1_TX_BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

#endif

//initialize UART2 after SD
#ifdef INIT_UART_2
    printf("Init UART2...\n");

    uart_config_t uart_config2 = 
    {
        .baud_rate = UART2_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
        .flags = 0
    };  

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config2) );
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, UART2_RX_BUFFER_SIZE, UART2_TX_BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif

    printf("MEMORY Before Loop: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);
    heap_caps_print_heap_info(MALLOC_CAP_EXEC);

    handle_ground2air_config_packetEx1( s_ground2air_config_packet);
    handle_ground2air_config_packetEx2( true);
    set_ground2air_config_packet_handler(handle_ground2air_config_packet);
    set_ground2air_connect_packet_handler(handle_ground2air_connect_packet);
    set_ground2air_data_packet_handler(handle_ground2air_data_packet);
    set_ground2air_control_packet_handler(handle_ground2air_control_packet);

    LOG("WIFI channel: %d\n", s_ground2air_config_packet.dataChannel.wifi_channel );

    temperature_sensor_init();

    s_initialized = true;

    while (true)
    {
        int dt = millis() - s_stats_last_tp;
        if (dt >= 1000)
        {
            s_stats_last_tp = millis();

            s_actual_capture_fps = (int)(s_stats.video_frames)* 1000 / dt;
            s_actual_capture_fps_expected = (int)(s_stats.video_frames_expected)* 1000 / dt;
            s_max_wlan_outgoing_queue_usage = getMaxWlanOutgoingQueueUsage();
            s_min_wlan_outgoing_queue_usage_seen = getMinWlanOutgoingQueueUsageSeen();
            
            s_stats.wlan_error_count += s_fec_wlan_error_count;
            s_fec_wlan_error_count = 0;

            if (s_uart_verbose > 0 )
            {
                LOG("WLAN S: %d, R: %d, E: %d, F: %d, D: %d, %%: %d...%d || FPS: %d(%d), D: %d || SD D: %d, E: %d || TLM IN: %d OUT: %d\nSK1: %d SK2: %d, SK3: %d, Q: %d s: %d ovf:%d || sbg: %d || AIR: 0x%04x || GS: 0x%04x\n ",
                    s_stats.wlan_data_sent, s_stats.wlan_data_received, s_stats.wlan_error_count, s_stats.fec_spin_count,
                    s_stats.wlan_received_packets_dropped, s_min_wlan_outgoing_queue_usage_seen, s_max_wlan_outgoing_queue_usage, 
                    s_actual_capture_fps, s_actual_capture_fps_expected, s_stats.video_data, s_stats.sd_data, s_stats.sd_drops, 
                    s_stats.in_telemetry_data, s_stats.out_telemetry_data,
                    (int)(s_quality_framesize_K1*100),  (int)(s_quality_framesize_K2*100), (int)(s_quality_framesize_K3*100), 
                    s_quality, (s_stats.camera_frame_size_min + s_stats.camera_frame_size_max)/2, cam_ovf_count, s_dbg,
                    s_air_device_id, s_connected_gs_device_id); 
                print_cpu_usage();
            }

            s_max_frame_size = 0;
            s_dbg = 0;

            if ( s_stats.fec_spin_count > 0 )
            {
                s_wifi_ovf_time = esp_timer_get_time();
            }

            s_last_stats = s_stats;
            s_stats = Stats();

            temperature_sensor_read(&s_camera_temperature);

            if ( s_camera_stopped_requested != s_camera_stopped )
            {
                s_camera_stopped = s_camera_stopped_requested;
                if ( s_camera_stopped )
                {
                    LOG("Camera stopped\n");
                    esp_camera_deinit();
                }
                else
                {
                    LOG("Camera started\n");
                    init_camera();
                }
            }
        }

        dt = millis() - s_last_osd_packet_tp;
        if ( dt > 200 ) 
        {
            s_last_osd_packet_tp = millis();
            
            if ( s_camera_stopped )
            {
                s_fec_encoder.lock();

                if ( !g_osd.isLocked() && (g_osd.isChanged() || (s_osdUpdateCounter == 15)) )
                {
                    send_air2ground_osd_packet();
                    s_osdUpdateCounter = 0;
                }
                else
                {
                    s_osdUpdateCounter++;
                }
                
                s_fec_encoder.unlock();
            }
        }

        if ( s_accept_connection_timeout_ms != 0 ) 
        {
            if ( s_accept_connection_timeout_ms < millis() )
            {
                unpairGS();
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
        //esp_task_wdt_reset();

        checkButton();

        // update_status_led();

#ifdef ENABLE_PROFILER
        if ( s_profiler.full() || s_profiler.timedOut() )
        {
            LOG("Profiler stopped!\n");
            s_profiler.stop();
            s_profiler.save();
            s_profiler.clear();
        } 
#endif

#ifdef UART_MSP_OSD
        //the msp.loop() should be called every ~10ms
        //115200 BAUD is 11520 bytes per second or 115 bytes per 10 ms
        //with UART RX buffer of 512 we are save with periods 10...40ms
        g_msp.loop();
#endif

        if ((s_restart_time!=0) && ( esp_timer_get_time()>s_restart_time))
        {
            esp_restart();
        }

    }

}

/*
Air receive:
1) packet_received_cb()- called by Wifi library when wifi packet is received
  - feeds data to s_fec_decoder.decode_data(). No data type checking at all.

2) s_fec_decoder.decode_data()
 - allocates item in m_decoder.packet_pool
 - concatenates small packets until mtu size (because wifi packets may be broken to smaller parts by wifi layer)
 - enquees packets into m_decoder.packet_queue

3) decoder_task_proc()  
 - retrives item from m_decoder.packet_queue
 - inserts either in m_decoder.block_packets or m_decoder.block_fec_packets
 - inserts into s_wlan_incoming_queue either received or restored packets
 - signals s_wifi_rx_task

4) s_wifi_rx_task
  - parses packets: Ground2Air_Header::Type::Config, Ground2Air_Header::Type::Data
 

Air send:
1) camera_data_available callback from camera library
 - send_air2ground_video_packet() - passes Air2Ground_Video_Packet to s_fec_encode.
 - flush_encode_packet() - concatenates data until mtu size and passes to m_encoder.packet_queue
 
 2) encoder_task_proc()
  - gathers packets in m_encoder.block_packets
  - calls fec_encode(()
  - add_to_wlan_outgoing_queue() - places encoded packets into s_wlan_outgoing_queue

3) wifi_tx_proc
 - reads s_wlan_outgoing_queue
 - calls esp_wifi_80211_tx() 

*/
