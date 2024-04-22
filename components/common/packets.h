#pragma once

#include "structures.h"

#define PACKET_VERSION 1

#pragma pack(push, 1) // exact fit - no padding

enum class WIFI_Rate : uint8_t
{
    /*  0 */ RATE_B_2M_CCK,
    /*  1 */ RATE_B_2M_CCK_S,
    /*  2 */ RATE_B_5_5M_CCK,
    /*  3 */ RATE_B_5_5M_CCK_S,
    /*  4 */ RATE_B_11M_CCK,
    /*  5 */ RATE_B_11M_CCK_S,

    /*  6 */ RATE_G_6M_ODFM,
    /*  7 */ RATE_G_9M_ODFM,
    /*  8 */ RATE_G_12M_ODFM,
    /*  9 */ RATE_G_18M_ODFM,
    /* 10 */ RATE_G_24M_ODFM,
    /* 11 */ RATE_G_36M_ODFM,
    /* 12 */ RATE_G_48M_ODFM,
    /* 13 */ RATE_G_54M_ODFM,

    /* 14 */ RATE_N_6_5M_MCS0,
    /* 15 */ RATE_N_7_2M_MCS0_S,
    /* 16 */ RATE_N_13M_MCS1,
    /* 17 */ RATE_N_14_4M_MCS1_S,
    /* 18 */ RATE_N_19_5M_MCS2,
    /* 19 */ RATE_N_21_7M_MCS2_S,
    /* 20 */ RATE_N_26M_MCS3,
    /* 21 */ RATE_N_28_9M_MCS3_S,
    /* 22 */ RATE_N_39M_MCS4,
    /* 23 */ RATE_N_43_3M_MCS4_S,
    /* 24 */ RATE_N_52M_MCS5,
    /* 25 */ RATE_N_57_8M_MCS5_S,
    /* 26 */ RATE_N_58M_MCS6,
    /* 27 */ RATE_N_65M_MCS6_S,
    /* 28 */ RATE_N_65M_MCS7,
    /* 29 */ RATE_N_72M_MCS7_S,
};

static constexpr size_t AIR2GROUND_MTU = WLAN_MAX_PAYLOAD_SIZE - 6; //6 is the fec header size

///////////////////////////////////////////////////////////////////////////////////////

constexpr size_t GROUND2AIR_DATA_MAX_SIZE = 64;

struct Ground2Air_Header
{
    enum class Type : uint8_t
    {
        Telemetry,
        Config,
    };

    Type type = Type::Telemetry; 
    uint32_t size = 0;
    uint8_t crc = 0;
};

constexpr size_t  GROUND2AIR_DATA_MAX_PAYLOAD_SIZE = GROUND2AIR_DATA_MAX_SIZE - sizeof(Ground2Air_Header);

struct Ground2Air_Data_Packet : Ground2Air_Header
{
    uint8_t payload[GROUND2AIR_DATA_MAX_PAYLOAD_SIZE];
};
static_assert(sizeof(Ground2Air_Data_Packet) <= GROUND2AIR_DATA_MAX_SIZE, "");

enum class Resolution : uint8_t
{
    QVGA,   //320x240
    CIF,    //400x296
    HVGA,   //480x320
    VGA,    //640x480
    SVGA,   //800x600
    XGA,    //1024x768
    SXGA,   //1280x1024
    UXGA,   //1600x1200
};

struct Ground2Air_Config_Packet : Ground2Air_Header
{
    uint8_t ping = 0; //used for latency measurement
    int8_t wifi_power = 20;//dBm
    WIFI_Rate wifi_rate = WIFI_Rate::RATE_G_24M_ODFM;
    uint8_t wifi_channel = 11;
    uint8_t fec_codec_k = 6;
    uint8_t fec_codec_n = 12;
    uint16_t fec_codec_mtu = AIR2GROUND_MTU;
    uint8_t air_record_btn = 0; //incremented each time button is pressed on gs

    struct Camera
    {
        Resolution resolution = Resolution::SVGA;
        uint8_t fps_limit = 35;
        uint8_t quality = 0;//0 - 63
        int8_t brightness = 0;//-2 - 2
        int8_t contrast = 0;//-2 - 2
        int8_t saturation = 1;//-2 - 2
        int8_t sharpness = 3;//-3 - 3
        uint8_t denoise = 0;
        uint8_t special_effect = 0;//0 - 6
        bool awb = true;
        bool awb_gain = true;
        uint8_t wb_mode = 0;//0 - 4
        bool aec = true;
        bool aec2 = true;
        int8_t ae_level = 1;//-2 - 2
        uint16_t aec_value = 204;//0 - 1200
        bool agc = false;
        uint8_t agc_gain = 0;//0 - 30
        uint8_t gainceiling = 3;//0 - 6
        bool bpc = true;
        bool wpc = true;
        bool raw_gma = true;
        bool lenc = true;
        bool hmirror = false;
        bool vflip = false;
        bool dcw = true;
    };
    Camera camera;
};
static_assert(sizeof(Ground2Air_Config_Packet) <= GROUND2AIR_DATA_MAX_SIZE, "");

///////////////////////////////////////////////////////////////////////////////////////

struct Air2Ground_Header
{
    enum class Type : uint8_t
    {
        Video,
        Telemetry
    };

    Type type = Type::Video; 
    uint32_t size = 0;
    uint8_t pong = 0; //used for latency measurement
    uint8_t version; //PACKET_VERSION
    uint8_t crc = 0;
};

struct Air2Ground_Video_Packet : Air2Ground_Header
{
    Resolution resolution;
    uint8_t part_index : 7;
    uint8_t last_part : 1;
    uint8_t wifi_queue : 7;
    uint8_t air_record_state : 1;
    WIFI_Rate curr_wifi_rate;
    uint32_t frame_index = 0;
    uint16_t freeSpaceGB16 : 12;
    uint16_t totalSpaceGB16 : 12;
    uint16_t curr_quality : 8;
    //data follows
};

struct Air2Ground_Data_Packet : Air2Ground_Header
{
};

static_assert(sizeof(Air2Ground_Video_Packet) == 20, "");

///////////////////////////////////////////////////////////////////////////////////////

#pragma pack(pop)

