#include "Comms.h"
#include <iostream>
#include <string>
#include <deque>
#include <mutex>
#include <algorithm>
#include <cstdio>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include "Clock.h"
#include "IHAL.h"
#include "PI_HAL.h"
#include "imgui.h"
#include "HUD.h"
#include "Log.h"
#include "Video_Decoder.h" 
#include "crc.h"
#include "packets.h"
#include <thread>
#include "imgui_impl_opengl3.h"
#include "main.h"

#include "socket.h"

#ifdef TEST_LATENCY
extern "C"
{
#include "pigpio.h"
}
#endif
/*

Changed on the PI:

- Disable the compositor from raspi-config. This will increase FPS
- Change from fake to real driver: dtoverlay=vc4-fkms-v3d to dtoverlay=vc4-kms-v3d

*/

std::unique_ptr<IHAL> s_hal;
Comms s_comms;
Video_Decoder s_decoder;

/* This prints an "Assertion failed" message and aborts.  */
void __assert_fail(const char* __assertion, const char* __file, unsigned int __line, const char* __function)
{
    printf("assert: %s:%d: %s: %s", __file, __line, __function, __assertion);
    fflush(stdout);
    //    abort();
}

static std::thread s_comms_thread;

static std::mutex s_ground2air_config_packet_mutex;
static Ground2Air_Config_Packet s_ground2air_config_packet;

#ifdef TEST_LATENCY
static uint32_t s_test_latency_gpio_value = 0;
static Clock::time_point s_test_latency_gpio_last_tp = Clock::now();
#endif

struct{
    int socket_fd;
    bool record;
    FILE * record_file=nullptr;
    std::mutex record_mutex;
    int wifi_channel;
}s_groundstation_config;
float video_fps = 0;
static void comms_thread_proc()
{
    Clock::time_point last_stats_tp = Clock::now();
    Clock::time_point last_comms_sent_tp = Clock::now();
    uint8_t last_sent_ping = 0;
    Clock::time_point last_ping_sent_tp = Clock::now();
    Clock::duration ping_min = std::chrono::seconds(999);
    Clock::duration ping_max = std::chrono::seconds(0);
    Clock::duration ping_avg = std::chrono::seconds(0);
    size_t ping_count = 0;
    size_t sent_count = 0;
    size_t total_data = 0;
    int16_t min_rssi = 0;

    std::vector<uint8_t> video_frame;
    uint32_t video_frame_index = 0;
    uint8_t video_next_part_index = 0;

    struct RX_Data
    {
        std::array<uint8_t, AIR2GROUND_MTU> data;
        size_t size;
        int16_t rssi = 0;
    };

    RX_Data rx_data;


    


    while (true)
    {
        if (Clock::now() - last_stats_tp >= std::chrono::milliseconds(1000))
        {
            if (ping_count == 0)
            {
                ping_count = 0;
                ping_min = std::chrono::seconds(0);
                ping_max = std::chrono::seconds(0);
                ping_avg = std::chrono::seconds(0);
            }

            LOGI("Sent: {}, RX len: {}, RSSI: {}, Latency: {}/{}/{},vfps:{}", sent_count, total_data, min_rssi, 
                std::chrono::duration_cast<std::chrono::milliseconds>(ping_min).count(),
                std::chrono::duration_cast<std::chrono::milliseconds>(ping_max).count(),
                std::chrono::duration_cast<std::chrono::milliseconds>(ping_avg).count() / ping_count,video_fps);

            ping_min = std::chrono::seconds(999);
            ping_max = std::chrono::seconds(0);
            ping_avg = std::chrono::seconds(0);
            sent_count = 0;
            ping_count = 0;
            total_data = 0;
            min_rssi = 0;
            last_stats_tp = Clock::now();
        }

        if (Clock::now() - last_comms_sent_tp >= std::chrono::milliseconds(500))
        {
            std::lock_guard<std::mutex> lg(s_ground2air_config_packet_mutex);
            auto& config = s_ground2air_config_packet;
            config.ping = last_sent_ping; 
            config.type = Ground2Air_Header::Type::Config;
            config.size = sizeof(config);
            config.crc = 0;
            config.crc = crc8(0, &config, sizeof(config)); 
            s_comms.send(&config, sizeof(config), true);
            last_comms_sent_tp = Clock::now();
            last_ping_sent_tp = Clock::now();
            sent_count++;
        }

#ifdef TEST_LATENCY
        if (s_test_latency_gpio_value == 0 && Clock::now() - s_test_latency_gpio_last_tp >= std::chrono::milliseconds(200))
        {
            s_test_latency_gpio_value = 1;
            gpioWrite(17, s_test_latency_gpio_value);
            s_test_latency_gpio_last_tp = Clock::now();
#   ifdef TEST_DISPLAY_LATENCY
            s_decoder.inject_test_data(s_test_latency_gpio_value);
#   endif
        }
        if (s_test_latency_gpio_value != 0 && Clock::now() - s_test_latency_gpio_last_tp >= std::chrono::milliseconds(50))
        {
            s_test_latency_gpio_value = 0;
            gpioWrite(17, s_test_latency_gpio_value);
            s_test_latency_gpio_last_tp = Clock::now();
#   ifdef TEST_DISPLAY_LATENCY
            s_decoder.inject_test_data(s_test_latency_gpio_value);
#   endif
        }
#endif        

#ifdef TEST_DISPLAY_LATENCY
        std::this_thread::yield();

        //pump the comms to avoid packages accumulating
        s_comms.process();
        s_comms.receive(rx_data.data.data(), rx_data.size);
#else
        //receive new packets
        do
        {
            s_comms.process();
            if (!s_comms.receive(rx_data.data.data(), rx_data.size))
            {
                std::this_thread::yield();
                break;
            }

            rx_data.rssi = (int16_t)s_comms.get_input_dBm();

            //filter bad packets
            Air2Ground_Header& air2ground_header = *(Air2Ground_Header*)rx_data.data.data();
            if (air2ground_header.type != Air2Ground_Header::Type::Video)
            {
                LOGE("Unknown air packet: {}", air2ground_header.type);
                break;
            }

            uint32_t video_packet_size = air2ground_header.size;
            if (video_packet_size > rx_data.size)
            {
                LOGE("Video frame {}: data too big: {} > {}", video_frame_index, video_packet_size, rx_data.size);
                break;
            }

            if (video_packet_size < sizeof(Air2Ground_Video_Packet))
            {
                LOGE("Video frame {}: data too small: {} > {}", video_frame_index, video_packet_size, sizeof(Air2Ground_Video_Packet));
                break;
            }

            size_t payload_size = video_packet_size - sizeof(Air2Ground_Video_Packet);
            Air2Ground_Video_Packet& air2ground_video_packet = *(Air2Ground_Video_Packet*)rx_data.data.data();
            uint8_t crc = air2ground_video_packet.crc;
            air2ground_video_packet.crc = 0;
            uint8_t computed_crc = crc8(0, rx_data.data.data(), sizeof(Air2Ground_Video_Packet));
            if (crc != computed_crc)
            {
                LOGE("Video frame {}, {} {}: crc mismatch: {} != {}", air2ground_video_packet.frame_index, (int)air2ground_video_packet.part_index, payload_size, crc, computed_crc);
                break;
            }

            if (air2ground_video_packet.pong == last_sent_ping)
            {
                last_sent_ping++;
                auto d = (Clock::now() - last_ping_sent_tp) / 2;
                ping_min = std::min(ping_min, d);
                ping_max = std::max(ping_max, d);
                ping_avg += d;
                ping_count++;
            }

            total_data += rx_data.size;
            min_rssi = std::min(min_rssi, rx_data.rssi);
            //LOGI("OK Video frame {}, {} {} - CRC OK {}. {}", air2ground_video_packet.frame_index, (int)air2ground_video_packet.part_index, payload_size, crc, rx_queue.size());

            if ((air2ground_video_packet.frame_index + 200 < video_frame_index) ||                 //frame from the distant past? TX was restarted
                (air2ground_video_packet.frame_index > video_frame_index)) //frame from the future and we still have other frames enqueued? Stale data
            {
                //if (video_next_part_index > 0) //incomplete frame
                //   s_decoder.decode_data(video_frame.data(), video_frame.size());

                //if (video_next_part_index > 0)
                //    LOGE("Aborting video frame {}, {}", video_frame_index, video_next_part_index);

                video_frame.clear();
                video_frame_index = air2ground_video_packet.frame_index;
                video_next_part_index = 0;
            }
            if (air2ground_video_packet.frame_index == video_frame_index && air2ground_video_packet.part_index == video_next_part_index)
            {
                video_next_part_index++;
                size_t offset = video_frame.size();
                video_frame.resize(offset + payload_size);
                memcpy(video_frame.data() + offset, rx_data.data.data() + sizeof(Air2Ground_Video_Packet), payload_size);

                if (video_next_part_index > 0 && air2ground_video_packet.last_part != 0)
                {
                    //LOGI("Received frame {}, {}, size {}", video_frame_index, video_next_part_index, video_frame.size());
                    s_decoder.decode_data(video_frame.data(), video_frame.size());
                    if(s_groundstation_config.record){
                        std::lock_guard<std::mutex> lg(s_groundstation_config.record_mutex);
                        fwrite(video_frame.data(),video_frame.size(),1,s_groundstation_config.record_file);
                    }
                    if(s_groundstation_config.socket_fd>0){
                        send_data_to_udp(s_groundstation_config.socket_fd,video_frame.data(),video_frame.size());
                    }
                    video_next_part_index = 0;
                    video_frame.clear();
                }
            }
        } 
        while (false);
#endif
    }
}

static inline ImVec2 operator+(const ImVec2& lhs, const ImVec2& rhs)
{
    return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y);
}
static inline ImVec2 ImRotate(const ImVec2& v, float cos_a, float sin_a)
{
    return ImVec2(v.x * cos_a - v.y * sin_a, v.x * sin_a + v.y * cos_a);
}
void ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size, float angle, float uvAngle)
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    ImVec2 pos[4] =
        {
            center + ImRotate(ImVec2(-size.x * 0.5f, -size.y * 0.5f), cos_a, sin_a),
            center + ImRotate(ImVec2(+size.x * 0.5f, -size.y * 0.5f), cos_a, sin_a),
            center + ImRotate(ImVec2(+size.x * 0.5f, +size.y * 0.5f), cos_a, sin_a),
            center + ImRotate(ImVec2(-size.x * 0.5f, +size.y * 0.5f), cos_a, sin_a)};

    cos_a = cosf(uvAngle);
    sin_a = sinf(uvAngle);
    ImVec2 uvCenter(0.5f, 0.5f);
    ImVec2 uvs[4] =
        {
            uvCenter + ImRotate(ImVec2(-0.5f, -0.5f), cos_a, sin_a),
            uvCenter + ImRotate(ImVec2(+0.5f, -0.5f), cos_a, sin_a),
            uvCenter + ImRotate(ImVec2(+0.5f, +0.5f), cos_a, sin_a),
            uvCenter + ImRotate(ImVec2(-0.5f, +0.5f), cos_a, sin_a)};

    draw_list->AddImageQuad(tex_id, pos[0], pos[1], pos[2], pos[3], uvs[0], uvs[1], uvs[2], uvs[3], IM_COL32_WHITE);
}

int run(char* argv[])
{
    HUD hud(*s_hal);

    ImGuiIO& io = ImGui::GetIO();


    s_decoder.init(*s_hal);

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    s_comms_thread = std::thread(&comms_thread_proc);
    Ground2Air_Config_Packet config=s_ground2air_config_packet;
    config.wifi_rate = WIFI_Rate::RATE_G_18M_ODFM;//RATE_G_18M_ODFM;

    config.camera.resolution = Resolution::QVGA;
    config.camera.fps_limit = 0;
    config.camera.quality = 8;

    size_t video_frame_count = 0;


    Clock::time_point last_stats_tp = Clock::now();
    Clock::time_point last_tp = Clock::now();

    auto f = [&config,&argv]{

        ImGui::Begin("HAL");
        {
            {
                int value = config.wifi_power;
                ImGui::SliderInt("Power", &value, 2, 20);
                config.wifi_power = value;
            }
            {
                static int value = (int)config.wifi_rate;
                ImGui::SliderInt("Rate", &value, (int)WIFI_Rate::RATE_B_2M_CCK, (int)WIFI_Rate::RATE_N_72M_MCS7_S);
                config.wifi_rate = (WIFI_Rate)value;
            }
            {
                int value = (int)config.camera.resolution;
                ImGui::SliderInt("Resolution", &value, 0, 7);
                config.camera.resolution = (Resolution)value;
            }
            {
                int value = (int)config.camera.fps_limit;
                ImGui::SliderInt("FPS", &value, 0, 100);
                config.camera.fps_limit = (uint8_t)value;
            }
            {
                int value = config.camera.quality;
                ImGui::SliderInt("Quality", &value, 0, 63);
                config.camera.quality = value;
            }
            {
                int value = config.camera.gainceiling;
                ImGui::SliderInt("Gain", &value, 0, 6);
                config.camera.gainceiling = (uint8_t)value;
            }
            {
                int value = config.camera.sharpness;
                ImGui::SliderInt("Sharpness", &value, -1, 6);
                config.camera.sharpness = (int8_t)value;
            }
            {
                int value = config.camera.denoise;
                ImGui::SliderInt("Denoise", &value, 0, 0xFF);
                config.camera.denoise = (int8_t)value;
            }
            {
                ImGui::SliderInt("WIFI Channel", &s_groundstation_config.wifi_channel, 1, 12);
            }
            {
                //ImGui::Checkbox("LC", &config.camera.lenc);
                //ImGui::SameLine();
                //ImGui::Checkbox("DCW", &config.camera.dcw);
                //ImGui::SameLine();
                //ImGui::Checkbox("H", &config.camera.hmirror);
                //ImGui::SameLine();
                //ImGui::Checkbox("V", &config.camera.vflip);
                //ImGui::SameLine();
                //ImGui::Checkbox("Raw", &config.camera.raw_gma);
                //ImGui::SameLine();
                bool last_record=s_groundstation_config.record;
                ImGui::Checkbox("Record", &config.dvr_record);
                ImGui::Checkbox("GS Record",&s_groundstation_config.record);
                if(s_groundstation_config.record != last_record){
                    std::lock_guard<std::mutex> lg(s_groundstation_config.record_mutex);
                    if(s_groundstation_config.record){
                        auto time=std::time({});
                        char filename[]="yyyy-mm-dd-hh:mm:ss.mjpeg";
                        std::strftime(filename, sizeof(filename), "%F-%T.mjpeg", std::localtime(&time));
                        s_groundstation_config.record_file=fopen(filename,"wb+");

                        LOGI("start record:{}",std::string(filename));
                    }else{
                        fflush(s_groundstation_config.record_file);
                        fclose(s_groundstation_config.record_file);
                        s_groundstation_config.record_file=nullptr;
                    }
                }
            }
            if (ImGui::Button("Exit")){
                if(s_groundstation_config.record){
                    std::lock_guard<std::mutex> lg(s_groundstation_config.record_mutex);
                    fflush(s_groundstation_config.record_file);
                    fclose(s_groundstation_config.record_file); 
                }
                abort();
            }
            if (ImGui::Button("Restart")){
                char tempstr[30];
                sprintf(tempstr,"ESPVTX_WIFI_CHN=%d",s_groundstation_config.wifi_channel);
                putenv(tempstr);
                execv(argv[0],argv);
            }

            ImGui::Text("%.3f ms/frame (%.1f FPS) %.1f VFPS", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate, video_fps);
        }
        ImGui::End();

        std::lock_guard<std::mutex> lg(s_ground2air_config_packet_mutex);
        s_ground2air_config_packet = config;
    };

    s_hal->add_render_callback(f);
    while (true)
    {
        s_decoder.unlock_output();
        size_t count = s_decoder.lock_output();
        video_frame_count += count;
        s_hal->set_video_channel(s_decoder.get_video_texture_id());

        s_hal->process();

        if (Clock::now() - last_stats_tp >= std::chrono::milliseconds(1000))
        {
            last_stats_tp = Clock::now();
            video_fps = video_frame_count;
            video_frame_count = 0;
        }

        Clock::time_point now = Clock::now();
        Clock::duration dt = now - last_tp;
        last_tp = now;
        io.DeltaTime = std::chrono::duration_cast<std::chrono::duration<float> >(dt).count();

    }

    return 0;
}

int main(int argc, const char* argv[])
{

    init_crc8_table();

    s_hal.reset(new PI_HAL());
    if (!s_hal->init())
        return -1;

#ifdef TEST_LATENCY
    gpioSetMode(17, PI_OUTPUT);
#endif

    Comms::RX_Descriptor rx_descriptor;
    rx_descriptor.coding_k = s_ground2air_config_packet.fec_codec_k;
    rx_descriptor.coding_n = s_ground2air_config_packet.fec_codec_n;
    rx_descriptor.mtu = s_ground2air_config_packet.fec_codec_mtu;
    rx_descriptor.interfaces = {"wlxc01c30222455"};
    Comms::TX_Descriptor tx_descriptor;
    tx_descriptor.coding_k = 2;
    tx_descriptor.coding_n = 6;
    tx_descriptor.mtu = GROUND2AIR_DATA_MAX_SIZE;
    tx_descriptor.interface = "wlxc01c30222455";

    for(int i=1;i<argc;++i){
        auto temp = std::string(argv[i]);
        auto next = i!=argc-1? std::string(argv[i+1]):std::string("");
        auto check_argval = [&next](std::string arg_name){
            if(next==""){throw std::string("please input correct ")+arg_name;}
        };
        if(temp=="--tx"){
            check_argval("tx");
            tx_descriptor.interface = next; 
            i++;
        }else if(temp=="-p"){
            check_argval("port");
            s_groundstation_config.socket_fd=udp_socket_init(std::string("127.0.0.1"),std::stoi(next));
            i++;
        }else if(temp=="-k"){
            check_argval("k");
            rx_descriptor.coding_k = std::stoi(next);
            s_ground2air_config_packet.fec_codec_k =  std::stoi(next);
            i++;
            LOGI("set rx fec_k to {}",rx_descriptor.coding_k);
        }else if(temp=="-n"){
            check_argval("n");
            rx_descriptor.coding_n = std::stoi(next);
            s_ground2air_config_packet.fec_codec_n =  std::stoi(next);
            i++;
            LOGI("set rx fec_n to {}",rx_descriptor.coding_n);
        }else if(temp=="--rx"){
            rx_descriptor.interfaces.clear();
        }else{
            rx_descriptor.interfaces.push_back(temp);
        }
    }

    {
        char *temp = getenv("ESPVTX_WIFI_CHN");
        if(temp){
            s_groundstation_config.wifi_channel = atoi(temp);
        }else{
            s_groundstation_config.wifi_channel = 11;
        }
    }
    

    if (!s_comms.init(rx_descriptor, tx_descriptor))
        return -1;

    for (const auto& itf: rx_descriptor.interfaces)
    {
        system(fmt::format("iwconfig {} channel 11", itf).c_str());
    }

    int result = run((char **)argv);

    s_hal->shutdown();

    return result;
}
