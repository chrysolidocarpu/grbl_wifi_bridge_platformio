// GRBL WiFi bridge for ESP32-S3
// Laptop joins the ESP's WiFi AP -> raw TCP -> ESP32-S3 USB host -> CH340 on the laser.
// LightBurn: GRBL device, WiFi/TCP, IP 192.168.4.1, port TCP_PORT.

#include <cstring>
#include <memory>
#include <cstdlib>
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp.hpp"

using namespace esp_usb;

// ---- Settings -------------------------------------------------------------
#define AP_SSID      "Acmer-S1"
#define AP_PASS      "AcmerS1234"   // min 8 characters
#define AP_CHANNEL   6
#define TCP_PORT     23
#define GRBL_BAUD    115200
#define AIR_GPIO         4          // avoid 0,3,45,46 (strapping), 19/20 (USB), 26-32, 43/44
#define AIR_PULSED_GPIO  5          // avoid 0,3,45,46 (strapping), 19/20 (USB), 26-32, 43/44
#define AIR_PULSED_ACTIVE_HIGH  1   // 1: PWM high = MOSFET on
#define M_CODE_AIRASSIST 7          // M7: Air Assist
#define PULSE_FREQ_HZ    20000
#define PULSE_RES_BITS   10
#define PULSE_TARGET_DUTY_PCT 40    // M7 without S: duty reached at the end of the ramp
#define PULSE_S_MAX      1000       // M7 Pn: n / PULSE_S_MAX = duty fraction
#define PULSE_RAMP_MS    5000       // M7 ramp-up time
#define AIR_ACTIVE_HIGH  1          // 1: GPIO high = air on
// ---------------------------------------------------------------------------

static const char *TAG = "grbl_bridge";

static std::unique_ptr<CdcAcmDevice> g_vcp;     // the laser, when connected
static SemaphoreHandle_t g_vcp_lock;            // protects g_vcp
static SemaphoreHandle_t g_disconnect_sem;      // given when the laser is unplugged
static StreamBufferHandle_t g_usb_rx;           // laser -> TCP
static volatile int g_client = -1;              // current TCP client socket
static volatile bool g_air_off_pending = false;

static inline void air_set(bool on)
{
    gpio_set_level((gpio_num_t)AIR_GPIO, (on == (AIR_ACTIVE_HIGH != 0)) ? 1 : 0);
}

static const uint32_t PULSE_MAX_DUTY = (1u << PULSE_RES_BITS) - 1;

static uint32_t pulse_duty_raw(uint32_t pct)
{
    uint32_t d = PULSE_MAX_DUTY * pct / 100;
    return AIR_PULSED_ACTIVE_HIGH ? d : PULSE_MAX_DUTY - d;
}

static void pulse_init()
{
    ledc_timer_config_t t = {};
    t.speed_mode = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = (ledc_timer_bit_t)PULSE_RES_BITS;
    t.timer_num = LEDC_TIMER_0;
    t.freq_hz = PULSE_FREQ_HZ;
    t.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&t);

    ledc_channel_config_t c = {};
    c.gpio_num = AIR_PULSED_GPIO;
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = LEDC_CHANNEL_0;
    c.timer_sel = LEDC_TIMER_0;
    c.duty = pulse_duty_raw(0);
    ledc_channel_config(&c);
    ledc_fade_func_install(0);
}

// Hardware fade from the current duty to pct over ms (0 = immediate).
static void pulse_fade_to(uint32_t pct, uint32_t ms)
{
    ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                 pulse_duty_raw(pct), ms ? ms : 1, LEDC_FADE_NO_WAIT);
}

static void pulse_start(uint32_t pct)
{
    pulse_fade_to(pct > 100 ? 100 : pct, PULSE_RAMP_MS);
}

// Returns the S word value on the line (ignoring comments), or -1 if absent.
static long find_s_word(const char *line)
{
    bool in_paren = false;
    for (const char *p = line; *p; ++p) {
        char c = *p;
        if (in_paren) { if (c == ')') in_paren = false; continue; }
        if (c == '(') { in_paren = true; continue; }
        if (c == ';') break;
        if (c == 'S' || c == 's') {
            char *end;
            double v = strtod(p + 1, &end);
            if (end != p + 1) return (long)v;
        }
    }
    return -1;
}

static void pulse_stop()
{
    // ledc_stop leaves the pin at the idle level: off for either polarity
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, AIR_PULSED_ACTIVE_HIGH ? 0 : 1);
}

static void air_init()
{
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << AIR_GPIO;
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    air_set(false);
    pulse_init();
    pulse_stop();
}

static void outputs_off()
{
    air_set(false);
    pulse_stop();
}

static void handle_gcode_line(const char *line)
{
    bool in_paren = false;
    for (const char *p = line; *p; ++p) {
        char c = *p;
        if (in_paren) { if (c == ')') in_paren = false; continue; }
        if (c == '(') { in_paren = true; continue; }
        if (c == ';') break;
        if (c == 'M' || c == 'm') {
            char *end;
            long v = strtol(p + 1, &end, 10);
            if (end == p + 1) continue;
            if (v == 7) {
                g_air_off_pending = false;
                long s = find_s_word(line);
                uint32_t pct = PULSE_TARGET_DUTY_PCT;
                if (s >= 0) pct = (uint32_t)(s > PULSE_S_MAX ? PULSE_S_MAX : s) * 1000 / PULSE_S_MAX;
                pulse_start(pct);
                ESP_LOGI(TAG, "Pulsed output ramping to %u%%", (unsigned)pct);
            } else if (v == 8) {
                g_air_off_pending = false;
                uint32_t pct = 100;
                pulse_start(pct);
                ESP_LOGI(TAG, "Pulsed output ramping to max %u%%", (unsigned)pct);
            } else if (v == 9) {
                g_air_off_pending = true;     // switch off once GRBL is idle
            }

            p = end - 1;
        }
    }
}

static char   g_line[160];
static size_t g_line_len = 0;
static bool   g_line_overflow = false;

static void gcode_reset()
{
    g_line_len = 0;
    g_line_overflow = false;
}

// Called with every byte block that goes client -> laser.
static void gcode_feed(const uint8_t *d, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        uint8_t c = d[i];
        if (c == 0x18) {                       // GRBL soft reset: stop everything
            g_air_off_pending = false;
            outputs_off();
            gcode_reset();
            continue;
        }
        if (c == '?' || c == '!' || c == '~') continue;   // real-time chars
        if (c == '\n' || c == '\r') {
            if (g_line_len && !g_line_overflow) {
                g_line[g_line_len] = 0;
                handle_gcode_line(g_line);
            }
            gcode_reset();
            continue;
        }
        if (g_line_len < sizeof(g_line) - 1) g_line[g_line_len++] = c;
        else g_line_overflow = true;
    }
}

// Called with every byte block that comes laser -> client.
// Looks for "<Idle" in status reports to finish a pending air-off.
static void scan_for_idle(const uint8_t *d, size_t n)
{
    static const char pat[] = "<Idle";
    static size_t m = 0;
    for (size_t i = 0; i < n; i++) {
        if (d[i] == (uint8_t)pat[m]) {
            if (++m == sizeof(pat) - 1) {
                m = 0;
                if (g_air_off_pending) {
                    g_air_off_pending = false;
                    outputs_off();
                    ESP_LOGI(TAG, "Outputs OFF");
                }
            }
        } else {
            m = (d[i] == '<') ? 1 : 0;
        }
    }
}

// ---- USB callbacks ---------------------------------------------------------
static void usb_event_cb(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "USB error %d", (int)event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "Laser unplugged");
        g_air_off_pending = false;
        outputs_off();                             // <-- added
        xSemaphoreGive(g_disconnect_sem);
        break;
    default:
        break;
    }
}

static bool usb_rx_cb(const uint8_t *data, size_t len, void *user_arg)
{
    // Keep this fast. Drop data if the buffer is full.
    scan_for_idle(data, len);  
    xStreamBufferSend(g_usb_rx, data, len, 0);
    return true;
}

// ---- USB host tasks --------------------------------------------------------
static void usb_lib_task(void *arg)
{
    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static void usb_manager_task(void *arg)
{
    cdc_acm_host_device_config_t cfg = {};
    cfg.connection_timeout_ms = 2000;
    cfg.out_buffer_size = 512;
    cfg.event_cb = usb_event_cb;
    cfg.data_cb = usb_rx_cb;
    cfg.user_arg = nullptr;

    while (true) {
        xSemaphoreTake(g_disconnect_sem, 0);   // clear stale signal

        ESP_LOGI(TAG, "Waiting for laser on USB...");
        std::unique_ptr<CdcAcmDevice> dev(VCP::open(&cfg));
        if (!dev) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        cdc_acm_line_coding_t lc = {};
        lc.dwDTERate = GRBL_BAUD;
        lc.bCharFormat = 0;      // 1 stop bit
        lc.bParityType = 0;      // none
        lc.bDataBits = 8;
        dev->line_coding_set(&lc);
        // Keep DTR/RTS low: toggling them can reset some GRBL boards.
        dev->set_control_line_state(false, false);

        xSemaphoreTake(g_vcp_lock, portMAX_DELAY);
        g_vcp = std::move(dev);
        xSemaphoreGive(g_vcp_lock);
        ESP_LOGI(TAG, "Laser connected at %d baud", GRBL_BAUD);

        xSemaphoreTake(g_disconnect_sem, portMAX_DELAY);   // wait for unplug

        xSemaphoreTake(g_vcp_lock, portMAX_DELAY);
        g_vcp.reset();
        xSemaphoreGive(g_vcp_lock);
    }
}

// ---- TCP side --------------------------------------------------------------
// Laser -> client
static void tcp_sender_task(void *arg)
{
    uint8_t buf[256];
    while (true) {
        size_t n = xStreamBufferReceive(g_usb_rx, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n == 0) continue;
        int fd = g_client;
        if (fd < 0) continue;                  // nobody connected: discard
        size_t off = 0;
        while (off < n) {
            int r = send(fd, buf + off, n - off, 0);
            if (r <= 0) break;
            off += r;
        }
    }
}

// Client -> laser (also accepts connections)
static void tcp_server_task(void *arg)
{
    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(TCP_PORT);
    if (bind(listen_fd, (sockaddr *)&addr, sizeof(addr)) != 0 || listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "bind/listen failed");
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "TCP server on port %d", TCP_PORT);

    while (true) {
        int fd = accept(listen_fd, nullptr, nullptr);
        if (fd < 0) continue;

        int nodelay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        int keepalive = 1, idle = 5, intvl = 2, cnt = 3;
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));

        g_client = fd;
        gcode_reset();
        ESP_LOGI(TAG, "Client connected");

        uint8_t buf[256];
        int n;
        while ((n = recv(fd, buf, sizeof(buf), 0)) > 0) {
            xSemaphoreTake(g_vcp_lock, portMAX_DELAY);
            if (g_vcp) {
                g_vcp->tx_blocking(buf, n, 500);
            }
            xSemaphoreGive(g_vcp_lock);
            gcode_feed(buf, n);  
        }

        g_client = -1;
        g_air_off_pending = false;
        outputs_off();
        close(fd);
        ESP_LOGI(TAG, "Client disconnected");
    }
}

// ---- WiFi access point -------------------------------------------------------
static void wifi_start_ap()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();    // default IP 192.168.4.1

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wc = {};
    strncpy((char *)wc.ap.ssid, AP_SSID, sizeof(wc.ap.ssid));
    strncpy((char *)wc.ap.password, AP_PASS, sizeof(wc.ap.password));
    wc.ap.ssid_len = strlen(AP_SSID);
    wc.ap.channel = AP_CHANNEL;
    wc.ap.max_connection = 2;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);         // lowest latency
    ESP_LOGI(TAG, "AP '%s' up, address 192.168.4.1", AP_SSID);
}

// ---- Entry point -------------------------------------------------------------
extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    air_init();
    g_vcp_lock = xSemaphoreCreateMutex();
    g_disconnect_sem = xSemaphoreCreateBinary();
    g_usb_rx = xStreamBufferCreate(4096, 1);

    wifi_start_ap();

    // USB host
    usb_host_config_t host_cfg = {};
    host_cfg.skip_phy_setup = false;
    host_cfg.intr_flags = ESP_INTR_FLAG_LEVEL1;
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, nullptr, 5, nullptr);
    ESP_ERROR_CHECK(cdc_acm_host_install(nullptr));
    VCP::register_driver<CH34x>();

    xTaskCreate(usb_manager_task, "usb_mgr", 6144, nullptr, 4, nullptr);
    xTaskCreate(tcp_sender_task, "tcp_tx", 4096, nullptr, 4, nullptr);
    xTaskCreate(tcp_server_task, "tcp_srv", 4096, nullptr, 4, nullptr);
}
