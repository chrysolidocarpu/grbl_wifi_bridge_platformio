// GRBL WiFi bridge for ESP32-S3
// Laptop -> WiFi AP -> raw TCP -> ESP32-S3 USB host -> CH340 laser
//
// Manual controls:
//   Power button: 200 ms power-board button pulse; toggle tracked state.
//   Laser button: send GRBL M5.
//   Air button: toggle air assist between 40% PWM and OFF.
//
// Safety:
//   USB loss: laser OFF request, air OFF, power board OFF if tracked ON.
//   TCP loss: laser OFF request, air OFF; leave power board ON.
//   GRBL M7: air assist ramp to configured target.
//   GRBL M8: air assist ramp to 100%.
//   GRBL M9: turn air assist off when GRBL reports Idle.

#include <cstring>
#include <cstdlib>
#include <memory>
#include <atomic>

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
#include "lwip/tcp.h"
#include "unistd.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp.hpp"

using namespace esp_usb;


// ============================================================================
// SETTINGS
// ============================================================================

#define AP_SSID       "Acmer-S1"
#define AP_PASS       "AcmerS1234"
#define AP_CHANNEL    6
#define TCP_PORT      23
#define GRBL_BAUD     115200


// ---- Air assist ------------------------------------------------------------

// Existing digital enable output: PRESERVED
#define AIR_GPIO                  4
#define AIR_ACTIVE_HIGH           1

// Existing PWM output: PRESERVED, DO NOT CHANGE
#define AIR_PULSED_GPIO           5
#define AIR_PULSED_ACTIVE_HIGH    1

#define M_CODE_AIRASSIST          7
#define PULSE_FREQ_HZ             20000
#define PULSE_RES_BITS            10
#define PULSE_TARGET_DUTY_PCT     40
#define PULSE_S_MAX               1000
#define PULSE_RAMP_MS             10000


// ---- Power board -----------------------------------------------------------

// Control output connected to the power-board button interface
#define PWR_CTRL_GPIO             6
#define PWR_CTRL_ACTIVE_HIGH      1

// Manual button and status LED
#define PWR_BUTTON_GPIO           13
#define PWR_LED_GPIO              12
#define PWR_LED_ACTIVE_HIGH       1


// ---- Laser -----------------------------------------------------------------

// Laser is controlled through GRBL, not a physical enable output
#define LSR_BUTTON_GPIO           15
#define LSR_LED_GPIO              7
#define LSR_LED_ACTIVE_HIGH       1


// ---- System indicators -----------------------------------------------------

#define FIRST_BOOT_GPIO           8
#define FIRST_BOOT_ACTIVE_HIGH    1

#define USB_BOOT_GPIO             9
#define USB_STATE_ACTIVE_HIGH     1


// ---- Air-assist manual button and LED --------------------------------------

#define AIR_BUTTON_GPIO           10
#define AIR_LED_GPIO              11
#define AIR_LED_ACTIVE_HIGH       1


// ---- Button handling --------------------------------------------------------

#define BUTTON_DEBOUNCE_MS        30
#define BUTTON_POLL_MS            10

// Buttons are wired between GPIO and GND.
#define BUTTON_PRESSED_LEVEL      0


// ============================================================================
// STATE
// ============================================================================

static const char *TAG = "grbl_bridge";

static std::unique_ptr<CdcAcmDevice> g_vcp;

static SemaphoreHandle_t g_vcp_lock;
static SemaphoreHandle_t g_disconnect_sem;
static StreamBufferHandle_t g_usb_rx;

static volatile int g_client = -1;
static volatile bool g_air_off_pending = false;

// These are mutable software-tracked states, NOT #define constants.
static std::atomic<bool> PWR_STATE{false};
static std::atomic<bool> LSR_STATE{false};
static std::atomic<bool> AIR_STATE{false};
static std::atomic<bool> FIRST_BOOT{false};
static std::atomic<bool> USB_ESTABLISHED{false};


// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static void pulse_init(void);
static void pulse_start(uint32_t pct);
static void pulse_stop(void);

static void forward_to_laser(const uint8_t *d, size_t n);
static void outputs_off(void);
static void send_grbl_m5(void);

static void Emergency_usb_lost(void);
static void Emergency_tcp_connection_lost(void);


// ============================================================================
// GPIO HELPERS
// ============================================================================

static inline void set_indicator(
    gpio_num_t gpio,
    bool on,
    int active_high)
{
    gpio_set_level(
        gpio,
        (on == (active_high != 0)) ? 1 : 0
    );
}


// Air enable output
static inline void air_assist_set(bool on)
{
    set_indicator(
        (gpio_num_t)AIR_GPIO,
        on,
        AIR_ACTIVE_HIGH
    );
}


// Air status LED
static inline void air_led_set(bool on)
{
    set_indicator(
        (gpio_num_t)AIR_LED_GPIO,
        on,
        AIR_LED_ACTIVE_HIGH
    );
}


// Power-board button interface
static inline void power_state_set(bool on)
{
    set_indicator(
        (gpio_num_t)PWR_CTRL_GPIO,
        on,
        PWR_CTRL_ACTIVE_HIGH
    );
}


// Power status LED
static inline void power_led_set(bool on)
{
    set_indicator(
        (gpio_num_t)PWR_LED_GPIO,
        on,
        PWR_LED_ACTIVE_HIGH
    );
}


// Laser status LED
static inline void laser_led_set(bool on)
{
    set_indicator(
        (gpio_num_t)LSR_LED_GPIO,
        on,
        LSR_LED_ACTIVE_HIGH
    );
}


// First-boot indicator
static inline void first_boot_set(bool on)
{
    set_indicator(
        (gpio_num_t)FIRST_BOOT_GPIO,
        on,
        FIRST_BOOT_ACTIVE_HIGH
    );
}


// USB connection indicator
static inline void usb_state_set(bool on)
{
    set_indicator(
        (gpio_num_t)USB_BOOT_GPIO,
        on,
        USB_STATE_ACTIVE_HIGH
    );
}


// ============================================================================
// POWER BOARD
// ============================================================================

// Simulate a momentary button press.
// The output is released after 200 ms.
static void power_button_press(void)
{
    power_state_set(true);

    vTaskDelay(pdMS_TO_TICKS(200));

    power_state_set(false);
}


// Toggle the expected power-board state.
//
// IMPORTANT: the software cannot verify the actual board state without
// hardware feedback. This toggles the tracked state.
static void Cycle_PWR_STATE(void)
{
    bool next_state = !PWR_STATE.load();

    power_button_press();

    PWR_STATE.store(next_state);
    power_led_set(next_state);

    // Record that the first commanded power-button cycle has occurred.
    if (!FIRST_BOOT.load()) {
        FIRST_BOOT.store(true);
        first_boot_set(true);
    }

    ESP_LOGI(
        TAG,
        "Power-board state commanded: %s",
        next_state ? "ON" : "OFF"
    );
}


// ============================================================================
// LASER
// ============================================================================

// Request M5 over USB.
static void send_grbl_m5(void)
{
    static const uint8_t cmd[] = {'M', '5', '\n'};

    forward_to_laser(cmd, sizeof(cmd));

    // This indicates the commanded state, not confirmed laser output.
    LSR_STATE.store(false);
    laser_led_set(false);

    ESP_LOGI(TAG, "Laser OFF requested using M5");
}


static void laser_state_off(void)
{
    send_grbl_m5();
}


// ============================================================================
// AIR ASSIST
// ============================================================================

static void air_assist_on(uint32_t pct)
{
    if (pct > 100) {
        pct = 100;
    }

    g_air_off_pending = false;

    // Enable the air-assist hardware.
    air_assist_set(true);

    // Start the existing PWM ramp.
    pulse_start(pct);

    AIR_STATE.store(true);
    air_led_set(true);

    ESP_LOGI(TAG, "Air assist ON, target %lu%%", (unsigned long)pct);
}


static void air_assist_off(void)
{
    g_air_off_pending = false;

    // Disable both the digital enable and PWM output.
    air_assist_set(false);
    pulse_stop();

    AIR_STATE.store(false);
    air_led_set(false);

    ESP_LOGI(TAG, "Air assist OFF");
}


// Manual button: toggle between 40% PWM and OFF.
static void Cycle_AIR_ASSIST(void)
{
    if (AIR_STATE.load()) {
        air_assist_off();
    } else {
        air_assist_on(PULSE_TARGET_DUTY_PCT);
    }
}


// ============================================================================
// PWM
// ============================================================================

static const uint32_t PULSE_MAX_DUTY =
    (1u << PULSE_RES_BITS) - 1;


static uint32_t pulse_duty_raw(uint32_t pct)
{
    uint32_t d = PULSE_MAX_DUTY * pct / 100;

    return AIR_PULSED_ACTIVE_HIGH
        ? d
        : PULSE_MAX_DUTY - d;
}


static void pulse_init(void)
{
    ledc_timer_config_t t = {};

    t.speed_mode = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = (ledc_timer_bit_t)PULSE_RES_BITS;
    t.timer_num = LEDC_TIMER_0;
    t.freq_hz = PULSE_FREQ_HZ;
    t.clk_cfg = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t c = {};

    c.gpio_num = AIR_PULSED_GPIO;
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = LEDC_CHANNEL_0;
    c.timer_sel = LEDC_TIMER_0;
    c.duty = pulse_duty_raw(0);

    ESP_ERROR_CHECK(ledc_channel_config(&c));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}


// Hardware fade from current duty to requested percentage.
static void pulse_fade_to(uint32_t pct, uint32_t ms)
{
    esp_err_t err = ledc_set_fade_time_and_start(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_0,
        pulse_duty_raw(pct),
        ms ? ms : 1,
        LEDC_FADE_NO_WAIT
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PWM fade failed: %s", esp_err_to_name(err));
    }
}


static void pulse_start(uint32_t pct)
{
    if (pct > 100) {
        pct = 100;
    }

    pulse_fade_to(pct, PULSE_RAMP_MS);
}


static void pulse_stop(void)
{
    // Stop PWM at the inactive physical level.
    esp_err_t err = ledc_stop(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_0,
        AIR_PULSED_ACTIVE_HIGH ? 0 : 1
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PWM stop returned: %s", esp_err_to_name(err));
    }
}


// ============================================================================
// GRBL G-CODE PARSING
// ============================================================================

// Returns the S word value outside comments, or -1 if absent.
static long find_s_word(const char *line)
{
    bool in_paren = false;

    for (const char *p = line; *p; ++p) {
        char c = *p;

        if (in_paren) {
            if (c == ')') {
                in_paren = false;
            }
            continue;
        }

        if (c == '(') {
            in_paren = true;
            continue;
        }

        if (c == ';') {
            break;
        }

        if (c == 'S' || c == 's') {
            char *end;
            double v = strtod(p + 1, &end);

            if (end != p + 1) {
                return (long)v;
            }
        }
    }

    return -1;
}


// Interpret commands for safety outputs and the laser indicator.
static void handle_gcode_line(const char *line)
{
    bool in_paren = false;

    for (const char *p = line; *p; ++p) {
        char c = *p;

        if (in_paren) {
            if (c == ')') {
                in_paren = false;
            }
            continue;
        }

        if (c == '(') {
            in_paren = true;
            continue;
        }

        if (c == ';') {
            break;
        }

        if (c != 'M' && c != 'm') {
            continue;
        }

        char *end;
        long v = strtol(p + 1, &end, 10);

        if (end == p + 1) {
            continue;
        }

        switch (v) {
        case 3:
        case 4:
            // Laser commanded ON.
            LSR_STATE.store(true);
            laser_led_set(true);
            break;

        case 5:
            // Laser commanded OFF.
            LSR_STATE.store(false);
            laser_led_set(false);
            break;

        case 7: {
            long s = find_s_word(line);

            uint32_t pct = PULSE_TARGET_DUTY_PCT;

            if (s >= 0) {
                if (s > PULSE_S_MAX) {
                    s = PULSE_S_MAX;
                }

                pct = (uint32_t)s;
            }

            air_assist_on(pct);

            ESP_LOGI(TAG, "GRBL M7: air assist %lu%%",
                     (unsigned long)pct);
            break;
        }

        case 8:
            air_assist_on(100);
            ESP_LOGI(TAG, "GRBL M8: air assist 100%%");
            break;

        case 9:
            // Preserve the existing behavior: switch off when GRBL is Idle.
            pulse_stop();
            g_air_off_pending = true;
            break;

        default:
            break;
        }

        p = end - 1;
    }
}


// Remove S words from M7 lines so the laser firmware doesn't interpret
// the air-assist S value as a laser power setting.
static size_t strip_s_from_m7(const char *line, char *out)
{
    bool has_m7 = false;
    bool in_paren = false;

    for (const char *p = line; *p; ++p) {
        char c = *p;

        if (in_paren) {
            if (c == ')') {
                in_paren = false;
            }
            continue;
        }

        if (c == '(') {
            in_paren = true;
            continue;
        }

        if (c == ';') {
            break;
        }

        if (c == 'M' || c == 'm') {
            char *end;
            long v = strtol(p + 1, &end, 10);

            if (end == p + 1) {
                continue;
            }

            if (v == M_CODE_AIRASSIST) {
                has_m7 = true;
            }

            p = end - 1;
        }
    }

    size_t o = 0;
    in_paren = false;
    bool in_comment = false;

    for (const char *p = line; *p; ++p) {
        char c = *p;

        if (!has_m7 || in_comment) {
            out[o++] = c;
            continue;
        }

        if (in_paren) {
            out[o++] = c;
            if (c == ')') {
                in_paren = false;
            }
            continue;
        }

        if (c == '(') {
            in_paren = true;
            out[o++] = c;
            continue;
        }

        if (c == ';') {
            in_comment = true;
            out[o++] = c;
            continue;
        }

        if (c == 'S' || c == 's') {
            char *end;
            strtod(p + 1, &end);

            if (end != p + 1) {
                p = end - 1;
                continue;
            }
        }

        out[o++] = c;
    }

    out[o] = 0;
    return o;
}


// ============================================================================
// G-CODE STREAM PROCESSING
// ============================================================================

static char g_line[160];
static size_t g_line_len = 0;
static bool g_line_overflow = false;


static void gcode_reset(void)
{
    g_line_len = 0;
    g_line_overflow = false;
}


static void forward_to_laser(const uint8_t *d, size_t n)
{
    if (!n || !g_vcp_lock) {
        return;
    }

    xSemaphoreTake(g_vcp_lock, portMAX_DELAY);

    if (g_vcp) {
        g_vcp->tx_blocking(
            const_cast<uint8_t *>(d),
            n,
            500
        );
    }

    xSemaphoreGive(g_vcp_lock);
}


static void gcode_feed(const uint8_t *d, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        uint8_t c = d[i];

        if (c == 0x18) {
            // GRBL soft reset: shut down air-assist outputs.
            g_air_off_pending = false;
            outputs_off();
            gcode_reset();

            forward_to_laser(&c, 1);
            continue;
        }

        if (c == '?' || c == '!' || c == '~') {
            // GRBL real-time characters.
            forward_to_laser(&c, 1);
            continue;
        }

        if (c == '\n' || c == '\r') {
            if (!g_line_overflow && g_line_len) {
                g_line[g_line_len] = 0;

                handle_gcode_line(g_line);

                char out[sizeof(g_line)];

                size_t o = strip_s_from_m7(g_line, out);

                forward_to_laser((const uint8_t *)out, o);
            }

            forward_to_laser(&c, 1);
            gcode_reset();
            continue;
        }

        if (g_line_overflow) {
            forward_to_laser(&c, 1);
        } else if (g_line_len < sizeof(g_line) - 1) {
            g_line[g_line_len++] = c;
        } else {
            g_line_overflow = true;

            forward_to_laser(
                (const uint8_t *)g_line,
                g_line_len
            );

            forward_to_laser(&c, 1);
        }
    }
}


// ============================================================================
// GRBL STATUS SCANNER
// ============================================================================

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
                    air_assist_off();

                    ESP_LOGI(TAG, "GRBL Idle: air assist OFF");
                }
            }
        } else {
            m = (d[i] == '<') ? 1 : 0;
        }
    }
}


// ============================================================================
// EMERGENCY SAFETY
// ============================================================================

static void Emergency_usb_lost(void)
{
    ESP_LOGE(TAG, "USB lost: emergency shutdown");

    // Attempt M5, although transmission cannot be guaranteed after USB loss.
    laser_state_off();

    air_assist_off();

    // Power off only if software currently expects the board to be ON.
    if (PWR_STATE.load()) {
        Cycle_PWR_STATE();
    }

    USB_ESTABLISHED.store(false);
    usb_state_set(false);
}


static void Emergency_tcp_connection_lost(void)
{
    ESP_LOGW(TAG, "TCP client lost: laser and air assist OFF");

    laser_state_off();
    air_assist_off();

    // Intentionally leave the power board on.
}


// ============================================================================
// USB CALLBACKS
// ============================================================================

static void usb_event_cb(
    const cdc_acm_host_dev_event_data_t *event,
    void *user_ctx)
{
    (void)user_ctx;

    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "USB error %d", (int)event->data.error);
        break;

    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "Laser unplugged");

        USB_ESTABLISHED.store(false);
        usb_state_set(false);

        g_air_off_pending = false;
        outputs_off();

        // Wake the USB manager so it can perform safety handling.
        if (g_disconnect_sem) {
            xSemaphoreGive(g_disconnect_sem);
        }
        break;

    default:
        break;
    }
}


static bool usb_rx_cb(
    const uint8_t *data,
    size_t len,
    void *user_arg)
{
    (void)user_arg;

    scan_for_idle(data, len);

    xStreamBufferSend(g_usb_rx, data, len, 0);

    return true;
}


// ============================================================================
// USB HOST TASKS
// ============================================================================

static void usb_lib_task(void *arg)
{
    (void)arg;

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
    (void)arg;

    cdc_acm_host_device_config_t cfg = {};

    cfg.connection_timeout_ms = 2000;
    cfg.out_buffer_size = 512;
    cfg.event_cb = usb_event_cb;
    cfg.data_cb = usb_rx_cb;
    cfg.user_arg = nullptr;

    while (true) {
        xSemaphoreTake(g_disconnect_sem, 0);

        ESP_LOGI(TAG, "Waiting for laser on USB...");

        std::unique_ptr<CdcAcmDevice> dev(VCP::open(&cfg));

        if (!dev) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        cdc_acm_line_coding_t lc = {};

        lc.dwDTERate = GRBL_BAUD;
        lc.bCharFormat = 0;
        lc.bParityType = 0;
        lc.bDataBits = 8;

        dev->line_coding_set(&lc);

        // Keep DTR/RTS low.
        dev->set_control_line_state(false, false);

        xSemaphoreTake(g_vcp_lock, portMAX_DELAY);
        g_vcp = std::move(dev);
        xSemaphoreGive(g_vcp_lock);

        USB_ESTABLISHED.store(true);
        usb_state_set(true);

        ESP_LOGI(TAG, "Laser connected at %d baud", GRBL_BAUD);

        // Wait for disconnect.
        xSemaphoreTake(g_disconnect_sem, portMAX_DELAY);

        // Perform safety handling while cleaning up the disconnected device.
        Emergency_usb_lost();

        xSemaphoreTake(g_vcp_lock, portMAX_DELAY);
        g_vcp.reset();
        xSemaphoreGive(g_vcp_lock);
    }
}


// ============================================================================
// TCP TASKS
// ============================================================================

// Laser -> TCP client
static void tcp_sender_task(void *arg)
{
    (void)arg;

    uint8_t buf[256];

    while (true) {
        size_t n = xStreamBufferReceive(
            g_usb_rx,
            buf,
            sizeof(buf),
            pdMS_TO_TICKS(100)
        );

        if (n == 0) {
            continue;
        }

        int fd = g_client;

        if (fd < 0) {
            continue;
        }

        size_t off = 0;

        while (off < n) {
            int r = send(fd, buf + off, n - off, 0);

            if (r <= 0) {
                break;
            }

            off += r;
        }
    }
}


// TCP server: client -> laser
static void tcp_server_task(void *arg)
{
    (void)arg;

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if (listen_fd < 0) {
        ESP_LOGE(TAG, "Socket creation failed");
        vTaskDelete(nullptr);
        return;
    }

    int yes = 1;

    setsockopt(
        listen_fd,
        SOL_SOCKET,
        SO_REUSEADDR,
        &yes,
        sizeof(yes)
    );

    sockaddr_in addr = {};

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(TCP_PORT);

    if (bind(listen_fd, (sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "bind/listen failed");
        close(listen_fd);
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "TCP server on port %d", TCP_PORT);

    while (true) {
        int fd = accept(listen_fd, nullptr, nullptr);

        if (fd < 0) {
            continue;
        }

        int nodelay = 1;

        setsockopt(
            fd,
            IPPROTO_TCP,
            TCP_NODELAY,
            &nodelay,
            sizeof(nodelay)
        );

        int keepalive = 1;
        int idle = 5;
        int intvl = 2;
        int cnt = 3;

        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));

        g_client = fd;
        gcode_reset();

        ESP_LOGI(TAG, "TCP client connected");

        uint8_t buf[256];
        int n;

        while ((n = recv(fd, buf, sizeof(buf), 0)) > 0) {
            gcode_feed(buf, (size_t)n);
        }

        g_client = -1;

        g_air_off_pending = false;

        Emergency_tcp_connection_lost();

        close(fd);

        ESP_LOGI(TAG, "TCP client disconnected");
    }
}


// ============================================================================
// BUTTON HANDLING
// ============================================================================

struct Button {
    gpio_num_t pin;
    bool last_read;
    bool stable_state;
    TickType_t last_change;
};


static void button_init(Button *b, gpio_num_t pin)
{
    b->pin = pin;

    bool initial = gpio_get_level(pin);

    b->last_read = initial;
    b->stable_state = initial;
    b->last_change = xTaskGetTickCount();
}


static bool button_pressed_event(Button *b)
{
    bool reading = gpio_get_level(b->pin);
    TickType_t now = xTaskGetTickCount();

    if (reading != b->last_read) {
        b->last_read = reading;
        b->last_change = now;
    }

    if ((now - b->last_change) >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
        if (reading != b->stable_state) {
            b->stable_state = reading;

            if (reading == BUTTON_PRESSED_LEVEL) {
                return true;
            }
        }
    }

    return false;
}


static void button_task(void *arg)
{
    (void)arg;

    Button air_button;
    Button power_button;
    Button laser_button;

    button_init(&air_button, (gpio_num_t)AIR_BUTTON_GPIO);
    button_init(&power_button, (gpio_num_t)PWR_BUTTON_GPIO);
    button_init(&laser_button, (gpio_num_t)LSR_BUTTON_GPIO);

    while (true) {
        if (button_pressed_event(&air_button)) {
            ESP_LOGI(TAG, "Manual air-assist button");
            Cycle_AIR_ASSIST();
        }

        if (button_pressed_event(&power_button)) {
            ESP_LOGI(TAG, "Manual power-board button");
            Cycle_PWR_STATE();
        }

        if (button_pressed_event(&laser_button)) {
            ESP_LOGI(TAG, "Manual laser OFF button");
            laser_state_off();
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}


// ============================================================================
// OUTPUT INITIALIZATION
// ============================================================================

static void gpio_init_all(void)
{
    // Digital control outputs and indicator LEDs.
    const uint64_t output_mask =
        (1ULL << AIR_GPIO) |
        (1ULL << PWR_CTRL_GPIO) |
        (1ULL << AIR_LED_GPIO) |
        (1ULL << PWR_LED_GPIO) |
        (1ULL << LSR_LED_GPIO) |
        (1ULL << FIRST_BOOT_GPIO) |
        (1ULL << USB_BOOT_GPIO);

    gpio_config_t out = {};

    out.pin_bit_mask = output_mask;
    out.mode = GPIO_MODE_OUTPUT;
    out.pull_up_en = GPIO_PULLUP_DISABLE;
    out.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&out));

    // Manual button inputs, active LOW, with internal pull-ups.
    const uint64_t input_mask =
        (1ULL << AIR_BUTTON_GPIO) |
        (1ULL << PWR_BUTTON_GPIO) |
        (1ULL << LSR_BUTTON_GPIO);

    gpio_config_t in = {};

    in.pin_bit_mask = input_mask;
    in.mode = GPIO_MODE_INPUT;
    in.pull_up_en = GPIO_PULLUP_ENABLE;
    in.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&in));

    // Start in safe output states.
    air_assist_set(false);
    power_state_set(false);

    AIR_STATE.store(false);
    PWR_STATE.store(false);
    LSR_STATE.store(false);
    FIRST_BOOT.store(false);
    USB_ESTABLISHED.store(false);

    air_led_set(false);
    power_led_set(false);
    laser_led_set(false);
    first_boot_set(false);
    usb_state_set(false);

    pulse_init();
    pulse_stop();
}


static void outputs_off(void)
{
    air_assist_off();
}


// ============================================================================
// WIFI ACCESS POINT
// ============================================================================

static void wifi_start_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wc = {};

    strncpy(
        (char *)wc.ap.ssid,
        AP_SSID,
        sizeof(wc.ap.ssid)
    );

    strncpy(
        (char *)wc.ap.password,
        AP_PASS,
        sizeof(wc.ap.password)
    );

    wc.ap.ssid_len = strlen(AP_SSID);
    wc.ap.channel = AP_CHANNEL;
    wc.ap.max_connection = 2;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "AP '%s' up, address 192.168.4.1", AP_SSID);
}


// ============================================================================
// ENTRY POINT
// ============================================================================

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);

    // Configure GPIO before starting tasks.
    gpio_init_all();

    g_vcp_lock = xSemaphoreCreateMutex();
    g_disconnect_sem = xSemaphoreCreateBinary();
    g_usb_rx = xStreamBufferCreate(4096, 1);

    if (!g_vcp_lock || !g_disconnect_sem || !g_usb_rx) {
        ESP_LOGE(TAG, "Failed to allocate FreeRTOS resources");
        abort();
    }

    wifi_start_ap();

    // USB host setup.
    usb_host_config_t host_cfg = {};

    host_cfg.skip_phy_setup = false;
    host_cfg.intr_flags = ESP_INTR_FLAG_LEVEL1;

    ESP_ERROR_CHECK(usb_host_install(&host_cfg));

    xTaskCreate(
        usb_lib_task,
        "usb_lib",
        4096,
        nullptr,
        5,
        nullptr
    );

    ESP_ERROR_CHECK(cdc_acm_host_install(nullptr));

    VCP::register_driver<CH34x>();

    // Start application tasks.
    xTaskCreate(
        usb_manager_task,
        "usb_mgr",
        6144,
        nullptr,
        4,
        nullptr
    );

    xTaskCreate(
        tcp_sender_task,
        "tcp_tx",
        4096,
        nullptr,
        4,
        nullptr
    );

    xTaskCreate(
        tcp_server_task,
        "tcp_srv",
        4096,
        nullptr,
        4,
        nullptr
    );

    xTaskCreate(
        button_task,
        "buttons",
        4096,
        nullptr,
        4,
        nullptr
    );
}