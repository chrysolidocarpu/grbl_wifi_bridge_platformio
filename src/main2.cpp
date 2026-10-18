// GRBL WiFi bridge for ESP32-S3
// Laptop -> WiFi AP -> raw TCP -> ESP32-S3 USB host -> CH340 laser
//
// Behaviour summary
// -----------------
// Boot:
//   The ESP and the power board are fed by the same PSU, so the power board
//   is always OFF when the ESP boots. After a short delay the ESP presses the
//   power-board button ONCE and tracks the board as ON.
//
// USB link, three states:
//   1. NO DEVICE   : nothing detected.            standby LED off, USB LED off
//   2. STANDBY     : CH340 enumerated, but the S1 does not talk yet (it only
//                    answers after the physical button on the laser is
//                    pressed). The ESP polls "$I" until GRBL answers.
//                                                  standby LED blinks, USB LED off
//   3. ARMED       : first valid GRBL reply received (swallowed, not forwarded).
//                    From now on losing USB is an EMERGENCY.
//                                                  standby LED off, USB LED on
//   There is NO software disarm. Only the physical button on the laser arms it.
//
// Shared safe_state(send_stop, cut_power):
//   always : forget half-built G-code line, laser state/LED off, air off,
//            air PWM back to 0
//   send_stop : hard stop to GRBL: '!' then 0x18 (real-time bytes, not queued)
//   cut_power : press the power-board button (only if tracked state is ON)
//
//   Event                      send_stop   cut_power
//   Laser button               yes         no
//   TCP client lost            yes         no
//   Client sends 0x18          (already)   no
//   USB lost while ARMED       no (no link) YES
//   USB lost while not armed   no          no
//   Power button (hold 3 s)    -           board ON (only if tracked OFF)
//   Power button (short press) -           board OFF (only if tracked ON)
//
// Manual controls:
//   Power button: hold POWER_HOLD_ON_MS to switch the board ON (LED blinks
//                 while holding, solid when reached); a short press switches
//                 it OFF at once. Each uses a 200 ms power-board button pulse.
//   Laser button: hard stop ('!' + 0x18).
//   Air button  : toggle air assist between 40% PWM and OFF.
//
// Air assist:
//   GRBL M7: air assist ramp to configured target (S word, if present).
//   GRBL M8: air assist ramp to 100%.
//   GRBL M9: handled immediately by the ESP: fade down to AIR_MIN_DUTY_PCT
//            over AIR_RAMP_DOWN_MS, then switch the air assist OFF. From a
//            level at or below the minimum it fades to 0 on the same slope.

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
#include "freertos/timers.h"

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

// ---- GPIO list ------------------------------------------------------------
#define AIR_GPIO                  4 // air assist digital enable output
#define AIR_PULSED_GPIO           5 // pulsed PWM output

#define PWR_CTRL_GPIO             6 // power-board button interface
#define AIR_BUTTON_GPIO           10 // manual air-assist button input
#define PWR_BUTTON_GPIO           13 // manual power button input
#define LSR_BUTTON_GPIO           15 // manual laser button input

#define LSR_LED_GPIO              7 // laser status LED
#define USB_BOOT_GPIO             9  // USB LED: ON only when the link is ARMED (valid GRBL reply received)
#define AIR_LED_GPIO              11 // air-assist status LED
#define PWR_LED_GPIO              12 // power-board status LED
#define STANDBY_LED_GPIO          18 // Standby LED: blinks while the USB device is present but not yet armed

// ---- Settings --------------------------------------------------------------
#define AIR_MIN_DUTY_PCT          40 // M7 without S: duty reached at the end of the ramp
#define PULSE_RAMP_MS             5000  // M7 ramp-up time
#define AIR_RAMP_DOWN_MS          5000  // M9 ramp-down time 
                                        // Time of the ramp-down from the current duty to AIR_MIN_DUTY_PCT.
                                        // The air assist is switched off when this time has elapsed.



// ============================================================================
// No changes beyond this point without careful consideration of the consequences.
// ============================================================================

// ---- Air assist ------------------------------------------------------------

#define AIR_ACTIVE_HIGH           1     // 1: GPIO high = air on
#define AIR_PULSED_ACTIVE_HIGH    1     // 1: PWM high = MOSFET on  
#define M_CODE_AIRASSIST          7     // M7: Air Assist
#define PULSE_FREQ_HZ             20000 // PWM frequency
#define PULSE_RES_BITS            10    // PWM resolution
#define PULSE_S_MAX               1000  // M7 Pn: n / PULSE_S_MAX = duty fraction

// ---- Power board -----------------------------------------------------------

#define PWR_CTRL_ACTIVE_HIGH      1 // GPIO high = button pressed, Control output connected to the power-board button interface
#define PWR_LED_ACTIVE_HIGH       1 // GPIO high = LED on, Power-board status LED

// Delay between ESP boot and the automatic power-on press, so rails and
// GPIOs have settled and the press cannot be a start-up glitch.
#define POWER_ON_DELAY_MS         1500
#define POWER_PRESS_MS            200 // Length of the simulated button press

// Manual power button: hold this long to switch the board ON.
// A short press switches it OFF immediately.
#define POWER_HOLD_ON_MS          3000  // How long the power button must be held to switch the board ON
#define POWER_HOLD_BLINK_MS       150   // Power LED blink period while the button is being held

// ---- Laser -----------------------------------------------------------------

// Laser is controlled through GRBL, not a physical enable output
#define LSR_LED_ACTIVE_HIGH       1

// ---- USB link indicators ---------------------------------------------------

// USB LED: ON only when the link is ARMED (valid GRBL reply received).
// (Pin name kept from the old code.)
#define USB_STATE_ACTIVE_HIGH     1

// Standby LED: blinks while the USB device is present but not yet armed
#define STANDBY_LED_ACTIVE_HIGH   1


// ---- USB arming ------------------------------------------------------------

// Poll "$I" this often while in STANDBY
#define ARM_POLL_MS               1000

// Standby LED toggles this often (full blink period = 2x)
#define STANDBY_BLINK_MS          500

// Granularity of the USB manager loop
#define MGR_TICK_MS               250


// ---- Hard stop -------------------------------------------------------------

// How long the stop bytes wait for the USB write lock before sending anyway
#define STOP_LOCK_TIMEOUT_MS      50

// USB write timeout for the stop bytes
#define STOP_TX_TIMEOUT_MS        100


// ---- Air-assist manual button and LED --------------------------------------
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
static SemaphoreHandle_t g_pwr_lock;
static SemaphoreHandle_t g_disconnect_sem;
static StreamBufferHandle_t g_usb_rx;

static volatile int g_client = -1;

// Tick of the most recent commanded power-board OFF. Used so that a late
// GRBL reply during shutdown cannot flip the tracked state back to ON.
static std::atomic<TickType_t> g_last_off_tick{0};

// One-shot timer: switches the air assist off at the end of an M9 ramp-down.
static TimerHandle_t g_air_down_timer = nullptr;

// Asks the TCP task to throw away its half-built G-code line.
// (The line buffer is owned by the TCP task; other tasks only raise this flag.)
static std::atomic<bool> g_line_flush{false};

// Mutable software-tracked states, NOT #define constants.
static std::atomic<bool> PWR_STATE{false};    // power board tracked ON/OFF
static std::atomic<bool> LSR_STATE{false};    // laser commanded ON/OFF
static std::atomic<bool> AIR_STATE{false};    // air assist ON/OFF
static std::atomic<bool> USB_PRESENT{false};  // CH340 enumerated
static std::atomic<bool> USB_ARMED{false};    // GRBL answered: safety is live


// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static void pulse_init(void);
static void pulse_start(uint32_t pct);
static void pulse_stop(void);

static void forward_to_laser(const uint8_t *d, size_t n);
static void safe_state(bool send_stop, bool cut_power, const char *why);


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
    set_indicator((gpio_num_t)AIR_GPIO, on, AIR_ACTIVE_HIGH);
}


// Air status LED
static inline void air_led_set(bool on)
{
    set_indicator((gpio_num_t)AIR_LED_GPIO, on, AIR_LED_ACTIVE_HIGH);
}


// Power-board button interface
static inline void power_state_set(bool on)
{
    set_indicator((gpio_num_t)PWR_CTRL_GPIO, on, PWR_CTRL_ACTIVE_HIGH);
}


// Power status LED
static inline void power_led_set(bool on)
{
    set_indicator((gpio_num_t)PWR_LED_GPIO, on, PWR_LED_ACTIVE_HIGH);
}


// Laser status LED
static inline void laser_led_set(bool on)
{
    set_indicator((gpio_num_t)LSR_LED_GPIO, on, LSR_LED_ACTIVE_HIGH);
}


// USB LED: lit only while ARMED
static inline void usb_state_set(bool on)
{
    set_indicator((gpio_num_t)USB_BOOT_GPIO, on, USB_STATE_ACTIVE_HIGH);
}


// Standby LED: blinks while waiting for the laser to answer
static inline void standby_led_set(bool on)
{
    set_indicator((gpio_num_t)STANDBY_LED_GPIO, on, STANDBY_LED_ACTIVE_HIGH);
}


// ============================================================================
// POWER BOARD
// ============================================================================

// Simulate a momentary button press.
// The output is released after POWER_PRESS_MS.
static void power_button_press(void)
{
    power_state_set(true);

    vTaskDelay(pdMS_TO_TICKS(POWER_PRESS_MS));

    power_state_set(false);
}


// Record the new tracked state FIRST, then press the button.
// Setting the state before the press means a concurrent USB-loss handler
// already sees the final state and cannot press a second time.
// Caller must hold g_pwr_lock.
static void power_press_locked(bool next_state)
{
    PWR_STATE.store(next_state);
    power_led_set(next_state);

    if (!next_state) {
        g_last_off_tick.store(xTaskGetTickCount());
    }

    power_button_press();

    ESP_LOGI(
        TAG,
        "Power-board state commanded: %s",
        next_state ? "ON" : "OFF"
    );
}


// Press only if the board is tracked OFF.
static void power_on_if_off(void)
{
    xSemaphoreTake(g_pwr_lock, portMAX_DELAY);

    if (!PWR_STATE.load()) {
        power_press_locked(true);
    }

    xSemaphoreGive(g_pwr_lock);
}


// Press only if the board is tracked ON.
static void power_off_if_on(void)
{
    xSemaphoreTake(g_pwr_lock, portMAX_DELAY);

    if (PWR_STATE.load()) {
        power_press_locked(false);
    }

    xSemaphoreGive(g_pwr_lock);
}


// One-shot task: automatic power-on after boot.
static void boot_power_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(POWER_ON_DELAY_MS));

    ESP_LOGI(TAG, "Boot: powering the power board on");
    power_on_if_off();

    vTaskDelete(nullptr);
}


// ============================================================================
// PWM
// ============================================================================

static const uint32_t PULSE_MAX_DUTY =
    (1u << PULSE_RES_BITS) - 1;

// True while the PWM channel is running (not held at its idle level by
// ledc_stop). Used so every ramp starts from 0% after a stop.
static bool s_pwm_running = false;


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

    s_pwm_running = true;
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

    // After a stop the channel is parked at its idle level. Restart it at an
    // explicit 0% so the ramp always begins from zero.
    if (!s_pwm_running) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pulse_duty_raw(0));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        s_pwm_running = true;
    }

    pulse_fade_to(pct, PULSE_RAMP_MS);
}


static void pulse_stop(void)
{
    // Abort any running fade, then park the pin at the inactive level.
    ledc_fade_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    esp_err_t err = ledc_stop(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_0,
        AIR_PULSED_ACTIVE_HIGH ? 0 : 1
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PWM stop returned: %s", esp_err_to_name(err));
    }

    s_pwm_running = false;
}


// ============================================================================
// AIR ASSIST
// ============================================================================

static void air_assist_on(uint32_t pct)
{
    if (pct > 100) {
        pct = 100;
    }

    // A new "on" request cancels a pending M9 shutdown.
    if (g_air_down_timer) {
        xTimerStop(g_air_down_timer, 0);
    }

    // Enable the air-assist hardware.
    air_assist_set(true);

    // Start the PWM ramp.
    pulse_start(pct);

    AIR_STATE.store(true);
    air_led_set(true);

    ESP_LOGI(TAG, "Air assist ON, target %lu%%", (unsigned long)pct);
}


static void air_assist_off(void)
{
    // Cancel any pending M9 shutdown (this is the shutdown).
    if (g_air_down_timer) {
        xTimerStop(g_air_down_timer, 0);
    }

    // Disable both the digital enable and PWM output.
    air_assist_set(false);
    pulse_stop();

    AIR_STATE.store(false);
    air_led_set(false);

    ESP_LOGI(TAG, "Air assist OFF");
}


// Current PWM output as a percentage (0..100), read back from the hardware
// so it is correct in the middle of a fade as well.
static uint32_t pulse_current_pct(void)
{
    uint32_t raw = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    if (raw > PULSE_MAX_DUTY) {
        raw = PULSE_MAX_DUTY;
    }

    uint32_t active = AIR_PULSED_ACTIVE_HIGH ? raw : PULSE_MAX_DUTY - raw;

    return active * 100 / PULSE_MAX_DUTY;
}


// Timer callback: the M9 ramp-down time has elapsed, switch the air off.
static void air_down_timer_cb(TimerHandle_t t)
{
    (void)t;

    ESP_LOGI(TAG, "M9 ramp-down finished: air assist OFF");
    air_assist_off();
}


// GRBL M9: fade the air assist out, then switch it off. Always a fade.
//   - air off                          : nothing to do
//   - above AIR_MIN_DUTY_PCT           : fade down to the minimum over
//                                        AIR_RAMP_DOWN_MS, then off
//   - at/below the minimum (but > 0)   : fade down to 0, same slope as the
//                                        ramp above (time scales with the
//                                        current level), then off
//   - already 0%                       : off now
// The shutdown happens when the fade time has elapsed (one-shot timer).
// M7/M8/off/safe_state cancel or replace a pending shutdown.
static void air_assist_ramp_down(void)
{
    if (!AIR_STATE.load()) {
        return;
    }

    uint32_t cur = pulse_current_pct();

    // Debug: read-back of the PWM at the moment of M9 (should be a plausible
    // 0..100 value, also in the middle of a ramp).
    ESP_LOGI(TAG, "M9: PWM currently at %lu%%", (unsigned long)cur);

    if (cur == 0 || !g_air_down_timer) {
        air_assist_off();
        return;
    }

    uint32_t target_pct;
    uint32_t fade_ms;

    if (cur > AIR_MIN_DUTY_PCT) {
        // Above the minimum: down to the minimum, then off.
        target_pct = AIR_MIN_DUTY_PCT;
        fade_ms = AIR_RAMP_DOWN_MS;
    } else {
        // At or below the minimum: fade to zero with the same slope as the
        // ramp above (full AIR_RAMP_DOWN_MS corresponds to going from 100%
        // to AIR_MIN_DUTY_PCT; here the time scales with the current level).
        target_pct = 0;
        fade_ms = (uint32_t)((uint64_t)AIR_RAMP_DOWN_MS * cur /
                             (AIR_MIN_DUTY_PCT ? AIR_MIN_DUTY_PCT : 1));
    }

    if (fade_ms < 100) {
        fade_ms = 100;
    }

    pulse_fade_to(target_pct, fade_ms);

    // Shut the air off when the fade is done. Changing the period also
    // (re)starts the timer.
    xTimerChangePeriod(g_air_down_timer, pdMS_TO_TICKS(fade_ms), 0);

    ESP_LOGI(TAG, "GRBL M9: fading to %lu%% over %lu ms, then off",
             (unsigned long)target_pct, (unsigned long)fade_ms);
}


// Manual button: toggle between 40% PWM and OFF.
static void Cycle_AIR_ASSIST(void)
{
    if (AIR_STATE.load()) {
        air_assist_off();
    } else {
        air_assist_on(AIR_MIN_DUTY_PCT);
    }
}


// ============================================================================
// SAFETY: HARD STOP AND SHARED SAFE STATE
// ============================================================================

// Send the GRBL hard stop: '!' (feed hold) immediately followed by 0x18
// (soft reset). Both are real-time bytes: GRBL acts on them at once and they
// bypass its line buffer, unlike a queued M5.
//
// Does nothing when no USB device is present (nothing to send to).
//
// The USB write lock is tried for STOP_LOCK_TIMEOUT_MS only. If another task
// is stuck in a write, the stop is sent anyway: a late stop is worse than a
// garbled stream. (g_vcp is only destroyed by the USB manager AFTER it has
// cleared USB_PRESENT, so skipping the lock is safe in practice.)
static void hard_stop_send(void)
{
    if (!USB_PRESENT.load()) {
        ESP_LOGW(TAG, "Hard stop skipped: no USB device");
        return;
    }

    static uint8_t seq[2] = { '!', 0x18 };

    bool locked = g_vcp_lock &&
        xSemaphoreTake(g_vcp_lock, pdMS_TO_TICKS(STOP_LOCK_TIMEOUT_MS)) == pdTRUE;

    if (!locked) {
        ESP_LOGE(TAG, "Hard stop: USB lock busy, sending without lock");
    }

    if (g_vcp) {
        esp_err_t err = g_vcp->tx_blocking(seq, sizeof(seq), STOP_TX_TIMEOUT_MS);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Hard stop write failed: %s", esp_err_to_name(err));
        }
    }

    if (locked) {
        xSemaphoreGive(g_vcp_lock);
    }

    ESP_LOGW(TAG, "Hard stop sent ('!' + 0x18)");
}


// The single place that puts the machine in its safe state.
//
//   send_stop : send '!' + 0x18 to GRBL
//   cut_power : press the power-board button (only if tracked ON)
//
// Always (regardless of the switches):
//   - discard the half-built G-code line
//   - laser tracked state and LED off
//   - air assist off, PWM back to 0
static void safe_state(bool send_stop, bool cut_power, const char *why)
{
    ESP_LOGW(TAG, "SAFE STATE: %s (stop=%d, power cut=%d)",
             why, (int)send_stop, (int)cut_power);

    // 1. The stop goes first: it is the part that matters most.
    if (send_stop) {
        hard_stop_send();
    }

    // 2. Drop any partially received G-code line.
    g_line_flush.store(true);

    // 3. Laser is off (commanded state, not confirmed output).
    LSR_STATE.store(false);
    laser_led_set(false);

    // 4. Air assist off, PWM back to zero.
    air_assist_off();

    // 5. Optional: cut the whole machine.
    if (cut_power) {
        power_off_if_on();
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

            uint32_t pct = AIR_MIN_DUTY_PCT; // default if no S word

            if (s >= 0) {
                if (s > PULSE_S_MAX) {
                    s = PULSE_S_MAX;
                }

                pct = (uint32_t)s * 100 / PULSE_S_MAX;
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
            // Handled immediately: ramp down to the minimum, then off.
            air_assist_ramp_down();
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

// The line buffer belongs to the TCP task. Other tasks never touch it
// directly; they raise g_line_flush instead.
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

        // A safety event elsewhere asked for the half-built line to be dropped.
        if (g_line_flush.exchange(false)) {
            gcode_reset();
        }

        if (c == 0x18) {
            // GRBL soft reset sent by the client: pass it on, then clean up.
            // The stop itself is not repeated (the client just sent it).
            forward_to_laser(&c, 1);
            gcode_reset();

            safe_state(false, false, "soft reset from client");
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
// USB ARMING
// ============================================================================

// Line accumulator for the replies received while NOT armed.
static char s_arm_line[96];
static size_t s_arm_len = 0;


static void arm_parser_reset(void)
{
    s_arm_len = 0;
}


// Does this line prove that GRBL is really talking to us?
// Accepts the answer to "$I" ([VER:..] / [OPT:..] / ok), the boot banner
// ("Grbl ...") and error replies (the controller is clearly alive).
static bool is_valid_grbl_line(const char *line)
{
    return strstr(line, "[VER:") ||
           strstr(line, "[OPT:") ||
           strstr(line, "Grbl")  ||
           strstr(line, "ok")    ||
           strstr(line, "error:");
}


// STANDBY -> ARMED. Called from the USB receive callback.
static void usb_arm_now(void)
{
    USB_ARMED.store(true);

    standby_led_set(false);
    usb_state_set(true);

    // The S1 is talking over USB, so the power board is certainly ON.
    // Correct the tracked state if it says OFF. Nothing is pressed.
    // Skipped while a power press is running or right after a commanded OFF,
    // so a late reply during shutdown cannot flip the state back to ON.
    if (!PWR_STATE.load() &&
        (xTaskGetTickCount() - g_last_off_tick.load()) >= pdMS_TO_TICKS(2000) &&
        xSemaphoreTake(g_pwr_lock, 0) == pdTRUE) {
        PWR_STATE.store(true);
        power_led_set(true);
        xSemaphoreGive(g_pwr_lock);

        ESP_LOGW(TAG, "Armed: power board tracked state corrected to ON");
    }

    ESP_LOGI(TAG, "USB link ARMED: loss of USB now triggers emergency shutdown");
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

        // All handling happens in the USB manager task (one place,
        // one time). It decides emergency or plain cleanup based on
        // whether the link was armed.
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

    if (!USB_ARMED.load()) {
        // STANDBY: nobody is listening yet. Look for the first valid GRBL
        // reply and swallow everything (nothing is forwarded to LightBurn).
        for (size_t i = 0; i < len; i++) {
            char c = (char)data[i];

            if (c == '\n' || c == '\r') {
                if (s_arm_len) {
                    s_arm_line[s_arm_len] = 0;
                    s_arm_len = 0;

                    if (is_valid_grbl_line(s_arm_line)) {
                        usb_arm_now();
                        return true;    // swallow the rest of this chunk too
                    }
                }
            } else if (s_arm_len < sizeof(s_arm_line) - 1) {
                s_arm_line[s_arm_len++] = c;
            } else {
                // Overlong garbage line: start over.
                s_arm_len = 0;
            }
        }

        return true;
    }

    // ARMED: normal operation, forward everything to the TCP side.
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

    const uint32_t blink_ticks = STANDBY_BLINK_MS / MGR_TICK_MS;
    const uint32_t poll_ticks  = ARM_POLL_MS / MGR_TICK_MS;

    static const uint8_t poll_cmd[] = { '$', 'I', '\n' };

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

        // ---- STANDBY: device present, not armed yet ----
        arm_parser_reset();
        USB_ARMED.store(false);
        USB_PRESENT.store(true);
        usb_state_set(false);

        ESP_LOGI(TAG, "Laser enumerated at %d baud: STANDBY, polling $I", GRBL_BAUD);

        uint32_t tick = 0;
        bool blink = true;

        standby_led_set(blink);
        forward_to_laser(poll_cmd, sizeof(poll_cmd));

        // Wait for disconnect. While not armed: blink and poll $I.
        // Once armed the polling stops and this just waits.
        while (true) {
            if (xSemaphoreTake(g_disconnect_sem,
                               pdMS_TO_TICKS(MGR_TICK_MS)) == pdTRUE) {
                break;
            }

            if (USB_ARMED.load()) {
                continue;
            }

            tick++;

            if (tick % blink_ticks == 0) {
                blink = !blink;
                standby_led_set(blink);
            }

            if (tick % poll_ticks == 0) {
                forward_to_laser(poll_cmd, sizeof(poll_cmd));
            }
        }

        // ---- Device gone ----
        bool was_armed = USB_ARMED.exchange(false);

        USB_PRESENT.store(false);
        standby_led_set(false);
        usb_state_set(false);

        if (was_armed) {
            // Armed: this is an emergency. No link to send a stop over,
            // so cut the whole machine (air off, laser state off, power off).
            ESP_LOGE(TAG, "USB lost while ARMED: emergency shutdown");
            safe_state(false, true, "USB lost while armed");
        } else {
            // Never armed: just a plain unplug, no power cut.
            safe_state(false, false, "USB removed (not armed)");
        }

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
        g_line_flush.store(false);

        ESP_LOGI(TAG, "TCP client connected");

        uint8_t buf[256];
        int n;

        while ((n = recv(fd, buf, sizeof(buf), 0)) > 0) {
            gcode_feed(buf, (size_t)n);
        }

        g_client = -1;

        // Client gone: hard stop the laser, air off. Power board stays on.
        safe_state(true, false, "TCP client lost");

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


// Power button: hold POWER_HOLD_ON_MS to switch ON, short press to switch OFF.
//   board tracked OFF : press starts the hold; LED blinks; at POWER_HOLD_ON_MS
//                       the board is switched on (LED goes solid). Releasing
//                       early cancels.
//   board tracked ON  : a press switches it OFF immediately (no release wait).
static void power_button_service(Button *b)
{
    static bool holding = false;
    static TickType_t hold_start = 0;
    static TickType_t last_blink = 0;
    static bool blink = false;

    TickType_t now = xTaskGetTickCount();

    if (button_pressed_event(b)) {
        if (PWR_STATE.load()) {
            ESP_LOGI(TAG, "Power button: short press, board OFF");
            power_off_if_on();
        } else {
            holding = true;
            hold_start = now;
            last_blink = now;
            blink = true;
            power_led_set(true);
            ESP_LOGI(TAG, "Power button: hold %d ms to switch ON", POWER_HOLD_ON_MS);
        }
    }

    if (!holding) {
        return;
    }

    bool still_down = (b->stable_state == (BUTTON_PRESSED_LEVEL != 0));

    if (!still_down) {
        // Released too early: cancel and show the real state again.
        holding = false;
        power_led_set(PWR_STATE.load());
        ESP_LOGI(TAG, "Power button released early, no action");
        return;
    }

    if ((now - hold_start) >= pdMS_TO_TICKS(POWER_HOLD_ON_MS)) {
        holding = false;
        ESP_LOGI(TAG, "Power button: hold complete, board ON");
        power_on_if_off();      // LED goes solid ON
        return;
    }

    if ((now - last_blink) >= pdMS_TO_TICKS(POWER_HOLD_BLINK_MS)) {
        last_blink = now;
        blink = !blink;
        power_led_set(blink);
    }
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

        power_button_service(&power_button);

        if (button_pressed_event(&laser_button)) {
            ESP_LOGW(TAG, "Manual laser button: hard stop");
            safe_state(true, false, "laser button");
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
        (1ULL << USB_BOOT_GPIO) |
        (1ULL << STANDBY_LED_GPIO);

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
    PWR_STATE.store(false);     // board and ESP start together: board is OFF
    LSR_STATE.store(false);
    USB_PRESENT.store(false);
    USB_ARMED.store(false);

    air_led_set(false);
    power_led_set(false);
    laser_led_set(false);
    usb_state_set(false);
    standby_led_set(false);

    pulse_init();
    pulse_stop();
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
    g_pwr_lock = xSemaphoreCreateMutex();
    g_disconnect_sem = xSemaphoreCreateBinary();
    g_usb_rx = xStreamBufferCreate(4096, 1);

    g_air_down_timer = xTimerCreate(
        "air_down",
        pdMS_TO_TICKS(AIR_RAMP_DOWN_MS),
        pdFALSE,                 // one-shot
        nullptr,
        air_down_timer_cb
    );

    if (!g_air_down_timer) {
        ESP_LOGE(TAG, "Failed to create air-down timer");
        abort();
    }

    if (!g_vcp_lock || !g_pwr_lock || !g_disconnect_sem || !g_usb_rx) {
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

    // One-shot: press the power-board button once, shortly after boot.
    xTaskCreate(
        boot_power_task,
        "boot_pwr",
        3072,
        nullptr,
        3,
        nullptr
    );
}
