// Example file - Public Domain
// Need help? http://bit.ly/bluepad32-help

#include <btstack_run_loop.h>
#include <hardware/structs/ioqspi.h>
#include <hardware/sync.h>
#include <hardware/uart.h>
#include <pico_servo.h>
#include <pico/cyw43_arch.h>
#include <pico/rand.h>
#include <pico/stdlib.h>
#include <uni.h>

#include "gamepad_data.h"
#include "sdkconfig.h"

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

// Defined in my_platform.c
struct uni_platform* get_my_platform(void);

struct gamepad_data gamepad = { .connected = false, .ready = false, .new_data = false, .joycon_side = ' ' };

typedef enum
{
  ServoHead = 8,
  ServoDoor = 9,
  ServoExtend = 10,
  ServoLighter = 11,
  LedFlame = 13
} Pins;

#define SERVO_DOOR_CLOSED 1000
#define SERVO_DOOR_OPEN 1800
#define SERVO_EXTEND_IN 1900
#define SERVO_EXTEND_OUT 1080
#define SERVO_LIGHTER_DOWN 2000
#define SERVO_LIGHTER_UP 940

void led(bool on)
{
  static bool last = 0;

  // don't set the LED again if it's already in the specified state - hoping
  // this avoids unnecessary communication with CYW43
  if (on == last) { return; }

  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
  last = on;
}

// Temporarily changes the QSPI_SS_N/BOOTSEL to be an input to read the BOOTSEL
// button. While we are reading the button, we cannot execute any code from
// flash, so that is why we this function disables interrupts and writes
// directly to hardware registers (or calls inline functions).
bool __no_inline_not_in_flash_func(bootsel_is_pressed)()
{
  uint32_t flags = save_and_disable_interrupts();

  hw_write_masked(&ioqspi_hw->io[1].ctrl,
                  GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                  IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

  // Delay for 1 to 2 us.
  uint32_t start = timer_hw->timerawl;
  while ((uint32_t)(timer_hw->timerawl - start) < 2);

  bool r = !(sio_hw->gpio_hi_in & (1 << 1));

  hw_clear_bits(&ioqspi_hw->io[1].ctrl, IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

  restore_interrupts(flags);

  return r;
}

void sound_setup()
{
  uart_init(uart0, 9600);
  gpio_set_function(28, GPIO_FUNC_UART);
  uart_write_blocking(uart0, (uint8_t[]){0x7E, 0x02, 0x0C, 0xEF}, 4); // reset
  sleep_ms(500);
}

void play_sound(uint16_t index)
{
  static uint8_t buf[6] = {0x7E, 0x04, 0x03, 0x00, 0x01, 0xEF};
  buf[3] = (index >> 8) & 0xFF;
  buf[4] = index & 0xFF;
  uart_write_blocking(uart0, buf, 6);
}

void play_random_sound()
{
  const uint8_t count = 9; // number of random sounds; index 2 through [count + 1]
  static uint8_t prev_sound = 255;

  uint8_t sound = rand() / (RAND_MAX / (count - 1) + 1); // random number between 0 and [count - 2]
  if (sound >= prev_sound) { sound++; } // shift results >= prev so we don't play the same sound twice
  prev_sound = sound;
  play_sound(2 + sound);
}

typedef enum
{
  ArmStowedIdle,
  ArmStowed,
  ArmDoorOpen,
  ArmExtended,
  ArmThumbUp,
  ArmThumbUpIdle
} ArmState;

ArmState arm_state = ArmThumbUp; // so that we can run the retract sequence on startup
alarm_id_t active_alarm = 0;

int64_t arm_extend_cb(__unused alarm_id_t id, __unused void * user_data)
{
  //printf("arm_extend_cb state=%u\n", arm_state);

  switch (arm_state)
  {
    case ArmStowedIdle:
    case ArmStowed:
      servo_microseconds(ServoDoor, SERVO_DOOR_OPEN);
      arm_state = ArmDoorOpen;
      return 300 * 1000;

    case ArmDoorOpen:
      play_sound(1);
      servo_microseconds(ServoExtend, SERVO_EXTEND_OUT);
      arm_state = ArmExtended;
      return 400 * 1000;

    case ArmExtended:
      servo_microseconds(ServoLighter, SERVO_LIGHTER_UP);
      gpio_put(LedFlame, 1);
      arm_state = ArmThumbUp;
      return 300 * 1000;

    case ArmThumbUp:
      servo_microseconds(ServoLighter, 0);
      arm_state = ArmThumbUpIdle;
      return 0;

    default:
      return 0;
  }
}

void arm_extend()
{
  cancel_alarm(active_alarm);
  active_alarm = add_alarm_in_us(0, arm_extend_cb, NULL, true);
}

int64_t arm_retract_cb(__unused alarm_id_t id, __unused void * user_data)
{
  //printf("arm_retract_cb state=%u\n", arm_state);

  switch (arm_state)
  {
    case ArmThumbUpIdle:
    case ArmThumbUp:
      servo_microseconds(ServoLighter, SERVO_LIGHTER_DOWN);
      gpio_put(LedFlame, 0);
      arm_state = ArmExtended;
      return 200 * 1000;

    case ArmExtended:
      servo_microseconds(ServoExtend, SERVO_EXTEND_IN);
      arm_state = ArmDoorOpen;
      return 300 * 1000;

    case ArmDoorOpen:
      servo_microseconds(ServoDoor, SERVO_DOOR_CLOSED);
      arm_state = ArmStowed;
      return 300 * 1000;

    case ArmStowed:
      servo_microseconds(ServoDoor, 0);
      servo_microseconds(ServoLighter, 0);
      arm_state = ArmStowedIdle;
      return 0;

    default:
      return 0;
  }
}

void arm_retract()
{
  cancel_alarm(active_alarm);
  active_alarm = add_alarm_in_us(0, arm_retract_cb, NULL, true);
}

void servo_setup()
{
  servo_init();
  servo_clock_auto();

  servo_attach(ServoHead);
  servo_attach(ServoDoor);
  servo_attach(ServoExtend);
  servo_attach(ServoLighter);

  // yeah yeah I know this isn't a servo
  gpio_init(LedFlame);
  gpio_set_dir(LedFlame, GPIO_OUT);

  arm_retract();
}

int gamepad_setup()
{
  // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
  if (cyw43_arch_init())
  {
    loge("failed to initialise cyw43_arch\n");
    return -1;
  }

  led(1);

  // Must be called before uni_init()
  uni_platform_set_custom(get_my_platform());

  // Initialize BP32
  uni_init(0, NULL);

  led(0);

  return 0;
}

void process_buttons()
{
  static uint16_t prev_buttons = 0;

  int arm_button, sound_button;

  switch (gamepad.joycon_side)
  {
    case 'L':
      arm_button = BUTTON_X;
      sound_button = BUTTON_B;
      break;

    case 'R':
      arm_button = BUTTON_B;
      sound_button = BUTTON_X;
      break;

    default:
      arm_button = BUTTON_Y;
      sound_button = BUTTON_A;
  }

  if ((gamepad.buttons & arm_button) && !(prev_buttons & arm_button))
  {
    //printf("^ pressed state=%u\n", arm_state);

    switch (arm_state)
    {
      case ArmStowedIdle:
      //case ArmStowed:
        arm_extend();
        break;

      case ArmThumbUpIdle:
      //case ArmThumbUp:
        arm_retract();
        break;

      default:
        // arm is moving - don't interrupt it
    }
  }
  if ((gamepad.buttons & sound_button) && !(prev_buttons & sound_button))
  {
    play_random_sound();
  }
  prev_buttons = gamepad.buttons;
}

void update_head()
{
  int32_t axis;

  switch (gamepad.joycon_side)
  {
    case 'L':
      axis = gamepad.axis_y;
      break;

    case 'R':
      axis = -gamepad.axis_y;
      break;

    default:
      axis = -gamepad.axis_x;
  }

  const int32_t deadzone = 100;
  if (axis > deadzone)
  {
    axis -= deadzone;
  }
  else if (axis < -deadzone)
  {
    axis += deadzone;
  }
  else
  {
    axis = 0;
  }

  servo_microseconds(ServoHead, 1500 + axis);
}

void loop()
{
  static bool pairing = false;

  if (!pairing && bootsel_is_pressed())
  {
    pairing = true;
    uni_bt_disconnect_device_safe(0);
    uni_bt_del_keys_safe();
    uni_bt_enable_new_connections_safe(true);
  }

  if (gamepad.connected)
  {
    pairing = false;

    // ~1 Hz short blink
    led(((to_ms_since_boot(get_absolute_time()) >> 4) & 0x3F) == 0x3F);

    if (absolute_time_diff_us(gamepad.last_update, get_absolute_time()) > 5000000)
    {
      // disconnect if we haven't heard from the gamepad in 5s. It seems to have
      // trouble reconnecting after that though...
      printf("timeout disconnect\n");
      uni_bt_disconnect_device_safe(0);

      servo_microseconds(ServoHead, 0);
    }
    else if (gamepad.ready)
    {
      if (gamepad.new_data)
      {
        process_buttons();
        update_head();
        gamepad.new_data = false;
      }
    }
  }
  else if (pairing)
  {
    // ~4 Hz fast blink
    led((to_ms_since_boot(get_absolute_time()) >> 6) & 1);
  }
  else
  {
    // ~0.5 Hz slow blink
    led((to_ms_since_boot(get_absolute_time()) >> 10) & 1);
  }
}

int main()
{
  srand(get_rand_32());

  stdio_usb_init();
  sound_setup();
  servo_setup();
  gamepad_setup();

  while (true)
  {
    loop();
  }

  return 0;
}