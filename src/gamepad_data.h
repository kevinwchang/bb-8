struct gamepad_data
{
  bool connected;
  absolute_time_t last_update;
  uint16_t buttons;
};
extern struct gamepad_data gamepad;