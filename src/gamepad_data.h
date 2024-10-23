struct gamepad_data
{
  bool connected;
  absolute_time_t last_update;
  bool new_data;
  int32_t axis_x;
  uint16_t buttons;
};
extern struct gamepad_data gamepad;