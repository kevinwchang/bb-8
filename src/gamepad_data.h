struct gamepad_data
{
  bool connected;
  bool ready;
  absolute_time_t last_update;
  bool new_data;
  int32_t axis_x;
  int32_t axis_y;
  uint16_t buttons;
  char joycon_side;
};
extern struct gamepad_data gamepad;