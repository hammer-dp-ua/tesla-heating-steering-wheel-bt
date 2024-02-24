static const char START_STOP_BUTTON_COMMAND[] = {0xCD, 0x00};
static const char START_STOP_LONG_PRESS_BUTTON_COMMAND[] = {0x23, 0x02};
static const char VOLUME_PLUS_BUTTON_COMMAND[] = {0xE9, 0x00};
static const char VOLUME_MINUS_BUTTON_COMMAND[] = {0xEA, 0x00};
static const char VOLUME_PLUS_OR_MINUS_LONG_PRESS_BUTTON_COMMAND[] = {0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00};
static const char FORVARD_BUTTON_COMMAND[] = {0xB5, 0x00};
static const char FORVARD_LONG_PRESS_BUTTON_COMMAND[] = {0x40, 0x00, 0x00, 0x00};
static const char BACK_BUTTON_COMMAND[] = {0xB6, 0x00};
static const char BACK_LONG_PRESS_BUTTON_COMMAND[] = {0x30, 0x00, 0x00, 0x00};

typedef enum {
    START_STOP_BUTTON,
    START_STOP_LONG_PRESS_BUTTON,
    VOLUME_PLUS_BUTTON,
    VOLUME_MINUS_BUTTON,
    VOLUME_PLUS_OR_MINUS_LONG_PRESS_BUTTON,
    FORVARD_BUTTON,
    FORVARD_LONG_PRESS_BUTTON,
    BACK_BUTTON,
    BACK_LONG_PRESS_BUTTON
} bt_button_t;

struct bt_button_command_s {
    bt_button_t button;
    const char *data;
    unsigned char data_length;
};

typedef struct bt_button_command_s bt_button_command_t;

const unsigned char bt_button_commands_length = 9;
const bt_button_command_t bt_button_commands[] = {
    { .button = START_STOP_BUTTON, .data = START_STOP_BUTTON_COMMAND, .data_length = sizeof(START_STOP_BUTTON_COMMAND) / sizeof(START_STOP_BUTTON_COMMAND[0]) },
    { .button = START_STOP_LONG_PRESS_BUTTON, .data = START_STOP_LONG_PRESS_BUTTON_COMMAND, .data_length = sizeof(START_STOP_LONG_PRESS_BUTTON_COMMAND) / sizeof(START_STOP_LONG_PRESS_BUTTON_COMMAND[0]) },
    { .button = VOLUME_PLUS_BUTTON, .data = VOLUME_PLUS_BUTTON_COMMAND, .data_length = sizeof(VOLUME_PLUS_BUTTON_COMMAND) / sizeof(VOLUME_PLUS_BUTTON_COMMAND[0]) },
    { .button = VOLUME_MINUS_BUTTON, .data = VOLUME_MINUS_BUTTON_COMMAND, .data_length = sizeof(VOLUME_MINUS_BUTTON_COMMAND) / sizeof(VOLUME_MINUS_BUTTON_COMMAND[0]) },
    { .button = VOLUME_PLUS_OR_MINUS_LONG_PRESS_BUTTON, .data = VOLUME_PLUS_OR_MINUS_LONG_PRESS_BUTTON_COMMAND, .data_length = sizeof(VOLUME_PLUS_OR_MINUS_LONG_PRESS_BUTTON_COMMAND) / sizeof(VOLUME_PLUS_OR_MINUS_LONG_PRESS_BUTTON_COMMAND[0]) },
    { .button = FORVARD_BUTTON, .data = FORVARD_BUTTON_COMMAND, .data_length = sizeof(FORVARD_BUTTON_COMMAND) / sizeof(FORVARD_BUTTON_COMMAND[0]) },
    { .button = FORVARD_LONG_PRESS_BUTTON, .data = FORVARD_LONG_PRESS_BUTTON_COMMAND, .data_length = sizeof(FORVARD_LONG_PRESS_BUTTON_COMMAND) / sizeof(FORVARD_LONG_PRESS_BUTTON_COMMAND[0]) },
    { .button = BACK_BUTTON, .data = BACK_BUTTON_COMMAND, .data_length = sizeof(BACK_BUTTON_COMMAND) / sizeof(BACK_BUTTON_COMMAND[0]) },
    { .button = BACK_LONG_PRESS_BUTTON, .data = BACK_LONG_PRESS_BUTTON_COMMAND, .data_length = sizeof(BACK_LONG_PRESS_BUTTON_COMMAND) / sizeof(BACK_LONG_PRESS_BUTTON_COMMAND[0]) }
};