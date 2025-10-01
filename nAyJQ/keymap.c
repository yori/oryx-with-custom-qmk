#include QMK_KEYBOARD_H
#include "version.h"
#define MOON_LED_LEVEL LED_LEVEL
#ifndef ZSA_SAFE_RANGE
#define ZSA_SAFE_RANGE SAFE_RANGE
#endif

enum custom_keycodes {
  RGB_SLD = ZSA_SAFE_RANGE,
  HSV_0_0_255,
  ST_MACRO_0,
  ST_MACRO_1,
  MAC_SPOTLIGHT,
  HR_A,
  HR_S,
  HR_D,
  HR_F,
  HR_J,
  HR_K,
  HR_L,
  HR_SCLN,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
};

#define DUAL_FUNC_0 LT(5, KC_V)
#define DUAL_FUNC_1 LT(13, KC_F11)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    TD(DANCE_0),    KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           TD(DANCE_1),    
    KC_GRAVE,       KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSLS,        
    KC_TAB,         KC_A,           KC_S,           KC_D,           KC_F,           KC_G,                                           KC_H,           KC_J,           KC_K,           KC_L,           KC_SCLN,        KC_QUOTE,       
    MAC_SPOTLIGHT,  KC_Z,           KC_X,           KC_C,           KC_V,           KC_B,                                           KC_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_SLASH,       KC_RIGHT_CTRL,  
                                                    LT(1, KC_ENTER),KC_ESCAPE,                                      KC_BSPC,        LT(2, KC_SPACE)
  ),
  [1] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         
    KC_TRANSPARENT, KC_EXLM,        KC_AT,          KC_HASH,        KC_DLR,         KC_PERC,                                        KC_COMMA,       KC_7,           KC_8,           KC_9,           KC_EQUAL,       KC_F12,         
    KC_TRANSPARENT, KC_LCBR,        KC_TRANSPARENT, KC_TRANSPARENT, KC_RCBR,        CW_TOGG,                                        KC_DOT,         KC_4,           KC_5,           KC_6,           KC_MINUS,       KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, HSV_0_0_255,    RGB_TOG,                                        KC_ASTR,        KC_1,           KC_2,           KC_3,           KC_0,           KC_SLASH,       
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [2] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         
    KC_TRANSPARENT, KC_CIRC,        KC_AMPR,        KC_ASTR,        KC_UNDS,        KC_PLUS,                                        KC_PAGE_UP,     KC_TRANSPARENT, KC_UP,          KC_MINUS,       KC_EQUAL,       KC_F12,         
    KC_TRANSPARENT, MT(MOD_LSFT, KC_LBRC),DUAL_FUNC_0,    DUAL_FUNC_1,    MT(MOD_LGUI, KC_RBRC),CW_TOGG,                                        KC_PGDN,        KC_LEFT,        KC_DOWN,        KC_RIGHT,       KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_LPRN,        KC_TRANSPARENT, KC_TRANSPARENT, KC_RPRN,        KC_TRANSPARENT,                                 ST_MACRO_0,     ST_MACRO_1,     KC_HOME,        KC_END,         KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, LGUI(KC_Q),     LGUI(KC_W),     LGUI(KC_E),     LGUI(KC_R),     LGUI(KC_T),                                     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_LEFT_SHIFT,  KC_LEFT_CTRL,   KC_LEFT_ALT,    KC_LEFT_GUI,    LGUI(KC_G),                                     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, LGUI(KC_Z),     LGUI(KC_X),     LGUI(KC_C),     LGUI(KC_V),     LGUI(KC_B),                                     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 LGUI(KC_BSPC),  KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_RIGHT_GUI,   KC_RIGHT_ALT,   KC_RIGHT_CTRL,  KC_RIGHT_SHIFT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};

const char chordal_hold_layout[MATRIX_ROWS][MATRIX_COLS] PROGMEM = LAYOUT(
  'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R', 
  'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R', 
  'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R', 
  'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R', 
  '*', '*', '*', '*'
);

bool get_hold_on_other_key_press(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
        case LT(1,KC_ENTER):
        case LT(2,KC_SPACE):
            // Only select the hold action if thumb key was recently tapped prior to this press.
            return record->tap.count == 1;
        default:
            // Do not select the hold action when another key is tapped.
            return false;
    }
}

bool get_permissive_hold(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
        case LT(1,KC_ENTER):
        case LT(2,KC_SPACE):
        case MT(MOD_LSFT, KC_A):
        case MT(MOD_RSFT, KC_SCLN):
            // Immediately select the hold action when another key is tapped.
            return true;
        default:
            // Do not select the hold action when another key is tapped.
            return false;
    }
}

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case KC_BSPC:
            return TAPPING_TERM -150;
        default:
            return TAPPING_TERM;
    }
}


extern rgb_config_t rgb_matrix_config;

RGB hsv_to_rgb_with_value(HSV hsv) {
  RGB rgb = hsv_to_rgb( hsv );
  float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
  return (RGB){ f * rgb.r, f * rgb.g, f * rgb.b };
}

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [1] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {139,218,204}, {0,0,0}, {0,0,0}, {139,218,204}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {86,197,198}, {86,197,198}, {86,197,198}, {0,0,0}, {0,0,0}, {0,0,0}, {86,197,198}, {86,197,198}, {86,197,198}, {0,0,0}, {0,0,0}, {0,0,0}, {86,197,198}, {86,197,198}, {86,197,198}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [2] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,218,204}, {86,250,255}, {86,250,255}, {0,218,204}, {0,0,0}, {0,0,0}, {74,255,255}, {0,0,0}, {0,0,0}, {74,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255}, {0,0,0}, {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,0}, {0,0,0}, {41,255,255}, {41,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [3] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {27,249,255}, {27,249,255}, {27,249,255}, {27,249,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [4] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {27,249,255}, {27,249,255}, {27,249,255}, {27,249,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb_with_value(hsv);
        rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (!keyboard_config.disable_layer_led) { 
    switch (biton32(layer_state)) {
      case 1:
        set_layer_color(1);
        break;
      case 2:
        set_layer_color(2);
        break;
      case 3:
        set_layer_color(3);
        break;
      case 4:
        set_layer_color(4);
        break;
     default:
        if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
          rgb_matrix_set_color_all(0, 0, 0);
        }
    }
  } else {
    if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
      rgb_matrix_set_color_all(0, 0, 0);
    }
  }

  return true;
}



typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[2];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_ESCAPE);
        tap_code16(KC_ESCAPE);
        tap_code16(KC_ESCAPE);
    }
    if(state->count > 3) {
        tap_code16(KC_ESCAPE);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_ESCAPE); break;
        case DOUBLE_TAP: register_code16(KC_MEDIA_PREV_TRACK); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_ESCAPE); register_code16(KC_ESCAPE);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_ESCAPE); break;
        case DOUBLE_TAP: unregister_code16(KC_MEDIA_PREV_TRACK); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_ESCAPE); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_MEDIA_PLAY_PAUSE);
        tap_code16(KC_MEDIA_PLAY_PAUSE);
        tap_code16(KC_MEDIA_PLAY_PAUSE);
    }
    if(state->count > 3) {
        tap_code16(KC_MEDIA_PLAY_PAUSE);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_MEDIA_PLAY_PAUSE); break;
        case DOUBLE_TAP: register_code16(KC_MEDIA_NEXT_TRACK); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_MEDIA_PLAY_PAUSE); register_code16(KC_MEDIA_PLAY_PAUSE);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_MEDIA_PLAY_PAUSE); break;
        case DOUBLE_TAP: unregister_code16(KC_MEDIA_NEXT_TRACK); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_MEDIA_PLAY_PAUSE); break;
    }
    dance_state[1].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
};

// --- Home-row mods gated by same-side thumb hold ---------------------------
// Put this in keymap.c. Replace the LEFT/RIGHT_THUMB_POS arrays with your
// board's physical positions for the thumb keys you want to count as "held".
// Then place HR_* custom keycodes (below) on your home row in the keymap.
//
// Behavior:
// - If you press a home-row key while the same-side thumb is down -> acts as MOD (hold)
// - If no thumb is down -> acts as the letter
// - If pressed nearly simultaneously: a small grace window (HR_GRACE_MS) resolves to MOD
// - If the letter was sent and you press the same-side thumb afterward, it flips to MOD

// Tunables
#ifndef HR_GRACE_MS
    #define HR_GRACE_MS 40  // grace window for near-simultaneous thumb+home-row (ms)
#endif

// --- Identify your thumb *positions* (row, col) on each side ----------------
// TODO: Replace these with the actual positions for your keyboard.
// You can quickly discover them by enabling key logging or printing row/col
// in process_record_user for your thumb keys.
typedef struct { uint8_t row, col; } pos_t;

static const pos_t LEFT_THUMB_POS[]  = {
    /* {row, col}, {row, col}, ... */
    // {5, 0}, {5, 1}, {5, 2},  // ← Example only; fill with your real positions
    {4, 1}
};
static const pos_t RIGHT_THUMB_POS[] = {
    /* {row, col}, {row, col}, ... */
    // {5, 13}, {5, 14}, {5, 15}, // ← Example only; fill with your real positions
    {4, 2}
};

static inline bool pos_match(uint8_t row, uint8_t col, const pos_t *list, size_t n) {
    for (size_t i = 0; i < n; i++) if (list[i].row == row && list[i].col == col) return true;
    return false;
}

// Map each HR_* to its tap (letter) and its modifier, and which side it belongs to.
typedef enum { HR_IDLE, HR_PENDING, HR_SENT_LETTER, HR_HELD_MOD } hr_state_t;
typedef struct {
    uint16_t keycode;   // HR_* custom code
    uint16_t tap_kc;    // the letter (e.g., KC_A)
    uint8_t  mod_mask;  // e.g., MOD_BIT(KC_LCTL)
    bool     is_left;   // true=left side, false=right side
    hr_state_t state;
    uint16_t t_started; // for grace window
} hrm_entry_t;

// Typical HRM mapping (customize to taste).
static hrm_entry_t hrm[] = {
    {HR_A,    KC_A,    MOD_BIT(KC_LSFT), true,  HR_IDLE, 0},
    {HR_S,    KC_S,    MOD_BIT(KC_LCTL), true,  HR_IDLE, 0},
    {HR_D,    KC_D,    MOD_BIT(KC_LALT), true,  HR_IDLE, 0},
    {HR_F,    KC_F,    MOD_BIT(KC_LGUI), true,  HR_IDLE, 0},
    {HR_J,    KC_J,    MOD_BIT(KC_RGUI), false, HR_IDLE, 0},
    {HR_K,    KC_K,    MOD_BIT(KC_RALT), false, HR_IDLE, 0},
    {HR_L,    KC_L,    MOD_BIT(KC_RCTL), false, HR_IDLE, 0},
    {HR_SCLN, KC_SCLN, MOD_BIT(KC_RSFT), false, HR_IDLE, 0},
};
#define HR_COUNT (sizeof(hrm) / sizeof(hrm[0]))

static inline int8_t hr_index_from_keycode(uint16_t kc) {
    for (uint8_t i = 0; i < HR_COUNT; i++) if (hrm[i].keycode == kc) return i;
    return -1;
}

// Thumb state (updated by *position*)
static bool left_thumb_down  = false;
static bool right_thumb_down = false;

static inline bool same_side_thumb_down(bool is_left) {
    return is_left ? left_thumb_down : right_thumb_down;
}

static void hr_flip_to_mod(uint8_t i) {
    if (hrm[i].state == HR_HELD_MOD) return;
    if (hrm[i].state == HR_SENT_LETTER) {
        unregister_code16(hrm[i].tap_kc);  // retract the letter if already sent
    }
    hrm[i].state = HR_HELD_MOD;
    register_mods(hrm[i].mod_mask);
}

static void hr_send_letter_if_pending(uint8_t i) {
    if (hrm[i].state == HR_PENDING) {
        hrm[i].state = HR_SENT_LETTER;
        register_code16(hrm[i].tap_kc);
    }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  // Track thumb holds by *physical position*
  if (record->event.pressed) {
      if (pos_match(record->event.key.row, record->event.key.col, LEFT_THUMB_POS,  ARRAY_SIZE(LEFT_THUMB_POS))) {
          left_thumb_down = true;
      } else if (pos_match(record->event.key.row, record->event.key.col, RIGHT_THUMB_POS, ARRAY_SIZE(RIGHT_THUMB_POS))) {
          right_thumb_down = true;
      }
  } else {
      if (pos_match(record->event.key.row, record->event.key.col, LEFT_THUMB_POS,  ARRAY_SIZE(LEFT_THUMB_POS))) {
          left_thumb_down = false;
      } else if (pos_match(record->event.key.row, record->event.key.col, RIGHT_THUMB_POS, ARRAY_SIZE(RIGHT_THUMB_POS))) {
          right_thumb_down = false;
      }
  }
  
  // Handle our custom home-row keys
  int8_t idx = hr_index_from_keycode(keycode);
  if (idx >= 0) {
      hrm_entry_t *k = &hrm[idx];
  
      if (record->event.pressed) {
          if (same_side_thumb_down(k->is_left)) {
              // Thumb already down → act as MOD immediately
              k->state = HR_HELD_MOD;
              register_mods(k->mod_mask);
          } else {
              // Start pending; wait briefly to see if thumb goes down
              k->state     = HR_PENDING;
              k->t_started = timer_read();
          }
      } else { // key released
          switch (k->state) {
              case HR_HELD_MOD:
                  unregister_mods(k->mod_mask);
                  break;
              case HR_SENT_LETTER:
                  unregister_code16(k->tap_kc);
                  break;
              case HR_PENDING:
                  // Fast tap that ended before grace expired and before letter was sent:
                  tap_code16(k->tap_kc);
                  break;
              default:
                  break;
          }
          k->state = HR_IDLE;
      }
      return false; // we've handled HR_*; don't let QMK process it further
  }
  
  // If a thumb key went down, convert any *pending* or even *letter-sent* same-side HR to MOD.
  if (record->event.pressed &&
      (pos_match(record->event.key.row, record->event.key.col, LEFT_THUMB_POS, ARRAY_SIZE(LEFT_THUMB_POS)) ||
       pos_match(record->event.key.row, record->event.key.col, RIGHT_THUMB_POS, ARRAY_SIZE(RIGHT_THUMB_POS)))) {
      bool thumb_is_left = pos_match(record->event.key.row, record->event.key.col, LEFT_THUMB_POS, ARRAY_SIZE(LEFT_THUMB_POS));
      for (uint8_t i = 0; i < HR_COUNT; i++) {
          if (hrm[i].state == HR_PENDING || hrm[i].state == HR_SENT_LETTER) {
              if (hrm[i].is_left == thumb_is_left) {
                  hr_flip_to_mod(i);
              }
          }
      }
  }

  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_LGUI(SS_LSFT(SS_TAP(X_LBRC))));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_LGUI(SS_LSFT(SS_TAP(X_RBRC))));
    }
    break;
    case MAC_SPOTLIGHT:
      HCS(0x221);

    case DUAL_FUNC_0:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_LPRN);
        } else {
          unregister_code16(KC_LPRN);
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_LEFT_CTRL);
        } else {
          unregister_code16(KC_LEFT_CTRL);
        }  
      }  
      return false;
    case DUAL_FUNC_1:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_RPRN);
        } else {
          unregister_code16(KC_RPRN);
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_LEFT_ALT);
        } else {
          unregister_code16(KC_LEFT_ALT);
        }  
      }  
      return false;
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_0_0_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(0,0,255);
      }
      return false;
  }
  return true;
}

// Resolve pending HR taps after the grace window.
void matrix_scan_user(void) {
    // const uint16_t now = timer_read();
    for (uint8_t i = 0; i < HR_COUNT; i++) {
        if (hrm[i].state == HR_PENDING) {
            if (same_side_thumb_down(hrm[i].is_left)) {
                hr_flip_to_mod(i);
            } else if (timer_elapsed(hrm[i].t_started) > HR_GRACE_MS) {
                hr_send_letter_if_pending(i);
            }
        }
    }
}
