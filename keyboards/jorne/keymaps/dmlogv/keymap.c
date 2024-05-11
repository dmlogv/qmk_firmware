// Copyright 2021 Joric (@joric)
// SPDX-License-Identifier: GPL-2.0-or-later
#include QMK_KEYBOARD_H


enum layers {
    _EN = 0,
    _RU,
    _NUMPAD,
    _F,
    _MOUSEPAD,
    _LOWER,
    _RAISE,
    _ADJUST,
};


// Tap Dance
enum {
    TD_SS_HS
};

#define _ATDD ACTION_TAP_DANCE_DOUBLE

tap_dance_action_t tap_dance_actions[] = {
    // _RU
    [TD_SS_HS] = _ATDD(KC_M, KC_RBRC)
};

#define SS_HS TD(TD_SS_HS)


// Misc
enum custom_keycodes {
    RGBRST = SAFE_RANGE
};

#define F12_RGU MT(MOD_RGUI, KC_F12)
#define PLS_LCT MT(MOD_LCTL, KC_PPLS)
#define EQL_LCT MT(MOD_LCTL, KC_PEQL)
#define QUO_RCT MT(MOD_RCTL, KC_QUOT)
#define APP_RCT MT(MOD_RCTL, KC_APP)
#define MIN_RCT MT(MOD_RCTL, KC_MINS)
#define EQL_LAL MT(MOD_LALT, KC_EQL)
#define BSL_RAL MT(MOD_RALT, KC_BSLS)

#define BSP_RSH MT(MOD_RSFT, KC_BSPC)
#define SPC_LSH MT(MOD_LSFT, KC_SPC)

#define DEL_RSE LT(_RAISE, KC_DEL)
#define TAB_LWR LT(_LOWER, KC_TAB)
#define ENT_LWR LT(_LOWER, KC_ENT)
#define ESC_RSE LT(_RAISE, KC_ESC)

#define W_MOUSE LT(_MOUSEPAD, KC_W)
#define   E_NUM LT(_NUMPAD, KC_E)
#define     R_F LT(_F, KC_R)

#define  F_LGUI MT(MOD_LGUI, KC_F)
#define  D_LALT MT(MOD_LALT, KC_D)
#define  A_LCTL MT(MOD_LCTL, KC_A)

#define  J_RGUI MT(MOD_RGUI, KC_J)
#define  K_RALT MT(MOD_RALT, KC_K)
#define SC_RCTL MT(MOD_RCTL, KC_SCLN)

#define HYPR_EN HYPR_T(KC_0)
#define HYPR_RU HYPR_T(KC_1)


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [_EN] = LAYOUT(
      XXXXXXX,  KC_GRV,    KC_Q, W_MOUSE,   E_NUM,     R_F,    KC_T,         KC_Y,    KC_U,    KC_I,    KC_O,    KC_P, KC_LBRC, XXXXXXX,
               HYPR_EN,  A_LCTL,    KC_S,  D_LALT,  F_LGUI,    KC_G,         KC_H,  J_RGUI,  K_RALT,    KC_L, SC_RCTL, QUO_RCT,
               HYPR_RU,    KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,         KC_N,    KC_M, KC_COMM,  KC_DOT, KC_SLSH, BSL_RAL,
                                          ESC_RSE, SPC_LSH, TAB_LWR,      ENT_LWR, BSP_RSH, DEL_RSE
    ),

    [_RU] = LAYOUT(
      _______, _______, _______, _______, _______, _______, _______,      _______, _______, _______, _______, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,      _______, _______, _______, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,      _______,   SS_HS, _______, _______, _______, _______,
                                          _______, _______, _______,      _______, _______, _______
    ),

    [_NUMPAD] = LAYOUT(
      _______, _______, _______, _______, _______, _______, _______,      _______,    KC_7,    KC_8,    KC_9, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,      _______,    KC_4,    KC_5,    KC_6, _______, _______,
               _______, _______, _______, _______, _______, _______,      _______,    KC_1,    KC_2,    KC_3, _______, _______,
                                          _______, _______, _______,      _______,    KC_0, KC_PCMM
    ),

    [_F] = LAYOUT(
      _______, _______, _______, _______, _______, _______, _______,      _______,   KC_F7,   KC_F8,    KC_F9,  KC_F10,  KC_F11, _______,
               _______, _______, _______, _______, _______, _______,      _______,   KC_F4,   KC_F5,    KC_F6,  KC_F12, _______,
               _______, _______, _______, _______, _______, _______,      _______,   KC_F1,   KC_F2,    KC_F3, _______, _______,
                                          _______, _______, _______,      _______, _______, _______
    ),

    [_MOUSEPAD] = LAYOUT(
      _______, _______, _______, _______, _______, _______, _______,      _______, KC_WH_U, KC_MS_U, _______, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,      _______, KC_MS_L, KC_BTN1, KC_MS_R, _______, _______,
               _______, _______, _______, _______, _______, _______,      _______, KC_WH_D, KC_MS_D, _______, _______, _______,
                                          _______, _______, _______,      KC_BTN1, KC_BTN2, KC_BTN3
    ),

    [_LOWER] = LAYOUT(
      _______, KC_UNDS, KC_F1,   KC_F2,   KC_F3,   KC_UNDS, KC_LBRC,      KC_RBRC,  KC_EQL,   KC_F8,   KC_F9,  KC_F10,  KC_F11, F12_RGU,
               PLS_LCT, KC_EXLM, KC_AT,   KC_HASH, KC_MINS, KC_LPRN,      KC_RPRN, KC_PLUS, KC_ASTR, KC_LPRN, KC_RPRN, MIN_RCT,
               EQL_LAL, KC_1,    KC_2,    KC_3,    KC_SCLN, KC_LCBR,      KC_RCBR, KC_COLN,    KC_8,    KC_9,    KC_0, _______,
                                          _______, _______, _______,      ENT_LWR, BSP_RSH, DEL_RSE
    ),

    [_RAISE] = LAYOUT(
      _______, KC_NUM,  KC_PSLS, KC_P7,   KC_P8,   KC_P9,   KC_PMNS,      KC_VOLU, KC_HOME, KC_PSCR, KC_PGUP, KC_SCRL, KC_CAPS, _______,
               EQL_LCT, KC_PAST, KC_P4,   KC_P5,   KC_P6,   KC_PPLS,      KC_MUTE, KC_LEFT, KC_UP,   KC_RGHT, KC_INS,  APP_RCT,
               _______, KC_P0,   KC_P1,   KC_P2,   KC_P3,   KC_PCMM,      KC_VOLD, KC_END,  KC_DOWN, KC_PGDN, KC_PAUS, _______,
                                          _______, _______, _______,      _______, _______, _______
    ),

    [_ADJUST] = LAYOUT(
      QK_BOOT, RGBRST,  AS_UP,   AS_TOGG, AS_DOWN, _______, _______,      _______, _______, AS_DOWN, AS_TOGG, AS_UP,   RGBRST,  QK_BOOT,
               RGB_TOG, RGB_HUI, RGB_SAI, RGB_VAI, _______, _______,      _______, _______, RGB_VAI, RGB_SAI, RGB_HUI, RGB_TOG,
               RGB_MOD, RGB_HUD, RGB_SAD, RGB_VAD, _______, _______,      _______, _______, RGB_VAD, RGB_SAD, RGB_HUD, RGB_MOD,
                                          _______, SH_TOGG, _______,      _______, SH_TOGG, _______
    ),

};


layer_state_t layer_state_set_user(layer_state_t state) {
    return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}


bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case RGBRST:
            #ifdef RGBLIGHT_ENABLE
                if (record->event.pressed) {
                    eeconfig_update_rgblight_default();
                    rgblight_enable();
                }
            #endif
            break;

        case HYPR_EN:
            if (record->tap.count && record->event.pressed) {
                layer_off(_RU);
                tap_code16(KC_F18);
                return false;
            }
            break;

        case HYPR_RU:
            if (record->tap.count && record->event.pressed) {
                layer_on(_RU);
                tap_code16(KC_F19);
                return false;
            }
            break;
    }

    return true;
}


oled_rotation_t oled_init_user(oled_rotation_t rotation) {
    return OLED_ROTATION_270;
}


static void render_layer_state(void) {
    // Host Keyboard Layer Status
    oled_write_P(PSTR("Layer\n"), false);
    switch (get_highest_layer(layer_state)) {
        case _EN:
            oled_write_P(PSTR("   EN\n"), false);
            break;
        case _RU:
            oled_write_P(PSTR("   RU\n"), false);
            break;
        case _NUMPAD:
            oled_write_P(PSTR(" NUMS\n"), false);
            break;
        case _F:
            oled_write_P(PSTR("    F\n"), false);
            break;
        case _MOUSEPAD:
            oled_write_P(PSTR(" MICE\n"), false);
            break;
        case _LOWER:
            oled_write_P(PSTR("LOWER\n"), false);
            break;
        case _RAISE:
            oled_write_P(PSTR("RAISE\n"), false);
            break;
        case _ADJUST:
            oled_write_P(PSTR("  ADJ\n"), false);
            break;
        default:
            oled_write_P(PSTR("#UNDEF\n"), false);
    }
    oled_write_P(PSTR("-----\n"), false);


    // Host Keyboard LED Status
    led_t led_state = host_keyboard_led_state();
    oled_write_P(led_state.num_lock ? PSTR("1 ") : PSTR("  "), false);
    oled_write_P(led_state.caps_lock ? PSTR("A ") : PSTR("  "), false);
    oled_write_P(led_state.scroll_lock ? PSTR("S") : PSTR(" "), false);
}


bool oled_task_user(void) {
    if (is_keyboard_left()) {
        render_layer_state();
    } else {
        oled_write_P(PSTR("WPM: "), false);
        oled_write(get_u8_str(get_current_wpm(), ' '), false);
    }
    
    return false;
}
