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
    TD_T_LBRC,
    TD_Y_RBRC,
    TD_G_MINS,
    TD_H_EQL,

    TD_YE_YO,
    TD_SS_HS
};

#define _ATDD ACTION_TAP_DANCE_DOUBLE

tap_dance_action_t tap_dance_actions[] = {
    // _EN
    [TD_T_LBRC] = _ATDD(KC_T, KC_LBRC),
    [TD_Y_RBRC] = _ATDD(KC_Y, KC_RBRC),
    [TD_G_MINS] = _ATDD(KC_G, KC_MINS),
    [TD_H_EQL]  = _ATDD(KC_H, KC_EQL),

    // _RU
    [TD_YE_YO] = _ATDD(KC_T, KC_GRV),
    [TD_SS_HS] = _ATDD(KC_M, KC_RBRC)
};

#define T_LBRC TD(TD_T_LBRC)
#define Y_RBRC TD(TD_Y_RBRC)
#define G_MINS TD(TD_G_MINS)
#define H_EQL  TD(TD_H_EQL)

#define YE_YO TD(TD_YE_YO)
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
#define TAB_RSE LT(_RAISE, KC_TAB)
#define ENT_LWR LT(_LOWER, KC_ENT)
#define ESC_LWR LT(_LOWER, KC_ESC)

#define   R_NUM LT(_NUMPAD, KC_R)
#define     F_F LT(_F, KC_F)
#define V_MOUSE LT(_MOUSEPAD, KC_V)

#define HYPR_EN HYPR_T(KC_0)
#define HYPR_RU HYPR_T(KC_1)

#define GRV_LGU MT(MOD_LGUI, KC_GRV)
#define LBR_RGU MT(MOD_RGUI, KC_LBRC)


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [_EN] = LAYOUT(
      HYPR_EN, GRV_LGU, KC_Q,    KC_W,    KC_E,      R_NUM,  T_LBRC,       Y_RBRC,    KC_U,    KC_I,    KC_O,    KC_P, LBR_RGU, HYPR_RU,
               KC_LCTL, KC_A,    KC_S,    KC_D,        F_F,  G_MINS,        H_EQL,    KC_J,    KC_K,    KC_L, KC_SCLN, QUO_RCT,
               KC_LALT, KC_Z,    KC_X,    KC_C,    V_MOUSE,    KC_B,         KC_N,    KC_M, KC_COMM,  KC_DOT, KC_SLSH, BSL_RAL,
                                          TAB_RSE, SPC_LSH, ENT_LWR,      ESC_LWR, BSP_RSH, DEL_RSE
    ),

    [_RU] = LAYOUT(
      _______, _______, _______, _______, _______, _______,   YE_YO,      _______, _______, _______, _______, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,      _______, _______, _______, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,      _______,   SS_HS, _______, _______, _______, _______,
                                          _______, _______, _______,      _______, _______, _______
    ),

    [_NUMPAD] = LAYOUT(
      _______, _______, _______, _______, _______, _______, _______,         KC_7,    KC_8,    KC_9, _______, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,         KC_4,    KC_5,    KC_6, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,         KC_1,    KC_2,    KC_3, _______, _______, _______,
                                          _______, _______, _______,         KC_0, _______, _______
    ),

    [_F] = LAYOUT(
      _______, _______, _______, _______, _______, _______, _______,        KC_F7,   KC_F8,    KC_F9,  KC_F10,  KC_F11,  KC_F12, _______,
               _______, _______, _______, _______, _______, _______,        KC_F4,   KC_F5,    KC_F6, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,        KC_F1,   KC_F2,    KC_F3, _______, _______, _______,
                                          _______, _______, _______,      _______, _______, _______
    ),

    [_MOUSEPAD] = LAYOUT(
      _______, _______, _______, _______, _______, _______, _______,      KC_WH_U, KC_MS_U, _______, _______, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,      KC_MS_L, KC_BTN1, KC_MS_R, _______, _______, _______,
               _______, _______, _______, _______, _______, _______,      KC_WH_D, KC_MS_D, _______, _______, _______, _______,
                                          _______, _______, _______,      KC_BTN1, KC_BTN2, KC_BTN3
    ),

    [_LOWER] = LAYOUT(
      _______, KC_UNDS, KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,        KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  F12_RGU,
               PLS_LCT, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  KC_PERC,      KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, MIN_RCT,
               EQL_LAL, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,         KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    _______,
                                          _______, _______, _______,      ESC_LWR, BSP_RSH, DEL_RSE
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
    return true;
}

#ifdef oled_task_kb
    #define oled_task_kb(void) { return true; }
#endif
