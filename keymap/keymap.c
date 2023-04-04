/* Copyright 2015-2017 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "keymap_nordic.h"
#include "keymap_swedish.h"
#include "muse.h"
#include "song_list.h"

#ifdef AUDIO_ENABLE
float se_song[][2] = SONG(QWERTY_SOUND);
float us_song[][2] = SONG(COLEMAK_SOUND);
float plover_song[][2] = SONG(PLOVER_SOUND);
float plover_gb_song[][2] = SONG(PLOVER_GOODBYE_SOUND);
float numpad_song[][2] = SONG(NUM_LOCK_ON_SOUND);
float rickroll[][2] = SONG(RICK_ROLL);
#endif

extern keymap_config_t keymap_config;

enum planck_layers { _QWERTY, _LOWER, _RAISE, _NUMPAD, _PLOVER, _ADJUST };

enum planck_keycodes {
  QWERTY = SAFE_RANGE,
  LOWER,
  RAISE,
  PLOVER,
  EXT_PLV,
  R_ARROW,
  KC_STD,
  NUMPAD,
  RICKRL,
  BACKLIT
};

#define CTRL_ESC LCTL_T(KC_ESC)
#define MACOS_LCK LCTL(LSFT(KC_SYSTEM_POWER))
#define CTRL_LEFT LCTL(KC_LEFT)
#define CTRL_RGHT LCTL(KC_RIGHT)
#define CTRL_UP LCTL(KC_UP)
#define CTRL_DOWN LCTL(KC_DOWN)
#define NUMBER MO(_NUMBER)
#define SYMBOL MO(_SYMBOL)
#define NAVIGATE MO(_NAVIGATE)

enum { TD_QUOTE = 0 };

tap_dance_action_t tap_dance_actions[] = {
    [TD_QUOTE] = ACTION_TAP_DANCE_DOUBLE(KC_QUOTE, KC_DOUBLE_QUOTE)};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    // clang-format off
    /* Qwerty International
     * ,-----------------------------------------------------------------------------------.
     * | Tab  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |
     * P  | Bksp |
     * |------+------+------+------+------+-------------+------+------+------+------+------|
     * | Esc  |   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  |
     * ;  | ' / "|
     * |------+------+------+------+------+------|------+------+------+------+------+------|
     * | Shift|   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |
     * /  | Enter|
     * |------+------+------+------+------+------+------+------+------+------+------+------|
     * | Ctrl |  Del | Win  | Alt  |Lower |    Space    |Raise | Left | Down |
     * Up  |Right |
     * `-----------------------------------------------------------------------------------'
     */
    [_QWERTY] = LAYOUT_planck_grid(
        KC_TAB, KC_Q, KC_W, KC_E, KC_R, KC_T, KC_Y, KC_U, KC_I, KC_O, KC_P,
        KC_BSPC, KC_ESC, KC_A, KC_S, KC_D, KC_F, KC_G, KC_H, KC_J, KC_K, KC_L,
        KC_SCLN, TD(TD_QUOTE), KC_LSFT, KC_Z, KC_X, KC_C, KC_V, KC_B, KC_N,
        KC_M, KC_COMM, KC_DOT, KC_SLSH, KC_ENT, KC_LCTL, KC_DEL, KC_LGUI,
        KC_LALT, LOWER, KC_SPC, KC_SPC, RAISE, KC_LEFT, KC_DOWN, KC_UP,
        KC_RGHT),

    /* Lower International
     * ,-----------------------------------------------------------------------------------.
     * |   `  |   !  |   @  |   #  |   $  |   %  |   ^  |   &  |   *  |   (  |
     * )  |      |
     * |------+------+------+------+------+-------------+------+------+------+------+------|
     * |      | Sup1 | Sup2 | Sup3 | Sup4 | Sup5 |      |   _  |   +  |   {  |
     * }  |   |  |
     * |------+------+------+------+------+------|------+------+------+------+------+------|
     * |      |      |  ->  | std::|      |      |      | Home | End  |      |
     * |      |
     * |------+------+------+------+------+------+------+------+------+------+------+------|
     * |      |      |      |      |      |             |      | Next | Vol- |
     * Vol+ | Play |
     * `-----------------------------------------------------------------------------------'
     */
    [_LOWER] = LAYOUT_planck_grid(
        KC_GRV, KC_EXLM, KC_AT, KC_HASH, KC_DLR, KC_PERC, KC_CIRC, KC_AMPR,
        KC_ASTR, KC_LPRN, KC_RPRN, _______, _______, G(KC_1), G(KC_2), G(KC_3),
        G(KC_4), G(KC_5), _______, KC_UNDS, KC_PLUS, KC_LCBR, KC_RCBR, KC_PIPE,
        _______, _______, R_ARROW, KC_STD, _______, _______, _______, KC_HOME,
        KC_END, _______, _______, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, KC_MNXT, KC_VOLD, KC_VOLU, KC_MPLY),

    /* Raise International
     * ,-----------------------------------------------------------------------------------.
     * |      |   1  |   2  |   3  |   4  |   5  |   6  |   7  |   8  |   9  |
     * 10 |      |
     * |------+------+------+------+------+-------------+------+------+------+------+------|
     * |      |   *  |   +  |   -  |  =   |   /  |      |   -  |   =  |   [  |
     * ]  |   \  |
     * |------+------+------+------+------+------|------+------+------+------+------+------|
     * |      |      |MS_WDN|MS_WUP|MS_LCL|MS_RCL|      | MS_L | MS_DN| MS_UP|
     * MS_R |      |
     * |------+------+------+------+------+------+------+------+------+------+------+------|
     * |      |PrtScr|      |      |      |             |      | Next | Vol- |
     * Vol+ | Play |
     * `-----------------------------------------------------------------------------------'
     */
    [_RAISE] = LAYOUT_planck_grid(
        _______, KC_1, KC_2, KC_3, KC_4, KC_5, KC_6, KC_7, KC_8, KC_9, KC_0,
        _______, _______, KC_ASTR, KC_PLUS, KC_MINS, KC_EQL, KC_SLSH, _______,
        _______, _______, KC_LBRC, KC_RBRC, KC_BSLS, _______, KC_WH_D, KC_WH_U,
        KC_BTN2, KC_BTN1, _______, _______, KC_MS_L, KC_MS_D, KC_MS_U, KC_MS_R,
        _______, _______, KC_PSCR, _______, _______, _______, KC_SPC, KC_SPC,
        _______, KC_MNXT, KC_VOLD, KC_VOLU, KC_MPLY),

    /* Numpad SE
     * ,-----------------------------------------------------------------------------------.
     * |  TAB |   7  |   8  |   9  |   *  |      |      |      |      |      |
     * |      |
     * |------+------+------+------+------+-------------+------+------+------+------+------|
     * |  Esc |   4  |   5  |   6  |   -  |      |      |      |      |      |
     * |      |
     * |------+------+------+------+------+------|------+------+------+------+------+------|
     * |  Bspc|   1  |   2  |   3  |   +  |      |      |      |      |      |
     * |      |
     * |------+------+------+------+------+------+------+------+------+------+------+------|
     * |  Ent |   0  |   ,  |   /  |      |             |      |      |      |
     * |      |
     * `-----------------------------------------------------------------------------------'
     */
    [_NUMPAD] = LAYOUT_planck_grid(
        KC_TAB, KC_P7, KC_P8, KC_P9, KC_MPLY, _______, _______, _______,
        _______, _______, _______, _______, KC_ESC, KC_P4, KC_P5, KC_P6,
        KC_PMNS, _______, _______, _______, _______, _______, _______, _______,
        KC_BSPC, KC_P1, KC_P2, KC_P3, KC_PPLS, _______, _______, _______,
        _______, _______, _______, _______, KC_PENT, KC_P0, KC_PDOT, KC_PSLS,
        _______, _______, _______, _______, _______, _______, _______,
        _______),

    /* Plover layer (http://opensteno.org)
     * ,-----------------------------------------------------------------------------------.
     * |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |
     * #  |   #  |
     * |------+------+------+------+------+------+------+------+------+------+------+------|
     * |      |   S  |   T  |   P  |   H  |   *  |   *  |   F  |   P  |   L  |
     * T  |   D  |
     * |------+------+------+------+------+------+------+------+------+------+------+------|
     * |      |   S  |   K  |   W  |   R  |   *  |   *  |   R  |   B  |   G  |
     * S  |   Z  |
     * |------+------+------+------+------+------+------+------+------+------+------+------|
     * | Exit |      |      |   A  |   O  |             |   E  |   U  |      |
     * |      |
     * `-----------------------------------------------------------------------------------'
     */
    [_PLOVER] = LAYOUT_planck_grid(
        KC_1, KC_1, KC_1, KC_1, KC_1, KC_1, KC_1, KC_1, KC_1, KC_1, KC_1, KC_1,
        XXXXXXX, KC_Q, KC_W, KC_E, KC_R, KC_T, KC_Y, KC_U, KC_I, KC_O, KC_P,
        KC_LBRC, XXXXXXX, KC_A, KC_S, KC_D, KC_F, KC_G, KC_H, KC_J, KC_K, KC_L,
        KC_SCLN, KC_QUOT, EXT_PLV, XXXXXXX, XXXXXXX, KC_C, KC_V, XXXXXXX,
        XXXXXXX, KC_N, KC_M, XXXXXXX, XXXXXXX, XXXXXXX),

    /* Adjust (Lower + Raise)
     * ,-----------------------------------------------------------------------------------.
     * |  F1  |  F2  |  F3  |  F4  |  F5  |  F6  |  F7  |  F8  |  F9  |  F10 |
     * F11 |  F12 |
     * |------+------+------+------+------+-------------+------+------+------+------+------|
     * | Reset| RICKR|      |Mus on|AudOn |MidiOn|AGswap| sv_SE| en_US| NMPAD|
     * PLOVR|      |
     * |------+------+------+------+------+------|------+------+------+------+------+------|
     * | Debug|Voice-|Voice+|MusOff|AudOff|MidOff|AGnorm|      |      | NMLCK|
     * Caps |      |
     * |------+------+------+------+------+------+------+------+------+------+------+------|
     * |      |      |      |      |      |             |      |      |      |
     * |      |
     * `-----------------------------------------------------------------------------------'
     */
    [_ADJUST] = LAYOUT_planck_grid(
        KC_F1, KC_F2, KC_F3, KC_F4, KC_F5, KC_F6, KC_F7, KC_F8, KC_F9, KC_F10, KC_F11, KC_F12,
        RESET, RICKRL, _______, MU_ON, AU_ON, MI_ON, AG_SWAP, _______, QWERTY, NUMPAD, PLOVER, _______,
        _______, _______, _______, MU_OFF, AU_OFF, MI_OFF, AG_NORM, _______, _______, KC_NUM, KC_CAPS, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______)
    // clang-format on
};

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
  case QWERTY:
    if (record->event.pressed) {
      set_single_persistent_default_layer(_QWERTY);
#ifdef AUDIO_ENABLE
      PLAY_SONG(us_song);
#endif
    }
    return false;
    break;
  case NUMPAD:
    if (record->event.pressed) {
      set_single_persistent_default_layer(_NUMPAD);
#ifdef AUDIO_ENABLE
      PLAY_SONG(numpad_song);
#endif
    }
  case LOWER:
    if (record->event.pressed) {
      layer_on(_LOWER);
      update_tri_layer(_LOWER, _RAISE, _ADJUST);
    } else {
      layer_off(_LOWER);
      update_tri_layer(_LOWER, _RAISE, _ADJUST);
    }
    return false;
    break;
  case RAISE:
    if (record->event.pressed) {
      layer_on(_RAISE);
      update_tri_layer(_LOWER, _RAISE, _ADJUST);
    } else {
      layer_off(_RAISE);
      update_tri_layer(_LOWER, _RAISE, _ADJUST);
    }
    return false;
    break;
  case PLOVER:
    if (record->event.pressed) {
#ifdef AUDIO_ENABLE
      stop_all_notes();
      PLAY_SONG(plover_song);
#endif
      layer_off(_ADJUST);
      layer_on(_PLOVER);
      if (!eeconfig_is_enabled()) {
        eeconfig_init();
      }
      keymap_config.raw = eeconfig_read_keymap();
      keymap_config.nkro = 1;
      eeconfig_update_keymap(keymap_config.raw);
    }
    return false;
    break;
  case EXT_PLV:
    if (record->event.pressed) {
#ifdef AUDIO_ENABLE
      PLAY_SONG(plover_gb_song);
#endif
      layer_off(_PLOVER);
      layer_on(_QWERTY);
    }
    return false;
    break;
  case BACKLIT:
    if (record->event.pressed) {
      register_code(KC_RSFT);
#ifdef BACKLIGHT_ENABLE
      backlight_step();
#endif
#ifdef KEYBOARD_planck_rev5
      PORTE &= ~(1 << 6);
#endif
    } else {
      unregister_code(KC_RSFT);
#ifdef KEYBOARD_planck_rev5
      PORTE |= (1 << 6);
#endif
    }
    return false;
    break;
  case R_ARROW:
    if (record->event.pressed) {
      SEND_STRING("->");
    } else {
    }
    break;
  case KC_STD:
    if (record->event.pressed) {
      SEND_STRING("std::");
    } else {
    }
    break;
  case RICKRL:
    if (record->event.pressed) {
      PLAY_SONG(rickroll);
    }
    return false;
    break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
  if (muse_mode) {
    if (clockwise) {
      muse_tempo += 1;
    } else {
      muse_tempo -= 1;
    }
  } else {
    if (clockwise) {
#ifdef MOUSEKEY_ENABLE
      tap_code(KC_MS_WH_DOWN);
#else
      tap_code(KC_PGDN);
#endif
    } else {
#ifdef MOUSEKEY_ENABLE
      tap_code(KC_MS_WH_UP);
#else
      tap_code(KC_PGUP);
#endif
    }
  }
}

void dip_update(uint8_t index, bool active) {
  switch (index) {
  case 0:
    if (active) {
#ifdef AUDIO_ENABLE
      // PLAY_SONG(plover_song);
#endif
      layer_on(_ADJUST);
    } else {
#ifdef AUDIO_ENABLE
      // PLAY_SONG(plover_gb_song);
#endif
      layer_off(_ADJUST);
    }
    break;
  case 1:
    if (active) {
      muse_mode = true;
    } else {
      muse_mode = false;
#ifdef AUDIO_ENABLE
      stop_all_notes();
#endif
    }
  }
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
  if (muse_mode) {
    if (muse_counter == 0) {
      uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
      if (muse_note != last_muse_note) {
        stop_note(compute_freq_for_midi_note(last_muse_note));
        play_note(compute_freq_for_midi_note(muse_note), 0xF);
        last_muse_note = muse_note;
      }
    }
    muse_counter = (muse_counter + 1) % muse_tempo;
  }
#endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
  default:
    return true;
  }
}
