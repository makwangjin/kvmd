/*****************************************************************************
#                                                                            #
#   KVMD - The main PiKVM daemon.                                            #
#                                                                            #
#   Copyright (C) 2018-2024  Maxim Devaev <mdevaev@gmail.com>                #
#                                                                            #
#   This program is free software: you can redistribute it and/or modify     #
#   it under the terms of the GNU General Public License as published by     #
#   the Free Software Foundation, either version 3 of the License, or        #
#   (at your option) any later version.                                      #
#                                                                            #
#   This program is distributed in the hope that it will be useful,          #
#   but WITHOUT ANY WARRANTY; without even the implied warranty of           #
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            #
#   GNU General Public License for more details.                             #
#                                                                            #
#   You should have received a copy of the GNU General Public License        #
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.   #
#                                                                            #
*****************************************************************************/


#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"

#include "ph_types.h"
#include "ph_tools.h"
#include "ph_outputs.h"
#include "ph_usb.h"
#include "ph_ps2.h"
#include "ph_com.h"
#include "ph_proto.h"
#include "ph_cmds.h"
#include "ph_debug.h"

/*
 * ====================================================================
 * [수정] CH9329 시리얼 프로토콜 번역기 코드 (main.c에 삽입)
 * main.c는 ph_cmds.h와 ph_usb.h를 포함하므로, 링커 오류가 발생하지 않습니다.
 * ====================================================================
 */

// CH9329 프로토콜 상수 정의
#define CH_HEADER_1 0x57
#define CH_HEADER_2 0xAB
#define CH_CMD_TYPE 0x00 // 고정값
#define CH_CMD_MOUSE 0x01
#define CH_CMD_KEYBOARD 0x02

// CH9329 패킷 길이 정의
#define CH_LEN_MOUSE 0x05
#define CH_LEN_KEYBOARD 0x08

// 파서(Parser)의 상태를 정의
typedef enum {
    CH_WAIT_HEADER_1,
    CH_WAIT_HEADER_2,
    CH_WAIT_CMD_TYPE,
    CH_WAIT_CMD,
    CH_WAIT_LEN,
    CH_READ_DATA,
    CH_WAIT_CHECKSUM
} ch_parser_state_t;

// CH9329 패킷 구조체 정의 (u8 타입 사용)
static struct {
    u8 cmd;
    u8 len;
    u8 data[8]; // 최대 8바이트 (키보드 기준)
    u8 checksum;
    u8 data_index;
} ch_packet;

static ch_parser_state_t ch_parser_state = CH_WAIT_HEADER_1;

/**
 * @brief CH9329 패킷을 해석하고 PiKVM의 HID 함수를 호출
 */
static void ch9329_process_packet()
{
    if (ch_packet.cmd == CH_CMD_KEYBOARD && ch_packet.len == CH_LEN_KEYBOARD)
    {
        // u8 타입 사용
        u8 modifier = ch_packet.data[0];
        u8 keycodes[6];
        keycodes[0] = ch_packet.data[2];
        keycodes[1] = ch_packet.data[3];
        keycodes[2] = ch_packet.data[4];
        keycodes[3] = ch_packet.data[5];
        keycodes[4] = ch_packet.data[6];
        keycodes[5] = ch_packet.data[7];
        
        // PiKVM의 *검증된* "ph_hid_keyboard_report" 함수 호출
        // (ph_cmds.h -> ph_usb.h -> ph_hid.h 에 정의되어 있음)
        ph_hid_keyboard_report(modifier, keycodes);
    }
    else if (ch_packet.cmd == CH_CMD_MOUSE && ch_packet.len == CH_LEN_MOUSE)
    {
        // u8, s8 타입 사용
        u8 buttons = ch_packet.data[0];
        s8 x = (s8)ch_packet.data[1];
        s8 y = (s8)ch_packet.data[2];
        s8 wheel = (s8)ch_packet.data[3];

        // PiKVM의 *검증된* "ph_hid_mouse_report" 함수 호출
        ph_hid_mouse_report(buttons, x, y, wheel);
    }
}

/**
 * @brief UART에서 1바이트를 읽어 CH9329 상태 머신을 실행
 */
static void ch9329_parse_byte(u8 ch) // u8 타입 사용
{
    // C89 표준을 준수하기 위해 변수를 함수 맨 위에 선언합니다.
    u8 calculated_checksum = 0; // u8 타입 사용
    int i = 0; 

    switch (ch_parser_state)
    {
        case CH_WAIT_HEADER_1:
            if (ch == CH_HEADER_1) ch_parser_state = CH_WAIT_HEADER_2;
            break;
        case CH_WAIT_HEADER_2:
            if (ch == CH_HEADER_2) ch_parser_state = CH_WAIT_CMD_TYPE;
            else ch_parser_state = CH_WAIT_HEADER_1;
            break;
        case CH_WAIT_CMD_TYPE:
            if (ch == CH_CMD_TYPE) ch_parser_state = CH_WAIT_CMD;
            else ch_parser_state = CH_WAIT_HEADER_1;
            break;
        case CH_WAIT_CMD:
            if (ch == CH_CMD_MOUSE || ch == CH_CMD_KEYBOARD) {
                ch_packet.cmd = ch;
                ch_parser_state = CH_WAIT_LEN;
            } else ch_parser_state = CH_WAIT_HEADER_1;
            break;
        case CH_WAIT_LEN:
            if ((ch_packet.cmd == CH_CMD_MOUSE && ch == CH_LEN_MOUSE) ||
                (ch_packet.cmd == CH_CMD_KEYBOARD && ch == CH_LEN_KEYBOARD)) {
                ch_packet.len = ch;
                ch_packet.data_index = 0;
                ch_parser_state = CH_READ_DATA;
            } else ch_parser_state = CH_WAIT_HEADER_1;
            break;
        case CH_READ_DATA:
            ch_packet.data[ch_packet.data_index++] = ch;
            if (ch_packet.data_index >= ch_packet.len) {
                ch_parser_state = CH_WAIT_CHECKSUM;
            }
            break;
        case CH_WAIT_CHECKSUM:
            ch_packet.checksum = ch;
            calculated_checksum = CH_HEADER_1 + CH_HEADER_2 + CH_CMD_TYPE + 
                                  ch_packet.cmd + ch_packet.len;
            
            for (i = 0; i < ch_packet.len; i++) {
                calculated_checksum += ch_packet.data[i];
            }

            if ((u8)calculated_checksum == ch_packet.checksum) { // u8 타입 사용
                ch9329_process_packet();
            }
            ch_parser_state = CH_WAIT_HEADER_1; 
            break;
        default:
            ch_parser_state = CH_WAIT_HEADER_1;
            break;
    }
}
/*
 * ====================================================================
 * (사용자 추가 부분 끝)
 * ====================================================================
 */


static bool _reset_required = false;


static u8 _handle_request(const u8 *data) { // 8 bytes
	// FIXME: See kvmd/kvmd#80
	// Should input buffer be cleared in this case?
	if (data[0] == PH_PROTO_MAGIC && ph_crc16(data, 6) == ph_merge8_u16(data[6], data[7])) {
#		define HANDLE(x_handler, x_reset) { \
				x_handler(data + 2); \
				if (x_reset) { _reset_required = true; } \
				return PH_PROTO_PONG_OK; \
			}
		switch (data[1]) {
			case PH_PROTO_CMD_PING:				return PH_PROTO_PONG_OK;
			case PH_PROTO_CMD_SET_KBD:			HANDLE(ph_cmd_set_kbd, true);
			case PH_PROTO_CMD_SET_MOUSE:		HANDLE(ph_cmd_set_mouse, true);
			case PH_PROTO_CMD_SET_CONNECTED:	return PH_PROTO_PONG_OK; // Arduino AUM
			case PH_PROTO_CMD_CLEAR_HID:		HANDLE(ph_cmd_send_clear, false);
			case PH_PROTO_CMD_KBD_KEY:			HANDLE(ph_cmd_kbd_send_key, false);
			case PH_PROTO_CMD_MOUSE_BUTTON:		HANDLE(ph_cmd_mouse_send_button, false);
			case PH_PROTO_CMD_MOUSE_ABS:		HANDLE(ph_cmd_mouse_send_abs, false);
			case PH_PROTO_CMD_MOUSE_REL:		HANDLE(ph_cmd_mouse_send_rel, false);
			case PH_PROTO_CMD_MOUSE_WHEEL:		HANDLE(ph_cmd_mouse_send_wheel, false);
			case PH_PROTO_CMD_REPEAT:			return 0;
		}
#		undef HANDLE
		return PH_PROTO_RESP_INVALID_ERROR;
	}
	return PH_PROTO_RESP_CRC_ERROR;
}

static void _send_response(u8 code) {
	static u8 prev_code = PH_PROTO_RESP_NONE;
	if (code == 0) {
		code = prev_code; // Repeat the last code
	} else {
		prev_code = code;
	}

	u8 resp[8] = {0};
	resp[0] = PH_PROTO_MAGIC_RESP;

	if (code & PH_PROTO_PONG_OK) {
		resp[1] = PH_PROTO_PONG_OK;
		if (_reset_required) {
			resp[1] |= PH_PROTO_PONG_RESET_REQUIRED;
		}
		resp[2] = PH_PROTO_OUT1_DYNAMIC;

		resp[1] |= ph_cmd_get_offlines();
		resp[1] |= ph_cmd_kbd_get_leds();
		resp[2] |= ph_g_outputs_active;
		resp[3] |= ph_g_outputs_avail;
	} else {
		resp[1] = code;
	}

	ph_split16(ph_crc16(resp, 6), &resp[6], &resp[7]);

	ph_com_write(resp);

	if (_reset_required) {
		watchdog_reboot(0, 0, 100); // Даем немного времени чтобы отправить ответ, а потом ребутимся
	}
}


/*
 * ====================================================================
 * [수정] _data_handler 함수를 하이재킹합니다.
 * ====================================================================
 */
static void _data_handler(const u8 *data) {
	// _send_response(_handle_request(data)); // <-- [수정] 기존 PiKVM 프로토콜 비활성화

    /*
     * [수정] 하이재킹: 8바이트 버퍼(data)를 CH9329 번역기에 1바이트씩 주입합니다.
     */
    int i;
    for (i = 0; i < 8; i++) {
        ch9329_parse_byte(data[i]);
    }

    /*
     * [수정] CH340(control3)은 응답을 기다리지 않으므로,
     * _send_response() 함수를 호출하지 않습니다.
     */
}

/*
 * ====================================================================
 * [수정] _timeout_handler 함수를 비활성화합니다.
 * ====================================================================
 */
static void _timeout_handler(void) {
	// _send_response(PH_PROTO_RESP_TIMEOUT_ERROR); // <-- [수정] 비활성화
    /*
     * [수정] CH9329 프로토콜은 타임아웃 개념을 사용하지 않으므로,
     * 타임아웃 콜백을 무시합니다.
     */
}


int main(void) {
	//ph_debug_act_init();
	//ph_debug_uart_init();
	ph_outputs_init();
	ph_ps2_init();
	ph_usb_init(); // Тут может быть инициализация USB-CDC для бриджа
	
    // [수정] CH9329 파서 초기화 코드를 ph_com_init 전에 추가합니다.
    ch_parser_state = CH_WAIT_HEADER_1;

    ph_com_init(_data_handler, _timeout_handler);

	while (true) {
		ph_usb_task();
		ph_ps2_task();
		if (!_reset_required) {
			ph_com_task();
			//ph_debug_act_pulse(100);
		}
	}
	return 0;
}
