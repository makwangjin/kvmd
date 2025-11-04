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

// [링커 오류 수정] 키보드 상태 저장을 위한 변수 추가
static u8 ch_kbd_modifier = 0;
static u8 ch_kbd_keycodes[6] = {0};

/**
 * @brief 6-Key 롤오버(CH9329)를 1-Key 이벤트(PiKVM)로 변환
 */
static void ch9329_process_keyboard_state(u8 new_modifier, u8 new_keycodes[6])
{
    int i, j;
    bool found;

    // 1. 모디파이어 키 처리
    if (ch_kbd_modifier != new_modifier) {
        // (간결성을 위해 모디파이어 키의 개별 Press/Release는 생략합니다.
        // PiKVM의 ph_cmd_kbd_send_key는 모디파이어를 다루지 않습니다.)
        ch_kbd_modifier = new_modifier;
    }

    // 2. 키 떼기 (Release) 이벤트 전송
    // (이전 리포트에는 있었지만, 새 리포트에는 없는 키를 찾습니다)
    for (i = 0; i < 6; i++) {
        if (ch_kbd_keycodes[i] == 0) continue; // 이전 키가 비어있으면 스킵
        
        found = false;
        for (j = 0; j < 6; j++) {
            if (ch_kbd_keycodes[i] == new_keycodes[j]) {
                found = true;
                break;
            }
        }
        
        if (!found) {
            // 이 키(ch_kbd_keycodes[i])를 떼었습니다.
            u8 args[2];
            args[0] = ch_kbd_keycodes[i];
            args[1] = 0; // 0 = Release
            ph_cmd_kbd_send_key(args);
        }
    }

    // 3. 키 누름 (Press) 이벤트 전송
    // (새 리포트에는 있지만, 이전 리포트에는 없던 키를 찾습니다)
    for (i = 0; i < 6; i++) {
        if (new_keycodes[i] == 0) continue; // 새 키가 비어있으면 스킵
        
        found = false;
        for (j = 0; j < 6; j++) {
            if (new_keycodes[i] == ch_kbd_keycodes[j]) {
                found = true;
                break;
            }
        }
        
        if (!found) {
            // 이 키(new_keycodes[i])를 눌렀습니다.
            u8 args[2];
            args[0] = new_keycodes[i];
            args[1] = 1; // 1 = Press
            ph_cmd_kbd_send_key(args);
        }
    }

    // 4. 현재 상태를 이전 상태로 저장
    for (i = 0; i < 6; i++) {
        ch_kbd_keycodes[i] = new_keycodes[i];
    }
}


/**
 * @brief CH9329 패킷을 해석하고 PiKVM의 HID 함수를 호출
 */
static void ch9329_process_packet()
{
    if (ch_packet.cmd == CH_CMD_KEYBOARD && ch_packet.len == CH_LEN_KEYBOARD)
    {
        // [링커 오류 수정]
        // 6KRO 리포트를 1-Key 이벤트로 변환하는 새 함수 호출
        u8 modifier = ch_packet.data[0];
        u8* keycodes = &ch_packet.data[2]; // data[2]~[7]
        ch9329_process_keyboard_state(modifier, keycodes);
    }
    else if (ch_packet.cmd == CH_CMD_MOUSE && ch_packet.len == CH_LEN_MOUSE)
    {
        // 마우스 로직은 링커 오류가 없었으므로 10차 수정안과 동일
        u8 buttons = ch_packet.data[0];
        s8 x = (s8)ch_packet.data[1];
        s8 y = (s8)ch_packet.data[2];
        s8 wheel = (s8)ch_packet.data[3];
        
        u8 button_args[2];
        button_args[0] = (buttons & 0x07); 
        button_args[1] = ((buttons >> 3) & 0x03); 
        ph_cmd_mouse_send_button(button_args);

        u8 rel_args[2];
        rel_args[0] = (u8)x;
        rel_args[1] = (u8)y;
        ph_cmd_mouse_send_rel(rel_args);
        
        u8 wheel_args[2];
        wheel_args[0] = (u8)wheel; // Vertical wheel
        wheel_args[1] = 0;         // Horizontal wheel (CH9329는 지원 안함)
        ph_cmd_mouse_send_wheel(wheel_args);
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

/*
 * ====================================================================
 * [수정] _handle_request 함수는 사용되지 않으므로 비워둡니다.
 * (로그에 'unused' 경고가 뜨지만 무시합니다.)
 * ====================================================================
 */
static u8 _handle_request(const u8 *data) { // 8 bytes
	// (기존 내용 모두 삭제 또는 주석 처리)
    return 0; 
}

/*
 * ====================================================================
 * [수정] _send_response 함수는 사용되지 않으므로 비워둡니다.
 * (로그에 'unused' 경고가 뜨지만 무시합니다.)
 * ====================================================================
 */
static void _send_response(u8 code) {
	// (기존 내용 모두 삭제 또는 주석 처리)
}


/*
 * ====================================================================
 * [수정] _data_handler 함수를 하이재킹합니다.
 * ====================================================================
 */
static void _data_handler(const u8 *data) {
	// _send_response(_handle_request(data)); // <-- 기존 PiKVM 프로토콜 비활성화

    /*
     * 하이재킹: 8바이트 버퍼(data)를 CH9329 번역기에 1바이트씩 주입합니다.
     */
    int i;
    for (i = 0; i < 8; i++) {
        ch9329_parse_byte(data[i]);
    }
}

/*
 * ====================================================================
 * [수정] _timeout_handler 함수를 비활성화합니다.
 * ====================================================================
 */
static void _timeout_handler(void) {
	// _send_response(PH_PROTO_RESP_TIMEOUT_ERROR); // <-- 비활성화
}


int main(void) {
	//ph_debug_act_init();
	//ph_debug_uart_init();
	ph_outputs_init();
	ph_ps2_init();
	ph_usb_init();
	
    // [수정] CH9329 파서 초기화 코드를 ph_com_init 전에 추가합니다.
    ch_parser_state = CH_WAIT_HEADER_1;
    // [수정] CH9329 키보드 상태 초기화
    ch_kbd_modifier = 0;
    for(int i=0; i<6; i++) { ch_kbd_keycodes[i] = 0; }


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
