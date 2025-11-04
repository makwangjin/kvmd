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
#include "ph_cmds.h" // <--- [검증] ph_cmd_... 함수들이 이 헤더에 정의되어 있습니다.
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

/*
 * [링커 오류 수정]
 * ph_hid_... 함수 대신, main.c가 포함하는 "ph_cmds.h"에 정의된
 * PiKVM의 중간 레벨 함수를 호출합니다.
 */
static void ch9329_process_packet()
{
    if (ch_packet.cmd == CH_CMD_KEYBOARD && ch_packet.len == CH_LEN_KEYBOARD)
    {
        /*
         * PiKVM V3의 키보드 함수(ph_cmd_kbd_send_key)는
         * CH9329의 6-key 롤오버 방식과 호환되지 않습니다.
         * 대신 ph_usb_kbd_report 함수를 직접 호출해야 합니다.
         * (ph_usb.h는 main.c에 포함되어 있습니다.)
         */
        u8 modifier = ch_packet.data[0];
        u8 keycodes[6];
        keycodes[0] = ch_packet.data[2];
        keycodes[1] = ch_packet.data[3];
        keycodes[2] = ch_packet.data[4];
        keycodes[3] = ch_packet.data[5];
        keycodes[4] = ch_packet.data[6];
        keycodes[5] = ch_packet.data[7];
        
        ph_usb_kbd_report(modifier, keycodes); // ph_usb.h에 정의됨
    }
    else if (ch_packet.cmd == CH_CMD_MOUSE && ch_packet.len == CH_LEN_MOUSE)
    {
        /*
         * PiKVM V3의 마우스 함수(ph_cmd_mouse_send_rel 등)를 호출합니다.
         */
        u8 buttons = ch_packet.data[0];
        s8 x = (s8)ch_packet.data[1];
        s8 y = (s8)ch_packet.data[2];
        s8 wheel = (s8)ch_packet.data[3];
        
        // 버튼 상태 전송 (ph_cmds.h에 정의됨)
        u8 button_args[2];
        // CH9329 (Bit 0:L, 1:R, 2:M) -> PiKVM (Bit 0:L, 1:R, 2:M)
        button_args[0] = (buttons & 0x07); 
        // CH9329 (Bit 3:B4, 4:B5) -> PiKVM (Bit 0:B4, 1:B5)
        button_args[1] = ((buttons >> 3) & 0x03); 
        // PiKVM 함수 호출 (ph_cmds.h에 정의됨)
        ph_cmd_mouse_send_button(button_args);

        // 상대 좌표 이동 (ph_cmds.h에 정의됨)
        u8 rel_args[2];
        rel_args[0] = (u8)x;
        rel_args[1] = (u8)y;
        ph_cmd_mouse_send_rel(rel_args);
        
        // 휠 스크롤 (ph_cmds.h에 정의됨)
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

    /*
     * CH340(control3)은 응답을 기다리지 않으므로,
     * _send_response() 함수를 호출하지 않습니다.
     */
}

/*
 * ====================================================================
 * [수정] _timeout_handler 함수를 비활성화합니다.
 * ====================================================================
 */
static void _timeout_handler(void) {
	// _send_response(PH_PROTO_RESP_TIMEOUT_ERROR); // <-- 비활성화
    /*
     * CH9329 프로토콜은 타임아웃 개념을 사용하지 않으므로,
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
