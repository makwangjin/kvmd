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


/*
 * ====================================================================
 * [수정] 원본 PiKVM 헤더파일을 모두 포함합니다.
 * ====================================================================
 */
#include "ph_com_uart.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "ph_types.h" // u8, s8 타입을 사용하기 위해 포함
#include "ph_hid.h"   // KVM 엔진(키보드/마우스 함수)을 불러옵니다.


/*
 * ====================================================================
 * CH9329 시리얼 프로토콜 번역기 코드 (사용자 추가 부분)
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
        // PiKVM의 *검증된* 키보드 함수 호출
        hid_keyboard_report(modifier, keycodes);
    }
    else if (ch_packet.cmd == CH_CMD_MOUSE && ch_packet.len == CH_LEN_MOUSE)
    {
        // u8, s8 타입 사용
        u8 buttons = ch_packet.data[0];
        s8 x = (s8)ch_packet.data[1];
        s8 y = (s8)ch_packet.data[2];
        s8 wheel = (s8)ch_packet.data[3];
        // PiKVM의 *검증된* 마우스 함수 호출
        hid_mouse_report(buttons, x, y, wheel);
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

// --- PiKVM V3의 기존 코드 시작 ---

#define _BUS		uart1
#define _SPEED		115200
#define _RX_PIN		21
#define _TX_PIN		20
#define _TIMEOUT_US	100000


// --- 이 변수들은 더 이상 사용되지 않으므로 삭제하지 않습니다. ---
static u8 _buf[8] = {0};
static u8 _index = 0;
static u64 _last_ts = 0;
static void (*_data_cb)(const u8 *) = NULL;
static void (*_timeout_cb)(void) = NULL;
// -------------------------------------------------------------------


/*
 * ====================================================================
 * [수정] PiKVM V3의 init 함수를 하이재킹합니다.
 * ====================================================================
 */
void ph_com_uart_init(void (*data_cb)(const u8 *), void (*timeout_cb)(void)) {
    // CH9329 번역기를 초기화합니다.
    ch_parser_state = CH_WAIT_HEADER_1;
	
    // 펌웨어에 정의된 핀(GP20, GP21)으로 UART를 초기화합니다.
    uart_init(_BUS, _SPEED);
	gpio_set_function(_RX_PIN, GPIO_FUNC_UART);
	gpio_set_function(_TX_PIN, GPIO_FUNC_UART);
}


/*
 * ====================================================================
 * [수정] PiKVM V3의 task 함수를 하이재킹합니다.
 * ====================================================================
 */
void ph_com_uart_task(void) {
    // UART로 들어오는 모든 바이트를 CH9329 파서로 보냅니다.
    if (uart_is_readable(_BUS)) {
		u8 ch = (u8)uart_getc(_BUS); // u8 타입 사용
        ch9329_parse_byte(ch); // <--- 우리가 추가한 "번역기" 함수 호출
	}
}


/*
 * =G==================================================================
 * [수정 없음] 이 함수는 사용되지 않지만, 그대로 둡니다.
 * ====================================================================
 */
void ph_com_uart_write(const u8 *data) {
	uart_write_blocking(_BUS, data, 8);
}
