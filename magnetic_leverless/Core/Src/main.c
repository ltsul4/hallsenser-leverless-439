/* main.c */
#include "main.h"
#include "adc.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_hid.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// --- 定数定義 ---
#define KEY_COUNT 20
#define TRAVEL_DISTANCE_MM 3.2f
#define ADC_MAX 4095.0f
#define RAW_DEADZONE 60 
#define MAX_MACROS 8

// WebHID Commands
#define CMD_GET_CONFIG    0x10 
#define CMD_SET_CONFIG    0x20 
#define CMD_READ_SENSORS  0x30 
#define CMD_CALIBRATE     0x40 
#define CMD_SAVE_SETTINGS 0x50 
#define CMD_SET_SOCD      0x60 
#define CMD_SET_MAPPING   0x70 
#define CMD_SET_MACRO     0x80 

// Flash Memory
#define FLASH_STORAGE_ADDR 0x0801F800
#define FLASH_MAGIC_NUM    0xDEADBEE3 

// --- ボタンID定義 ---
#define BTN_NONE      0
#define BTN_UP        1
#define BTN_DOWN      2
#define BTN_LEFT      3
#define BTN_RIGHT     4
#define BTN_START     5
#define BTN_BACK      6
#define BTN_L3        7
#define BTN_R3        8
#define BTN_LB        9
#define BTN_RB        10
#define BTN_GUIDE     11
#define BTN_A         12
#define BTN_B         13
#define BTN_X         14
#define BTN_Y         15
#define BTN_LT        16 
#define BTN_RT        17 
// 50~57: Macro 1~8

// XInput Bitmask
#define XINPUT_DPAD_UP    0x0001
#define XINPUT_DPAD_DOWN  0x0002
#define XINPUT_DPAD_LEFT  0x0004
#define XINPUT_DPAD_RIGHT 0x0008
#define XINPUT_START      0x0010
#define XINPUT_BACK       0x0020
#define XINPUT_LB         0x0100
#define XINPUT_RB         0x0200
#define XINPUT_BTN_A      0x1000
#define XINPUT_BTN_B      0x2000
#define XINPUT_BTN_X      0x4000
#define XINPUT_BTN_Y      0x8000
#define XINPUT_THUMB_L    0x0040
#define XINPUT_THUMB_R    0x0080
#define XINPUT_GUIDE      0x0400

// --- キーマップ (デフォルト) ---
uint8_t key_map[KEY_COUNT] = {
    BTN_RIGHT, BTN_DOWN, BTN_LEFT, BTN_R3, BTN_L3, BTN_GUIDE, BTN_BACK, BTN_START, 
    BTN_RB,    BTN_LB,   BTN_Y,    BTN_X,  BTN_R3, BTN_A,     BTN_B,    BTN_LT, 
    BTN_RT,    BTN_L3,   BTN_UP,   BTN_R3
};

// --- マクロ定義 ---
uint32_t custom_macros[MAX_MACROS] = {0};

// --- 構造体 ---
typedef struct {
    uint16_t resting_val;
    uint16_t max_diff;
    float current_mm;
    float anchor_mm;
    bool is_pressed;
    float actuation_point;
    float rt_press;
    float rt_release;
    float top_deadzone;
    float bottom_deadzone;
} KeyState_t;

// Flash保存用構造体
typedef struct {
    uint32_t magic;
    float ap[KEY_COUNT];
    float rt[KEY_COUNT];
    float top_dz[KEY_COUNT];
    float bottom_dz[KEY_COUNT];
    uint8_t socd_mode;
    uint8_t key_map[KEY_COUNT];
    uint32_t macros[MAX_MACROS];
    uint8_t reserved[3];
} FlashConfig_t;

typedef struct __attribute__((packed)) {
    uint8_t msg_type; uint8_t pkt_size; 
    uint16_t buttons; uint8_t lt; uint8_t rt;
    int16_t lx; int16_t ly; int16_t rx; int16_t ry; 
    uint8_t reserved[6]; 
} XInputReport_t;

// --- グローバル変数 ---
extern USBD_HandleTypeDef hUsbDeviceFS;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

uint8_t g_boot_mode = 0; 
volatile bool g_req_calibrate = false;
volatile bool g_req_save = false;

uint8_t g_socd_mode = 0; 

// Last Win用
bool last_left_state = false;
bool last_right_state = false;
bool last_up_state = false;
bool last_down_state = false;
int  last_lr_winner = 0; 
int  last_ud_winner = 0; 

uint16_t adc_raw_values[KEY_COUNT];
KeyState_t key_states[KEY_COUNT];
uint8_t webhid_tx_buf[64];

// --- プロトタイプ宣言 ---
void SystemClock_Config(void);
void Error_Handler(void);
void MUX1_Select(uint8_t ch);
void MUX2_Select(uint8_t ch);
void scan_all_keys_fast(uint16_t *buffer);
void Process_Rapid_Trigger(int key_idx, uint16_t raw_val);
void Send_Reports(void);
void Save_Config(void);
void Load_Config(void);
extern uint8_t USBD_WebHID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);
extern uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);

// --- 初期化 ---
void Setup_Keys(void) {
    HAL_Delay(200);
    scan_all_keys_fast(adc_raw_values);
    HAL_Delay(50);
    scan_all_keys_fast(adc_raw_values);

    for(int i=0; i<KEY_COUNT; i++) {
        key_states[i].resting_val = adc_raw_values[i];
        key_states[i].max_diff = 200; 
        key_states[i].actuation_point = 1.2f;
        key_states[i].rt_press = 0.5f;
        key_states[i].rt_release = 0.5f;
        key_states[i].top_deadzone = 0.2f;
        key_states[i].bottom_deadzone = 0.1f;
        key_states[i].is_pressed = false;
        key_states[i].anchor_mm = 0.0f;
        key_states[i].current_mm = 0.0f;
    }
    g_socd_mode = 0; 
    memset(custom_macros, 0, sizeof(custom_macros));
    
    uint8_t default_map[KEY_COUNT] = {
        BTN_RIGHT, BTN_DOWN, BTN_LEFT, BTN_R3, BTN_L3, BTN_GUIDE, BTN_BACK, BTN_START, 
        BTN_RB,    BTN_LB,   BTN_Y,    BTN_X,  BTN_R3, BTN_A,     BTN_B,    BTN_LT, 
        BTN_RT,    BTN_L3,   BTN_UP,   BTN_R3
    };
    memcpy(key_map, default_map, KEY_COUNT);
}

// --- Flash 保存/読み込み ---
void Save_Config(void) {
    FlashConfig_t cfg;
    cfg.magic = FLASH_MAGIC_NUM;
    cfg.socd_mode = g_socd_mode;
    for(int i=0; i<KEY_COUNT; i++) {
        cfg.ap[i] = key_states[i].actuation_point;
        cfg.rt[i] = key_states[i].rt_press;
        cfg.top_dz[i] = key_states[i].top_deadzone;
        cfg.bottom_dz[i] = key_states[i].bottom_deadzone;
        cfg.key_map[i] = key_map[i];
    }
    for(int i=0; i<MAX_MACROS; i++) {
        cfg.macros[i] = custom_macros[i];
    }

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_STORAGE_ADDR;
    EraseInitStruct.NbPages = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        uint32_t *pData = (uint32_t*)&cfg;
        for (int i = 0; i < sizeof(FlashConfig_t) / 4; i++) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR + (i * 4), pData[i]);
        }
    }
    HAL_FLASH_Lock();
}

void Load_Config(void) {
    FlashConfig_t *pCfg = (FlashConfig_t*)FLASH_STORAGE_ADDR;
    if (pCfg->magic == FLASH_MAGIC_NUM) {
        g_socd_mode = pCfg->socd_mode;
        for(int i=0; i<KEY_COUNT; i++) {
            key_states[i].actuation_point = pCfg->ap[i];
            key_states[i].rt_press = pCfg->rt[i];
            key_states[i].rt_release = pCfg->rt[i];
            key_states[i].top_deadzone = pCfg->top_dz[i];
            key_states[i].bottom_deadzone = pCfg->bottom_dz[i];
            key_map[i] = pCfg->key_map[i];
        }
        for(int i=0; i<MAX_MACROS; i++) {
            custom_macros[i] = pCfg->macros[i];
        }
    }
}

// --- MUX制御 ---
void short_delay(void) { for(volatile int i = 0; i < 30; i++) { __NOP(); } }
void MUX1_Select(uint8_t ch) {
    HAL_GPIO_WritePin(MUX1_A_GPIO_Port, MUX1_A_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX1_B_GPIO_Port, MUX1_B_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX1_C_GPIO_Port, MUX1_C_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
void MUX2_Select(uint8_t ch) {
    HAL_GPIO_WritePin(MUX2_A_GPIO_Port, MUX2_A_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX2_B_GPIO_Port, MUX2_B_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX2_C_GPIO_Port, MUX2_C_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// --- ADCスキャン ---
void scan_all_keys_fast(uint16_t *buffer) {
    for (int ch = 0; ch < 8; ch++) {
        MUX2_Select(ch); MUX1_Select(ch); short_delay(); 
        HAL_ADC_Start(&hadc4); HAL_ADC_Start(&hadc3); HAL_ADC_Start(&hadc2);
        if (HAL_ADC_PollForConversion(&hadc4, 1) == HAL_OK) buffer[ch] = HAL_ADC_GetValue(&hadc4);
        if (HAL_ADC_PollForConversion(&hadc3, 1) == HAL_OK) {
            uint16_t val = HAL_ADC_GetValue(&hadc3);
            if (ch < 4) buffer[ch + 8] = val; else if (ch == 7) buffer[12] = val; 
        }
        if (HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK) {
            uint16_t val = HAL_ADC_GetValue(&hadc2);
            switch(ch) { 
                case 7: buffer[13]=val; break; case 6: buffer[14]=val; break; 
                case 5: buffer[15]=val; break; case 4: buffer[16]=val; break; 
                case 3: buffer[17]=val; break; case 2: buffer[18]=val; break; 
                case 1: buffer[19]=val; break; 
            }
        }
    }
}

// --- ラピッドトリガー ---
void Process_Rapid_Trigger(int key_idx, uint16_t raw) {
    KeyState_t *k = &key_states[key_idx];
    int diff = (int)raw - (int)k->resting_val;
    uint16_t abs_diff = abs(diff);

    if (abs_diff < RAW_DEADZONE) abs_diff = 0;
    if (abs_diff > k->max_diff) k->max_diff = abs_diff;

    float norm = (float)abs_diff / (float)k->max_diff;
    if (norm > 1.0f) norm = 1.0f;
    
    float pos = norm * TRAVEL_DISTANCE_MM;

    if (pos < k->top_deadzone) pos = 0.0f;
    k->current_mm = pos;

    bool in_bottom_zone = (pos > (TRAVEL_DISTANCE_MM - k->bottom_deadzone));

    if (k->is_pressed) {
        if (pos < (k->anchor_mm - k->rt_release)) {
            if (!in_bottom_zone) { k->is_pressed = false; k->anchor_mm = pos; }
            else { k->anchor_mm = pos; }
        } 
        else if (pos > k->anchor_mm) { k->anchor_mm = pos; }
        if (pos <= 0.1f) k->is_pressed = false; 
    } else {
        if (pos > (k->anchor_mm + k->rt_press) && pos >= k->actuation_point) { k->is_pressed = true; k->anchor_mm = pos; } 
        else if (pos < k->anchor_mm) { k->anchor_mm = pos; }
    }
}

// --- レポート送信 (マクロ修正版) ---
void Send_Reports(void) {
    static XInputReport_t prev_report = {0};
    XInputReport_t report = {0};
    report.msg_type = 0x00;
    report.pkt_size = 0x14;

    uint16_t btns_mask = 0;
    uint8_t trigger_l = 0;
    uint8_t trigger_r = 0;
    bool up=false, down=false, left=false, right=false;

    for(int i = 0; i < KEY_COUNT; i++) {
        if (key_states[i].is_pressed) {
            uint8_t id = key_map[i];
            
            // マクロ処理 (ID 50-57)
            if (id >= 50 && id < 50 + MAX_MACROS) {
                uint32_t m = custom_macros[id - 50];
                // 下位16bitがボタンマスク
                uint16_t m_btns = (m & 0xFFFF);
                // 次の8bitがLT, その次がRT
                uint8_t m_lt = (m >> 8) & 0xFF; // ★修正: シフト量を修正
                uint8_t m_rt = (m >> 16) & 0xFF;

                // ★修正: ビットマスクを直接OR演算
                btns_mask |= m_btns;
                
                // トリガーは最大値を採用
                if (m_lt) trigger_l = 255;
                if (m_rt) trigger_r = 255;

                // 方向キーの状態を更新 (SOCD用)
                if (m_btns & XINPUT_DPAD_UP) up = true;
                if (m_btns & XINPUT_DPAD_DOWN) down = true;
                if (m_btns & XINPUT_DPAD_LEFT) left = true;
                if (m_btns & XINPUT_DPAD_RIGHT) right = true;
            }
            else {
                switch(id) {
                    case BTN_UP:    up=true; break;
                    case BTN_DOWN:  down=true; break;
                    case BTN_LEFT:  left=true; break;
                    case BTN_RIGHT: right=true; break;
                    case BTN_START: btns_mask |= XINPUT_START; break;
                    case BTN_BACK:  btns_mask |= XINPUT_BACK; break;
                    case BTN_L3:    btns_mask |= XINPUT_THUMB_L; break;
                    case BTN_R3:    btns_mask |= XINPUT_THUMB_R; break;
                    case BTN_LB:    btns_mask |= XINPUT_LB; break;
                    case BTN_RB:    btns_mask |= XINPUT_RB; break;
                    case BTN_GUIDE: btns_mask |= XINPUT_GUIDE; break;
                    case BTN_A:     btns_mask |= XINPUT_BTN_A; break;
                    case BTN_B:     btns_mask |= XINPUT_BTN_B; break;
                    case BTN_X:     btns_mask |= XINPUT_BTN_X; break;
                    case BTN_Y:     btns_mask |= XINPUT_BTN_Y; break;
                    case BTN_LT:    trigger_l = 255; break;
                    case BTN_RT:    trigger_r = 255; break;
                    default: break;
                }
            }
        }
    }

    // SOCD
    if (g_socd_mode == 2) {
        if (left && !last_left_state) last_lr_winner = -1;
        if (right && !last_right_state) last_lr_winner = 1;
        if (up && !last_up_state) last_ud_winner = 1;
        if (down && !last_down_state) last_ud_winner = -1;
        last_left_state = left; last_right_state = right;
        last_up_state = up; last_down_state = down;
    }

    if (left && right) {
        if (g_socd_mode == 0) { btns_mask &= ~(XINPUT_DPAD_LEFT | XINPUT_DPAD_RIGHT); }
        else if (g_socd_mode == 1) { btns_mask &= ~(XINPUT_DPAD_LEFT | XINPUT_DPAD_RIGHT); }
        else if (g_socd_mode == 2) { 
            if (last_lr_winner == -1) { btns_mask |= XINPUT_DPAD_LEFT; btns_mask &= ~XINPUT_DPAD_RIGHT; }
            else { btns_mask |= XINPUT_DPAD_RIGHT; btns_mask &= ~XINPUT_DPAD_LEFT; }
        }
    } else {
        if (left) btns_mask |= XINPUT_DPAD_LEFT;
        if (right) btns_mask |= XINPUT_DPAD_RIGHT;
    }
    
    if (up && down) { 
        if (g_socd_mode == 0) { btns_mask &= ~(XINPUT_DPAD_UP | XINPUT_DPAD_DOWN); }
        else if (g_socd_mode == 1) { btns_mask |= XINPUT_DPAD_UP; btns_mask &= ~XINPUT_DPAD_DOWN; } 
        else if (g_socd_mode == 2) { 
            if (last_ud_winner == 1) { btns_mask |= XINPUT_DPAD_UP; btns_mask &= ~XINPUT_DPAD_DOWN; }
            else { btns_mask |= XINPUT_DPAD_DOWN; btns_mask &= ~XINPUT_DPAD_UP; }
        }
    } else {
        if (up) btns_mask |= XINPUT_DPAD_UP;
        if (down) btns_mask |= XINPUT_DPAD_DOWN;
    }

    report.buttons = btns_mask;
    report.lt = trigger_l;
    report.rt = trigger_r;

    if (memcmp(&report, &prev_report, sizeof(XInputReport_t)) != 0) {
        if (USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report)) == USBD_OK) {
            prev_report = report;
        }
    }
}

// --- WebHID コールバック ---
void WebHID_DataReceived_Callback(uint8_t* data, uint32_t len) {
    uint8_t cmd = data[0]; 
    
    if (cmd == CMD_READ_SENSORS) {
        static uint8_t tx[64];
        memset(tx, 0, 64);
        tx[0] = CMD_READ_SENSORS;
        for(int i=0; i<KEY_COUNT; i++) {
            float val = key_states[i].current_mm / TRAVEL_DISTANCE_MM;
            if (val > 1.0f) val = 1.0f; else if (val < 0.0f) val = 0.0f;
            tx[i+1] = (uint8_t)(val * 255.0f);
        }
        for(int i=0; i<KEY_COUNT; i++) tx[i+21] = key_states[i].is_pressed ? 1 : 0;
        for(int i=0; i<KEY_COUNT; i++) {
            float val = key_states[i].anchor_mm / TRAVEL_DISTANCE_MM;
            if (val > 1.0f) val = 1.0f; else if (val < 0.0f) val = 0.0f;
            tx[i+41] = (uint8_t)(val * 255.0f);
        }
        USBD_WebHID_SendReport(&hUsbDeviceFS, tx, 64);
    }
    else if (cmd == CMD_GET_CONFIG) { 
        static uint8_t tx[64];
        memset(tx, 0, 64);
        tx[0] = CMD_GET_CONFIG;
        tx[1] = g_socd_mode;
        for(int i=0; i<KEY_COUNT; i++) tx[i+2] = (uint8_t)(key_states[i].actuation_point * 10.0f);
        for(int i=0; i<KEY_COUNT; i++) tx[i+22] = (uint8_t)(key_states[i].rt_press * 10.0f);
        for(int i=0; i<KEY_COUNT; i++) tx[i+42] = key_map[i];
        
        USBD_WebHID_SendReport(&hUsbDeviceFS, tx, 64);
    }
    else if (cmd == CMD_SET_CONFIG) {
        uint8_t id = data[1];
        if (id < KEY_COUNT) {
            float ap = (float)data[2] / 10.0f;
            if (ap < 0.1f) ap = 0.1f; if (ap > 3.0f) ap = 3.0f;
            key_states[id].actuation_point = ap;

            float rt = (float)data[3] / 10.0f;
            if (rt < 0.1f) rt = 0.1f; if (rt > 3.0f) rt = 3.0f;
            key_states[id].rt_press = rt; key_states[id].rt_release = rt;
            
            float top = (float)data[4] / 100.0f;
            float btm = (float)data[5] / 100.0f;
            key_states[id].top_deadzone = top;
            key_states[id].bottom_deadzone = btm;
        }
    }
    else if (cmd == CMD_SET_SOCD) { g_socd_mode = data[1]; }
    else if (cmd == CMD_SET_MAPPING) { 
        uint8_t id = data[1];
        uint8_t btn = data[2];
        if (id < KEY_COUNT) key_map[id] = btn;
    }
    else if (cmd == CMD_SET_MACRO) {
        uint8_t macro_id = data[1];
        if (macro_id < MAX_MACROS) {
            // ★修正: JS側でパックしたデータを正しく復元
            // JS: val = buttons | (LT << 8) | (RT << 16)
            // C:  custom_macros = val
            uint32_t val = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
            custom_macros[macro_id] = val;
        }
    }
    else if (cmd == CMD_CALIBRATE) { g_req_calibrate = true; }
    else if (cmd == CMD_SAVE_SETTINGS) { g_req_save = true; }
}

// --- Main ---
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_ADC4_Init();
    
    // 1. 仮の初期化 (モード判定用)
    // ここでは基準点はまだ確定させない
    HAL_Delay(200);
    scan_all_keys_fast(adc_raw_values);
    
    // 2. モード判定
    int diff_up_down = abs((int)adc_raw_values[18] - (int)adc_raw_values[1]);
    if (diff_up_down > 500) { 
        g_boot_mode = 1; // Config Mode
    } else {
        g_boot_mode = 0; // Xbox Mode
    }

    // 3. 設定読み込み
    Load_Config();

    // 4. USB接続開始
    MX_USB_DEVICE_Init();
    
    // ★修正: 設定モードの場合、指が離れるのを待ってから基準点を取る
    if (g_boot_mode == 1) {
        // UPボタンの値が安定する（指が離れる）まで待つ簡易ロジック
        // あるいは、Webツール接続後にキャリブレーションを必須とする
        
        // 今回は「起動時は変な値でもOK、Webツールでキャリブレーションする」という運用が確実ですが、
        // 少しでもマシにするなら、ここで少し待ってから再取得します。
        HAL_Delay(1000); // 1秒待つ (この間に指を離してもらう)
        Setup_Keys(); 
    } else {
        // Xboxモードなら即座に初期化
        Setup_Keys();
    }
    
    while (1) {
        if (g_req_calibrate) { HAL_Delay(100); Setup_Keys(); g_req_calibrate = false; }
        if (g_req_save) { Save_Config(); g_req_save = false; }

        scan_all_keys_fast(adc_raw_values);
        for(int i=0; i<KEY_COUNT; i++) Process_Rapid_Trigger(i, adc_raw_values[i]);

        if (g_boot_mode == 0) Send_Reports();
    }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2; 
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; 
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5; 
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}
void Error_Handler(void) { __disable_irq(); while (1) { } }