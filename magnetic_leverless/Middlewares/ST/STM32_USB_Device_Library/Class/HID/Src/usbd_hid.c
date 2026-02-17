/* usbd_hid.c */
#include "usbd_hid.h"
#include "usbd_ctlreq.h"
#include "usbd_core.h"
#include "main.h"

extern void WebHID_DataReceived_Callback(uint8_t* data, uint32_t len);
uint8_t WebHID_RxBuffer[64]; 

static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t *USBD_HID_GetCfgDesc(uint16_t *length);
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum); // ★追加

// クラス構造体に DataOut を登録 (これが重要！)
USBD_ClassTypeDef USBD_HID = {
    USBD_HID_Init,
    USBD_HID_DeInit,
    USBD_HID_Setup,
    NULL, /* EP0_TxSent */
    NULL, /* EP0_RxReady */
    USBD_HID_DataIn,
    USBD_HID_DataOut, // ★ここに追加
    NULL, /* SOF */
    NULL,
    NULL,
    USBD_HID_GetCfgDesc,
    USBD_HID_GetCfgDesc,
    USBD_HID_GetCfgDesc,
    NULL,
};

// --- レポート記述子 (Config Mode用) ---
__ALIGN_BEGIN static uint8_t HID_ReportDesc_Config[] __ALIGN_END = {
    0x06, 0x60, 0xFF, // Usage Page (Vendor Specific)
    0x09, 0x01,       // Usage (Vendor Usage 1)
    0xA1, 0x01,       // Collection (Application)
    
    // Input (Device -> PC) [EP1 IN]
    0x09, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x40, 0x81, 0x02, 
    
    // Output (PC -> Device) [EP2 OUT] ★Feature(0xB1)からOutput(0x91)に変更
    0x09, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x40, 0x91, 0x02, 
    
    0xC0              // End Collection
};

// --- 構成記述子 (Config Mode用) ---
#define CONFIG_DESC_SIZE_WEB 41
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc_Web[CONFIG_DESC_SIZE_WEB] __ALIGN_END = {
    0x09, 0x02, CONFIG_DESC_SIZE_WEB, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x32,
    // Interface 0: WebHID (Generic HID)
    0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, sizeof(HID_ReportDesc_Config), 0x00,
    0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x01, // EP1 IN (64 bytes)
    0x07, 0x05, 0x02, 0x03, 0x40, 0x00, 0x01  // EP2 OUT (64 bytes)
};

// --- 構成記述子 (Xbox Mode用) ---
#define CONFIG_DESC_SIZE_XBOX 48
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc_Xbox[CONFIG_DESC_SIZE_XBOX] __ALIGN_END = {
    0x09, 0x02, CONFIG_DESC_SIZE_XBOX, 0x00, 0x01, 0x01, 0x00, 0xA0, 0xFA, 
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x5D, 0x01, 0x00, 
    0x10, 0x21, 0x10, 0x01, 0x01, 0x24, 0x81, 0x14, 0x03, 0x00, 0x03, 0x13, 0x02, 0x00, 0x03, 0x00,
    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x01, 
    0x07, 0x05, 0x02, 0x03, 0x20, 0x00, 0x08  
};

static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    uint16_t mps = (g_boot_mode == 0) ? 32 : 64;

    USBD_LL_OpenEP(pdev, 0x81, USBD_EP_TYPE_INTR, mps);
    USBD_LL_OpenEP(pdev, 0x02, USBD_EP_TYPE_INTR, mps);

    pdev->pClassData = USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

    // ★重要: Config Modeなら、すぐに受信待ち状態にする
    if (g_boot_mode == 1) {
        USBD_LL_PrepareReceive(pdev, 0x02, WebHID_RxBuffer, 64);
    }

    return USBD_OK;
}

static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
    USBD_LL_CloseEP(pdev, 0x81); 
    USBD_LL_CloseEP(pdev, 0x02);
    if(pdev->pClassData != NULL) USBD_free(pdev->pClassData);
    return USBD_OK;
}

static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
    uint16_t len = 0; uint8_t *pbuf = NULL;
    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
        case USB_REQ_TYPE_STANDARD:
            if (req->bRequest == USB_REQ_GET_DESCRIPTOR) {
                if (g_boot_mode == 1 && (req->wValue >> 8) == 0x22) {
                    pbuf = HID_ReportDesc_Config;
                    len = sizeof(HID_ReportDesc_Config);
                }
            }
            break;
    }
    if (len > 0) { len = MIN(len, req->wLength); USBD_CtlSendData(pdev, pbuf, len); }
    return USBD_OK;
}

static uint8_t *USBD_HID_GetCfgDesc(uint16_t *length) {
    if (g_boot_mode == 0) {
        *length = sizeof(USBD_HID_CfgDesc_Xbox);
        return USBD_HID_CfgDesc_Xbox;
    } else {
        *length = sizeof(USBD_HID_CfgDesc_Web);
        return USBD_HID_CfgDesc_Web;
    }
}

// ★追加: データ受信完了時に呼ばれる関数 (EP2 OUT)
static uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    if (epnum == 0x02) {
        // 受信したデータを処理
        WebHID_DataReceived_Callback(WebHID_RxBuffer, 64);
        
        // 次のデータ受信に備える
        USBD_LL_PrepareReceive(pdev, 0x02, WebHID_RxBuffer, 64);
    }
    return USBD_OK;
}

static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
    return USBD_OK;
}

uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len) {
    return USBD_LL_Transmit(pdev, 0x81, report, len);
}

uint8_t USBD_WebHID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len) {
    return USBD_LL_Transmit(pdev, 0x81, report, len);
}