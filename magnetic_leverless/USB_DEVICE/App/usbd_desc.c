/* usbd_desc.c */
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"
#include "main.h" // g_boot_mode を参照するために必要

// Xbox 360 Mode
#define USBD_VID_XBOX     0x045E
#define USBD_PID_XBOX     0x028E

// Config Mode (ST Micro Generic HID)
#define USBD_VID_CONF     0x0483
#define USBD_PID_CONF     0x5751 
#define USBD_LANGID_STRING     1033
#define USBD_MANUFACTURER_STRING     "STM32 Community"
#define USBD_PRODUCT_STRING          "Magnetic Leverless"
#define USBD_CONFIGURATION_STRING    "Default Config"
#define USBD_INTERFACE_STRING        "XInput Interface"

static void Get_SerialNum(void);
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len);

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef FS_Desc = {
  USBD_FS_DeviceDescriptor,
  USBD_FS_LangIDStrDescriptor,
  USBD_FS_ManufacturerStrDescriptor,
  USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor,
  USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor
};

// Composite Device Descriptor (IAD)
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12,                       /* bLength */
  USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
  0x00, 0x02,                 /* bcdUSB */
  0x00,                       /* bDeviceClass (Configモード用に00に変更推奨) */
  0x00,                       /* bDeviceSubClass */
  0x00,                       /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,           /* bMaxPacketSize */
  LOBYTE(USBD_VID_XBOX), HIBYTE(USBD_VID_XBOX), /* デフォルト値 */
  LOBYTE(USBD_PID_XBOX), HIBYTE(USBD_PID_XBOX), /* デフォルト値 */
  0x00, 0x02,                 /* bcdDevice */
  USBD_IDX_MFC_STR,
  USBD_IDX_PRODUCT_STR,
  USBD_IDX_SERIAL_STR,
  USBD_MAX_NUM_CONFIGURATION
};

__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
     USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING, LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)
};

__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;
__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
  USB_SIZ_STRING_SERIAL, USB_DESC_TYPE_STRING,
};

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  
  if (g_boot_mode == 0) {
      // Xbox Mode
      USBD_FS_DeviceDesc[4] = 0xFF; // Class (Vendor Specific)
      USBD_FS_DeviceDesc[5] = 0xFF; // SubClass
      USBD_FS_DeviceDesc[6] = 0xFF; // Protocol
      
      USBD_FS_DeviceDesc[8] = LOBYTE(USBD_VID_XBOX);
      USBD_FS_DeviceDesc[9] = HIBYTE(USBD_VID_XBOX);
      USBD_FS_DeviceDesc[10] = LOBYTE(USBD_PID_XBOX);
      USBD_FS_DeviceDesc[11] = HIBYTE(USBD_PID_XBOX);
  } else {
      // Config Mode (Generic HID)
      USBD_FS_DeviceDesc[4] = 0x00; // Class (Defined at Interface level)
      USBD_FS_DeviceDesc[5] = 0x00;
      USBD_FS_DeviceDesc[6] = 0x00;

      USBD_FS_DeviceDesc[8] = LOBYTE(USBD_VID_CONF);
      USBD_FS_DeviceDesc[9] = HIBYTE(USBD_VID_CONF);
      USBD_FS_DeviceDesc[10] = LOBYTE(USBD_PID_CONF);
      USBD_FS_DeviceDesc[11] = HIBYTE(USBD_PID_CONF);
  }

  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  *length = sizeof(USBD_LangIDDesc); return USBD_LangIDDesc;
}
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_StrDesc, length); return USBD_StrDesc;
}
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length); return USBD_StrDesc;
}
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  *length = USB_SIZ_STRING_SERIAL; Get_SerialNum(); return (uint8_t *) USBD_StringSerial;
}
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING, USBD_StrDesc, length); return USBD_StrDesc;
}
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  USBD_GetString((uint8_t *)USBD_INTERFACE_STRING, USBD_StrDesc, length); return USBD_StrDesc;
}

static void Get_SerialNum(void) {
  uint32_t deviceserial0, deviceserial1, deviceserial2;
  deviceserial0 = *(uint32_t *) DEVICE_ID1;
  deviceserial1 = *(uint32_t *) DEVICE_ID2;
  deviceserial2 = *(uint32_t *) DEVICE_ID3;
  deviceserial0 += deviceserial2;
  if (deviceserial0 != 0) {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}

static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len) {
  uint8_t idx = 0;
  for (idx = 0; idx < len; idx++) {
    if (((value >> 28)) < 0xA) pbuf[2 * idx] = (value >> 28) + '0';
    else pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    value = value << 4;
    pbuf[2 * idx + 1] = 0;
  }
}