#if !defined( __SKY_USB_CHARGING_H__ )
#define __SKY_USB_CHARGING_H__

#define USB_CHARGING_MAGIC_NUMBER 0x20303419
#define USB_CHARGING_MUST_ENTER 0x34192030
#define USB_CHARGING_MUST_SKIP 0x85991899
#define USB_CHARGING_UNKNOWN 0x59120938

typedef struct
{
        unsigned int  magic_num;
        unsigned int  reset_reason;
}usb_charging_info_type;
#endif /* __SKY_USB_CHARGING_H__ */
