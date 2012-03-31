#ifndef _PANTECH_LOOPBACK_H_
#define _PANTECH_LOOPBACK_H_

#define LOOPBACK_BUF_SIZE 64*1024
#define SKT_USB_CMD_LOOPBACK_APP 0xD0
#define SKT_USB_CMD_PORT_CONTROL 0xD1

typedef struct {
  uint16_t  run;  //SKT_USB_CMD_LOOPBACK_APP-> wValue off(0), on(1) 
  uint16_t  open; //SKT_USB_CMD_PORT_CONTROL->wValue close(0), open(1)
  uint16_t  type; //SKT_USB_CMD_PORT_CONTROL->wIndex modem(0), diag(1), obex(2), max(3)
} usb_test_cmd_type;

char *pantech_usb_test_get_loopback_buffer(void);
uint16_t pantech_usb_test_get_run(void);
uint16_t pantech_usb_test_get_open(void);
uint16_t pantech_usb_test_get_port_type(void);
uint32_t pantech_usb_test_copy_buffer(char *buffer, int length);
void pantech_usb_test_send_loopback(void *context, void *endpoint); 
void pantech_usb_test_bind_loopback(void *endpoint);

extern void obex_loopback_open(void);
extern void obex_loopback_close(void);
#endif
