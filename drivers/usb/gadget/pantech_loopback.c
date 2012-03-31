#if defined(CONFIG_HSUSB_PANTECH_USB_TEST)

#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#include "pantech_loopback.h"

usb_test_cmd_type loopback_usb_test_cmd;

static char *loopback_buffer = NULL;
static struct usb_request *loopback_usb_request = NULL;
static int loopback_buffer_send_count = 0; 
static int loopback_buffer_bulk_count;
static int loopback_buffer_sended_bulk_count;

char *pantech_usb_test_get_loopback_buffer(void)
{
	return loopback_buffer;
}
EXPORT_SYMBOL(pantech_usb_test_get_loopback_buffer);

uint16_t pantech_usb_test_get_run(void)
{
	return loopback_usb_test_cmd.run;
}
EXPORT_SYMBOL(pantech_usb_test_get_run);

uint16_t pantech_usb_test_get_open(void)
{
	return loopback_usb_test_cmd.open;
}
EXPORT_SYMBOL(pantech_usb_test_get_open);

uint16_t pantech_usb_test_get_port_type(void)
{
	return loopback_usb_test_cmd.type;
}
EXPORT_SYMBOL(pantech_usb_test_get_port_type);

uint32_t pantech_usb_test_copy_buffer(char *buffer, int length)
{
	int count;

	if(!buffer || !loopback_buffer || (loopback_buffer_send_count >= LOOPBACK_BUF_SIZE))	return 0;

	count = loopback_buffer_send_count +length;

	if((loopback_buffer_send_count + length) > LOOPBACK_BUF_SIZE){
		count = LOOPBACK_BUF_SIZE - loopback_buffer_send_count;
	} else {
		count = length;
	}
	
	memcpy(&loopback_buffer[loopback_buffer_send_count], buffer, count);

	loopback_buffer_send_count += count;
	return count;
}
EXPORT_SYMBOL(pantech_usb_test_copy_buffer);

static void pantech_usb_test_null(struct usb_ep *ep, struct usb_request *req)
{
	//printk("sended 0 length packed\n");
}

static void pantech_usb_test_loopback_in_complete(struct usb_ep *ept, struct usb_request *req)
{
	if(req->status == 0)
		loopback_buffer_sended_bulk_count++;
	else printk("oops!!! SEND ERROR\n");

	if(loopback_buffer_sended_bulk_count >= loopback_buffer_bulk_count) {
		//printk("usb_test_loopback send complete[%d]\n", loopback_buffer_send_count);

		/* send 0-length packet */
		if((loopback_buffer_send_count % ept->maxpacket) == 0) {
			loopback_usb_request->complete = pantech_usb_test_null;
			loopback_usb_request->length = 0;
			usb_ep_queue(ept, loopback_usb_request, GFP_ATOMIC);
		}

		loopback_buffer_send_count = 0;
		loopback_buffer_bulk_count = 0;
		loopback_buffer_sended_bulk_count = 0;
		loopback_usb_request->context = NULL;
		loopback_usb_request->length = 0;
		loopback_usb_request->buf = NULL;

	} else if((loopback_buffer_bulk_count - 1) == loopback_buffer_sended_bulk_count) {
		if(loopback_buffer_send_count % 4096 == 0) {
			loopback_usb_request->length = 4096;
		} else {
			loopback_usb_request->length = loopback_buffer_send_count % 4096;
		}
		loopback_usb_request->buf = &loopback_buffer[loopback_buffer_sended_bulk_count * 4096];
		usb_ep_queue(ept, loopback_usb_request, GFP_ATOMIC);
	} else  {
		loopback_usb_request->buf = &loopback_buffer[loopback_buffer_sended_bulk_count * 4096];
		usb_ep_queue(ept, loopback_usb_request, GFP_ATOMIC);
	}
}

void pantech_usb_test_send_loopback(void *context, void *endpoint) 
{
	struct usb_ep *ept = (struct usb_ep *)endpoint;

	if(!context || !ept) return;


	if(!loopback_usb_request) {
		printk("loopback_usb_request is null\n");
	}

	if(!loopback_buffer) {
		printk("loopback_buffer is null\n");
	}

	//loopback_buffer_send_count = length;
	loopback_buffer_bulk_count = (loopback_buffer_send_count % 4096 == 0)? loopback_buffer_send_count/4096 : loopback_buffer_send_count/4096 + 1;
	loopback_buffer_sended_bulk_count = 0;

	if(loopback_buffer_bulk_count == 0) {
		printk("usb_test_loopback bulk count == 0\n");
		return;
	}

	//printk("pantech_usb_test_send request[%d]\n", loopback_buffer_send_count);

	loopback_usb_request->context = context;

	if(loopback_buffer_bulk_count == 1) {
		if(loopback_buffer_send_count % 4096 == 0) {
			loopback_usb_request->length = 4096;
		} else {
			loopback_usb_request->length = loopback_buffer_send_count;
		}
	} else {
		loopback_usb_request->length = 4096;
	}

	loopback_usb_request->buf = loopback_buffer;
	loopback_usb_request->complete = pantech_usb_test_loopback_in_complete;

	usb_ep_queue(ept, loopback_usb_request, GFP_ATOMIC);
}
EXPORT_SYMBOL(pantech_usb_test_send_loopback);


void pantech_usb_test_bind_loopback(void *endpoint)
{
	struct usb_ep *ep	= (struct usb_ep *)endpoint;
	if(!ep) return;

	if(!loopback_usb_request) {
		loopback_usb_request = usb_ep_alloc_request(ep, GFP_ATOMIC);
		if(!loopback_usb_request) {
			printk("error bind loopback\n");
		}
	}
	
	if(!loopback_buffer) {
		loopback_buffer = kmalloc(LOOPBACK_BUF_SIZE, GFP_ATOMIC);
		if(!loopback_buffer) {
			printk("error allocation loopback buffer\n");
		} else {
			printk("success allocation loopback buffer\n");
		}
	}	

	loopback_usb_test_cmd.run = 0;
	loopback_usb_test_cmd.open = 0;
	loopback_usb_test_cmd.type = 0xff;
}

#endif/*CONFIG_HSUSB_PANTECH_USB_TEST*/
