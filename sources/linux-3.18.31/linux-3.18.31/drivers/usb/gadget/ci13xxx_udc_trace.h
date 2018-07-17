#undef TRACE_SYSTEM
#define TRACE_SYSTEM ci13xxx_udc
#define TRACE_INCLUDE_FILE ci13xxx_udc_trace

#if !defined(_TRACE_CI13XXX_MSM) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_CI13XXX_MSM

#include <linux/usb/ch9.h>
#include <linux/tracepoint.h>

struct usb_ctrlrequest;

TRACE_EVENT(ci13xxx_udc_req,

	TP_PROTO(struct usb_ctrlrequest req),

	TP_ARGS(req),

	TP_STRUCT__entry(
		__field( u8, bRequestType )
		__field( u8, bRequest )
		__field( int16_t, wValue )
		__field( int16_t, wIndex )
		__field( int16_t, wLength )
	),

	TP_fast_assign(
		__entry->bRequestType	= req.bRequestType;
		__entry->bRequest	= req.bRequest;
		__entry->wValue		= le16_to_cpu(req.wValue);
		__entry->wIndex		= le16_to_cpu(req.wIndex);
		__entry->wLength	= le16_to_cpu(req.wLength);
	),

	TP_printk("bRequest=%s (%x) bRequestType=%x (%s|%s) wValue=%x wIndex=%x wLength=%d",
		__print_symbolic(__entry->bRequest,
				{ 0x00,"USB_REQ_GET_STATUS" },
				{ 0x01,"USB_REQ_CLEAR_FEATURE" },
				{ 0x03,"USB_REQ_SET_FEATURE" },
				{ 0x05,"USB_REQ_SET_ADDRESS" },
				{ 0x06,"USB_REQ_GET_DESCRIPTOR" },
				{ 0x07,"USB_REQ_SET_DESCRIPTOR" },
				{ 0x08,"USB_REQ_GET_CONFIGURATION" },
				{ 0x09,"USB_REQ_SET_CONFIGURATION" },
				{ 0x0A,"USB_REQ_GET_INTERFACE" },
				{ 0x0B,"USB_REQ_SET_INTERFACE" },
				{ 0x0C,"USB_REQ_SYNCH_FRAME" },
				{ 0x30,"USB_REQ_SET_SEL" },
				{ 0x31,"USB_REQ_SET_ISOCH_DELAY" }),
		__entry->bRequest,

		__entry->bRequestType,
		(__entry->bRequestType & USB_DIR_IN) ? "I" : "O",
		(__entry->bRequestType & USB_TYPE_MASK) ?
		__print_flags(__entry->bRequestType & USB_TYPE_MASK, "|",
				{ 0x01,"C" }, { 0x02,"V" }, { 0x03,"R" }) : "S",

		__entry->wValue, __entry->wIndex, __entry->wLength)
);

#endif /* _TRACE_CI13XXX_MSM */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#include <trace/define_trace.h>
