/*
  * for hardware configs
  *
  * date:	2010.9.8
  * autor:	Merinvs
  * modify:	James 2010.9.15,2010.9.20
  *
  */
#ifndef _HWCONFIG_H_
#define _HWCONFIG_H_

/*
  * board type
  */
#define BOARD_EB100 		0
#define BOARD_EP1 			1
#define BOARD_EP2 			2
#define BOARD_EP3 			3
#define BOARD_EP4 			4
#define BOARD_IMX375 		6
#define BOARD_EP1_4 		7

#define BOARD_ARGS	 BOARD_EP1_4
/*
  *display
  */
#define DISPLAY_6_600_800 	0
#define DISPLAY_5_600_800 	1
#define DISPLAY_8_1024_768	2
#define DISPLAY_9_1200_825 	3

#define DISPLAY_ARGS 	DISPLAY_9_1200_825
/*
  *controller
  */
#define CON_PVI 			0
#define CON_EPSON_GP 		1
#define CON_EPSON_M 		2

#define CON_ARGS 	CON_EPSON_M
/*
  * keyboard
  */
#define KEY_6_NTX 			0
#define KEY_5_PB360 		1
#define KEY_COOKIE  		2
#define KEY_9D7LINEAR 		3
#define KEY_FOXCONN 		4

#define KEY_ARGS KEY_FOXCONN
/*
  * display orientation
  */
#define ORIEN_TOPDOWN 			1

#define ORIEN_ARGS	ORIEN_TOPDOWN
/*
  * g-sensor
  */
#define GSENSOR_NONE 			0
#define GSENSOR_MMA7455_1  		1
#define GSENSOR_MMA7455_2 		2
#define GSENSOR_TS1013 			3
#define GSENSOR_ADXL345_9		4
#define GSENSOR_ADXL345_6		5

#ifdef HWCONFIG_EP3
	#define GSENSOR_ARGS 	GSENSOR_ADXL345_9
#endif
#ifdef HWCONFIG_EP4
	#define GSENSOR_ARGS 	GSENSOR_ADXL345_9
#endif
#ifdef HWCONFIG_EP1
	#define GSENSOR_ARGS 	GSENSOR_ADXL345_6
#endif
#ifdef HWCONFIG_EP2
	#define GSENSOR_ARGS 	GSENSOR_ADXL345_6
#endif


/*
  * audio
  */
#define AUDIO_NONE 			0
#define AUDIO_UDA1345 		1
#define AUDIO_ALC5623 		2
#define AUDIO_FOX 			3

#define AUDIO_ARGS	AUDIO_FOX
/*
  * USB
  */
#define USB_NONE  			0
#define USB_ISP1582 		1
#define USB_RTS5105 		2
#define USB_INTERNAL 		3

#define USB_ARGS	USB_INTERNAL

/*
  * touchscreen
  */
#define TOUCH_NONE 			0
#define TOUCH_S3C_RESIST	1
#define TOUCH_FOX 			2

#define TOUCH_ARGS	TOUCH_FOX
/*
  * bluetooth module
  */
#define BLUETOOTH_NONE 		0
#define BLUETOOTH_WG7210 	1 
#define BLUETOOTH_FOX 		2

#define BLUETOOTH_ARGS	BLUETOOTH_FOX
/*
  *WiFi module
  */
#define WIFI_NONE			0
#define WIFI_WG7210			1
#define WIFI_GM9601 		2
#define WIFI_FOX			3

#define WIFI_ARGS	WIFI_FOX
/*
  *usb host
  */


/*
  *captouch
  */
#define CAP_TOUCH_NONE 		0
#define CAP_TOUCH_AD7147_1  1

#define CAP_TOUCH_ARGS	CAP_TOUCH_NONE
#ifdef HWCONFIG_EP2
	#define CAP_TOUCH_ARGS	CAP_TOUCH_NONE
#endif
#ifdef HWCONFIG_EP4
	#define CAP_TOUCH_ARGS	CAP_TOUCH_NONE
#endif
#ifdef HWCONFIG_EP1
	#define CAP_TOUCH_ARGS	CAP_TOUCH_AD7147_1
#endif
#ifdef HWCONFIG_EP3
	#define CAP_TOUCH_ARGS	CAP_TOUCH_AD7147_1
#endif
/*
  *cdma/3g
  */
#define CDMA_NONE 			0
#define CDMA_FOX 			1

#define CDMA_ARGS		CDMA_NONE
#ifdef HWCONFIG_EP2
	#define CDMA_ARGS		CDMA_NONE
#endif
#ifdef HWCONFIG_EP4
	#define CDMA_ARGS		CDMA_NONE
#endif
#ifdef HWCONFIG_EP1
	#define CDMA_ARGS		CDMA_FOX
#endif
#ifdef HWCONFIG_EP3
	#define CDMA_ARGS		CDMA_FOX
#endif
#if 0
#define HW_CONFIG 	(unsigned long long)((BOARD_ARGS<<0)|(DISPLAY_ARGS<<4)|(CON_ARGS<<8)|(KEY_ARGS<<12)|(ORIEN_ARGS<<16)|\
						(GSENSOR_ARGS<<20)|(AUDIO_ARGS<<24)|(USB_ARGS<<28)|(TOUCH_ARGS<<32)|(BLUETOOTH_ARGS<<36)|\
							(WIFI_ARGS<<40)|(CAP_TOUCH_ARGS<<52)|(CDMA_ARGS<<56))
#endif

#define HW_CONFIG_L 	(unsigned long long)((BOARD_ARGS<<0)|(DISPLAY_ARGS<<4)|(CON_ARGS<<8)|(KEY_ARGS<<12)|(ORIEN_ARGS<<16)|\
							(GSENSOR_ARGS<<20)|(AUDIO_ARGS<<24)|(USB_ARGS<<28))

#define HW_CONFIG_H		(unsigned long long)((TOUCH_ARGS<<0)|(BLUETOOTH_ARGS<<4)|\
								(WIFI_ARGS<<8)|(CAP_TOUCH_ARGS<<20)|(CDMA_ARGS<<24))
#define PLATFORM "epx"
#ifdef HWCONFIG_EP1
	#define PLATFORM "ep1"
#endif
#ifdef HWCONFIG_EP2
	#define PLATFORM "ep2"
#endif
#ifdef HWCONFIG_EP3
	#define PLATFORM "ep3"
#endif
#ifdef HWCONFIG_EP4
	#define PLATFORM "ep4"
#endif						
#endif
