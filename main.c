/*
 * wiicc2vusb.c
 *
 * Created: 04.11.2023 16:02:22
 * Author : BH
 *
 * Using - https://github.com/gblargg/vusb-joystick
 *       - https://github.com/TarkanAl-Kazily/Wii_Classic_Controller/blob/master/Wii_Classic_Controller.ino
 *       - https://www.instructables.com/USB-Wii-Classic-Controller/
 *       - more
 */ 

#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "usbdrv/usbdrv.h"

#define TWI_FREQ 400000L
#include "twi/twi.h"

//-----------------------------------------------------------------------------
// Defines, Constants
//-----------------------------------------------------------------------------

#define true  1
#define false 0

#define CLASSIC_BYTE_COUNT    (6)
						      
#define I2C_PIN_SDA           (4)
#define I2C_PIN_SCL           (5)
						      
#define MASK_DOWN             (1 << 6)
#define MASK_LEFT             (1 << 1)
#define MASK_UP               (1 << 0)
#define MASK_RIGHT            (1 << 7)
#define MASK_SEL              (1 << 4)
#define MASK_HOME             (1 << 3)
#define MASK_START            (1 << 2)
#define MASK_Y                (1 << 5)
#define MASK_X                (1 << 3)
#define MASK_A                (1 << 4)
#define MASK_B                (1 << 6)
#define MASK_L                (1 << 5)
#define MASK_R                (1 << 1)
#define MASK_ZL               (1 << 7)
#define MASK_ZR               (1 << 2)

#define INITIAL_DELAY_VALUE   ( 100000UL )  // These values are "cycles" so not an exact time
#define RECONNECT_DELAY_VALUE ( 200000UL )  // These values are "cycles" so not an exact time 

#define LED_ON  PORTC &= ~1
#define LED_OFF PORTC |=  1

/*    Bit
  Byte  7 6 5 4 3 2 1 0
  0 RX<4:3> |  LX<5:0>
  1 RX<2:1> |  LY<5:0>
  2 RX<0>   |  LT<4:3>  |  RY<4:0>
  3 LT<2:0> |  RT<4:0>
  4 BDR BDD BLT B-  BH  B+  BRT Null
  5 BZL BB  BY  BA  BX  BZR BDL BDU
*/

typedef struct ClassicController 
{
  int8_t  LeftX;
  int8_t  LeftY;
  int8_t  RightX;
  int8_t  RightY;
  int8_t  LeftT;
  int8_t  RightT;
  uint8_t ButtonDown;
  uint8_t ButtonLeft;
  uint8_t ButtonUp;
  uint8_t ButtonRight;
  uint8_t ButtonSelect;
  uint8_t ButtonHome;
  uint8_t ButtonStart;
  uint8_t ButtonY;
  uint8_t ButtonX;
  uint8_t ButtonA;
  uint8_t ButtonB;
  uint8_t ButtonL;
  uint8_t ButtonR;
  uint8_t ButtonZL;
  uint8_t ButtonZR;
}ClassicController_t;

const uint8_t       CLASSIC_IDENT = 0x52;    // I2C address is 0x52

// X/Y joystick w/ 8-bit readings (-127 to +127), 16 digital buttons
PROGMEM const char usbHidReportDescriptor [USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
	0x05, 0x01,     // USAGE_PAGE (Generic Desktop)
	0x09, 0x05,     // USAGE (Game Pad)
	0xa1, 0x01,     // COLLECTION (Application)
	0x09, 0x01,     //   USAGE (Pointer)
	0xa1, 0x00,     //   COLLECTION (Physical)
	0x09, 0x30,     //     USAGE (X)
	0x09, 0x31,     //     USAGE (Y)
	0x15, 0x81,     //   LOGICAL_MINIMUM (-127)
	0x25, 0x7f,     //   LOGICAL_MAXIMUM (127)
	0x75, 0x08,     //   REPORT_SIZE (8)
	0x95, 0x02,     //   REPORT_COUNT (2)
	0x81, 0x02,     //   INPUT (Data,Var,Abs)
	0xc0,           // END_COLLECTION
	0x05, 0x09,     // USAGE_PAGE (Button)
	0x19, 0x01,     //   USAGE_MINIMUM (Button 1)
	0x29, 0x10,     //   USAGE_MAXIMUM (Button 16)
	0x15, 0x00,     //   LOGICAL_MINIMUM (0)
	0x25, 0x01,     //   LOGICAL_MAXIMUM (1)
	0x75, 0x01,     // REPORT_SIZE (1)
	0x95, 0x10,     // REPORT_COUNT (16)
	0x81, 0x02,     // INPUT (Data,Var,Abs)
	0xc0            // END_COLLECTION
};

//-----------------------------------------------------------------------------
// Local variables
//-----------------------------------------------------------------------------

// Report format: Y, X, buttons (up to 16)
static uint8_t main_ayReport    [4]; // current
static uint8_t main_ayReportOut [4]; // last sent over USB

ClassicController_t main_tClassicController;

uint8_t main_ayTwiBuffer[8];

int16_t main_iCenterLeftX; 
int16_t main_iCenterLeftY; 
int16_t main_iCenterRightX;
int16_t main_iCenterRightY;

uint32_t main_lDelayCount = 0UL;

uint8_t main_bToggleLED = false;

//-----------------------------------------------------------------------------
// Local functions
//-----------------------------------------------------------------------------

void ClassicControllerInit ( void )
{
  // initialize the Wii Classic Controller
  // disable encryption "the new way"
  
  main_ayTwiBuffer[0] = 0xF0; 
  main_ayTwiBuffer[1] = 0x55; 

  twi_writeTo( CLASSIC_IDENT, main_ayTwiBuffer, 2, true );
  
  _delay_us(1000); // the controller needs some time to process
  
  main_ayTwiBuffer[0] = 0xFB; 
  main_ayTwiBuffer[1] = 0x00; 
  
  twi_writeTo( CLASSIC_IDENT, main_ayTwiBuffer, 2, true );
  _delay_us(1000); // the controller needs some time to process
  
  // retrieve center value of sticks
  
  main_ayTwiBuffer[0] = 0x00;
  twi_writeTo( CLASSIC_IDENT, main_ayTwiBuffer, 1, true );
  _delay_us(1000); // the controller needs some time to process
  twi_readFrom( CLASSIC_IDENT, main_ayTwiBuffer, 6);
  
  // First value read after bootup
  main_iCenterLeftX  = (int16_t)((main_ayTwiBuffer[0] & 0x3F) << 2);
  main_iCenterLeftY  = (int16_t)((main_ayTwiBuffer[1] & 0x3F) << 2);
  main_iCenterRightX = (int16_t)((main_ayTwiBuffer[0] & 0xC0) | ((main_ayTwiBuffer[1] & 0xC0) >> 2) | ((main_ayTwiBuffer[2] & 0x80) >> 4));
  main_iCenterRightY = (int16_t)((main_ayTwiBuffer[2] & 0x1F) << 3);
}

void I2cInit( void )
{
  DDRC  &= ~( _BV( I2C_PIN_SCL ) | _BV( I2C_PIN_SDA ) ); // TWI pins as input
  // PORTC |= _BV( I2C_PIN_SCL ) | _BV( I2C_PIN_SDA );      // enable pull-ups (external Pullups are necessary!)
  
  twi_init();
}



uint8_t DataAvailable ( uint8_t *pArray, uint8_t yLength )
{
  uint8_t bRetval = 0;
  uint8_t yIndex  = 0;
  
  for ( yIndex = 0; ( yIndex < yLength ) && ( bRetval == 0 ); yIndex++ )
  {
	  if ( *(pArray+yIndex) != 0 ) bRetval = 1;
  }
  
  return bRetval;
}

void ClearTwiBuffer( void )
{
  main_ayTwiBuffer[0] = 0x00;
  main_ayTwiBuffer[1] = 0x00;
  main_ayTwiBuffer[2] = 0x00;
  main_ayTwiBuffer[3] = 0x00;
  main_ayTwiBuffer[4] = 0x00;
  main_ayTwiBuffer[5] = 0x00;
}

void ClassicControllerDecode( uint8_t *pBuffer, size_t len, ClassicController_t *pClassicController )
{
  if ( len >= CLASSIC_BYTE_COUNT )
  {
	pClassicController->LeftX  = ( pBuffer[0] & (64 - 1)) - 32; // 0 to 63
    pClassicController->LeftY  = ( pBuffer[1] & (64 - 1)) - 32; // 0 to 63 -> -32 to 31
    // 
    // pClassicController->RightX = (((pBuffer[2] >> 7) & 1) + ((pBuffer[1] >> 6) & 3) * 2 + ((pBuffer[0] >> 6) & 3) * 8) - 16; // 0 to 31 -> -16 to 15
    // pClassicController->RightY = (pBuffer[2] & (32 - 1)) - 16; // 0 to 31 -> -16 to 15
    
    // pClassicController->LeftX  -= main_iCenterLeftX;  // Substract Offset
    // pClassicController->LeftY  -= main_iCenterLeftY;  // Substract Offset
    // 													 
    // pClassicController->RightX -= main_iCenterRightX; // Substract Offset
    // pClassicController->RightY -= main_iCenterRightY; // Substract Offset
    	
    // pClassicController->LeftT        = ((( pBuffer[2] >> 5) & 3) * 8 + ((pBuffer[3] >> 5) & 7));
    // pClassicController->RightT       = ( pBuffer[3] & (32 - 1));
    
    
    // Digital signals
    pClassicController->ButtonDown   = (( pBuffer[4] & MASK_DOWN ) == 0);
    pClassicController->ButtonLeft   = (( pBuffer[5] & MASK_LEFT ) == 0);
    pClassicController->ButtonUp     = (( pBuffer[5] & MASK_UP   ) == 0);
    pClassicController->ButtonRight  = (( pBuffer[4] & MASK_RIGHT) == 0);
    pClassicController->ButtonSelect = (( pBuffer[4] & MASK_SEL  ) == 0);
    pClassicController->ButtonHome   = (( pBuffer[4] & MASK_HOME ) == 0);
    pClassicController->ButtonStart  = (( pBuffer[4] & MASK_START) == 0);
    pClassicController->ButtonY      = (( pBuffer[5] & MASK_Y    ) == 0);
    pClassicController->ButtonX      = (( pBuffer[5] & MASK_X    ) == 0);
    pClassicController->ButtonA      = (( pBuffer[5] & MASK_A    ) == 0);
    pClassicController->ButtonB      = (( pBuffer[5] & MASK_B    ) == 0);
    pClassicController->ButtonL      = (( pBuffer[4] & MASK_L    ) == 0);
    pClassicController->ButtonR      = (( pBuffer[4] & MASK_R    ) == 0);
    pClassicController->ButtonZL     = (( pBuffer[5] & MASK_ZL   ) == 0);
    pClassicController->ButtonZR     = (( pBuffer[5] & MASK_ZR   ) == 0);
    
    // For Testing
	// if ( pClassicController->ButtonA ) main_bToggleLED = true; 
    // if ( pClassicController->ButtonB ) main_bToggleLED = false; 
  }
}

static void HandleControllerInputs( ClassicController_t *pClassicController )
{
  static uint8_t bToggleAnalog = false;

  // Press A, B, Select and DPad Left/Right simultaneously
  // to toggle between Analog and Digital Input
  if (    ( pClassicController->ButtonA )
       && ( pClassicController->ButtonB )
       && ( pClassicController->ButtonSelect )
       && ( pClassicController->ButtonLeft   )
     )
  {
	  bToggleAnalog = true;
  }
  
  if (    ( pClassicController->ButtonA )
       && ( pClassicController->ButtonB )
       && ( pClassicController->ButtonSelect )
       && ( pClassicController->ButtonRight  )
     )
  {
	  bToggleAnalog = false;
  }
  
  // Set USB Data
  
  if ( bToggleAnalog )
  {
    int16_t iXAxis = (int16_t)pClassicController->LeftX *  4;  // X-Axis
    int16_t iYAxis = (int16_t)pClassicController->LeftY * -4;  // Y-Axis, inverted
	
    if ( iXAxis >  127 ) iXAxis =  127; 
    if ( iYAxis >  127 ) iYAxis =  127; 
    if ( iXAxis < -127 ) iXAxis = -127;
    if ( iYAxis < -127 ) iYAxis = -127;
	
    main_ayReport [0] = (uint8_t)iXAxis;  // could be done with << 2
	main_ayReport [1] = (uint8_t)iYAxis;
  }
  else
  {
  	if ( pClassicController->ButtonLeft  ) main_ayReport [0] = -127;  // X-Axis
  	if ( pClassicController->ButtonRight ) main_ayReport [0] =  127;  // X-Axis
  	if ( pClassicController->ButtonUp    ) main_ayReport [1] = -127;  // Y-Axis
  	if ( pClassicController->ButtonDown  ) main_ayReport [1] =  127;  // Y-Axis
  }
	
  main_ayReport [2] = (   ( pClassicController->ButtonA      << 0 )  // Buttons LSB
                        | ( pClassicController->ButtonB      << 1 )
                        | ( pClassicController->ButtonX      << 2 )
                        | ( pClassicController->ButtonY      << 3 )
                        | ( pClassicController->ButtonL      << 4 )
                        | ( pClassicController->ButtonR      << 5 )
                        | ( pClassicController->ButtonStart  << 6 )
                        | ( pClassicController->ButtonSelect << 7 ) );
  
  main_ayReport [3] = (   ( pClassicController->ButtonZL     << 0 )    // Buttons MSB
                        | ( pClassicController->ButtonZR     << 1 )
                        | ( pClassicController->ButtonHome   << 2 ) );

}

static void ReadJoy( void )
{
  static uint8_t bControllerDetected = false;

  // Reset all data
  main_ayReport [0] = 0;
  main_ayReport [1] = 0;
  main_ayReport [2] = 0;
  main_ayReport [3] = 0;
 
  if ( bControllerDetected )
  {
    ClearTwiBuffer();
    
    twi_writeTo( CLASSIC_IDENT, main_ayTwiBuffer, 1, true );
	_delay_us ( 500 ); // This delay is very important!
	twi_readFrom( CLASSIC_IDENT, main_ayTwiBuffer, CLASSIC_BYTE_COUNT );
	  
	bControllerDetected = DataAvailable( main_ayTwiBuffer, CLASSIC_BYTE_COUNT ); 
    if ( bControllerDetected )
	{
	  ClassicControllerDecode( main_ayTwiBuffer, CLASSIC_BYTE_COUNT, &main_tClassicController );
	  
      // Process Controller Data
	  HandleControllerInputs( &main_tClassicController );
	}
  }
  else
  {
	// Restart Controller (don't be too spammy)
	
	if ( main_lDelayCount >= RECONNECT_DELAY_VALUE )
	{
      main_ayTwiBuffer[0] = 0x00;
      twi_writeTo( CLASSIC_IDENT, main_ayTwiBuffer, 1, true );
	  _delay_us(500); // the controller needs some time to process
      twi_readFrom( CLASSIC_IDENT, main_ayTwiBuffer, CLASSIC_BYTE_COUNT );
	  
      bControllerDetected = DataAvailable( main_ayTwiBuffer, CLASSIC_BYTE_COUNT );
	  
      if ( bControllerDetected )
	  {
	    ClassicControllerInit();
		
		// Prepare first read
		twi_writeTo( CLASSIC_IDENT, main_ayTwiBuffer, 1, true );
	  }
	  main_lDelayCount = 0UL;
	}
	else
	{
	  main_lDelayCount++;
	}
  }	
  
  if ( bControllerDetected && !main_bToggleLED ) LED_ON;   
  else                                           LED_OFF;  
}


// Necessary for V-USB usage
uint8_t usbFunctionSetup( uint8_t data [8] )
{
	usbRequest_t const* rq = (usbRequest_t const*) data;

	if ( (rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_CLASS )
	return 0;
	
	switch ( rq->bRequest )
	{
		case USBRQ_HID_GET_REPORT: // HID joystick only has to handle this
		usbMsgPtr = (usbMsgPtr_t) main_ayReportOut;
		return sizeof main_ayReportOut;
		
		//case USBRQ_HID_SET_REPORT: // LEDs on joystick?
		
		default:
		return 0;
	}
}

void Init ( void )
{
  DDRC  |= 1;            // Set LED as Output
  
  I2cInit();		     // Initialize I2C driver

  sei();			     // Enable Interrupts
  
  _delay_ms(25);         // power up delay
  
  cli();                 // Disable Interrupts
  
  usbInit();             // start v-usb
  usbDeviceDisconnect(); // enforce USB re-enumeration, do this while interrupts are disabled!
  _delay_ms(250);
  usbDeviceConnect();
  	
  sei();                 // Enable Interrupts
}

int main(void)
{
	Init();
	 
	// Initial Delay value
	// without, USB device is not detected properly
	static uint32_t lDelay = 0;
	
	for ( ;; )
	{
	  usbPoll();
		
	  // Don't bother reading joy if previous changes haven't gone out yet.
	  // Forces delay after changes which serves to debounce controller as well.
	  if ( usbInterruptIsReady() )
	  {
	    // Read Gamepad from I2C
		if ( lDelay >= INITIAL_DELAY_VALUE ) ReadJoy();
		else                                 lDelay++;
		
		// Memory Compare: Don't send update unless joystick changed
		if ( memcmp( main_ayReportOut, main_ayReport, sizeof( main_ayReport ) ) )
		{
		  memcpy( main_ayReportOut, main_ayReport, sizeof( main_ayReport ) );
		  usbSetInterrupt( main_ayReportOut, sizeof( main_ayReportOut ) );
		}
	  }
	}
	
	return 0;
}

