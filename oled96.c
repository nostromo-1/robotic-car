/*************************************************************************
OLED control code, for the 0.96" SSD1306 128x64 OLED display via I2C interface.
Based on code written by Larry Bank (bitbank@pobox.com)




*****************************************************************************/

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <pthread.h>
#include <pigpio.h>

#include "oled96.h"
#include "fonts.c"

#define I2C_BUS 1   // i2c bus of display: Bus 0 for Rev 1 boards, bus 1 for newer boards


#define ERR(ret, format, arg...)                                       \
   {                                                                   \
         fprintf(stderr, "%s: " format "\n" , __func__ , ## arg);      \
         return ret;                                                   \
   }
   
   
   
static int iScreenOffset; // current write offset of screen data
static uint8_t ucScreen[1024]; // local copy of the image buffer
static int i2c_handle = -1;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    
static int oledWriteCommand(uint8_t);
static int oledWriteCommand2(uint8_t c, uint8_t d);
static int oledSetPosition(int x, int y);
static int oledWriteDataBlock(const uint8_t *ucBuf, int iLen);
static void RotateFont90(void);


// Opens a handle to the I2C device using pigpio library
// Initializes the OLED controller into "page mode"
// Prepares the font data for the orientation of the display
int oledInit(int iAddr)
{
int rc;
uint8_t initbuf[]={0x00,0xae,0xa8,0x3f,0xd3,0x00,0x40,0xa0,0xa1,0xc0,0xc8,
			0xda,0x12,0x81,0xff,0xa4,0xa6,0xd5,0x80,0x8d,0x14,0x20,0x02};

    i2c_handle = i2cOpen(I2C_BUS, iAddr, 0); 
    if (i2c_handle < 0) ERR(-1, "Cannot open display");     

    rc = i2cWriteDevice(i2c_handle, initbuf, sizeof(initbuf));
    if (rc < 0) goto rw_error;     
    
    oledFill(0);    // Set display memory to zero   
    oledWriteCommand(0xAF);  // turn on OLED
    
    oledSetContrast(128);
    RotateFont90(); // fix font orientation for OLED
    return 0;  

   /* error handling if read operation from I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(-1, "Cannot write data to display"); 
} 


// Sends a command to turn off the OLED display
// Closes the I2C file handle
void oledShutdown()
{
    if (i2c_handle < 0) return;
    pthread_mutex_lock(&mutex);
    oledWriteCommand(0xAE); // turn off OLED
    i2cClose(i2c_handle);
    i2c_handle = -1;
    pthread_mutex_unlock(&mutex);
}


// Send a single byte command to the OLED controller
static int oledWriteCommand(uint8_t c)
{
int rc;
    
    rc = i2cWriteByteData(i2c_handle, 0x00, c);  // 0x00 is the command introducer
    if (rc < 0) ERR(-1, "Error writing to display"); 
    return 0;
}


static int oledWriteCommand2(uint8_t c, uint8_t d)
{
int rc;
uint16_t value;
    
    value = d<<8 | c;
    // 0x00 is the command introducer
    rc = i2cWriteWordData(i2c_handle, 0x00, value);
    if (rc < 0) ERR(-1, "Error writing to display"); 
    return 0;
} 


int oledSetContrast(uint8_t ucContrast)
{
int rc;
   
   if (i2c_handle < 0) return -1;

   pthread_mutex_lock(&mutex);
	rc = oledWriteCommand2(0x81, ucContrast);
   pthread_mutex_unlock(&mutex);
	return rc;
} 



int oledSetInversion(bool invert)
{
int rc;

    if (i2c_handle < 0) return -1;

    pthread_mutex_lock(&mutex);    
    if (invert == false) rc = oledWriteCommand(0xA6);
    else rc = oledWriteCommand(0xA7);
    pthread_mutex_unlock(&mutex);
    return rc;
}


// Send commands to position the "cursor" to the given
// row and column. It assumes that the mutex is locked
static int oledSetPosition(int x, int y)
{
char buf[4];
int rc;

   if (y<0 || y>7 || x<0 || x>127) ERR(-1, "Invalid coordinates in display");
   buf[0] = 0;         // 0x00 is the command introducer
   buf[1] = 0xb0 | y;  // go to page Y
   buf[2] = 0x00 | (x & 0x0f);   // lower col addr
   buf[3] = 0x10 | (x >> 4);     // upper col addr
   rc = i2cWriteDevice(i2c_handle, buf, sizeof(buf));
   if (rc < 0) goto rw_error; 
	iScreenOffset = (y*128)+x;
   return 0;
   
   /* error handling if write operation to I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(-1, "Cannot write data to display");  
}



// Write a block of pixel data to the OLED
// Length can be anything from 1 to 128 (whole line)
static int oledWriteDataBlock(const uint8_t *ucBuf, int iLen)
{
uint8_t ucTemp[129];
int rc, rest;

	 ucTemp[0] = 0x40; // data command
    
    if (!ucBuf) ERR(-1, "Error writing to display: Invalid buffer");
    if (iLen < 0 || iLen > 128) ERR(-1, "Error writing to display: Invalid length for display data");
    if (iLen == 0) return 0;
    memcpy(&ucTemp[1], ucBuf, iLen);
    
	// Keep a copy in local buffer, taking care not to overflow to beginning of row (display in page mode)
    rest = 128 - iScreenOffset%128;
    if (rest >= iLen) {
        rc = i2cWriteDevice(i2c_handle, ucTemp, iLen+1);
        if (rc < 0) goto rw_error; 
        memcpy(&ucScreen[iScreenOffset], ucBuf, iLen);        
        iScreenOffset += iLen;
    }
    else {
        rc = i2cWriteDevice(i2c_handle, ucTemp, rest+1);
        if (rc < 0) goto rw_error;         
        memcpy(&ucScreen[iScreenOffset], ucBuf, rest);
        iScreenOffset += rest;
    } 
    return 0;
    
   /* error handling if write operation to I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(-1, "Cannot write data to display");     
}


// Set (or clear) an individual pixel
// The local copy of the frame buffer is used to avoid
// reading data from the display controller
// Coordinate system is pixels, not text rows (0-127, 0-63)
int oledSetPixel(int x, int y, uint8_t ucColor)
{
int i, rc;
uint8_t uc, ucOld;

	if (i2c_handle < 0) return -1;
   if (x<0 || x>127 || y<0 || y>63) ERR(-1, "Invalid coordinates for display");

	i = ((y >> 3) * 128) + x;
	uc = ucOld = ucScreen[i];
	uc &= ~(0x1 << (y & 7));
	if (ucColor) uc |= (0x1 << (y & 7));
	if (uc != ucOld) {  // pixel changed
      pthread_mutex_lock(&mutex);
		rc = oledSetPosition(x, y>>3);
		rc |= oledWriteDataBlock(&uc, 1);
      pthread_mutex_unlock(&mutex);
      if (rc < 0) goto rw_error;  
	}
   return 0;
    
   /* error handling if write operation to I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(-1, "Cannot write data to display");     
} 


// Draw a string of small (8x8) or large (16x24) characters
// At the given col+row
// String must have a maximum of 16 characters for small font and 8 for large font
int oledWriteString(int x, int y, const char *szMsg, bool bLarge)
{
int i, j, iLen, rc;
const uint8_t *s;
uint8_t buf[16*8];

	if (i2c_handle < 0) return -1; 
   if (y<0 || y>7 || x<0 || x>127) ERR(-1, "Invalid coordinates for display");
   if (!szMsg) ERR(-1, "Invalid string");
    
	iLen = strlen(szMsg);
	if (bLarge) {  // draw 16x24 font, 8 characters per line
        if (iLen>8) ERR(-1, "length of string with large font is over 8 characters");
        for (i=0; i<3; i++) {
            pthread_mutex_lock(&mutex);
            rc = oledSetPosition(x, y+i);
            for (j=0; j<iLen; j++) {
                s = &ucFont[9728 + (uint8_t)(szMsg[j]&0x7F)*64];  // 0x7F: large font has only 128 characters
                memcpy(buf+j*16, s+16*i, 16);
            }
            rc |= oledWriteDataBlock(buf, iLen*16);
            pthread_mutex_unlock(&mutex); 
            if (rc < 0) goto rw_error;  
        }
	}
	else {  // draw 8x8 font, 16 characters per line
      if (iLen>16) ERR(-1, "length is over 16 characters");
      pthread_mutex_lock(&mutex);
		rc = oledSetPosition(x, y);
		for (i=0; i<iLen; i++) memcpy(buf+i*8, &ucFont[(uint8_t)szMsg[i]*8], 8);
      rc |= oledWriteDataBlock(buf, iLen*8); // write character pattern
      pthread_mutex_unlock(&mutex);
      if (rc < 0) goto rw_error; 
	}

	return 0;
   
   /* error handling if write operation to I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(-1, "Cannot write data to display");    
} 


// Write an 8x8 bitmap to display
// graph is an 8 byte array, glyph must be turned 90 degrees to the right
int oledSetBitmap8x8(int x, int y, const uint8_t* graph)
{
   static const uint8_t empty[] = {0, 0, 0, 0, 0, 0, 0, 0};  // empty space
   const uint8_t *buf;
   int rc;
    
	if (i2c_handle < 0) return -1; 
   if (y<0 || y>7 || x<0 || x>127) ERR(-1, "Invalid coordinates for display");
   if (graph) buf = graph;
   else buf = empty;
    
   pthread_mutex_lock(&mutex);
   rc = oledSetPosition(x, y);
   rc |= oledWriteDataBlock(buf, 8);
   pthread_mutex_unlock(&mutex);
   
	return rc;
} 


// Fill the frame buffer with a byte pattern
// e.g. all off (0x00) or all on (0xff)
int oledFill(uint8_t ucData)
{
int y, rc;
uint8_t temp[128];

	if (i2c_handle < 0) return -1; 

	memset(temp, ucData, sizeof(temp));
	for (y=0; y<8; y++) {
      pthread_mutex_lock(&mutex);
		rc = oledSetPosition(0, y); // set to (0,Y)
		rc |= oledWriteDataBlock(temp, sizeof(temp)); // fill line with data byte
      pthread_mutex_unlock(&mutex);
      if (rc < 0) goto rw_error; 
	} 
    
	return 0;
   
   /* error handling if write operation to I2C bus failed */
rw_error:
   if (i2c_handle>=0) i2cClose(i2c_handle);
   i2c_handle = -1;
   ERR(-1, "Cannot write data to display");   
}



// Write a message in big font on display
// line: 0 or 1 (writes message in lines 2,3,4 or 5,6,7 respectively)
int oledBigMessage(int line, const char *msg)
{
static const char *empty = "        ";
char *buf;
int rc;
    
    if (line<0 || line>1) ERR(-1, "line must be 0 or 1");
    if (msg) buf = (char *)msg; 
    else buf = (char *)empty;
    
    rc = oledWriteString(0, 2+3*line, buf, true); 
    return rc;
}



// Fix the orientation of the font image data, defined in fonts.c
static void RotateFont90(void)
{
uint8_t ucTemp[64];
int i, j, x, y;
uint8_t c, c2, ucMask, *s, *d;

	// Rotate the 8x8 font
	for (i=0; i<256; i++) {  // fix 8x8 font by rotating it 90 deg clockwise
		s = &ucFont[i*8];
		ucMask = 0x1;
		for (y=0; y<8; y++) {
			c = 0;
			for (x=0; x<8; x++) {
				c >>= 1;
				if (s[x] & ucMask) c |= 0x80;
			}
			ucMask <<= 1;
			ucTemp[7-y] = c;
		}
		memcpy(s, ucTemp, 8);
	}
	// Rotate the 16x32 font
	for (i=0; i<128; i++) { // only 128 characters
		for (j=0; j<4; j++) {
			s = &ucFont[9728 + 12 + (i*64) + (j*16)];
			d = &ucTemp[j*16];
			ucMask = 0x1;
			for (y=0; y<8; y++) {
				c = c2 = 0;
				for (x=0; x<8; x++) {
					c >>= 1;
					c2 >>= 1;
					if (s[(x*2)] & ucMask) c |= 0x80;
					if (s[(x*2)+1] & ucMask) c2 |= 0x80;
				}
				ucMask <<= 1;
				d[7-y] = c;
				d[15-y] = c2;
			} // for y
		} // for j
		memcpy(&ucFont[9728 + (i*64)], ucTemp, 64);
	} // for i
} 

