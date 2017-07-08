/*************************************************************************
OLED control code, for the 0.96" SSD1306 128x64 OLED display.
Based on the following original code:

// OLED SSD1306 using the I2C interface
// Written by Larry Bank (bitbank@pobox.com)
// Project started 1/15/2017
//
// The I2C writes (through a file handle) can be single or multiple bytes.
// The write mode stays in effect throughout each call to write()
// To write commands to the OLED controller, start a byte sequence with 0x00,
// to write data, start a byte sequence with 0x40,
// The OLED controller is set to "page mode". This divides the display
// into 8 128x8 "pages" or strips. Each data write advances the output
// automatically to the next address. The bytes are arranged such that the LSB
// is the topmost pixel and the MSB is the bottom.
// The font data comes from another source and must be rotated 90 degrees
// (at init time) to match the orientation of the bits on the display memory.
// A copy of the display memory is maintained by this code so that single pixel
// writes can occur without having to read from the display controller.

*****************************************************************************/

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <pigpio.h>

#define I2C_BUS 1   // Bus 0 for Rev 1 boards, bus 1 for newer boards

extern unsigned char ucFont[];
static int iScreenOffset; // current write offset of screen data
static unsigned char ucScreen[1024]; // local copy of the image buffer
static int i2c_handle = -1;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    
static void oledWriteCommand(unsigned char);
static void RotateFont90(void);

// Opens a file system handle to the I2C device
// Initializes the OLED controller into "page mode"
// Prepares the font data for the orientation of the display
int oledInit(int iAddr)
{
int rc;
char initbuf[]={0x00,0xae,0xa8,0x3f,0xd3,0x00,0x40,0xa0,0xa1,0xc0,0xc8,
			0xda,0x12,0x81,0xff,0xa4,0xa6,0xd5,0x80,0x8d,0x14,
			0xaf,0x20,0x02};

    i2c_handle = i2cOpen(I2C_BUS, iAddr, 0);        
    if (i2c_handle < 0) {
        perror("oledInit: Cannot open display");
        return i2c_handle;
    }
    rc = i2cWriteDevice(i2c_handle, initbuf, sizeof(initbuf));
    if (rc < 0) {
        perror("oledInit: Error initializing display"); 
        i2cClose(i2c_handle);
        i2c_handle = -1;
        return rc;
    }        
    
    oledFill(0);    // Put display memory to zero    
   	RotateFont90(); // fix font orientation for OLED
    return 0;    
} /* oledInit() */

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
static void oledWriteCommand(unsigned char c)
{
    int rc;
    
    // 0x00 is the command introducer
    rc = i2cWriteByteData(i2c_handle, 0x00, c);
    if (rc < 0) perror("oledWriteCommand: Error writing to display");
} /* oledWriteCommand() */

static void oledWriteCommand2(unsigned char c, unsigned char d)
{
    int rc;
    unsigned int value;
    
    value = d<<8 | c;
    // 0x00 is the command introducer
    rc = i2cWriteWordData(i2c_handle, 0x00, value);
    if (rc < 0) perror("oledWriteCommand2: Error writing to display");
} /* oledWriteCommand2() */

int oledSetContrast(unsigned char ucContrast)
{
    if (i2c_handle < 0) return -1;

    pthread_mutex_lock(&mutex);
	oledWriteCommand2(0x81, ucContrast);
    pthread_mutex_unlock(&mutex);
	return 0;
} /* oledSetContrast() */



// Send commands to position the "cursor" to the given
// row and column
static void oledSetPosition(int x, int y)
{
    if (y<0 || y>7 || x<0 || x>127) {
        fprintf(stderr, "oledSetPosition: Invalid coordinates in display\n");
        return;
    }
	oledWriteCommand(0xb0 | y); // go to page Y
	oledWriteCommand(0x00 | (x & 0x0f)); // lower col addr
	oledWriteCommand(0x10 | ((x >> 4) & 0x0f)); // upper col addr
	iScreenOffset = (y*128)+x;
}



// Write a block of pixel data to the OLED
// Length can be anything from 1 to 128 (whole line)
static void oledWriteDataBlock(unsigned char *ucBuf, int iLen)
{
unsigned char ucTemp[129];
int rc, rest;

	ucTemp[0] = 0x40; // data command
    
    if (!ucBuf) {
        perror("oledWriteDataBlock: Invalid buffer");
        return;    
    }
    if (iLen < 1 || iLen > 128) {
        perror("oledWriteDataBlock: Invalid length for display data");
        return;
    }
    memcpy(&ucTemp[1], ucBuf, iLen);
    rc = i2cWriteDevice(i2c_handle, ucTemp, iLen+1);
    if (rc < 0) perror("oledWriteDataBlock: Error writing to i2c bus");
    
	// Keep a copy in local buffer, taking care not to overflow
    rest = sizeof(ucScreen) - iScreenOffset;
    if (rest >= iLen) {
        memcpy(&ucScreen[iScreenOffset], ucBuf, iLen);
        iScreenOffset += iLen;
    }
    else {
        memcpy(&ucScreen[iScreenOffset], ucBuf, rest);
        iScreenOffset += rest;
    }
}

// Set (or clear) an individual pixel
// The local copy of the frame buffer is used to avoid
// reading data from the display controller
// Coordinate system is pixels, not text rows (0-127, 0-63)
int oledSetPixel(int x, int y, unsigned char ucColor)
{
int i;
unsigned char uc, ucOld;

	if (i2c_handle < 0) return -1;
    if (x<0 || x>127 || y<0 || y>63) {
        fprintf(stderr, "oledSetPixel: Invalid coordinates in display\n");
        return -1;        
    }

	i = ((y >> 3) * 128) + x;
	uc = ucOld = ucScreen[i];
	uc &= ~(0x1 << (y & 7));
	if (ucColor) uc |= (0x1 << (y & 7));
	if (uc != ucOld) {  // pixel changed
        pthread_mutex_lock(&mutex);
		oledSetPosition(x, y>>3);
		oledWriteDataBlock(&uc, 1);
        pthread_mutex_unlock(&mutex);
	}
    return 0;
} /* oledSetPixel() */

// Draw a string of small (8x8) or large (16x24) characters
// At the given col+row
int oledWriteString(int x, int y, char *szMsg, int bLarge)
{
int i, j, iLen;
unsigned char *s;

	if (i2c_handle < 0) return -1; // not initialized
    if (y<0 || y>7 || x<0 || x>127) {
        fprintf(stderr, "oledWriteString: Invalid coordinates for display\n");
        return -1;
    }
    if (!szMsg) {
        fprintf(stderr, "oledWriteString: Invalid string\n");
        return -1;    
    }
    
	iLen = strlen(szMsg);
	if (bLarge) {  // draw 16x24 font, 8 characters per line
        for (i=0; i<3; i++) {
            pthread_mutex_lock(&mutex);
            oledSetPosition(x, y+i);
            for (j=0; j<iLen; j++) {
                s = &ucFont[9728 + (unsigned char)(szMsg[j]&0x7F)*64];  // 0x7F: only 128 characters
                oledWriteDataBlock(s+16*i, 16);
            }
            pthread_mutex_unlock(&mutex); 
        }
	}
	else {  // draw 8x8 font, 16 characters per line
        pthread_mutex_lock(&mutex);
		oledSetPosition(x, y);
		for (i=0; i<iLen; i++) {
			s = &ucFont[(unsigned char)szMsg[i]*8];
			oledWriteDataBlock(s, 8); // write character pattern
		}	
        pthread_mutex_unlock(&mutex);
	}

	return 0;
} /* oledWriteString() */

// Fill the frame buffer with a byte pattern
// e.g. all off (0x00) or all on (0xff)
int oledFill(unsigned char ucData)
{
int y;
unsigned char temp[128];

	if (i2c_handle < 0) return -1; // not initialized

	memset(temp, ucData, 128);
	for (y=0; y<8; y++) {
        pthread_mutex_lock(&mutex);
		oledSetPosition(0, y); // set to (0,Y)
		oledWriteDataBlock(temp, 128); // fill with data byte
        pthread_mutex_unlock(&mutex);
	} // for y
    
	return 0;
} /* oledFill() */

// Fix the orientation of the font image data
static void RotateFont90(void)
{
unsigned char ucTemp[64];
int i, j, x, y;
unsigned char c, c2, ucMask, *s, *d;

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
} /* RotateFont90() */
