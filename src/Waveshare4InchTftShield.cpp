
#include <Arduino.h>
//#include <Wire.h>
#include <SPI.h>

#include <Adafruit_GFX.h>

#include "Waveshare4InchTftShield.h"


namespace
{
#ifdef ARDUINO_ESP8266_WEMOS_D1R1
	//GPIO config
	//LCD

	//  D1 R1 pins are not 'general purpose". D5, D6 and D7 are actually *duplicates* of
	//  D11, D12 and D13, so you can't use them at all.  D9 is hard to use, it's part of
	//  the reboot process.  D1 is Serial TX, you probably want that for debugging.
	//  So, you can't plug the shield into a D1 R1, you need to map the pins.
	//
	constexpr unsigned int LCD_CS = D10; //  LCD Chip Select
	constexpr unsigned int LCD_BL = D8;  //  LCD Backlight
	constexpr unsigned int LCD_RST = D4;  //  LCD Reset
	constexpr unsigned int LCD_DC = D3;  //  LCD Data/Control

	constexpr unsigned int TP_CS = D0;

	constexpr unsigned int SD_CS = D2;
#elif defined ARDUINO_ESP32_DEV
	
	//  TO-DO - is this specific enough?  Pins below are for Wemos D1 R32, but that
	//  doesn't seem to have its own board.

	//  SERIOUSLY?  No analogWrite on this platform?  Idiots.
	void analogWrite(uint8_t pin, uint8_t val)
	{
		//  Shortcut - it's either on or off
		digitalWrite(pin, val ? HIGH : LOW);
	}

	constexpr unsigned int LCD_CS  = 5;  // 10; //  LCD Chip Select
	constexpr unsigned int LCD_BL  = 13; // 9;  //  LCD Backlight
	constexpr unsigned int LCD_RST = 12; // 8;  //  LCD Reset
	constexpr unsigned int LCD_DC  = 14; // 7;  //  LCD Data/Control

	constexpr unsigned int TP_CS = 17;   // 4;
	//constexpr unsigned int TP_IRQ = 3;
	//constexpr unsigned int TP_BUSY = 6;

	constexpr unsigned int SD_CS = 16;   // 5;

#else

	//GPIO config
	//LCD
	constexpr unsigned int LCD_CS = 10; //  LCD Chip Select
	constexpr unsigned int LCD_BL = 9;  //  LCD Backlight
	constexpr unsigned int LCD_RST = 8;  //  LCD Reset
	constexpr unsigned int LCD_DC = 7;  //  LCD Data/Control

	constexpr unsigned int TP_CS = 4;
	//constexpr unsigned int TP_IRQ = 3;
	//constexpr unsigned int TP_BUSY = 6;

	constexpr unsigned int SD_CS = 5;

#endif
	//  Dimensions in default rotation.
	constexpr int16_t LCD_WIDTH = 320;
	constexpr int16_t LCD_HEIGHT = 480;
}

//  Data sheets says min clock width is 66ns, for a max clock of 15 MHz.  Except, this
//  thing isn't *actually* SPI!  It's a 16 bit shift register connected to the parallel
//  interface, and that can run at 20 Mhz
SPISettings _tftSpiSettingsWrite(20000000, MSBFIRST, SPI_MODE0);

//  TFT reads are slower, 150 ns period.
//  Nevermind, Waveshare shield doesn't support reads at all!
//SPISettings _tftSpiSettingsRead(6500000, MSBFIRST, SPI_MODE0);

//  Touch screen wants 400 ns period.
SPISettings tsSpiSettings(2500000, MSBFIRST, SPI_MODE0);


Waveshare4InchTftShield::Waveshare4InchTftShield()
	:Adafruit_GFX(LCD_WIDTH, LCD_HEIGHT)
{
	//  Default Adafruit value
	pressureThreshhold = 10;

	//  Set input pins in a sane state, so SPI can be initialized
	pinMode(LCD_CS, OUTPUT);
	digitalWrite(LCD_CS, HIGH);
	pinMode(LCD_RST, OUTPUT);
	digitalWrite(LCD_RST, HIGH);
	pinMode(LCD_DC, OUTPUT);
	pinMode(LCD_BL, OUTPUT);
	analogWrite(LCD_BL, 0);

	pinMode(TP_CS, OUTPUT);
	digitalWrite(TP_CS, HIGH);
	//pinMode(TP_IRQ, INPUT_PULLUP);
	//pinMode(TP_BUSY, INPUT_PULLUP);

	pinMode(SD_CS, OUTPUT);
	digitalWrite(SD_CS, HIGH);
}

namespace
{
	inline void lcdWriteReg(uint8_t reg)
	{

		digitalWrite(LCD_DC, LOW);
		SPI.transfer(0);
		SPI.transfer(reg);
	}

	inline void lcdWriteData(uint8_t data)
	{

		digitalWrite(LCD_DC, HIGH);
		SPI.transfer(0);
		SPI.transfer(data);
	}

	inline void lcdWriteDataContinue(uint8_t data)
	{
		SPI.transfer(0);
		SPI.transfer(data);
	}


	inline void lcdWriteData16(uint16_t data)
	{
		lcdWriteData(uint8_t(data >> 8));
		lcdWriteDataContinue(uint8_t(data & 0xff));
	}

	inline void lcdWriteDataContinue16(uint16_t data)
	{
		lcdWriteDataContinue(uint8_t(data >> 8));
		lcdWriteDataContinue(uint8_t(data & 0xff));
	}

#ifdef ARDUINO_ARCH_AVR
	//  Version of SPI.transfer(...) that *doesn't* read data back into the buffer.
	//  

	//  Wrote this, then went with a bulk repeat implementation instead.
	//inline static void transferOut(void *buf, size_t count)
	//{
	//	if (count == 0) return;
	//	uint8_t *p = (uint8_t *)buf;
	//	SPDR = *p++;
	//	while (--count > 0)
	//	{
	//		uint8_t out = *p++;
	//		while (!(SPSR & _BV(SPIF)));
	//		SPDR = out;
	//	}
	//	while (!(SPSR & _BV(SPIF)));
	//}


	inline static void transfer16Repeat(uint16_t data, unsigned long count)
	{
		if (count == 0)
		{
			return;
		}

		//  Fast way to split a 16bit into into 2 8 bit ints.
		union
		{
			uint16_t val;
			struct
			{
				uint8_t lsb; uint8_t msb;
			};
		} in;
		in.val = data;

		//  Slightly faster using these, I think the optimizer can take advantage that
		//  there is no aliasing?
		const uint8_t msb = in.msb;
		const uint8_t lsb = in.lsb;

		//  Loop phase shifting AND Duff's device?  Inconceivable!
		SPDR = msb;
		count--;
		while (!(SPSR & _BV(SPIF)));
		SPDR = lsb;
		switch (count & 0x01)
		{
		case 0:
			while (count)
			{
				count--;
				while (!(SPSR & _BV(SPIF)));
				SPDR = msb;
				asm volatile("nop");
				while (!(SPSR & _BV(SPIF)));
				SPDR = lsb;
		case 1:
				count--;
				while (!(SPSR & _BV(SPIF)));
				SPDR = msb;
				asm volatile("nop");
				while (!(SPSR & _BV(SPIF)));
				SPDR = lsb;
			}
		}
		// MUST wait for final shift out to complete!  Otherwise subsequent commands
		// happen quickly enough to stomp on it.
		asm volatile("nop");
		while (!(SPSR & _BV(SPIF)));

	}
#endif

	inline void lcdWriteDataRepeat(uint16_t data, unsigned long count)
	{
		lcdWriteReg(0x2C);
		digitalWrite(LCD_DC, HIGH);

#ifdef ARDUINO_ARCH_ESP32
		//
		//  ESP8266 seems to have better bulk transfer APIs for SPI.  
		uint8_t pattern[2];
		pattern[0] = data >> 8;
		pattern[1] = data & 0xff;

		SPI.writePattern(pattern, 2, count);

#elif defined ARDUINO_ARCH_AVR
		//  On AVR it's pretty easy to write a better bulk repeat directly, so use that.
		transfer16Repeat(data, count);
#else
		//  Otherwise, just do it the boring way.
		for (unsigned long i = 0; i < count; i++)
		{
			SPI.transfer16(data);
		}
#endif
}

	inline void lcdWriteCommand(uint8_t reg, uint8_t data)
	{
		lcdWriteReg(reg);
		lcdWriteData(data);
	}

	inline void lcdWriteCommand(uint8_t reg, uint8_t data, uint8_t data2)
	{
		lcdWriteReg(reg);
		lcdWriteData(data);
		lcdWriteDataContinue(data2);
	}


	struct ActiveBounds
	{
		uint8_t data[8];
	};


	inline void lcdWriteActiveRect(uint16_t xUL, uint16_t yUL, uint16_t xSize, uint16_t ySize)
	{
		uint16_t xStart = xUL, xEnd = xUL + xSize - 1;
		uint16_t yStart = yUL, yEnd = yUL + ySize - 1;

		//  Writing datablocks is quite a bit faster than transfering out one byte at a
		//  time.
		ActiveBounds b = {0, (uint8_t)(xStart >> 8), 0, (uint8_t)(xStart & 0xFF), 0, (uint8_t)(xEnd >> 8), 0, (uint8_t)(xEnd & 0xFF)};
		lcdWriteReg(0x2a);
		digitalWrite(LCD_DC, HIGH);
		SPI.transfer((byte *)&b, sizeof(b));

		b = {0, (uint8_t)(yStart >> 8), 0, (uint8_t)(yStart & 0xFF), 0, (uint8_t)(yEnd >> 8), 0, (uint8_t)(yEnd & 0xFF)};
		lcdWriteReg(0x2b);
		digitalWrite(LCD_DC, HIGH);
		SPI.transfer((byte *)&b, sizeof(b));
	}
}


bool
Waveshare4InchTftShield::begin()
{
	return begin(0xff);
}


bool
Waveshare4InchTftShield::begin(uint8_t brightness)
{
	pinMode(LCD_CS, OUTPUT);
	digitalWrite(LCD_CS, HIGH);
	pinMode(LCD_RST, OUTPUT);
	digitalWrite(LCD_RST, HIGH);
	pinMode(LCD_DC, OUTPUT);
	pinMode(LCD_BL, OUTPUT);
	analogWrite(LCD_BL, 0);

	pinMode(TP_CS, OUTPUT);
	digitalWrite(TP_CS, HIGH);
	//pinMode(TP_IRQ, INPUT_PULLUP);
	//pinMode(TP_BUSY, INPUT_PULLUP);

	pinMode(SD_CS, OUTPUT);
	digitalWrite(SD_CS, HIGH);

	initializeLcd();
	setRotation(0);
	analogWrite(LCD_BL, brightness);
	return true;
}



uint8_t
Waveshare4InchTftShield::GetSdCardCS()
{
	return SD_CS;
}



void
Waveshare4InchTftShield::initializeLcd()
{

	//  Trigger hardware reset.
	digitalWrite(LCD_RST, HIGH);
	delay(5);
	digitalWrite(LCD_RST, LOW);
	delayMicroseconds(20);
	digitalWrite(LCD_RST, HIGH);

	//  TO-DO - how long after a reset until the screen can be used?  Doesn't seem to be
	//  specified in the datasheet.
	//  Experimentally, any less than this and the initial screen clear is incomplete.
	delay(65);

	startWrite();
	{
		//  Power control settings
		lcdWriteCommand(0xC0, 0x19, 0x1a);
		lcdWriteCommand(0xC1, 0x45, 0x00);
		lcdWriteCommand(0xC2, 0x33);        //  Power/Reset on default

		lcdWriteCommand(0xC5, 0x00, 0x28);  //  VCOM control

		lcdWriteCommand(0xB1, 0xA0, 0x11);  //  Frame rate control

		lcdWriteCommand(0xB4, 0x02);        //  Display Z Inversion

		lcdWriteReg(0xB6);                  //  Display Control Function      
		lcdWriteData(0x00);
		lcdWriteDataContinue(0x42);
		lcdWriteDataContinue(0x3B);

		lcdWriteReg(0xE0);                  //  Positive Gamma control
		lcdWriteData(0x1F);
		lcdWriteDataContinue(0x25);
		lcdWriteDataContinue(0x22);
		lcdWriteDataContinue(0x0B);
		lcdWriteDataContinue(0x06);
		lcdWriteDataContinue(0x0A);
		lcdWriteDataContinue(0x4E);
		lcdWriteDataContinue(0xC6);
		lcdWriteDataContinue(0x39);
		lcdWriteDataContinue(0x00);
		lcdWriteDataContinue(0x00);
		lcdWriteDataContinue(0x00);
		lcdWriteDataContinue(0x00);
		lcdWriteDataContinue(0x00);
		lcdWriteDataContinue(0x00);

		lcdWriteReg(0XE1);                  //  Negative Gamma control
		lcdWriteData(0x1F);
		lcdWriteDataContinue(0x3F);
		lcdWriteDataContinue(0x3F);
		lcdWriteDataContinue(0x0F);
		lcdWriteDataContinue(0x1F);
		lcdWriteDataContinue(0x0F);
		lcdWriteDataContinue(0x46);
		lcdWriteDataContinue(0x49);
		lcdWriteDataContinue(0x31);
		lcdWriteDataContinue(0x05);
		lcdWriteDataContinue(0x09);
		lcdWriteDataContinue(0x03);
		lcdWriteDataContinue(0x1C);
		lcdWriteDataContinue(0x1A);
		lcdWriteDataContinue(0x00);

		lcdWriteCommand(0x3A, 0x55);

		//  Set initial rotation to match AFX defaults - tall / narrow
		lcdWriteCommand(0xB6, 0x00, 0x22);
		lcdWriteCommand(0x36, 0x08);

		lcdWriteReg(0x11); // Sleep out

		//  Fill screen to black
		writeFillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x0000);
		
		lcdWriteReg(0x29);  // Turn on display
	}
	endWrite();
}


void
Waveshare4InchTftShield::drawPixel(
	int16_t x, int16_t y, uint16_t color)
{
	startWrite();

	writePixel(x, y, color);

	endWrite();
}

void
Waveshare4InchTftShield::startWrite()
{
	SPI.beginTransaction(_tftSpiSettingsWrite);
	digitalWrite(LCD_CS, LOW);
}

void
Waveshare4InchTftShield::writePixel(
	int16_t x, int16_t y, uint16_t color)
{
	if (x < 0) return;
	if (y < 0) return;

	if (x >= width()) return;
	if (y >= height()) return;

	writeFillRect2(x, y, 1, 1, color);
}


void
Waveshare4InchTftShield::writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	//  Negative widths, so swap left and right sides
	if (w < 0)
	{
		w = -w;
		x -= w;
	}

	//  Negative height, so swap top and bottom
	if (h < 0)
	{
		h = -h;
		y -= h;
	}

	// Left side offscreen, clip
	if (x < 0)
	{
		w += x;
		x = 0;
	}

	// Top offscreen, clip
	if (y < 0)
	{
		h += y;
		y = 0;
	}

	//  Rightside offscreen, clip
	if (x + w > width())
	{
		w = width() - x;
	}

	// bottom offscreen, clip
	if (y + h > height())
	{
		h = height() - y;
	}

	//  Entire width or entire height is offscreen
	if (w <= 0) return;
	if (h <= 0) return;

	// Now, 0 <= x <= x+w <= WIDTH
	// And, 0 <= y <= y+h <= HEIGHT
	lcdWriteActiveRect(x, y, w, h);
	lcdWriteDataRepeat(color, (int32_t)w * (int32_t)h);
}

void
Waveshare4InchTftShield::writeFillRect2(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	lcdWriteActiveRect(x, y, w, h);
	lcdWriteDataRepeat(color, (int32_t)w * (int32_t)h);
}

void
Waveshare4InchTftShield::writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	writeFillRect(x, y, 1, h, color);
}


void
Waveshare4InchTftShield::writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	writeFillRect(x, y, w, 1, color);
}


void
Waveshare4InchTftShield::endWrite()
{
	digitalWrite(LCD_CS, HIGH);
	SPI.endTransaction();
}

void
Waveshare4InchTftShield::setRotation(uint8_t r)
{
	uint8_t MemoryAccessControl_0x36 = 0;

	switch (r & 0x03)
	{
	case 0x00:
		MemoryAccessControl_0x36 = 0x08;
		break;

	case 0x01:
		MemoryAccessControl_0x36 = 0xA8;
		break;

	case 0x02:
		MemoryAccessControl_0x36 = 0xC8;
		break;

	case 0x03:
		MemoryAccessControl_0x36 = 0x68;
		break;
	}

	startWrite();
	{
		lcdWriteCommand(0x36, MemoryAccessControl_0x36);
	}
	endWrite();

	//  Don't forget to tell the base class!
	Adafruit_GFX::setRotation(r);
}


void
Waveshare4InchTftShield::invertDisplay(boolean i)
{
	startWrite();
	{
		lcdWriteReg(i ? 0x21 : 0x20);
	}
	endWrite();
}


void
Waveshare4InchTftShield::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	startWrite();
	writeFillRect(x, y, 1, h, color);
	endWrite();
}


void
Waveshare4InchTftShield::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	startWrite();
	writeFillRect(x, y, w, 1, color);
	endWrite();
}

void
Waveshare4InchTftShield::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	startWrite();
	writeFillRect(x, y, w, h, color);
	endWrite();
}

void
Waveshare4InchTftShield::fillScreen(uint16_t color)
{
	startWrite();
	writeFillRect2(0, 0, width(), height(), color);
	endWrite();
}


//  Non Adafruit_GFX APIs.
void
Waveshare4InchTftShield::setScreenBrightness(uint8_t brightness)
{
	analogWrite(LCD_BL, brightness);
}


void
Waveshare4InchTftShield::setIdleMode(bool idle)
{
	startWrite();
	{
		lcdWriteReg(idle ? 0x39 : 0x38);
	}
	endWrite();
}

//  Touchscreen interface

// increase or decrease the touchscreen oversampling. This is a little different than you make think:
// 1 is no oversampling, whatever data we get is immediately returned
// 2 is double-sampling and we only return valid data if both points are the same
// 3+ uses insert sort to get the median value.
// We found 2 is precise yet not too slow so we suggest sticking with it!

#define NUMSAMPLES 2


//  TSPoint code taken from Adafruit 'Touchscreen' library, MIT licence.
TSPoint::TSPoint(void)
{
	x = y = 0;
}

TSPoint::TSPoint(int16_t x0, int16_t y0, int16_t z0)
{
	x = x0;
	y = y0;
	z = z0;
}

bool TSPoint::operator==(TSPoint p1)
{
	return  ((p1.x == x) && (p1.y == y) && (p1.z == z));
}

bool TSPoint::operator!=(TSPoint p1)
{
	return  ((p1.x != x) || (p1.y != y) || (p1.z != z));
}

#if (NUMSAMPLES > 2)
static void insert_sort(int array[], uint8_t size)
{
	uint8_t j;
	int save;

	for (int i = 1; i < size; i++)
	{
		save = array[i];
		for (j = i; j >= 1 && save < array[j - 1]; j--)
			array[j] = array[j - 1];
		array[j] = save;
	}
}
#endif


//  Not implemented in Adafruit libraries?
//bool
//Waveshare4InchTftShield::isTouching()
//{
//	return false;
//}

namespace
{
	uint16_t readChannel(uint8_t channel)
	{
		uint16_t data = 0;

		SPI.beginTransaction(tsSpiSettings);
		digitalWrite(TP_CS, LOW);

		SPI.transfer(channel);

		//  Delay 8 serial clocks (3200 nS)
		delayMicroseconds(3);

		data = SPI.transfer(0x00);
		data <<= 8;
		data |= SPI.transfer(0x00);
		data >>= 3;

		digitalWrite(TP_CS, HIGH);
		SPI.endTransaction();

		return data;
	}
}

uint16_t
Waveshare4InchTftShield::pressure()
{
	uint32_t Z1 = readChannel(0b10110000);
	uint32_t Z2 = readChannel(0b11000000);

	if (Z1 > Z2) return 0;

	return (Z2 - Z1);
}


int16_t
Waveshare4InchTftShield::readTouchY()
{
	return readChannel(0b10010000);
}

int16_t
Waveshare4InchTftShield::readTouchX()
{
	return readChannel(0b11010000);
}

constexpr int slop = 7;

// Returns un-normalized data, oriented the same as the rotation 0 setting.
TSPoint
Waveshare4InchTftShield::getPoint()
{
	int x, y, z;
	int samples[NUMSAMPLES];
	uint8_t i, valid;

	valid = 1;

	//  Discard first reading, other code indicates it's noisy.
	readTouchX();

	for (i = 0; i < NUMSAMPLES; i++)
	{
		samples[i] = readTouchX();
	}

#if NUMSAMPLES > 2
	insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
	// Allow small amount of measurement noise, because capacitive
	// coupling to a TFT display's signals can induce some noise.
	if (samples[0] - samples[1] < -slop || samples[0] - samples[1] > slop)
	{
		valid = 0;
	}
	else
	{
		samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples
	}
#endif

	// Match 10 bit resolution of Adafruit touchplates
	x = (1023 - (samples[NUMSAMPLES / 2] >> 2));
	if (x == 1023) x = 0;


	for (i = 0; i < NUMSAMPLES; i++)
	{
		samples[i] = readTouchY();
	}

#if NUMSAMPLES > 2
	insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
	// Allow small amount of measurement noise, because capacitive
	// coupling to a TFT display's signals can induce some noise.
	if (samples[0] - samples[1] < -slop || samples[0] - samples[1] > slop)
	{
		valid = 0;
	}
	else
	{
		samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples
	}
#endif

	y = (1023 - (samples[NUMSAMPLES / 2] >> 2));

	if (!valid)
	{
		return TSPoint(0, 0, 0);
	}
	else if ((x == 0) && (y == 0))
	{
		return TSPoint(0, 0, 0);
	}
	else
	{
		z = (1023 - (pressure() >> 2));
	}

	return TSPoint(x, y, z);
}


namespace
{
	//  Some starting values that seem be just inside the actual limits.
	TSConfigData tscd = {100, 900, 75, 900};
}

const TSConfigData &
Waveshare4InchTftShield::getTsConfigData()
{
	return tscd;
}

void
Waveshare4InchTftShield::setTsConfigData(const TSConfigData &newData)
{
	tscd = newData;
}

void
Waveshare4InchTftShield::resetTsConfigData()
{
	tscd = {100, 900, 75, 900};
}


namespace
{
	void swap(int16_t &x, int16_t &y)
	{
		int16_t t = x;
		x = y;
		y = t;
	}
}
//  Normalize the touchscreen readings to the dimensions of the screen.  Automatically
//  adjusts the limits over time.  To calibrate, just run the stylus off each of the four
//  edges of the screen.
bool
Waveshare4InchTftShield::normalizeTsPoint(
	TSPoint &p)
{
	bool fReturn = false;

	if (p.x > 0)
	{
		if (p.x < tscd.xMin)
		{
			fReturn = true;
			tscd.xMin = p.x;
		}
		if (p.x > tscd.xMax)
		{
			fReturn = true;
			tscd.xMax = p.x;
		}
	}
	if (p.y > 0)
	{
		if (p.y < tscd.yMin)
		{
			fReturn = true;
			tscd.yMin = p.y;
		}
		if (p.y > tscd.yMax)
		{
			fReturn = true;
			tscd.yMax = p.y;
		}
	}
	p.x = map(p.x, tscd.xMin, tscd.xMax, 0, LCD_WIDTH - 1);

	//  Touchscreen area extends past bottom of screen.
	p.y = map(p.y, tscd.yMin, tscd.yMax, 0, LCD_HEIGHT + 10);
	if (p.y >= LCD_HEIGHT) p.y = LCD_HEIGHT - 1;

	switch (rotation)
	{
	case 0:
		break;

	case 1:
		swap(p.x, p.y);
		p.x = LCD_HEIGHT - 1 - p.x;
		break;

	case 2:
		p.x = LCD_WIDTH - 1 - p.x;
		p.y = LCD_HEIGHT - 1 - p.y;
		break;

	case 3:
		swap(p.x, p.y);
		p.y = LCD_WIDTH - 1 - p.y;
		break;
	}

	return fReturn;
}

