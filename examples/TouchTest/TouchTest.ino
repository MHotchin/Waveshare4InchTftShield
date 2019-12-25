/*
 Name:		TouchTest.ino
 Created:	2019-12-25 12:52:14 AM
 Author:	Michael
*/

// the setup function runs once when you press reset or power the board

#include <Arduino.h>

#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Waveshare4InchTftShield.h>

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

namespace
{
    Waveshare4InchTftShield Waveshield;
}

void setup() 
{
    Serial.begin(115200);

    while (!Serial);

    SPI.begin();
    Waveshield.begin();

    Waveshield.setRotation(1);
    Waveshield.setTextSize(2);
    Waveshield.print("Run stylus off each edge to calibrate!");

    Waveshield.setRotation(0);
}

int i = 0;

// the loop function runs over and over again until power down or reset
void loop()
{
    TSPoint p = Waveshield.getPoint();

    //  Remaps raw touchscreen values to screen co-ordinates.  Automatically handles
    //  rotation!
    Waveshield.normalizeTsPoint(p);

    Waveshield.fillCircle(p.x, p.y, 4, BLUE);

    if (millis() / 1000 > 10)
    {
        uint16_t color = i << 7 ^ i;
        Waveshield.drawFastHLine(0, i, Waveshield.width() - 1, color);

        i++;
        if (i >= Waveshield.height())
        {
            i = 0;
        }
    }
}
