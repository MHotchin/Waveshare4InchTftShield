# Waveshare4InchTftShield
Graphics and touchscreen drivers for Waveshare 4" shield.

This is an re-implementation of the 'Adafruit_GFX' graphics API.  This version is
substantially faster than the original Waveshare sample software, and provides an
integrated Touchscreen interface.  I don't have one to test on, but it should work for the
3.5" screen as well.

The touch screen will also calibrate itself automatically, and provides access to the
calibration data so that is can be stored (for example, in EEPROM) and then set, so that
the touch screen will maintain its calibration across reboots.

Under examples there is:
 - GraphicsTest
 - TouchTest

 GraphicsTest runs through the Adafruit_GFX test code, and gives a timing speed for it.
 This is about 25 seconds for the whole run.  The original Waveshare library used a cut
 down version of the Adafruit tests - that has been replicated as well so that times can
 be compared.  On a Mega 2560, the original library took about 70 seconds to complete.
 This library is down to just under 16 seconds.

 TouchTest is a *very* simple 'painting' program.  It shows how to read and normalize the
 touch points, and how the screen calibration is handled.  When first started, the drawing
 will not occur under the stylus.  Once the screen is calibrated, the drawng and stylus
 should be very closely aligned.

 To calibrate the screen, simply run the stylus off all four edges of the screen a few
 times.  This calibration can be retrieved using `getTsConfigData()`, saved, and restored
 later using `setTsConfigData()`.
