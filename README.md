# AutoScale
Automatic electronic scale.  This has mainly just been a project for fun, started simply with getting a load cell working on an MSP430. Eventually decided to try and make an auto dispensing scale (powder/granules). It grew beyond the capability of the MSP pretty quickly, at which point I moved to a STM32F405. Then I replaced the 16x2 character display with a 320x240 LCD (using an spi interface, so we could ditch the MSP that was running the 16x2 display previously)

Consists of a dump chamber for bulk volume, and an auger to precisely top up to desired weight.  Initial test with just the auger here: https://youtu.be/iJeQbjNJ7Eg

I have tried several 3d printed builds for this, none of which never were terribly practical (3d printer not tight enough tolerances). Most recent version should be in the CAD folder (currently aiming for something that can be easily milled via 3 axis CNC, in addition to 3d printed). 

Code compiled using Ti CCS version 10 and STM32CubeIDE version 1.8.0

# Parts List
- Load Cell (TAL221 100g) https://www.sparkfun.com/products/14727
- Load Cell Amplifier (HX711) https://www.sparkfun.com/products/13879
- Precision Voltage Reference (LM4040) https://www.adafruit.com/product/2200
- Microcontrollers
  - STM32F4 (Currently using STM32F405) https://www.sparkfun.com/products/17713
  - MSP430 Launchpad (MSP430G2553) 
- Motors
  - Nema 17 motors
    - https://www.amazon.com/dp/B07LF898KN?ref=ppx_yo2_dt_b_product_details&th=1
    - https://www.amazon.com/dp/B094CQDDKF?psc=1&ref=ppx_yo2_dt_b_product_details
  - Stepper Driver (Toshiba TC78H670FTG) https://www.sparkfun.com/products/16836
- LCD Display
  - 2.4" 320x240 LCD Display using a IL9341 controller
    - https://www.waveshare.com/2.4inch-LCD-Module.htm  (also available on amazon)
- 4x4 Button Array
- Breadboard(s)
- Lots of jumper wires
- i2c pullup resistors (docs specify 10k ohm; currently using 2 2.2k in series and its working fine)
  
# Display Code
Using Waveshare's provided code for the LCD display ( https://www.waveshare.com/wiki/2.4inch_LCD_Module#Using_with_STM32 )
