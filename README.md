# Rotating WS2812 persistence of vision (POV) demo [STM32C562RE]

[Unit RollerCAN](https://docs.m5stack.com/en/unit/Unit-RollerCAN) (M5Stack) offers a nice feature: "The device's axis features an optional electrical slip ring configuration, allowing the top Grove interface to remain connected to the bottom even during 360° rotation, enabling the expansion of additional modules on top while maintaining power and data transmission for the rotating part." On the RollerCAN side this interface is connected to the I2C peripheral. However, if used cautiously, i.e. making sure that the SDA and SCL are idling on the RollerCAN side, they may serve various other communication purposes, including push-pull lines. Here the Groove SCL line is used to send data to a string of [8 WS2812 RGB LEDs](https://allegro.pl/oferta/284-neopixel-listwa-8x-ws2812b-rgb-ws2812-arduino-10641656585). My initial idea was to use the built-in encoder for synchronisation purposes. Unfortunately, that cannot be done in the interrupt mode - readings from the encoder are served over the CAN bus only if polled for these. For the bus at 500 kbps, the precision of about 2 degrees is achieved - functional but clearly affecting the stability/stillness of the displayed image/text. As a quick fix/improvement, an [IR slot sensor](https://www.keyestudio.com/products/keyestudio-photo-interrupter-module-uno-r3-mega-2560-r3-video) (opto-interrupter) is added. The previous solution is kept in the code as a keepsake (`#define USE_ENCODER_FOR_SYNCHRONIZATION`). Enjoy playing with [visual persistence](https://en.wikipedia.org/wiki/Persistence_of_vision)!

![RollerCAN WS2812B arm](/Assets/Images/rollercan_ws2812_arm.jpg)
![RollerCAN WS2812B in action](/Assets/Images/rollercan_ws2812_pov_in_action.jpg)
![RollerCAN WS2812B photocell](/Assets/Images/ir_slot_sensor_photocell.jpg)

# Missing files?
Don't worry. Just open the .ioc2 file in STM32CubeMX2 and hit the yellow button to generate the IDE project. Then open the project in STM32CubeIDE for Visual Studio Code and build it.

# Further inspirations
* [Making of wireless energy transferred POV Clock Display](https://www.youtube.com/watch?v=sv5b2GO_gNE) (TecH BoyS ToyS)
* [Arduino POV Propeller Clock using 7-Segment LED Display](https://www.youtube.com/watch?v=Kez29cW7qoc) (Hobby Projects)

# Approaches utilizing I2C
* [Gravity: CH423 I2C 24 Digital IO Expansion Module](https://wiki.dfrobot.com/dfr0979/) (DFRobot)
* [SparkFun Qwiic Alphanumeric Display](https://www.sparkfun.com/sparkfun-qwiic-alphanumeric-display-purple.html) (SparkFun)

# For your children or younger siblings
* [Persistence of Vision (POV) Clock with 51515](https://www.antonsmindstorms.com/product/persistence-of-vision-pov-clock-with-51515/) (Antons Mindstorms)

# Call to action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2026_dzien_otwarty_we/Dzien_Otwarty_WE_2026_Control_Engineering_for_Hobbyists.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [bitluni's lab](https://www.youtube.com/@bitluni), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), [Max Imagination](https://www.youtube.com/@MaxImagination), [Nikodem Bartnik](https://www.youtube.com/@nikodembartnik), [Stuff Made Here](https://www.youtube.com/@StuffMadeHere), [Mario's Ideas](https://www.youtube.com/@marios_ideas), [Aaed Musa](https://www.aaedmusa.com/), [Haase Industries](https://www.youtube.com/@h1tec), [Marcin Plaza](https://www.youtube.com/MarcinPlaza), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them! Promote [README-driven learning](http://ufnalski.edu.pl/proceedings/sene2025/Ufnalski_PE_formatted_SENE_2025.pdf).

> [!WARNING]
> STM32C5 series - do try it at home and build your own POV toys :grey_exclamation:

230 challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned!

0xBU
