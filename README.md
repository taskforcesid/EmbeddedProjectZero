# EmbeddedProjectZero
Final Assignment

Complete this Assignment after you have (1) watched all videos; (2) reviewed all slides; (3) completed reading assignments.

This Assignment will certify that you have mastered all Embedded C skills that we covered in the lessons of this course.

Here are the requirements:

=1= Use STM32CubeIDE or STMCubeMX/TrueStudio to generate the initial code for this final assignment. 

=2= You will use the UART that is connected to the Arduino connector to display output from this assignment onto PuTTy (or similar) terminal emulator. (Similar to how you displayed output on the UART from a previous assignment.)

=3= You will use the Blue Button on the STM board to cycle through different demos. Each time you press the Blue button, the title of the demo will be sent to the UART and displayed on PuTTY. Importantly, pressing the Blue Button should generate an interrupt and change a "demo count" so that your main looping code changes to the next demo on every button press. When you power up "Demo 1" will auto start. When you press the Blue Button "Demo 1" ends and "Demo 2" starts. This continues until you reach the last demo, then you cycle back to Demo 1. Stated differently, pressing the Blue Button lets you cycle through the demos as many times as you like.

=4= The list of demos you should include are:

= "Demo 1: LL APIs".  Use the following APIs during demo 1: (a) get flash size  LL_GetFlashSize(); (b) get the device unique ID, LL_GetUID_Wordn(); c3) toggle the LED, LL_GPIO_TogglePin() at a 1 second rate. Display the Flash Size and GUID only when demo begins, but keep flashing the LED every 1 second until the Blue Button is pressed to advance to next demo.

= "Demo 2: HAL APIs". Use the following HAL APIs during demo 2: (a) get the device ID, HAL_GetDEVID(), (b) read the device unique ID, HAL_GetUIDwn(), and  (c)toggle the LED, HAL_GPIO_TogglePin() at a 2 second rate, using HAL_Delay() to sleep for the 2 seconds. Display the Dev ID info only when demo 2 begins, but keep flashing the LED every 2 second until the Blue Button is pressed to advance to next demo.

= "Demo 3: BSP APIs". Use the following BSP APIs during demo 3: (a) read the temperature, BSP_TSENSOR_ReadTemp() (b) turn the LED on every 3 seconds with BSP_LED_On() (c)  turn LED off, every 3 seconds with BSP_LED_Off(). In other words the LED should blink on/off at a 3-second rate (3 seconds on, 3 seconds off). Continue this until the Blue Button pressed to advance to the next demo.

= "Demo 4: BONUS!" This is optional. Use this demo to demonstrate an optional I/O device that is on the STM board. Or, connect additional I/O devices to the Arduio connectors. Some examples: (1) connect a push-button switch to one of the GPIO input pins on the Arduino connector, then read the status of the on-off push button; (2) connect an LED to one of the GPIO output pins, and toggle the LED at a 4 second cycle (4 seconds on, 4 seconds off). (3) Connect an ADC device to one of the ADC Arduino pins, and read the value. (4) connect an I2C or SPI device to the Arduino pins, and read the info from those devices.

NOTE: Demo 1, 2, and 3 are required. However Demo 4 is optional for extra credit.
