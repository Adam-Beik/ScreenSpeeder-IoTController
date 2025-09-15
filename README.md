# ScreenSpeeder IoT Controller
![animation of project deconstruction](https://github.com/Adam-Beik/ScreenSpeeder-IoTController/blob/main/deconstruct_compressed.gif)
![animation of project's navigation dial internals](https://github.com/Adam-Beik/ScreenSpeeder-IoTController/blob/main/gears_compressed.gif)
* Project repository for the ScreenSpeeder, a wireless navigation controller for the visually impaired.
* [Link to first demonstration video (updated video releasing October 2025)](https://www.youtube.com/watch?v=nY-d67pn6yI)
* [Link to technical report documentation](https://1drv.ms/w/c/bf5aecddce199c5e/EYxzLiALXJlNptYhbYVuunkBDRpJXIS_0LUYTRMiL-INpQ?e=VsDVbx)
* 3D-printed wireless navigation controller with a custom PCB that allows the visually impaired to navigate web content and enter text data using a combination of physical controls, gestures, and voice-to-text input. **Note:** Voice-to-text currently only functional using Windows devices.
* Comfortably worn on the wrist using adjustable straps.
* Uses BlueTooth to connect to target device and WiFi (optional) to exchange MQTT requests for a Node.JS (Node-RED) web dashboard.
* Sends browser and NVDA commands via BlueTooth HID input; NVDA used as screen-reading technology to read web content to user. 
* Firmware uses object-oriented C++ codebase; MQTT messaging uses JSON.
* See **platformio.ini** for a list of all open-source libraries used for this project.
* **Note: This project is subject to change. Libraries not included but are listed in the .ini file.**

Electronics
------
* Espressif ESP32-DevKitC-V4 microcontroller for central processing
* I2C addressing for the following sensor and output controls:
  * AS5600 Hall effect sensor for navigation dial
  * AXDL345 Accelerometer sensor for gesture input
  * DRV2605L Motor driver with FIT0774 haptic output for vibrational feedback
* UART devices:
  * DFPlayerMini MP3 module (with SD card) connected to AS01508MR-6-LW 8Ohm speaker for sound cues
* Other GPIO devices:
  * KY-040 Rotary encoder with quadrature decoding and push button for mode selection
  * WS2812 RGB LED for auxiliary information regarding current mode
  * SP/ST momentary push buttons for left-right or up-down navigation depending on mode
* Power management:
  * Rechargeable 3.7V 620mAh LiPo battery
  * Adafruit PowerBoost 500C for charging; 5V boost for added module stability (LED, DFPlayerMini)

Software
-----
* PlatformIO extension via Visual Studio Code for project development and debugging
* Node.JS via Node-RED for web dashboard and MQTT signalling
* C++ for firmware code; includes libraries (see **platformio.ini** for the full list)
* Autodesk Fusion 360 for modelling, schematics, and PCB design

Connecting Device
----
* Upon powering on the device (see **"Usage->Left-right buttons"**), enable BlueTooth on target device and pair with **ScreenSpeeder V** (or whatever you have named it in the `main` file).
* Ensure device is flat when powered ON for accelerometer calibration.
* Ensure NVDA and a browser application is running on the target device.
  * Controller can be used in **Default** mode to tab to and select apps on the homescreen via navigation dial and the rotary encoder's selector button (see **"Usage"**)
* Ensure WiFi is configured in the `main` file to access the web dashboard (must have Node.JS and Node-RED installed to host dashboard using command `node-red`).

Usage (See technical documentation for more information)
----
* **Navigation dial** tabs through web content depending on mode. Clockwise is **forward** and counter-clockwise is **backward**.
  
* **Rotary encoder button** "clicks" by sending a "Space" or "Enter" command depending on mode.
  * Rotary encoder knob cycles through four modes:
    * **Default mode:** Clockwise rotation of the **navigation dial** sends "Tab"; Counter-clockwise rotation sends "Shift+Tab"
    * **Header mode:** Rotation of the **navigation dial** cycles through **Header** elements of the web page
    * **Link mode:** Rotation of the **navigation dial** cycles through **Link** elements of the web page
    * **Form mode:** Rotation of the **navigation dial** cycles through **Form** elements (text fields, buttons) of the web page. Button sends "Enter" instead of "Space"
  * **Holding** encoder button for two seconds sends a **voice-to-text command**. Pressing the button submits the parsed text into the selected field
  * **Double-clicking** the encoder button re-reads the targeted line or element
  * **Triple-clicking** the encoder button focuses on the browser's navigation/search field **except in Link mode (see below)**
    * **Triple-clicking** in **Link** mode will **put the device into deep-sleep (essentially powering OFF)**
      
* **Left-right buttons** send arrow-key signals to move through elements forward or backward depending on mode.
  * Sends up and down arrow-key signals in all modes except **Form** mode, in which case it sends left and right arrow signals to move the text cursor
  * Should be conceptualized as "forward to next immediate element or line" and "backward to last immediate element or line"
  * While the device is in **deep-sleep (OFF)**, pressing the **left button** will **power the device ON**
    
* **Gesture** commands (assuming device is held in front of user on the forearm) **Note:** Assume arm is always in an **"L"** shape:
  *  Tilting the device >30° to the **right** (rotating shoulder so forearm moves upward from neutral) issues a **browser forward** command
  *  Tilting the device >30° to the **left** (rotating shoulder so forearm moves downward from neutral) issues a **browser backward** command
  *  Tilting the device >70° **toward** the user in an **upward roll** (as if checking one's watch) issues a **browser refresh** command
    * In **Header** mode, tilting the device >70° in a **downward roll** (as if showing one's watch to someone) issues a **browser refresh** command
    * In **Form** mode, the aforementioned **downward roll** will **clear the focused text field** in case an input was entered incorrectly


