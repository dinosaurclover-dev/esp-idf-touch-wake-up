# esp-idf-touch-wake-up
My project is hosed in a small, waterproof container. It has a USB waterproof, panel mounted socket for charging. A single charge of the battery will power the box contents, including the ESP, for about 10 hours.

I wanted a way of switching the unit on/off by touch. A simple touch detector is prone to accidental switching. Users can switch this unit on/off by tapping a label on the outside of the box for about 10 or 15 seconds. The user must tap once per second. Too slow or too fast will not trigger the switch.

To include this functionality in your project 

Step 1.
Copy the file subTouch.c and subTouch.h into you main folder.

Step 2.
Add the required depedency to your project. 
If your main directory does not already have a idf_component.yml file, create with the idf.py create-mainifest command. 
Add the lines
  touch_sens_examples_common:
    path: ${IDF_PATH}/examples/peripherals/touch_sensor/touch_sens_examples_common
to your manifest file.

Step 3.
In the CMakeLists.txt file in your main folder
add "subTouch.c" to the list of sources

The file example.c shows how to use it. Ensure that your app calls subTouchInit() and then subTouchScan(). This will start the task that monitors the touch input.

When the ESP is powered on or awoken from deep sleep, users will need to tap at one Hz for about 10 seconds. If they don't, the task will put the ESP into deep sleep after 30 seconds.

The user can save battery by switching the unit off when not needed. Just tap for about 10 or 15 seconds.
