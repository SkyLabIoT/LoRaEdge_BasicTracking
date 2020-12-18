# Update scripts

For updating the LR1110 modem-E firmware.

UPDATELRCHIP105 contains modem-E version 1.0.5

UPDATELRCHIP107 contains modem-E version 1.0.7

## How to update
Updating the modem firmware goes as follows. The host PC must have Phyton installed.

Upload the LRNodeUpdateTxToModem example sketch to the device.
Wait for blue LED to be on.
Start the Python script (UPDATELRCHIP.py).
Select the correct COM port in the python application.
Check if firmware is being send by looking at Python program and the device LED (should be flashing red).
Sending the firmware takes about 2 min.
The Python program displays done and closes 30 sec later when done.
The green LED on the device stays on when firmware is updated.
Upload the mode A device example sketch.
The firmware should now be upgraded to Modem-E and the example sketch should work.


The original firmware files can be found at https://github.com/Lora-net/radio_firmware_images




