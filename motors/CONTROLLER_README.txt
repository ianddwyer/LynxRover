install evdev using pip
identify the controllers event number for calling:
--see this link: https://core-electronics.com.au/guides/using-usb-and-bluetooth-controllers-with-python/ 
-- the part of finding the controller is most important
-- ls /dev/input
find which one disappears when you unplug and run the command again
then run:
-- cat /dev/input/eventX where X is the controller identified
To run controller, you must open both spyder and a terminal
Replace the number of your device in the python file to call the controller
open the program in terminal and type "sudo python <directory>/controller.py"
this gives admin for accessing the drive in which spyder does not have
paho does not run for the terminal, hence why this is necessary
run the program from spyder now that you have admin
