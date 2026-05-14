USB Commands STM32

This project uses a custom developed STM32 development board to accept and respond to text based commands using USB CDC.

The incoming USB serial data is first copied into a buffer inside the USB interrupt callback. The buffer is then checked using main while loop and compared against a list of known commands.

The list of commands are stored as strings to compare against (ignoring case) the incoming command. If a match is found, the command is processed and a response is generated via USB CDC using the CDC_Transmit_FS function. If a command match is not found, the response will be "Invalid Commmand! Type 'help'"

Note that this project must receive a full command packet inside of one USB interrupt service routine, otherwise the command buffer will be reset. To prevent this, a command index can be used to append data to the command buffer then the command can be processed and reset after a newline character (or other command termination symbol) is received or the buffer is filled.

The main function and command processing function can be found in /Core/Src/main.c
	main(): Initialize system clocks, peripherals, reset data buffers.
			+Infinite loop: Calls readCommand, blinks LED, uses blocking delay
	readCommand(): USB Command processor, checks incoming USB serial data and attempts to parse the commands
	checkButton(): Used to read the status of a button press using GPIO input pins (pulled up) connected to buttons via breadboard, which pull the button low when pressed.
	
	The USB Serial data is collected in the interrupt callback, in the file /USB_DEVICE/App/usbd_cdc_if.c
	CDC_Receive_FS(): Copies the incoming USB buffer into the command buffer and returns USBD_OK to ACK over USB and allow for data reception.
		Note: At the top of the file, a reference to the buffer created in main.c is made: extern uint8_t usbBuf[64];

The list of available commands can be found inside the readCommand() function.
Command List (Case Insensitive):
Command			Response
"led1"			Toggles LED1, responds "LED 1 toggled!\n"
"led2"			Toggles LED2, responds "LED 2 toggled!\n"
"led3"			Toggles LED3, responds "LED 3 toggled!\n"
"all leds"		Toggles all LEDs, responds "All leds toggled!\n"
"button1"		Checks Button 1 Status, responds "Button 1 Status: On/Off\n" (On if pressed, else Off)
"button2"		Checks Button 2 Status, responds "Button 2 Status: On/Off\n" (On if pressed, else Off)
"button3"		Checks Button 3 Status, responds "Button 3 Status: On/Off\n" (On if pressed, else Off)
"all buttons"	Checks all buttons, sends "Button N Status: On/Off\n" (On if pressed, else Off) for N=1,2,3
"help"			Responds with "Available Commands:\n"
							  "led1,led2,led3\n"
							  "button1,button2,button3\n"
							  "all leds, all buttons\n"
No Match		Responds with "Invalid Command! Type 'help'\n"
