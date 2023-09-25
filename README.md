# Embedded Systems Class Repository

Welcome to my repository for the Embedded Systems class! In this course, I worked with the TI-CC3200, a 32-bit Arm Cortex-M4 Wi-Fi® wireless MCU, to complete various labs and projects related to embedded systems development. Below, you'll find information about the labs I completed during the course.

## Lab 1: Development Tools Tutorial and Lab Exercise
This lab covered the basic software development tools that were used throughout the course, such as Code Composer Studio (CCS), CCS UniFlash, and the TI Pin Mux Tool.

## Lab 2: Serial Interfacing Using SPI and I2C

In this lab, I used SPI to interface the CC3200 LaunchPad to a color OLED display. I also used I2C to communicate with an on-board Bosch BMA222 acceleration sensor. The application program will detect the XY tilt of the board based on BMA222 readings and use the tilt to control an icon on the OLED display. I also used a Saleae logic probe to capture and display SPI and I2C signal waveforms

[Link to Lab 2 Folder](https://github.com/almtorres805/eec172/tree/main/Lab2_Ball_Movement_Display)

[Link to Lab 2 Report](https://github.com/almtorres805/eec172/blob/main/Lab2_Ball_Movement_Display/Lab2Report.pdf)

## Lab 3: IR Remote Control Texting Over an Asynchronous Serial (UART) Link

In this lab, I used the Saleae logic analyzer and an IR receiver module to characterize the 
transmissions for buttons from an AT&T IR remote control for a specific TV. I was assigned a 
unique TV code so that each person was decoding different waveforms. I then interfaced the CC3200 
Launchpad to the IR receiver module and wrote a program that uses interrupts to monitor the signal from the IR 
receiver module. By analyzing the series of pulses, the program was able to determine which button was 
pressed on the IR remote control programmed for my particular TV code. I then used the IR remote
control to compose text messages using the multi-tap text entry system (https://en.wikipedia.org/wiki/Multi-tap) 
and send text messages back and forth between two CC3200 LaunchPad boards over an asynchronous serial 
(UART) communication channel.

[Link to Lab 3 Folder](https://github.com/almtorres805/eec172/tree/main/lab3_Test_Messaging)

[Link to Lab 3 Report](https://github.com/almtorres805/eec172/tree/main/lab3_Test_Messaging/Lab3Report.pdf)

## Lab 4: DTMF Texting Over an Asynchronous Serial (UART) Link

Increasingly embedded systems include audio input. Consider for example Amazon’s Alexa or 
Google Home. In this lab the objective was to create a simple audio interface to our embedded systems, specifically a touch tone phone input. I designed a system to decode dual-tone multi-frequency (DTMF) audio signals coming from my phone, and used the input to compose text messages, similar to the way I used the IR Remote in 
Lab 3. The phone produced DTMF signals and I used the same multi-tap text entry system to 
generate text messages. I used SPI to interface to an external analog-to-digital converter (ADC) as well as the OLED. 
The final program was the same board-to-board texting application used in Lab 3, except I used DTMF 
audio signals instead of IR Remote signals to enter the characters.

[Goertzel Algorithm used for implementing and decoding DTMF tones](https://en.wikipedia.org/wiki/Goertzel_algorithm)

[Link to Lab 4 Folder](https://github.com/almtorres805/eec172/tree/main/lab4_DTMF_Text_Messaging)

[Link to Lab 4 Report](https://github.com/almtorres805/eec172/blob/main/lab4_DTMF_Text_Messaging/Lab%204%20Report.pdf)

## Lab 5: Introduction to AWS and RESTful API
This lab explored the RESTful API and how to connect with Amazon Web Services (AWS). First, I 
created a ‘thing’ in AWS and used the HTTP GET command to retrieve status information about the device ‘thing’. 

Then used code from Lab 3 to compose messages with an IR remote, which was sent to AWS 
using the HTTP POST command. I defined a rule in AWS to send the text to my email using Amazon’s 
Simple Notification Service (SNS).

I didn't include the code in this repository because the text messaging application from lab 3 was used, as well as the code to connect with AWS in the [final project](https://github.com/almtorres805/eec172/tree/main/Home_Security_Box).

## Final Project: Home Security Box
The objective of this lab was to use the REST API from Lab 5 to connect to a web service and do something interesting and creative.

The requirements were as followed: 

• Control or monitor something over the internet using the CC3200 LaunchPad.
• You must control or monitor your thing over the internet using some combination of the following
devices interfaced to the CC3200:
- IR remote control for text or numeric input
- Microphone
- DTMF input
- OLED for text or graphics display
- LEDs
- Switches
- Sensors, such as accelerometer, temperature sensor or any external sensors that you interface to
the CC3200.
• Your system must be a stand-alone embedded system and must operate without being connected to a
host computer. For your final presentation, you must use Uniflash to load your code into the external
serial flash instead of downloading from CCS.

My Home Security Box: Utilizing the CC3200 LaunchPad and AWS to create a security system. I set up the CC3200 to connect with the AWS IoT and utilized SNS to send push notifications to an associated email. The board also utilizes the built-in accelerometer to detect whether the device has been moved or not. The user will use the IR remote controller to activate/deactivate the security system.

More details regarding the implementation can be found [here]()

[Link to Home Security Box Folder](https://github.com/almtorres805/eec172/tree/main/Home_Security_Box)
## Additional Resources

- [TI-CC3200 Official Documentation](https://www.ti.com/product/CC3200)

Feel free to explore the individual lab folders for code samples, documentation, and detailed explanations of each lab's objectives and outcomes.

If you have any questions or would like to discuss anything related to this repository or the embedded systems class, please feel free to contact me at: almtorres805@gmail.com

Happy coding!
