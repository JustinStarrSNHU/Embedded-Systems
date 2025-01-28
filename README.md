# Embedded-Systems

Project Located in M7 directory <br><br>

M1/ Preparing you Ti Board<br>
M2/ Milestone 1 - pwmled2.c<br>
M3/ Milestone 2 - uart2echo.c<br>
M4/ Journal<br>
M5/ Milestone 3 - gpiointerrupt.c / SM Diagram<br>
M7 Project Report - SM Diagrams / gpiointerrupt.c<br>


## Summarize the project and what problem it was solving.

- For this project, we were tasked with creating a smart thermostat using the TI SimpleLink Wi-Fi- CC3220S wireless microcontroller LaunchPad development kit. The final goal was to create a prototype of the low-level thermostat that would demonstrate its functionality. For the prototype, the TMP006 temperature sensor on the development board was used to read the room temperature (via I2C), an LED to indicate the output to the thermostat where LED on = heat on (via GPIO), two buttons to increase and decrease the set temperature (via GPIO interrupt), and the UART to simulate the data being sent to the server. Also, a written report was created for the team to ensure that the system that was created was based on SysTec's business requirements and technical specifications. In the report, it was asked to architect the next phase of the project, which is to connect the thermostat to the cloud. Various hardware architectures were analyzed (from TI, Microchip, and Freescale) to recommend and justify the architecture decision.

## What did you do particularly well?

- I believe I did well in just about all aspects of the coursework. I was able to create a task scheduler that would be used to perform the various tasks that the thermostat needed to be able to do. I broke each function of the thermostat down into individual tasks and then created state machines that controlled the transitions and actions for each of the tasks. I was able to create SM diagrams to depict the actions of the task scheduler and state machines. I successfully coded the project, applying industry standards and best practices, and demonstrated its functionality successfully.

## Where could you improve?

- I believe the biggest area where I could improve is analyzing the various architectures of embedded systems.

## What tools and/or resources are you adding to your support network?

- I have gained valuable knowledge of how to successfully download and install the necessary IDE and SDK to complete work on a development board. I have learned how to better navigate an IDE to open declarations and a type hierarchy to learn how peripheral functions work. I learned how to successfully design state machine diagrams to demonstrate the functionality of an SM. Additionally, I have added to my knowledge of analyzing client requirements and ensuring I have met those requirements with thorough testing. Also, I have not previously worked with the C programming language and this was an excellent opportunity to expand my knowledge of various programming languages, how they work, and what their similarities and differences are.

## What skills from this project will be particularly transferable to other projects and/or coursework?

- I think the biggest thing that will be transferrable in the future is the ability to implement task schedulers in my code to accomplish more than one task at the various times they are supposed to happen. I also learned how to use the enum type better and how beneficial it can be when performing transitions and actions within a state machine. One of the biggest skills that has continued to be beneficial for me when completing projects and/or coursework is having the ability to fully understand the requirements. This will continue to transfer not only in my academics but also in my future career as the need to be able to understand client's requirements is paramount.

## How did you make this project maintainable, readable, and adaptable?

- One of the biggest ways a developer can make their code maintainable and readable is to ensure they are following industry standards and best practices. This includes practicing standard naming conventions and using in-line comments to document the code. This makes it so others can easily read your code and understand what is happening. As it concerns adaptability, one way that it can be adapted is by looking at the task scheduler. If we wanted the thermostat to have additional functionality, we could simply create new tasks with an SM for those tasks that the task scheduler would then also check. Also, having the DISPLAY macro made outputting data to the server (UART) more efficient by constructing how the data should be formatted. This saves time when calling the DISPLAY macro because you do not have to include the formatting each time you want to output information to the UART.
