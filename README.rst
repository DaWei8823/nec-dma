Goal
****
The goal of this project is to psoc62 firmware for recording, decoding and replaying infrared singals using the NEC protocol.  The NEC protocol is a common protocol for transmiting information from remote controls for TVs, air-conditions ect.

Implementation
**************
The implementation is designed to keep the CPU asleep most of the time. The main loop sets up an interrupt on the gpio connected to the infrared reciever.  The CPU then goes to sleep. An infrared transmition changes the state of this gpio and triggers the interrupt.  In the corresponding isr, we setup a timer with the TCPWM peripheral and a dma transfer to be triggered on sbsequent activity on the gpio. The source of this dma transfer is the counter of this TCPWM and the destination a buffer, which will be filled with the tickstamps of when the gpio state changed. The CPU stays asleep during these transmitions.  When the dma transfers complete, the CPU is woken up to process the samples in the buffer.  


Hardware
********
We use the CY8CKIT-062S4 board and an infrared receiver connected to a gpio.

Building
********
This project requires the cypress sdk from the ModusToolbox. run `make get-libs` on first build. Then you can just run `make build`


Flashing
********
Run `make program` with board connected to host over micro-usb

