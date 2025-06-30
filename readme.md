# NS1 Firmware
This is firmware to make a 62.5MS/s oscilloscope from a Pi Pico. This is done by utilizing the PIO and DMA on the RP2040 chip to clock in data at a very high rate.  Of course an analog front end and parallel ADC is also necessary. See [here](https://hackaday.io/project/197104-ns1-oscilloscope) for info on this hardware.
## Control
The oscilloscope system is controlled by a computer that the Pi Pico is connected to. This is done using basic serial over USB. The instrument control
software [voltpeek](https://github.com/schuyler4/voltpeek) provides a UI and Python API to control the oscilloscope. 
## Triggering
Currently there are two trigger modes, force trigger and normal trigger. Force trigger takes a snapshot of the signal when initiated, 
and normal trigger waits for an edge to capture. With normal trigger, there are equal number of points before and after the trigger 
event. That is, the trigger event is at the center of the capture. DMA ring buffers are utilized to accomplish this. Currently, 
rising edge and falling edge triggers are supported.      