We are less worried about how long after gantry signal fires to pico starting a measurement - the travel speed is low and absolute positioning sample to sample is not a huge requriement. Even latching the position trigger to a timestamp is less interesting, its kinda whenever the pico gets to doing it. What is more critical is the time of the pulse being sent out and the relevant samples being captured and marked for retrieval

- Make the PDM reader dual channel - can probably mess with the pin mappings? 
- Make a ISR that outputs voltage to speaker for customizable time length on RX on digital pin - 5ms at start
- A periodic tick pin that happens at some interval that correlates to a specific distance traveled on the z-axis. Needs to last only a short amount of time
- ~~A blanking capability so the recording halts while it's high - only needs to affect how data is forwarded over usb or whatever~~
- ~~Either break up data capture by periodic tick and timestamp and location mark them - this would require a struct being created and changing how the data is sent over USB~~
- [x] OR need to mark samples, starting on periodic tick and lasting for 20ms or whatever timing is picked
- make the capture dual channel for two mics, need to combine them in some way???
- Path planning to map indices to positions
