We are less worried about how long after gantry signal fires to pico starting a measurement - the travel speed is low and absolute positioning sample to sample is not a huge requriement. Even latching the position trigger to a timestamp is less interesting, its kinda whenever the pico gets to doing it. What is more critical is the time of the pulse being sent out and the relevant samples being captured and marked for retrieval

- Make the PDM reader dual channel - can probably mess with the pin mappings? 
- Make a ISR that outputs voltage to speaker for customizable time length on RX on digital pin - 5ms at start
- A periodic tick pin that happens at some interval that correlates to a specific distance traveled on the z-axis. Needs to last only a short amount of time
- ~~A blanking capability so the recording halts while it's high - only needs to affect how data is forwarded over usb or whatever~~
- ~~Either break up data capture by periodic tick and timestamp and location mark them - this would require a struct being created and changing how the data is sent over USB~~
- [x] OR need to mark samples, starting on periodic tick and lasting for 20ms or whatever timing is picked
- make the capture dual channel for two mics, need to combine them in some way???
- Path planning to map indices to positions





Optimizations:
That 2 048-byte gap every ~100 frames means the Pi still pauses long enough for the Pico’s eight-frame SPI queue to drain and drop one block. You’re close, but the pipeline still hiccups occasionally. Here’s how to eliminate those stragglers:

Extend the Pico queue depth. Right now SPI_STREAMER_FRAME_QUEUE_DEPTH is 8. Bump it to 32 (or more) in spi_streamer.h and recompile the firmware. That gives the Pico more breathing room whenever the Pi sleeps for a few milliseconds (e.g., due to scheduler jitter).

Clock multiple frames per transfer. Modify spi_capture_helper.c to set transfer.len = frame_bytes * batch_count (say batch of 32) and loop inside C to split the RX buffer into 530-byte blocks before writing to stdout. That keeps chip-select low for longer stretches and drastically reduces inter-frame dead time.

Increase the queue / reduce flush interval in Python. Use queue.Queue(maxsize=0) (already unbounded) but allocate a larger pending buffer and flush metadata less often (e.g., SPI_METADATA_FLUSH_INTERVAL = 256) so disk I/O can’t stall the consumer.

Prioritize the helper process. Launch spi_capture_helper with nice -n -10 or chrt --fifo to keep it from being preempted, and pin it (plus the Python reader thread) to a dedicated core using taskset. The small gaps you’re seeing are often caused by the Pi’s scheduler freezing your process for a tick.

Measure and log. Add instrumentation: count how many frames were queued when each gap occurred, log helper timing, etc. If you can detect queue depth approaching the limit, you can warn when the host falls behind.

Combining these (especially the first two) should eliminate the periodic 16 384-bit drops. The key is to give the Pico a deeper buffer and keep the Pi clocking without per-frame pauses.