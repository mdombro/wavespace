The numbers line up once you separate the three choke points:

PDM → SPI streamer: During the run the Pico built 53 348 frames but only pushed 45 323 onto MISO. The difference (7 5xx) shows up in the CLI as drop/underrun, meaning those payloads never left the RP2040—queue overflow or CS abort made us throw them away. That’s a big problem, but even if the Pi were perfect you’d only ever see 45 k frames, not 200 k.

SPI wire → raw dump: Of the ~45 k frames that did reach the bus, only 6 614 survive in raw_frames.bin. That means the capture helper, Pi kernel, or wiring is corrupting almost everything—either the Pi stops clocking after a few seconds, the helper dies early, or SPI noise is flipping lots of bits so header CRC checks fail. We know the file is ~65 MB, so the Pi did clock out a ton of bytes, but most aren’t valid frames anymore (headers show absurd payload lengths). That’s classic signal-integrity or clock mismatch: if SCK is too fast, the Pico underflows and the host sees garbage; if MOSI/MISO pairs are ringing, bytes get mangled.

Parser: count_spi_headers sees 6 834 sync words, 6 614 valid. That matches the “usable” frames the post-processor reports (529 in your earlier test because it gave up after hitting continuous garbage). The parser isn’t your bottleneck—it’s faithfully rejecting corrupted frames.

So what’s happening to all those frames? They die in two places before parsing:

~7 500 frames are dropped on the Pico because it can’t queuem/dma them fast enough. Fix: increase SPI_STREAMER_FRAME_QUEUE_DEPTH (and buffer sizes), ensure spi_streamer_task() runs constantly, and match the host’s SPI clock to your production rate so the FIFO never backs up. The new stats already show task_last/task_avg = 0 because you never call spi_streamer_task() while streaming is disabled—that’s another hint your CLI isn’t running the task loop often.

Of the remaining ~45 k frames, ~38 k get mangled in transit. Either the Pi is reading with the wrong SPI mode/speed, the capture helper is still requesting 20 frames per transaction and bailing on short reads (so 19 frames of data are identical to the previous block), or your wiring is so noisy that only 15 % of frames survive with intact headers/CRC. Use an oscilloscope or logic analyser to verify that MISO/CLK actually carry the 0xA5 0x5A header pattern; right now they clearly don’t.

To debug, tackle it in order:

On the Pico, leave streaming enabled and call spi_streamer_task() continuously (e.g., from hello_pdm_microphone main loop) while monitoring stats. If frames_built ≈ frames_sent and drop=underrun=0, move on. If not, increase the queue and check the data path feeding spi_streamer_push_block().

On the Pi, capture a short run with a slower SPI speed (1 MHz) and --frame-bytes 530. See if the parser now recovers all frames. If yes, bump the speed until errors reappear—this tells you the practical envelope.

If you still lose frames, hook up a logic analyser to CS/SCK/MISO and confirm the RP2040 is actually outputting valid frames and that CS never toggles unexpectedly. With tied-low CS, you must also disable the GPIO IRQ or it will still see edges from the Pi host.

Until both steps (Pico transmission and Pi capture) are solid, the post-processor will continue to report only a fraction of the expected frames because most of them never arrive intact.







SPI streamer

microphone-library-for-pico-forked/src/spi_streamer.c (lines 117-118) copies the frame header and then the completed 512-byte payload into the queue slot before DMA pushes it to the PIO. This is required because each queued frame needs a contiguous header+payload+CRC buffer. Optimization: precompute the static header template once and use memcpy only for the payload (with memcpy(frame->buffer + header_len, …) as today) or use memcpy-like DMA (e.g., dma_channel_configure in memory-to-memory mode) to offload the copy if CPU load becomes an issue. Another option is to build frames directly into the queue buffers (see next point) to avoid the second copy.
microphone-library-for-pico-forked/src/spi_streamer.c (line 290) copies new PDM data into partial_payload until a full payload is accumulated. This copy is unavoidable if data arrives in arbitrary chunk sizes, but you could eliminate it by writing directly into the queue buffer: when a block arrives, point partial_payload at the current queue frame and fill it in place. That would require refactoring queue management so partially-filled frames live in the ring instead of a separate array.
Overall for the streamer, the biggest optimization would be reducing duplicate copies by giving each queue entry a “fill pointer” and letting spi_streamer_push_block() write straight into the ring. A smaller tweak is to replace the header memcpy with a memcpy-free struct assignment (since the header is a packed struct) or pre-fill the static fields and only write the changing ones (start bit, flags, channel mask).
PDM/Analog microphone drivers

microphone-library-for-pico-forked/src/pdm_microphone.c (line 61) and src/analog_microphone.c (line 36) simply copy user-provided configuration structs into global state during initialization. These happen once per init, so no real optimization is needed; zero-cost alternative is pdm_mic.config = *config; if the struct allows direct assignment.
microphone-library-for-pico-forked/src/pdm_microphone.c (line 434) copies raw PDM samples from the DMA buffer into the user’s destination buffer each time a capture completes. To avoid this copy, expose the DMA buffer directly to the caller or use double-buffering with pointers handed off to the consumer instead of copying bytes.
USB descriptor

microphone-library-for-pico-forked/examples/usb_microphone/usb_descriptors.c (line 136) copies the language ID string into a descriptor buffer; this is trivial and doesn’t need optimization.
hello_pdm_microphone example

microphone-library-for-pico-forked/examples/hello_pdm_microphone/main.c (line 883) copies a captured PDM packet into the next ring slot. This is required because the producer hands over ownership to the ring. To optimize, allocate the packets in-place inside the ring and fill them directly instead of copying from a temporary.
main.c (line 910) copies a packet into the USB payload buffer before sending. You could switch to scatter/gather (queue pointers instead of copies) or reuse the same buffer if USB is the only consumer.
Similar copies appear in the main-discrete-measurements.c variant (lines ~660, 684, 744, 1012). The general optimization is the same: avoid staging buffers and write directly into the final destination or hand off buffer ownership rather than copying.
Summary
Most memcpy calls exist because the current design stages data in temporary buffers (e.g., partial_payload, packet structs) before handing it to the next subsystem. To reduce drops:

Refactor the SPI streamer so spi_streamer_push_block() fills payloads directly in the queue entries; this removes the partial_payload copy and halves memory bandwidth.
Increase SPI_STREAMER_FRAME_QUEUE_DEPTH (and SRM usage) so temporary surges don’t overflow the ring.
In data paths where you own both producer and consumer buffers (PDM capture ring, USB queue), replace “copy into slot” with “rotate pointer ownership” so heavy memcpys go away entirely.