**Where The Time Goes**

* `pdm_microphone_read_with_metadata` now calls pdm_unpack_buffer, which walks every bit, re-writes channel data, and tracks trigger state (src/pdm_microphone.c (line 343) and src/pdm_microphone.c (line 410)). That per-bit loop touches ~24 k iterations per block (8 k samples × 3 streams) with multiple function calls/branches, so the caller sits inside the ISR-critical section much longer. While that happens, on_pdm_samples_ready cannot drain DMA quickly enough, so block_pending stays true and subsequent DMA completions drop into the discard path, inflating “discarded” stats.
* Main loop indirectly slows because it waits on block_pending clearing; as soon as the bit-unpacking work dominates, the loop spends more time idling, and overruns show up.

**How To Pinpoint The Hot Spot**

* Instrument pdm_unpack_buffer: wrap the loop with GPIO toggles or cycle counters (e.g., use time_us_64() or cpuid_t counters) to log µs per block when trigger capture is on vs off.
* Track `pdm_microphone_read_with_metadata` duration by grabbing timestamps around it inside on_pdm_samples_ready. If disabling trigger capture drops the duration back to a few hundred µs, you’ve confirmed the new loop is the culprit.
* Watch DMA IRQ latency: add a counter for how many times the handler finds raw_buffer_ready_count == PDM_RAW_BUFFER_COUNT to catch queue overruns caused by slow consumers.
* Use the existing CLI stats as a coarse indicator; enable periodic stats and note how blocks_ready vs blocks_discarded changes as you toggle trigger capture.

**Optimization Directions**

1. Avoid Per-Bit CPU Work

    * Let DMA write channel bits straight into the existing channel-aligned layout: use two DMA channels chained—first transfers 2 kB of channel data, second transfers 1 kB of trigger bits into a side buffer. That removes the need to unpack anything in software.
    * Alternatively, keep the combined buffer but split it by DMA reconfiguring write address/counter once every sample burst (PIO shift out channel bits to FIFO 0, trigger bit to FIFO 1) so the CPU reads already-separated streams.
2. Lighter-weight Unpacking

    * Operate on 32-bit chunks instead of bit-by-bit: precompute masks to scatter trigger bits while copying bytes via memcpy, or use pio_sm_get_blocking in 32-bit mode to align data.
    * Use lookup tables (256 entries) that map “byte of interleaved bits” to “two bytes of channel bits + trigger mask info” to shrink loops dramatically.
3. Defer Trigger Analysis

    * Instead of rewriting the buffer, leave raw interleaved bytes and analyze trigger bits later in the host script. Add metadata about trigger-carry state plus an offset pointer so Python can parse it offline, eliminating the firmware penalty.
4. Limit Work Inside IRQ Context

    * Move pdm_unpack_buffer outside the interrupt-protected section: copy the raw buffer pointer to a queue and let the main loop perform unpacking, so DMA IRQ can fire again quickly.
    * If unpacking must stay near capture, add a third raw buffer so the ISR can hand off faster even when processing lags.
5. Reduce Metadata Resolution

    * If block-level trigger index suffices, have PIO detect the first rising edge via IRQ or a small state machine (e.g., wait 1 pin into irq). That way the firmware only records an index counter without rewriting the data buffer.


A practical next step: add timing logs around pdm_unpack_buffer, confirm it’s taking milliseconds, then prototype either the dual-DMA separation or moving unpacking out of the ISR to see which gives acceptable throughput.