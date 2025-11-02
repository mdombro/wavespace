##fid = fopen("/home/matt/projects/wavespace/microphone-library-for-pico-forked/raw_pdm.bin", "rb");
##bytes = fread(fid, Inf, "uint8");      % each element = one byte (8 bits)
##fclose(fid);
##
##% Unpack bits (MSB first)
##bits = zeros(numel(bytes)*8, 1, "uint8");
##for b = 1:8
##  bits(b:8:end) = bitget(bytes, 9-b);  % take bit 8,7,...,1 in that order
##end




## pdm_read_filter.m
## Read a raw 1-bit PDM stream (packed in bytes), low-pass filter at 100 kHz,
## and optionally decimate to a reasonable PCM rate.

## --- user settings ---------------------------------------------------------
fn      = "/home/matt/projects/wavespace/microphone-library-for-pico-forked/raw_pdm.bin";   % your input file
fs_in   = 2048000;               % PDM bit/sample rate (Hz). Change if not 3 MHz.
fc      = 100e3;             % desired low-pass cutoff (Hz)
dec_fac = [];                % leave [] to auto-pick; or set integer (e.g., 10)

## --- load & unpack bits ----------------------------------------------------
fid = fopen(fn, "rb");
assert(fid >= 0, "Couldn't open file: %s", fn);
bytes = fread(fid, Inf, "uint8");
fclose(fid);

bits = zeros(numel(bytes)*8, 1, "uint8");
for b = 1:8
  bits(b:8:end) = bitget(bytes, 9-b);   % MSB first
end
x = 2*double(bits) - 1;                 % map {0,1} -> {-1,+1}

## --- FIR design (Hamming-windowed sinc) -----------------------------------
## Choose transition width ~ 0.2*fc (adjust for sharper/looser filter)
trans   = 0.2 * fc;
dfn     = trans / (fs_in/2);                 % normalized transition (0..1)
% Rough order guess for Hamming window
N_est   = ceil(3.3/dfn);                     % rule of thumb
N       = max(64, 2*ceil(N_est/2));          % even order for nice symmetry

wc      = fc/(fs_in/2);                      % normalized cutoff (0..1)

% Build ideal sinc LPF (length N+1, symmetric about center n0 = N/2)
n  = (0:N).';
n0 = N/2;
m  = n - n0;
% ideal lowpass impulse response (normalized to Nyquist = 1)
h_ideal = wc * sinc(wc * m);                 % Octave sinc(x) = sin(pi x)/(pi x)

% Hamming window coded inline (no toolboxes)
w = 0.54 - 0.46*cos(2*pi*n/N);

% Windowed-sinc FIR
h = h_ideal .* w;

% Normalize gain to ~1 at DC
h = h / sum(h);

## --- zero-phase forward-backward filtering (no filtfilt) -------------------
% Forward
y1 = filter(h, 1, x);
% Backward
y2 = filter(h, 1, flipud(y1));
% Flip back
y_filt = flipud(y2);

## --- decimation factor -----------------------------------------------------
if isempty(dec_fac)
  fs_target = 2.5 * fc;                 % keep Nyquist comfortably above fc
  dec_fac   = max(1, floor(fs_in / fs_target));
end
fs_out = fs_in / dec_fac;

if dec_fac > 1
  y = y_filt(1:dec_fac:end);
else
  y = y_filt;
end

printf("PDM file:     %s\n", fn);
printf("Samples (in): %d  (fs_in = %.3f MHz)\n", numel(x), fs_in/1e6);
printf("FIR order:    %d  (fc = %.1f kHz)\n", N, fc/1e3);
printf("Decimation:   %d -> fs_out = %.1f kHz\n", dec_fac, fs_out/1e3);
printf("Samples (out): %d\n", numel(y));

## --- quick peeks -----------------------------------------------------------
% first few ms
ms = 5e-3;
nshow = min(numel(y), round(fs_out * ms));
figure(1); clf;
plot((0:nshow-1)/fs_out*1e3, y(1:nshow));
xlabel('Time (ms)'); ylabel('Amplitude'); grid on;
title('First few ms (filtered + decimated)');

% magnitude response via FFT (no freqz)
L = 4096;
Hf = fft(h, L);
f  = (0:L-1) * (fs_in / L);
magdb = 20*log10(abs(Hf)+1e-12);
figure(2); clf;
plot(f/1e3, magdb);
xlim([0, min(600, fs_in/2/1e3)]); ylim([-140 5]); grid on;
xlabel('Frequency (kHz)'); ylabel('Mag (dB)');
title('LPF magnitude response (FFT of h)');

## Optional: save WAV at fs_out (scaled)
audiowrite('pdm_filtered.wav', y / max(1, max(abs(y))), fs_out);

