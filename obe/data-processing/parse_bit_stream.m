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
fn      = "/home/matt/projects/wavespace/microphone-library-for-pico-forked/examples/hello_pdm_microphone/output.bin";   % your input file
fn = "/home/matthew/projects/wavespace/microphone-library-for-pico-forked/examples/hello_pdm_microphone/output.bin";
fs_in   = 4800000;               % PDM bit/sample rate (Hz). Change if not 3 MHz.
fc      = 80e3;             % desired low-pass cutoff (Hz)
bit_msbfirst= true;         % set false if your file is LSB-first
map_pm1     = true;         % true: {0,1}->{-1,+1}; false: keep {0,1}

## Decimation after the 100 kHz LPF. Keep fs_out >= ~2.5*fc (>=250 kHz for fc=100k)
dec_fac     = [];           % [] auto; or set integer (e.g., 12 -> 250 kHz)
target_mult = 2.5;          % Nyquist safety multiplier

## --- load & unpack bits ----------------------------------------------------
fid = fopen(fn, "rb"); assert(fid>=0, "Couldn't open %s", fn);
bytes = fread(fid, Inf, "uint8"); fclose(fid);

bits = zeros(numel(bytes)*8, 1, "uint8");
if bit_msbfirst
  for b = 1:8
    bits(b:8:end) = bitget(bytes, 9-b);   % MSB..LSB
  end
else
  for b = 1:8
    bits(b:8:end) = bitget(bytes, b);     % LSB..MSB
  end
end

x = double(bits);
if map_pm1, x = 2*x - 1; end             % map to -1/+1

## --- FIR design (Hamming-windowed sinc) -----------------------------------
trans   = 0.15 * fc;                      % transition width (Hz)
dfn     = trans / (fs_in/2);              % normalized transition (0..1)
N_est   = ceil(3.3/dfn);                  % Hamming rule-of-thumb
N       = max(512, 2*ceil(N_est/2));      % even order, >=128 taps
wc      = fc/(fs_in/2);                   % normalized cutoff

n  = (0:N).';
n0 = N/2;
m = n - n0;

h_ideal = wc * sinc(wc * m);
w = 0.54 - 0.46*cos(2*pi*n/N);
h = h_ideal .* w;
h = h / sum(h);                           % unity DC

## --- single-pass filtering + delay compensation ---------------------------
D = N/2;                                  % group delay
y_full = filter(h, 1, x);
if numel(y_full) > D
  y_filt = y_full(D+1:end);
else
  y_filt = y_full;
end

#figure(1)
#plot(y_filt)

scale = max(1, max(abs(y_filt)));
audiowrite("pdm_filtered.wav", y_filt/scale, fs_in);

## Remove DC to avoid clicks on playback
y_filt = y_filt - mean(y_filt);

## --- choose decimation factor ---------------------------------------------
if isempty(dec_fac)
  fs_target = target_mult * fc;           % e.g., 2.5 * 100 kHz = 250 kHz
  dec_fac   = max(1, floor(fs_in / fs_target));
end
fs_mid = fs_in / dec_fac;

## Decimate (after anti-aliasing above)
if dec_fac > 1
  y_mid = y_filt(1:dec_fac:end);
else
  y_mid = y_filt;
end

printf("In:  fs = %.3f MHz, samples = %d\n", fs_in/1e6, numel(x));
printf("LPF: fc = %.1f kHz, order = %d, trans ≈ %.1f kHz\n", fc/1e3, N, trans/1e3);
printf("Out: fs_mid = %.1f kHz (dec_fac = %d), samples = %d\n", fs_mid/1e3, dec_fac, numel(y_mid));

scale = max(1, max(abs(y_mid)));
audiowrite("pdm_filtered_decimated.wav", y_mid/scale, fs_mid);

## --- quick looks -----------------------------------------------------------
ms = 5e-3; nshow = min(numel(y_mid), round(fs_mid*ms));
figure(2); clf;
plot((0:nshow-1)/fs_mid*1e3, y_mid(1:nshow)); grid on;
xlabel('Time (ms)'); ylabel('Amplitude');
title(sprintf('Filtered (single-pass) @ %.0f kHz', fs_mid/1e3));

L = 8192; Hf = fft(h, L); f = (0:L-1)*(fs_in/L);
figure(3); clf;
plot(f/1e3, 20*log10(abs(Hf)+1e-12)); grid on; xlim([0, min(600, fs_in/2/1e3)]); ylim([-140,5]);
xlabel('Frequency (kHz)'); ylabel('Mag (dB)'); title('LPF magnitude (single-pass)');
## Optional: save WAV at fs_out (scaled)
##audiowrite('pdm_filtered.wav', y / max(1, max(abs(y))), fs_out);

