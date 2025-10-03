% MATLAB script to load and plot samples from a .inc file

% --- Parameters ---
   % your .inc file
Fs = 40000/11;              % Example: effective sample rate (Hz)
T_total = 10;       
% total playback time in seconds (optional override)

% --- Read file ---
rawText = fileread(filename);

% Remove curly braces and newlines
rawText = strrep(rawText, '{', '');
rawText = strrep(rawText, '}', '');
rawText = strrep(rawText, newline, ' ');

% Split into numbers
numStrings = strsplit(rawText, ',');
samples = str2double(numStrings);

% Remove NaNs (in case of trailing commas or blanks)
samples = samples(~isnan(samples));

% --- Build time vector ---
N = length(samples);
if exist('Fs','var') && ~isempty(Fs)
    t = (0:N-1)/Fs;  % time vector in seconds
elseif exist('T_total','var') && ~isempty(T_total)
    t = linspace(0, T_total, N);
else
    t = 0:N-1;  % just index if no Fs known
end

% --- Plot ---
figure;
plot(t, samples);
xlabel('Time (s)');
ylabel('Amplitude');
title(sprintf('Waveform from %s', filename));
grid on;
