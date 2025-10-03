% WAV to INC File Generator
% Generates 3 separate .inc files with 12-bit LUT arrays (0-4095)
% Output: drum.inc, piano.inc, guitar.inc

clear all;
close all;
clc;

%% Configuration
WAV_FILES = {'drum.wav', 'piano.wav', 'guitar.wav'};
ARRAY_NAMES = {'drum', 'piano', 'guitar'};
LUT_SIZE = 40000;  % 40,000 samples
BIT_RESOLUTION = 12;  % 12-bit DAC
MAX_VALUE = 2^BIT_RESOLUTION - 1;  % 4095
OFFSET = MAX_VALUE / 2;  % 2047.5 (DC offset)

%% Create figure for plotting
figure('Position', [100, 100, 1400, 400]);

%% Process each WAV file
for fileIdx = 1:length(WAV_FILES)
    
    % Read WAV file
    [audioData, fs] = audioread(WAV_FILES{fileIdx});
    
    % Convert to mono if stereo
    if size(audioData, 2) > 1
        audioData = mean(audioData, 2);
    end
    
    % Resize to LUT_SIZE samples using interpolation
    originalLength = length(audioData);
    x_original = 1:originalLength;
    x_new = linspace(1, originalLength, LUT_SIZE);
    audioData = interp1(x_original, audioData, x_new, 'linear');
    
    % Normalize to [-1, 1]
    audioData = audioData / max(abs(audioData));
    
    % Convert to 12-bit unsigned integer (0-4095)
    lut_12bit = round(audioData * OFFSET + OFFSET);
    lut_12bit = max(0, min(MAX_VALUE, lut_12bit));
    
    % Plot waveform (full size)
    subplot(1, 3, fileIdx);
    plot(lut_12bit);
    title(sprintf('%s - 12-bit LUT', upper(ARRAY_NAMES{fileIdx})));
    xlabel('Sample Index');
    ylabel('DAC Value');
    grid on;
    ylim([0, 4095]);
    
    % Generate .inc file
    inc_filename = sprintf('%s.inc', ARRAY_NAMES{fileIdx});
    fid = fopen(inc_filename, 'w');
    
    % Write array declaration
    fprintf(fid, 'const uint16_t %s[%d] = {\n', ARRAY_NAMES{fileIdx}, LUT_SIZE);
    
    % Write array values (12 values per line)
    for i = 1:length(lut_12bit)
        if mod(i-1, 12) == 0
            fprintf(fid, '    ');
        end
        
        if i < length(lut_12bit)
            fprintf(fid, '%4d, ', lut_12bit(i));
        else
            fprintf(fid, '%4d', lut_12bit(i));
        end
        
        if mod(i, 12) == 0 || i == length(lut_12bit)
            fprintf(fid, '\n');
        end
    end
    
    fprintf(fid, '};\n');
    fclose(fid);
end