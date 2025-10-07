function plot_samples(filename, playback_time)
% plot_samples(filename, playback_time)
% Plots integer samples vs time assuming zero-order hold playback.

    if nargin < 2
        error('Usage: plot_samples(filename, playback_time)');
    end
    if ~isfile(filename)
        error('File not found: %s', filename);
    end

    % --- Read file ---
    txt = fileread(filename);
    nums = regexp(txt, '[-+]?\d+', 'match');
    samples = str2double(nums);

    if isempty(samples)
        error('No integer data found in %s', filename);
    end

    % --- Time axis ---
    N = length(samples);
    dt = playback_time / N;
    t = 0:dt:playback_time;   % N+1 points

    % --- Plot with proper step alignment ---
    figure;
    stairs(t, [samples samples(end)], 'LineWidth', 1.2);
    xlabel('Time (s)');
    ylabel('Amplitude (integer value)');
    title(sprintf('Samples from %s', filename), 'Interpreter', 'none');
    grid on;

    % --- Show info ---
    fs = N / playback_time;
    fprintf('Samples: %d\nPlayback time: %.6f s\nSample rate: %.3f Hz\n', ...
        N, playback_time, fs);
end
