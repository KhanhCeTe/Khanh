%% PCM System Simulator - Main Script
% Omar ElShazli - Omar Hagras - John Besada - Hala Refaie
% Modified to use WAV file input
clear; close all; clc;
%% User Input Section
% Default values
default_L = 8;          % Quantization levels (bits)
reader = dsp.AudioFileReader('odonnellyougogirl.wav');
disp('Reading audio file...');
input_signal = [];
fs = 0;
while ~isDone(reader)
    audio_frame = reader();
    if isempty(input_signal)
        fs = reader.SampleRate;
    end
    input_signal = [input_signal; audio_frame];
end
release(reader);
% If stereo, convert to mono by averaging channels
if size(input_signal, 2) > 1
    input_signal = mean(input_signal, 2);
end
answer='16';
    bits = str2double(answer);
    L = 2^bits;  % Convert bits to levels

% Fixed parameters
mp = 1;  % Peak amplitude (normalized)
mu = 255;  % μ-law parameter
encoding_type = 'polar';  % Encoding type

% Normalize input signal to range [-mp, mp]
input_signal = input_signal / max(abs(input_signal)) * mp;
% --- Channel Parameters ---
% Choose channel type: 'awgn', 'rayleigh', 'rician', 'impulsive'
channel_selection = 'awgn'; % <-- Đặt kênh Rayleigh tại đây
fd = 10; % Maximum Doppler Shift (Hz) for fading channels (e.g., 10 for slow movement)
K_factor = 3; % K-factor for Rician channel (dB). Higher K means stronger LoS.
SNR_range = -10:2:10; % Dải SNR từ -10 dB đến 10 dB, bước nhảy 2 dB.
ber_results = zeros(size(SNR_range)); % Mảng để lưu trữ BER cho mỗi SNR

impulsive_noise_probability = 0.01; % Probability of an impulse occurring (0 to 1)
impulsive_noise_amplitude_factor = 10; % Factor to multiply max signal amplitude for impulse (e.g., 10 means 10 times max signal)

% Create time vector
t = (0:length(input_signal)-1)' / fs;
% If the input signal is very long, consider using only a portion for visualization
max_duration = 2; % seconds
if t(end) > max_duration
    samples_to_use = ceil(max_duration * fs);
    input_signal = input_signal(1:min(samples_to_use, length(input_signal)));
    t = t(1:min(samples_to_use, length(t)));
end

%% Process Signal through PCM System (Preprocessing outside SNR loop)
% Sampling (input is already sampled at fs)
sampled_signal = input_signal;
t_sampled = t;

% Quantization
step_size = (2 * mp) / (L - 1);
quantized_signal = quantize_signal(sampled_signal, L, mp, mu);

% Encoding (This part also runs only once)
[encoded_signal, tinhtinh] = encode_signal(quantized_signal, L, mp, step_size, encoding_type);
original_bits_for_ber = tinhtinh(:); % Ensure it's a column vector for BER calculation
len_ber_total = length(original_bits_for_ber); % Total bits for BER comparison

% Define t_encoded based on the full encoded_signal
t_encoded = linspace(0, t(end), length(encoded_signal));

%% Loop for BER vs SNR Calculation
% This loop processes the signal through the channel for each SNR value
for snr_idx = 1:length(SNR_range)
    current_SNR_dB = SNR_range(snr_idx);

    disp(['--- Simulating with ', upper(channel_selection), ' Channel at SNR = ', num2str(current_SNR_dB), ' dB ---']);

    % --- Channel Modeling (AWGN, Rayleigh, Rician, Impulsive) ---
    % Note: comm.RayleighChannel and comm.RicianChannel require Communications Toolbox.
    % Input signal for fading channels needs to be a column vector.
    if strcmp(channel_selection, 'rayleigh')
        rayleigh_channel = comm.RayleighChannel( ...
            'SampleRate', fs, ...
            'PathDelays', 0, ... % Assuming simple flat fading for now
            'AveragePathGains', 0, ...
            'MaximumDopplerShift', fd, ...
            'Visualization', 'off');
        faded_signal = rayleigh_channel(encoded_signal(:)); % Input as column vector
        received_signal_current_snr = awgn(faded_signal, current_SNR_dB, 'measured'); % Add AWGN after fading
        received_signal_current_snr = received_signal_current_snr(:); % Ensure received_signal is column vector
        release(rayleigh_channel); % Release channel object resources

    elseif strcmp(channel_selection, 'rician')
        rician_channel = comm.RicianChannel( ...
            'SampleRate', fs, ...
            'PathDelays', 0, ...
            'AveragePathGains', 0, ...
            'KFactor', K_factor, ...
            'MaximumDopplerShift', fd, ...
            'Visualization', 'off');
        faded_signal = rician_channel(encoded_signal(:)); % Input as column vector
        received_signal_current_snr = awgn(faded_signal, current_SNR_dB, 'measured'); % Add AWGN after fading
        received_signal_current_snr = received_signal_current_snr(:); % Ensure received_signal is column vector
        release(rician_channel); % Release channel object resources

    elseif strcmp(channel_selection, 'impulsive')
        received_signal_current_snr = awgn(encoded_signal(:), current_SNR_dB, 'measured');
        
        % Add Impulsive Noise
        num_samples = length(received_signal_current_snr);
        impulse_peak_amplitude = impulsive_noise_amplitude_factor * max(abs(encoded_signal(:)));
        
        impulse_locations = rand(num_samples, 1) < impulsive_noise_probability;
        impulse_noise = zeros(num_samples, 1);
        impulse_noise(impulse_locations) = impulse_peak_amplitude * (2*rand(sum(impulse_locations),1)-1); 
        
        received_signal_current_snr = received_signal_current_snr + impulse_noise;
        received_signal_current_snr = received_signal_current_snr(:); % Ensure it's column vector
        
    else % Default to 'awgn' if channel_selection is not recognized or is 'awgn'
        received_signal_current_snr = awgn(encoded_signal(:), current_SNR_dB, 'measured'); % Ensure input is column vector
        received_signal_current_snr = received_signal_current_snr(:); % Ensure output is column vector
    end

    % Decoding
    % Only need decoded_binary for BER calculation in this loop
    [~, decoded_binary] = decode_signal(received_signal_current_snr, L, mp, step_size, encoding_type);

    % --- BER Calculation ---
    recovered_bits_for_ber = decoded_binary(:); % Ensure it's a column vector

    % Ensure lengths match before calculating BER
    min_len_ber = min(length(original_bits_for_ber), length(recovered_bits_for_ber));
    original_bits_for_ber_truncated = original_bits_for_ber(1:min_len_ber);
    recovered_bits_for_ber_truncated = recovered_bits_for_ber(1:min_len_ber);

    % Use biterr function (requires Communications Toolbox)
    [numErrors_manual, ber_manual] = biterr(original_bits_for_ber_truncated, recovered_bits_for_ber_truncated);

    % Store BER result in the array
    ber_results(snr_idx) = ber_manual;

    disp(['Total bits compared: ', num2str(min_len_ber)]); % Show actual bits compared
    disp(['Bit errors: ', num2str(numErrors_manual)]);
    disp(['BER: ', num2str(ber_manual, '%.4e')]);
    fprintf('\n'); % Add empty line for readability

    % If you want to plot Constellation Diagram or Reconstructed Signal for a SPECIFIC SNR,
    % you would do it here or save these signals to an array.
    % For now, we'll use the last SNR's results for the final Constellation/Reconstructed plots.
end

%% Plotting BER vs SNR Curve (THIS IS THE PRIMARY PLOT YOU ARE LOOKING FOR)
figure('Name', ['BER vs SNR Curve for ' upper(channel_selection) ' Channel'], 'NumberTitle', 'on'); % Explicitly name figure
semilogy(SNR_range, ber_results, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6);
hold on;
grid on;

xlabel('SNR (dB)');
ylabel('Bit Error Rate (BER)');
title(['BER vs SNR for ', upper(channel_selection), ' Channel']);
xlim([min(SNR_range), max(SNR_range)]);
% Adjust ylim based on expected BER values. 1e-6 to 1 is a good general range.
ylim([max(1e-6, min(ber_results)/10), 1]); % Dynamically adjust lower limit, but not below 1e-6

% Add Theoretical BER for BPSK (for Polar NRZ) on AWGN channel
if strcmp(channel_selection, 'awgn') && strcmp(encoding_type, 'polar')
    Eb_No_linear = 10.^(SNR_range/10);
    ber_theoretical_bpsk_awgn = 0.5 * erfc(sqrt(Eb_No_linear));
    semilogy(SNR_range, ber_theoretical_bpsk_awgn, 'r--', 'LineWidth', 1.5);
    legend('Simulated BER', 'Theoretical BER (BPSK, AWGN)', 'Location', 'southwest');
else
    legend('Simulated BER', 'Location', 'southwest');
end
hold off; % Release hold after adding all elements to this figure

%% Other Plotting (Signal Waveforms and Constellation Diagram - using data from LAST SNR iteration)
% These figures are for visualization and will use the data from the last SNR run in the loop.

% Get reconstructed_signal and received_signal from the last SNR iteration
% (This is necessary because these variables were updated inside the loop)
[reconstructed_signal_last_snr, decoded_binary_last_snr] = decode_signal(received_signal_current_snr, L, mp, step_size, encoding_type);


% Figure 2: Source and Sampled Signal (No change)
figure('Name', 'Source and Sampled Signal', 'NumberTitle', 'on');
plot(t, input_signal, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_sampled, sampled_signal, 'r.', 'MarkerSize', 8); % Using 'r.' for sampled points
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Source and Sampled Signal');
legend('Source Signal', 'Sampled Signal');
hold off;

% Figure 3: Sampled and Quantized Signal
figure('Name', ['Quantized Signal (' num2str(log2(L)) ' bits)'], 'NumberTitle', 'on');
plot(t_sampled, sampled_signal, 'b-', 'LineWidth', 1);
hold on;
plot(t_sampled, quantized_signal, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title(['Quantized Signal (' num2str(log2(L)) ' bits)']);
legend('Original', 'Quantized');
hold off;

% Figure 4: Quantization Noise
figure('Name', ['Quantization Noise (' num2str(log2(L)) ' bits)'], 'NumberTitle', 'on');
plot(t_sampled, sampled_signal - quantized_signal, 'g-', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title(['Quantization Noise (' num2str(log2(L)) ' bits)']);
ylim([-step_size, step_size]);
hold off;

% Figure 5: Encoded Signal
figure('Name', 'Encoded Signal', 'NumberTitle', 'on');
plot(t_encoded, encoded_signal, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Amplitude');
title(['Encoded Signal (' encoding_type ')']);
hold off;

% Figure 6: Source and Reconstructed Signal
figure('Name', 'Source and Reconstructed Signal', 'NumberTitle', 'on');
plot(t, input_signal, 'c-', 'LineWidth', 1.5);
hold on;
plot(t, reconstructed_signal_last_snr, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Amplitude');
title('Source and Reconstructed Signal');
legend('Source Signal', 'Reconstructed Signal');
hold off;

% Figure 7: Frequency Domain Analysis
figure('Name', 'Frequency Domain Analysis', 'NumberTitle', 'on');
% Source signal spectrum
subplot(3,1,1);
[pxx_source, f_source] = periodogram(input_signal, [], [], fs);
plot(f_source, 10*log10(pxx_source));
grid on;
title('Source Signal Spectrum');
xlabel('Frequency (Hz)'); ylabel('Power/Frequency (dB/Hz)');
% Reconstructed signal spectrum
subplot(3,1,2);
[pxx_recon, f_recon] = periodogram(reconstructed_signal_last_snr, [], [], fs);
plot(f_recon, 10*log10(pxx_recon));
grid on;
title('Reconstructed Signal Spectrum');
xlabel('Frequency (Hz)'); ylabel('Power/Frequency (dB/Hz)');
% Quantization noise spectrum
subplot(3,1,3);
[pxx_noise, f_noise] = periodogram(sampled_signal - quantized_signal, [], [], fs);
plot(f_noise, 10*log10(pxx_noise));
grid on;
title('Quantization Noise Spectrum');
xlabel('Frequency (Hz)'); ylabel('Power/Frequency (dB/Hz)');
hold off;


% Figure 8: Constellation Diagram (Chòm sao tín hiệu)
figure('Name', 'Constellation Diagram of Received Symbols (Last SNR)', 'NumberTitle', 'on');
% The 'received_signal_current_snr' from the last iteration is used here
current_samples_per_bit = 50; % This value needs to match your encode_signal/decode_signal implementation
num_total_bits_const = floor(length(received_signal_current_snr) / current_samples_per_bit);
received_symbol_values_const = zeros(num_total_bits_const, 1);
original_transmitted_symbols_const = zeros(num_total_bits_const, 1); 

for i = 1:num_total_bits_const
    bit_start = (i - 1) * current_samples_per_bit + 1;
    bit_end = bit_start + current_samples_per_bit - 1;
    received_symbol_values_const(i) = mean(received_signal_current_snr(bit_start:bit_end));
    if (i <= length(tinhtinh))
        original_transmitted_symbols_const(i) = (2 * tinhtinh(i)) - 1; % Convert 0/1 to -1/1
    else
        original_transmitted_symbols_const(i) = NaN; % If tinhtinh is shorter, handle NaN
    end
end

% Remove NaNs if any
valid_indices_const = ~isnan(original_transmitted_symbols_const);
received_symbol_values_const = received_symbol_values_const(valid_indices_const);
original_transmitted_symbols_const = original_transmitted_symbols_const(valid_indices_const);

% Plot received symbols (blue dots)
plot(received_symbol_values_const, zeros(size(received_symbol_values_const)), 'b.', 'MarkerSize', 5);
hold on;
grid on;

% Plot ideal constellation points (red circles) for Polar NRZ (-1 and +1)
plot([-1, 1], [0, 0], 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('In-Phase Component (I)');
ylabel('Quadrature Component (Q)'); % For Polar NRZ, Q component is 0
title(['Constellation Diagram of Received Symbols (Last SNR: ' num2str(current_SNR_dB) ' dB)']);
legend('Received Symbols', 'Ideal Constellation Points', 'Location', 'southeast');
xlim([-1.5, 1.5]); % Adjust x-axis limits for better visibility
ylim([-0.5, 0.5]); % Limit y-axis near 0 as there's no Q component for Polar NRZ
hold off;


%% Final Error Metrics (from the last SNR iteration)
% Mean Square Error and SNR
mse = mean((input_signal - reconstructed_signal_last_snr).^2);
snr_output = 10 * log10(mean(input_signal.^2) / mse); % Renamed to avoid confusion with current_SNR_dB

% Calculate quantization noise and SQNR
quantization_noise = sampled_signal - quantized_signal;
quantization_noise_power = mean(quantization_noise.^2);
signal_power = mean(sampled_signal.^2);
sqnr = 10 * log10(signal_power / quantization_noise_power);

% Theoretical SQNR for uniform quantization = 6.02*bits + 1.76 dB
bits_per_sample = log2(L); % Renamed to avoid confusion with L
theoretical_sqnr = 6.02 * bits_per_sample + 1.76;

disp('--- Final Metrics (from last SNR iteration) ---');
disp(['Mean Square Error: ', num2str(mse)]);
disp(['Signal-to-Noise Ratio (dB) (Output): ', num2str(snr_output)]);
disp(['Signal-to-Quantization Noise Ratio (dB): ', num2str(sqnr)]);
disp(['Theoretical SQNR for uniform quantization (dB): ', num2str(theoretical_sqnr)]);
disp(['Bits per sample: ', num2str(bits_per_sample)]);
