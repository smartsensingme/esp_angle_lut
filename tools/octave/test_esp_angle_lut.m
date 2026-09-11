% TEST_ESP_ANGLE_LUT Run sensor-independent regression tests for the library.
%
% WHAT IT DOES
%   Verifies CRC compatibility, int16 serialization, LUT estimation and
%   correction at two sensor resolutions, and rejection of a nonmonotonic map.
%
% CALLED BY
%   A developer or automated test command. This file is a script rather than a
%   function and is not called internally by the reusable library.
%
% CALLS
%   esp_angle_lut_crc32_ieee(), esp_angle_lut_pack(),
%   esp_angle_lut_calculate(), esp_angle_lut_validate(), and
%   esp_angle_lut_apply().
%
% INPUTS/OUTPUTS
%   No external inputs and no returned value. A failed assertion stops with an
%   error; successful completion prints a confirmation message.

% Path block: make the reusable functions visible regardless of the directory
% from which this script was launched.
addpath (fileparts (mfilename ("fullpath")));

% CRC block: check the canonical ASCII test vector for CRC-32/IEEE.
known = esp_angle_lut_crc32_ieee (uint8 ("123456789"));
assert (known == uint32 (hex2dec ("CBF43926")));

% Serialization block: exercise zero, signed extrema, and both signs, then
% confirm esp_angle_lut_pack() calculates CRC over the emitted bytes.
[payload, crc] = esp_angle_lut_pack (int16 ([0, 1, -1, 32767, -32768]));
assert (numel (payload) == 10);
assert (crc == esp_angle_lut_crc32_ieee (payload));

% Synthetic-experiment block: create three constant-speed intervals and add a
% known deterministic angular nonlinearity at the second and fourth harmonics.
fs = 500;
t = (0:(24 * fs - 1))' / fs;
speed_rpm = [613 * ones(1, 8 * fs), 887 * ones(1, 8 * fs), ...
             619 * ones(1, 8 * fs)]';
true_turns = cumsum (speed_rpm / 60 / fs);
true_angle = mod (true_turns * 360, 360);
measured_angle = mod (true_angle + 0.8 * sin (2 * true_angle * pi / 180) + ...
                      0.45 * sin (4 * true_angle * pi / 180), 360);
segments = {(1:(8 * fs))', ((8 * fs + 1):(16 * fs))', ...
            ((16 * fs + 1):(24 * fs))'};

% Fourteen-bit block: calculate the default MT6701-scale table, require a large
% reduction in speed ripple, and independently validate its firmware rules.
calibration = esp_angle_lut_calculate (t, measured_angle, segments, 256, ...
                                       16384, 1024);
assert (calibration.speed_reduction_percent > 70);
assert (calibration.full_scale_counts == 16384);
[valid, report] = esp_angle_lut_validate ( ...
    calibration.correction_counts, calibration.full_scale_counts, ...
    calibration.max_abs_correction_counts);
assert (valid && report.monotonic);

% Accuracy block: apply the calculated LUT and compare the corrected angle with
% the ideal trajectory used to synthesize the measurement.
corrected = esp_angle_lut_apply (measured_angle, ...
                                 calibration.correction_counts, 16384);
phase_error = mod (corrected - true_angle + 180, 360) - 180;
assert (sqrt (mean (phase_error .^ 2)) < 0.15);

% Twelve-bit block: repeat estimation and application at another configured
% resolution to prove that the reusable algorithm is not tied to 14-bit data.
calibration_12_bit = esp_angle_lut_calculate (t, measured_angle, segments, ...
                                              256, 4096, 256);
corrected_12_bit = esp_angle_lut_apply ( ...
    measured_angle, calibration_12_bit.correction_counts, 4096);
phase_error_12_bit = mod (corrected_12_bit - true_angle + 180, 360) - 180;
assert (sqrt (mean (phase_error_12_bit .^ 2)) < 0.2);

% Rejection block: force one corrected step to reverse direction and confirm
% that host-side validation identifies the nonmonotonic angular map.
invalid = zeros (256, 1);
invalid(2) = -128;
[valid, report] = esp_angle_lut_validate (invalid, 16384, 1024);
assert (! valid && ! report.monotonic);

% Completion block: reaching this point means every assertion passed.
disp ("esp_angle_lut reusable Octave tests passed");
