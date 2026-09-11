function calibration = esp_angle_lut_calculate (time_s, raw_angle_deg, ...
                                                  segments, bin_count, ...
                                                  full_scale_counts, ...
                                                  max_abs_correction_counts)
  % ESP_ANGLE_LUT_CALCULATE Estimate and validate an angular correction LUT.
  %
  % calibration = esp_angle_lut_calculate (time_s, raw_angle_deg, segments, ...
  %                                         bin_count, full_scale_counts, ...
  %                                         max_abs_correction_counts)
  %
  % WHAT IT DOES
  %   Estimates the deterministic angular error of an absolute sensor from
  %   stable, approximately constant-speed rotations. Alternating revolutions
  %   are used for training and validation. The resulting periodic correction
  %   is filtered, quantized, validated with the firmware rules, serialized,
  %   and returned with before/after quality metrics.
  %
  % CALLED BY
  %   External calibration applications and test_esp_angle_lut.m. No other
  %   public function in this library calls it.
  %
  % CALLS
  %   Private helpers validate_configuration(), revolution_errors(),
  %   bin_median(), fill_circular(), circular_harmonic_smooth(),
  %   interp_cyclic(), and wrap_pi(); public functions
  %   esp_angle_lut_validate(), esp_angle_lut_apply(), and
  %   esp_angle_lut_pack().
  %
  % INPUTS
  %   time_s                    - Strictly increasing sample times in seconds.
  %   raw_angle_deg             - Raw cyclic sensor angle in degrees.
  %   segments                  - Cell array of index vectors. Each vector
  %                               selects one independent stable-speed interval.
  %   bin_count                 - LUT entry count. Optional; defaults to 256.
  %   full_scale_counts         - Sensor counts/revolution. Optional; 16384.
  %   max_abs_correction_counts - Firmware safety bound. Optional; 1024.
  %
  % OUTPUT
  %   calibration - Structure containing correction data, binary payload, CRC,
  %                 experiment metadata, and independent validation metrics.
  %
  % The application, rather than this reusable algorithm, is responsible for
  % selecting the stable segments from its particular experiment.

  % Interface block: enforce valid signatures and supply component-compatible
  % defaults for omitted configuration arguments.
  if (nargin < 3 || nargin > 6)
    print_usage ();
  endif
  if (nargin < 4)
    bin_count = 256;
  endif
  if (nargin < 5)
    full_scale_counts = 16384;
  endif
  if (nargin < 6)
    max_abs_correction_counts = 1024;
  endif
  validate_configuration (bin_count, full_scale_counts, ...
                          max_abs_correction_counts);

  % Sample-validation block: use column vectors internally and reject missing,
  % non-finite, mismatched, too-short, or nonchronological acquisition data.
  time_s = double (time_s(:));
  raw_angle_deg = double (raw_angle_deg(:));
  if (numel (time_s) != numel (raw_angle_deg) || numel (time_s) < 100 || ...
      any (! isfinite (time_s)) || any (! isfinite (raw_angle_deg)) || ...
      any (diff (time_s) <= 0))
    error ("Invalid angle-calibration sample vectors");
  endif
  if (! iscell (segments) || isempty (segments))
    error ("segments must be a non-empty cell array of sample indices");
  endif

  % Dataset-partition block: process each experiment interval independently.
  % Odd complete revolutions train the LUT; even revolutions are held out so
  % the reported improvement does not merely measure the fitting data.
  train_phase = [];
  train_error = [];
  validation_phase = [];
  validation_error = [];
  revolution_count = 0;
  usable_segments = 0;
  for segment_index = 1:numel (segments)
    % Check that this segment is an ordered, in-range integer index vector.
    indices = double (segments{segment_index}(:));
    if (isempty (indices) || any (! isfinite (indices)) || ...
        any (indices != fix (indices)) || any (indices < 1) || ...
        any (indices > numel (time_s)) || any (diff (indices) <= 0))
      error ("Calibration segment %d contains invalid indices", segment_index);
    endif
    [phase, error_rad, revolution] = revolution_errors ( ...
        time_s(indices), raw_angle_deg(indices));

    % Ignore intervals without enough complete revolutions to estimate error.
    if (isempty (phase))
      continue;
    endif

    % Accumulate training and withheld validation samples by revolution parity.
    usable_segments += 1;
    revolution_count += max (revolution);
    training = mod (revolution, 2) == 1;
    train_phase = [train_phase; phase(training)];
    train_error = [train_error; error_rad(training)];
    validation_phase = [validation_phase; phase(! training)];
    validation_error = [validation_error; error_rad(! training)];
  endfor

  % Coverage block: require enough independent angular observations for the
  % selected table resolution before attempting interpolation or smoothing.
  if (numel (train_error) < 4 * bin_count || ...
      numel (validation_error) < 2 * bin_count)
    error ("Not enough complete revolutions for a %d-bin calibration", ...
           bin_count);
  endif

  % Angular-profile block: robustly aggregate the training error by raw-angle
  % bin, then use circular interpolation to fill bins not directly observed.
  binned_error = bin_median (train_phase, train_error, bin_count);
  binned_error = fill_circular (binned_error);

  % Harmonic-selection block: begin with a compact eight-harmonic model and
  % progressively reduce its bandwidth until quantization yields a monotonic
  % corrected angle. This prevents the LUT from folding the angular mapping.
  selected_harmonics = min (8, floor (bin_count / 4));
  correction_counts = [];
  while (selected_harmonics >= 1)
    % Retain only the requested low-order periodic content of the error.
    smooth_error = circular_harmonic_smooth (binned_error, ...
                                             selected_harmonics);

    % Remove the arbitrary constant angle offset; the LUT targets periodic
    % nonlinearity and should not redefine the application's mechanical zero.
    smooth_error -= mean (smooth_error);

    % Convert bin-center estimates to bin-boundary corrections, matching the
    % interpolation nodes used by esp_angle_lut_apply() in the firmware.
    previous = [numel(smooth_error); (1:(numel(smooth_error) - 1))'];
    boundary_error = 0.5 * (smooth_error + smooth_error(previous));

    % Quantize radians to native counts and remove quantization-induced DC.
    candidate_double = round (boundary_error * full_scale_counts / ...
                              (2 * pi));
    candidate_double -= round (mean (candidate_double));

    % Reduce bandwidth rather than saturating entries that do not fit int16,
    % because entry-wise saturation could destroy map monotonicity.
    if (any (abs (candidate_double) > 32767))
      selected_harmonics -= 1;
      continue;
    endif
    candidate = int16 (candidate_double);

    % Apply structural rules with the full int16 bound during model search.
    [candidate_valid, report] = esp_angle_lut_validate ( ...
        candidate, full_scale_counts, 32767);
    if (candidate_valid && report.monotonic)
      correction_counts = candidate;
      break;
    endif
    selected_harmonics -= 1;
  endwhile

  % A calibration that cannot preserve angular order must never be exported.
  if (isempty (correction_counts))
    error ("Could not produce a monotonic correction LUT");
  endif

  % Firmware-limit block: after finding a monotonic model, enforce the tighter
  % correction-amplitude limit configured for the intended target firmware.
  [valid, validation] = esp_angle_lut_validate ( ...
      correction_counts, full_scale_counts, max_abs_correction_counts);
  if (! valid)
    if (! validation.correction_range)
      error ("Calculated correction exceeds the firmware safety limit");
    endif
    error ("Calculated LUT does not satisfy the firmware validation rules");
  endif

  % Independent phase-validation block: compare the withheld median error with
  % the cyclically interpolated correction at the centers of the same bins.
  centers_rad = ((0:(bin_count - 1))' + 0.5) * 2 * pi / bin_count;
  correction_rad = double (correction_counts) * 2 * pi / full_scale_counts;
  validation_raw = bin_median (validation_phase, validation_error, bin_count);
  validation_raw = fill_circular (validation_raw);
  correction_at_centers = interp_cyclic (correction_rad, centers_rad);
  validation_corrected = wrap_pi (validation_raw - correction_at_centers);
  raw_rms_deg = sqrt (mean (validation_raw .^ 2)) * 180 / pi;
  corrected_rms_deg = sqrt (mean (validation_corrected .^ 2)) * 180 / pi;
  reduction_percent = 100 * (1 - corrected_rms_deg / max (raw_rms_deg, eps));

  % Independent speed-validation block: apply the LUT to the complete captured
  % angle, differentiate each selected segment, and remove its linear trend.
  % The residual RMS measures periodic ripple rather than average acceleration.
  corrected_deg = esp_angle_lut_apply (raw_angle_deg, correction_counts, ...
                                       full_scale_counts);
  raw_speed_energy = 0;
  corrected_speed_energy = 0;
  speed_samples = 0;
  for segment_index = 1:numel (segments)
    indices = double (segments{segment_index}(:));

    % Convert unwrapped angular increments to revolutions per minute.
    dt = diff (time_s(indices));
    raw_speed = diff (unwrap (raw_angle_deg(indices) * pi / 180)) ./ dt * ...
                60 / (2 * pi);
    corrected_speed = diff (unwrap (corrected_deg(indices) * pi / 180)) ./ ...
                      dt * 60 / (2 * pi);
    if (isempty (raw_speed))
      continue;
    endif

    % Accumulate squared detrended residuals across every usable interval.
    raw_residual = detrend (raw_speed);
    corrected_residual = detrend (corrected_speed);
    raw_speed_energy += sum (raw_residual .^ 2);
    corrected_speed_energy += sum (corrected_residual .^ 2);
    speed_samples += numel (raw_residual);
  endfor

  % Refuse to publish speed metrics when the selected data cannot define them.
  if (speed_samples == 0)
    error ("Calibration segments contain no speed samples");
  endif
  raw_speed_rms_rpm = sqrt (raw_speed_energy / speed_samples);
  corrected_speed_rms_rpm = sqrt (corrected_speed_energy / speed_samples);
  speed_reduction_percent = 100 * (1 - corrected_speed_rms_rpm / ...
                                   max (raw_speed_rms_rpm, eps));

  % Serialization block: create the exact bytes and CRC accepted by firmware.
  [payload, payload_crc32] = esp_angle_lut_pack (correction_counts);

  % Result block: collect target metadata, table representations, payload, and
  % quality metrics in a self-describing structure suitable for a MAT file.
  calibration = struct ();
  calibration.format_version = 2;
  calibration.full_scale_counts = full_scale_counts;
  calibration.sensor_counts = full_scale_counts;
  calibration.max_abs_correction_counts = max_abs_correction_counts;
  calibration.bin_count = bin_count;
  calibration.correction_counts = correction_counts(:);
  calibration.correction_deg = double (correction_counts(:)) * 360 / ...
                               full_scale_counts;
  calibration.lut_angle_deg = (0:(bin_count - 1))' * 360 / bin_count;
  calibration.payload = payload;
  calibration.payload_crc32 = payload_crc32;
  calibration.selected_harmonics = selected_harmonics;
  calibration.revolution_count = revolution_count;
  calibration.usable_segment_count = usable_segments;
  calibration.phase_deg = centers_rad * 180 / pi;
  calibration.validation_raw_error_deg = validation_raw * 180 / pi;
  calibration.validation_corrected_error_deg = ...
      validation_corrected * 180 / pi;
  calibration.raw_rms_deg = raw_rms_deg;
  calibration.corrected_rms_deg = corrected_rms_deg;
  calibration.reduction_percent = reduction_percent;
  calibration.raw_speed_rms_rpm = raw_speed_rms_rpm;
  calibration.corrected_speed_rms_rpm = corrected_speed_rms_rpm;
  calibration.speed_reduction_percent = speed_reduction_percent;
endfunction

function validate_configuration (bin_count, full_scale_counts, ...
                                 max_abs_correction_counts)
  % VALIDATE_CONFIGURATION Validate configuration before sample processing.
  %
  % WHAT IT DOES
  %   Builds a zero correction table and submits it to the common validator.
  %   This checks bin-count, resolution, divisibility, power-of-two, and safety
  %   limit rules without duplicating them in the calibration routine.
  %
  % CALLED BY
  %   Only esp_angle_lut_calculate(). This is a private local function.
  %
  % CALLS
  %   esp_angle_lut_validate().
  %
  % INPUTS
  %   bin_count, full_scale_counts, max_abs_correction_counts - The three
  %   target-firmware configuration values being checked.
  %
  % OUTPUT
  %   None. Invalid configuration is reported by an exception.

  % Probe block: a zero LUT is inherently monotonic, so any rejection identifies
  % incompatible configuration metadata rather than calibration content.
  zeros_lut = zeros (bin_count, 1);
  esp_angle_lut_validate (zeros_lut, full_scale_counts, ...
                          max_abs_correction_counts);
endfunction

function [phase, error_rad, revolution] = revolution_errors (time_s, angle_deg)
  % REVOLUTION_ERRORS Estimate phase error within complete revolutions.
  %
  % WHAT IT DOES
  %   Unwraps one stable-speed segment, determines rotation direction, locates
  %   complete revolution boundaries, and compares the measured trajectory with
  %   uniform angular progress between consecutive boundary times.
  %
  % CALLED BY
  %   Only esp_angle_lut_calculate(), once per supplied segment. This is a
  %   private local function.
  %
  % CALLS
  %   wrap_pi().
  %
  % INPUTS
  %   time_s   - Segment sample times in seconds.
  %   angle_deg - Corresponding cyclic raw angles in degrees.
  %
  % OUTPUTS
  %   phase      - Raw measured phase in [0, 2*pi), in radians.
  %   error_rad  - Ideal uniform phase minus measured phase, wrapped to pi.
  %   revolution - Sequential complete-revolution number for each sample.

  % Unwrapping block: convert the cyclic measurement into continuous radians
  % and infer direction robustly from the median sample-to-sample change.
  radians = unwrap (angle_deg * pi / 180);
  direction = sign (median (diff (radians)));

  % Stationary-segment block: return empty outputs when rotation direction
  % cannot be identified, allowing the caller to skip this interval.
  if (direction == 0)
    phase = [];
    error_rad = [];
    revolution = [];
    return;
  endif

  % Boundary block: express progress as increasing regardless of direction and
  % identify every full-turn level completely crossed by the segment.
  progress = direction * radians;
  first_crossing = ceil (progress(1) / (2 * pi));
  last_crossing = floor (progress(end) / (2 * pi));
  levels = (first_crossing:last_crossing)' * 2 * pi;

  % At least two complete turn intervals require three boundary crossings.
  if (numel (levels) < 3)
    phase = [];
    error_rad = [];
    revolution = [];
    return;
  endif

  % Timing block: interpolate the instant of each revolution crossing from the
  % monotonic unwrapped angular trajectory.
  crossing_time = interp1 (progress, time_s, levels, "linear");

  % Per-turn block: construct an ideal constant-speed phase ramp independently
  % within each full revolution so slow speed changes do not become LUT error.
  phase = [];
  error_rad = [];
  revolution = [];
  for turn = 1:(numel (crossing_time) - 1)
    % Select samples between this turn's two interpolated boundary times.
    selected = find (time_s >= crossing_time(turn) & ...
                     time_s < crossing_time(turn + 1));

    % Map elapsed time linearly to ideal angular progress through the turn.
    fraction = (time_s(selected) - crossing_time(turn)) / ...
               (crossing_time(turn + 1) - crossing_time(turn));
    ideal = direction * (levels(turn) + 2 * pi * fraction);
    measured = radians(selected);

    % Store cyclic measured phase, wrapped error, and its source revolution.
    phase = [phase; mod(measured, 2 * pi)];
    error_rad = [error_rad; wrap_pi(ideal - measured)];
    revolution = [revolution; turn * ones(numel (selected), 1)];
  endfor
endfunction

function values = bin_median (phase, samples, bin_count)
  % BIN_MEDIAN Robustly aggregate samples into uniform angular bins.
  %
  % WHAT IT DOES
  %   Assigns each phase/sample pair to a cyclic bin and calculates the median
  %   of all samples observed in each bin. Unobserved bins remain NaN.
  %
  % CALLED BY
  %   Only esp_angle_lut_calculate(), for both training and validation data.
  %   This is a private local function.
  %
  % CALLS
  %   No function from this library.
  %
  % INPUTS
  %   phase     - Angular positions in radians.
  %   samples   - Values to aggregate at the corresponding positions.
  %   bin_count - Number of uniform bins in one revolution.
  %
  % OUTPUT
  %   values - Column vector of per-bin medians; missing bins contain NaN.

  % Assignment block: wrap phase, scale it to a one-based bin index, and guard
  % the upper endpoint against floating-point roundoff.
  bins = min (floor (mod (phase, 2 * pi) * bin_count / (2 * pi)) + 1, ...
              bin_count);

  % Aggregation block: use the median to reduce sensitivity to isolated noise
  % spikes or transient samples within otherwise stable rotations.
  values = NaN (bin_count, 1);
  for index = 1:bin_count
    selected = samples(bins == index);
    if (! isempty (selected))
      values(index) = median (selected);
    endif
  endfor
endfunction

function filled = fill_circular (values)
  % FILL_CIRCULAR Interpolate missing bins with cyclic boundary continuity.
  %
  % WHAT IT DOES
  %   Fills NaN entries by linear interpolation while treating the first and
  %   last bins as neighbors on a circle.
  %
  % CALLED BY
  %   Only esp_angle_lut_calculate(), for training and validation profiles.
  %   This is a private local function.
  %
  % CALLS
  %   No function from this library.
  %
  % INPUT
  %   values - One cyclic bin vector, possibly containing NaN entries.
  %
  % OUTPUT
  %   filled - Complete column vector with circularly interpolated gaps.

  % Coverage block: require measurements in at least one quarter of the angular
  % bins; otherwise interpolation would invent most of the correction profile.
  count = numel (values);
  valid = find (isfinite (values));
  if (numel (valid) < count / 4)
    error ("Too few angular bins were observed during calibration");
  endif

  % Circular-extension block: replicate valid samples one revolution before and
  % after their native positions, making ordinary interpolation wrap correctly.
  positions = valid - 1;
  query = (0:(count - 1))';
  extended_positions = [positions - count; positions; positions + count];
  extended_values = [values(valid); values(valid); values(valid)];
  filled = interp1 (extended_positions, extended_values, query, "linear");
endfunction

function smooth = circular_harmonic_smooth (values, harmonic_count)
  % CIRCULAR_HARMONIC_SMOOTH Low-pass a periodic profile in the FFT domain.
  %
  % WHAT IT DOES
  %   Keeps the DC term and a symmetric number of positive and negative Fourier
  %   harmonics, removes higher-frequency content, and returns a real profile.
  %
  % CALLED BY
  %   Only esp_angle_lut_calculate() while searching for a monotonic LUT. This
  %   is a private local function.
  %
  % CALLS
  %   No function from this library.
  %
  % INPUTS
  %   values         - Complete periodic angular-error profile.
  %   harmonic_count - Highest positive/negative harmonic to preserve.
  %
  % OUTPUT
  %   smooth - Real-valued low-pass periodic profile.

  % Transform block: represent the angular profile as complex Fourier bins.
  spectrum = fft (values);

  % Selection block: preserve DC and conjugate low-order components at both
  % ends of Octave's FFT ordering, then zero every rejected harmonic.
  keep = false (numel (values), 1);
  keep(1:(harmonic_count + 1)) = true;
  keep((end - harmonic_count + 1):end) = true;
  spectrum(! keep) = 0;

  % Reconstruction block: numerical roundoff can leave negligible imaginary
  % values, so explicitly retain the physically meaningful real component.
  smooth = real (ifft (spectrum));
endfunction

function result = interp_cyclic (values, phase)
  % INTERP_CYCLIC Linearly interpolate uniform samples around a circle.
  %
  % WHAT IT DOES
  %   Locates two neighboring bins for each requested phase and interpolates
  %   between them, using the first bin as the successor of the final bin.
  %
  % CALLED BY
  %   Only esp_angle_lut_calculate() during withheld-data validation. This is a
  %   private local function.
  %
  % CALLS
  %   No function from this library.
  %
  % INPUTS
  %   values - Uniform cyclic bin values.
  %   phase  - Query phases in radians.
  %
  % OUTPUT
  %   result - Interpolated values with the same shape as phase.

  % Coordinate block: wrap phase and decompose its scaled bin coordinate into
  % a lower integer index and a fractional position.
  count = numel (values);
  position = mod (phase, 2 * pi) * count / (2 * pi);
  first = floor (position);
  fraction = position - first;

  % Interpolation block: wrap the upper neighbor to bin one at the cycle end.
  result = values(first + 1) .* (1 - fraction) + ...
           values(mod(first + 1, count) + 1) .* fraction;
endfunction

function value = wrap_pi (value)
  % WRAP_PI Normalize radian angles to the principal signed interval.
  %
  % WHAT IT DOES
  %   Maps arbitrary radian values to [-pi, pi) using modular arithmetic.
  %
  % CALLED BY
  %   revolution_errors() and esp_angle_lut_calculate(). This is a private
  %   local function.
  %
  % CALLS
  %   No function from this library.
  %
  % INPUT/OUTPUT
  %   value - Scalar or array of radians, returned with its original shape.

  % Normalization block: shift by pi, wrap one full turn, and shift back.
  value = mod (value + pi, 2 * pi) - pi;
endfunction
