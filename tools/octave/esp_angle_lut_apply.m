function corrected_deg = esp_angle_lut_apply (raw_deg, correction_counts, ...
                                                full_scale_counts)
  % ESP_ANGLE_LUT_APPLY Apply a cyclic angular correction LUT.
  %
  % corrected_deg = esp_angle_lut_apply (raw_deg, correction_counts, ...
  %                                       full_scale_counts)
  %
  % WHAT IT DOES
  %   Applies, in degrees, the same cyclic linear interpolation implemented by
  %   esp_angle_lut_apply() in the ESP32 component. The input and output are
  %   wrapped to one revolution.
  %
  % CALLED BY
  %   esp_angle_lut_calculate(), the reusable tests, and external application
  %   scripts. No other function in this file calls it.
  %
  % CALLS
  %   esp_angle_lut_validate() before using the correction table.
  %
  % INPUTS
  %   raw_deg           - Scalar, vector, or matrix of raw angles in degrees.
  %   correction_counts - Signed correction at every uniformly spaced LUT bin.
  %   full_scale_counts - Native sensor counts per revolution. Optional;
  %                       defaults to 16384.
  %
  % OUTPUT
  %   corrected_deg - Corrected angles in [0, 360), with the same shape as
  %                   raw_deg.

  % Interface block: enforce the supported call signatures and default scale.
  if (nargin < 2 || nargin > 3)
    print_usage ();
  endif
  if (nargin < 3)
    full_scale_counts = 16384;
  endif

  % Configuration block: reject an unusable sensor resolution early.
  if (! isscalar (full_scale_counts) || ! isfinite (full_scale_counts) || ...
      full_scale_counts <= 0)
    error ("full_scale_counts must be a positive scalar");
  endif

  % LUT-validation block: normalize its shape and apply firmware-compatible
  % range and monotonicity rules before it can index the input data.
  correction_counts = double (correction_counts(:));
  [valid, report] = esp_angle_lut_validate (correction_counts, ...
                                            full_scale_counts, 32767);
  if (! valid)
    if (! report.correction_range)
      error ("LUT corrections are outside the firmware correction range");
    endif
    error ("The corrected angular map is not monotonic");
  endif
  bin_count = numel (correction_counts);

  % Input-normalization block: retain the caller's original array shape while
  % working internally with a column vector wrapped to one revolution.
  shape = size (raw_deg);
  raw = mod (double (raw_deg(:)), 360);

  % Bin-location block: determine the lower bin, its cyclic successor, and the
  % fractional angular distance between the two correction samples.
  position = raw * bin_count / 360;
  first = floor (position);
  fraction = position - first;
  first_index = first + 1;
  second_index = mod (first + 1, bin_count) + 1;

  % Interpolation block: blend adjacent corrections, convert native counts to
  % degrees, add the result, wrap it, and restore the original array shape.
  correction = correction_counts(first_index) .* (1 - fraction) + ...
               correction_counts(second_index) .* fraction;
  corrected_deg = reshape (mod (raw + correction * 360 / ...
                                full_scale_counts, 360), shape);
endfunction
