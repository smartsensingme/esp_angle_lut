function [valid, report] = esp_angle_lut_validate (correction_counts, ...
                                                    full_scale_counts, ...
                                                    max_abs_correction_counts)
  % ESP_ANGLE_LUT_VALIDATE Validate a LUT using the firmware's safety rules.
  %
  % [valid, report] = esp_angle_lut_validate (correction_counts, ...
  %                                            full_scale_counts, ...
  %                                            max_abs_correction_counts)
  %
  % WHAT IT DOES
  %   Checks configuration compatibility, integer representation, correction
  %   amplitude, and monotonicity of the corrected cyclic angular map. These
  %   are the same structural rules enforced before firmware installation.
  %
  % CALLED BY
  %   esp_angle_lut_calculate(), esp_angle_lut_apply(), the private
  %   validate_configuration() helper, tests, and external applications.
  %
  % CALLS
  %   No function from the esp_angle_lut Octave library.
  %
  % INPUTS
  %   correction_counts          - One signed correction per LUT bin.
  %   full_scale_counts          - Native sensor counts per revolution.
  %   max_abs_correction_counts  - Accepted correction magnitude. Optional;
  %                                defaults to the complete int16 range.
  %
  % OUTPUTS
  %   valid  - True only when correction range and monotonicity both pass.
  %   report - Structure containing configuration, numeric-range, amplitude,
  %            corrected-step, and monotonicity diagnostics.

  % Interface block: enforce the supported signatures and default safety
  % bound used when the caller only wants structural validation.
  if (nargin < 2 || nargin > 3)
    print_usage ();
  endif
  if (nargin < 3)
    max_abs_correction_counts = 32767;
  endif

  % Configuration block: reproduce the component's power-of-two and divisibility
  % constraints so a host cannot generate a table incompatible with firmware.
  corrections = double (correction_counts(:));
  bin_count = numel (corrections);
  if (bin_count < 16 || bin_count > 256 || ...
      bitand (bin_count, bin_count - 1) != 0 || ...
      ! isscalar (full_scale_counts) || ! isfinite (full_scale_counts) || ...
      full_scale_counts < 256 || full_scale_counts > 65536 || ...
      full_scale_counts != fix (full_scale_counts) || ...
      bitand (full_scale_counts, full_scale_counts - 1) != 0 || ...
      mod (full_scale_counts, bin_count) != 0)
    error (["The bin count and full scale must match the esp_angle_lut ", ...
            "power-of-two configuration rules"]);
  endif

  % Safety-limit block: require a finite positive limit representable by int16.
  if (! isscalar (max_abs_correction_counts) || ...
      ! isfinite (max_abs_correction_counts) || ...
      max_abs_correction_counts < 1 || max_abs_correction_counts > 32767)
    error ("max_abs_correction_counts is invalid");
  endif

  % Numeric-representation block: a valid payload must consist exclusively of
  % finite integer entries that preserve their value when encoded as int16.
  finite_integer = all (isfinite (corrections)) && ...
                   all (corrections == fix (corrections));
  int16_range = finite_integer && all (corrections >= -32768) && ...
                all (corrections <= 32767);

  % Angular-map block: each corrected step equals one nominal bin width plus
  % the change in correction between adjacent bins, including cyclic closure.
  maximum_correction = Inf;
  corrected_steps = NaN (bin_count, 1);
  if (finite_integer)
    maximum_correction = max (abs (corrections));
    bin_width = full_scale_counts / bin_count;
    next = corrections([2:end, 1]);
    corrected_steps = bin_width + next - corrections;
  endif

  % Report block: retain individual predicates and extrema so callers can
  % explain why a table failed instead of receiving only a Boolean result.
  report = struct ();
  report.bin_count = bin_count;
  report.full_scale_counts = full_scale_counts;
  report.maximum_correction_counts = maximum_correction;
  report.integer_values = finite_integer;
  report.int16_range = int16_range;
  report.correction_range = int16_range && ...
                            maximum_correction <= max_abs_correction_counts;
  report.minimum_corrected_step_counts = min (corrected_steps);
  report.maximum_corrected_step_counts = max (corrected_steps);

  % Decision block: positive steps preserve order; the upper bound rejects an
  % implausibly abrupt local expansion, matching validate_table() in C.
  report.monotonic = int16_range && all (corrected_steps > 0) && ...
                     all (corrected_steps <= 4 * full_scale_counts / bin_count);
  valid = report.correction_range && report.monotonic;
endfunction
