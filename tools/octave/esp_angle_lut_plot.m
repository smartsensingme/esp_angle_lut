function figure_handle = esp_angle_lut_plot (calibration, font_size)
  % ESP_ANGLE_LUT_PLOT Plot a LUT and its independent validation metrics.
  %
  % figure_handle = esp_angle_lut_plot (calibration, font_size)
  %
  % WHAT IT DOES
  %   Creates one figure containing the correction as a function of raw angle
  %   and the validation error before and after correction. The title also
  %   reports angular and instantaneous-speed RMS reductions.
  %
  % CALLED BY
  %   External application scripts. No function in this library calls it.
  %
  % CALLS
  %   No function from the esp_angle_lut Octave library; it uses Octave plotting
  %   primitives only.
  %
  % INPUTS
  %   calibration - Structure returned by esp_angle_lut_calculate().
  %   font_size   - Font size used for every graphical text object. Optional;
  %                 defaults to 12.
  %
  % OUTPUT
  %   figure_handle - Handle of the newly created figure.

  % Interface block: enforce supported signatures and establish presentation
  % defaults independently of the calling application's graphics state.
  if (nargin < 1 || nargin > 2)
    print_usage ();
  endif
  if (nargin < 2)
    font_size = 12;
  endif

  % Figure block: create a named, normalized window and return its handle so
  % callers can save, modify, or close this specific result later.
  figure_handle = figure ("name", "Calibracao angular", ...
                          "numbertitle", "off", "units", "normalized", ...
                          "position", [0.12, 0.12, 0.76, 0.72]);

  % LUT subplot: show the correction that will be interpolated by firmware.
  subplot (2, 1, 1);
  plot (calibration.lut_angle_deg, calibration.correction_deg, ...
        "linewidth", 1.3);
  grid on;
  ylabel ("correcao [deg]");
  title (sprintf ("LUT de %d pontos - CRC %08X", ...
                  calibration.bin_count, calibration.payload_crc32));

  % Validation subplot: compare withheld raw and corrected phase errors rather
  % than reusing the revolutions from which the LUT was estimated.
  subplot (2, 1, 2);
  plot (calibration.phase_deg, calibration.validation_raw_error_deg, ...
        "linewidth", 1.1, calibration.phase_deg, ...
        calibration.validation_corrected_error_deg, "linewidth", 1.2);
  grid on;
  xlabel ("angulo bruto [deg]");
  ylabel ("erro de fase [deg]");
  legend ("antes", "depois", "location", "northeast");
  title (sprintf (["Validacao: fase %.4f -> %.4f deg (%.1f%%); ", ...
                  "velocidade %.2f -> %.2f rpm (%.1f%%)"], ...
                  calibration.raw_rms_deg, ...
                  calibration.corrected_rms_deg, ...
                  calibration.reduction_percent, ...
                  calibration.raw_speed_rms_rpm, ...
                  calibration.corrected_speed_rms_rpm, ...
                  calibration.speed_reduction_percent));

  % Typography block: apply one requested size to axes, labels, titles, ticks,
  % and legends after every graphical object has been created.
  set (findall (figure_handle, "type", "axes"), "fontsize", font_size);
  set (findall (figure_handle, "type", "text"), "fontsize", font_size);
endfunction
