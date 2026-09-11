function [payload, crc] = esp_angle_lut_pack (correction_counts)
  % ESP_ANGLE_LUT_PACK Build the binary LUT payload and its integrity code.
  %
  % [payload, crc] = esp_angle_lut_pack (correction_counts)
  %
  % WHAT IT DOES
  %   Combines serialization and CRC calculation to produce the exact pair
  %   expected by esp_angle_lut_install() in the firmware.
  %
  % CALLED BY
  %   esp_angle_lut_calculate(), the reusable tests, and external applications.
  %
  % CALLS
  %   esp_angle_lut_int16_le_bytes() and esp_angle_lut_crc32_ieee().
  %
  % INPUT
  %   correction_counts - Signed LUT corrections representable as int16.
  %
  % OUTPUTS
  %   payload - Column vector containing the little-endian correction bytes.
  %   crc     - Scalar uint32 CRC-32/IEEE of payload.

  % Interface block: one complete correction table is required.
  if (nargin != 1)
    print_usage ();
  endif

  % Serialization block: construct the platform-independent wire payload.
  payload = esp_angle_lut_int16_le_bytes (correction_counts);

  % Integrity block: calculate the code over the exact bytes to be sent.
  crc = esp_angle_lut_crc32_ieee (payload);
endfunction
