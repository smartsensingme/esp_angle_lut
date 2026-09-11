function payload = esp_angle_lut_int16_le_bytes (values)
  % ESP_ANGLE_LUT_INT16_LE_BYTES Serialize signed corrections as bytes.
  %
  % payload = esp_angle_lut_int16_le_bytes (values)
  %
  % WHAT IT DOES
  %   Encodes signed 16-bit LUT entries in little-endian transmission order.
  %   Negative values are represented in two's complement.
  %
  % CALLED BY
  %   esp_angle_lut_pack() and external applications that need the raw payload.
  %
  % CALLS
  %   No function from the esp_angle_lut Octave library.
  %
  % INPUT
  %   values - Array of finite integer values in the int16 range.
  %
  % OUTPUT
  %   payload - Column vector of uint8 values containing low byte followed by
  %             high byte for each input entry.

  % Interface block: require one correction array.
  if (nargin != 1)
    print_usage ();
  endif

  % Validation block: convert only for inspection and reject values whose
  % meaning would change when represented as signed 16-bit integers.
  numeric = double (values(:));
  if (any (! isfinite (numeric)) || any (numeric != fix (numeric)) || ...
      any (numeric < -32768) || any (numeric > 32767))
    error ("LUT corrections must be signed 16-bit integers");
  endif

  % Representation block: modulo 2^16 yields the two's-complement bit pattern
  % for negative values without depending on the host machine's byte order.
  unsigned = mod (numeric, 65536);

  % Serialization block: interleave the low and high bytes explicitly so the
  % result is identical on little- and big-endian Octave hosts.
  payload = zeros (2 * numel (numeric), 1, "uint8");
  payload(1:2:end) = uint8 (mod (unsigned, 256));
  payload(2:2:end) = uint8 (floor (unsigned / 256));
endfunction
