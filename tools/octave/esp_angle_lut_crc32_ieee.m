function crc = esp_angle_lut_crc32_ieee (bytes)
  % ESP_ANGLE_LUT_CRC32_IEEE Calculate the firmware-compatible payload CRC.
  %
  % crc = esp_angle_lut_crc32_ieee (bytes)
  %
  % WHAT IT DOES
  %   Calculates reflected CRC-32/IEEE using polynomial 0xEDB88320, initial
  %   value 0xFFFFFFFF, and final XOR 0xFFFFFFFF. The result matches
  %   esp_crc32_le() as used by the ESP32 firmware.
  %
  % CALLED BY
  %   esp_angle_lut_pack(), the reusable tests, and external applications.
  %
  % CALLS
  %   No function from the esp_angle_lut Octave library.
  %
  % INPUT
  %   bytes - Numeric array interpreted as a sequence of uint8 values.
  %
  % OUTPUT
  %   crc - Scalar uint32 containing the IEEE CRC32.

  % Interface block: this low-level routine accepts exactly one payload.
  if (nargin != 1)
    print_usage ();
  endif

  % Lookup-table block: build the 256 remainders once. Declaring the table
  % persistent avoids repeating this initialization for every transmitted LUT.
  persistent table;
  if (isempty (table))
    polynomial = uint32 (hex2dec ("EDB88320"));
    table = zeros (256, 1, "uint32");
    for index = 0:255
      value = uint32 (index);
      for bit = 1:8
        if (bitand (value, uint32 (1)))
          value = bitxor (bitshift (value, -1), polynomial);
        else
          value = bitshift (value, -1);
        endif
      endfor
      table(index + 1) = value;
    endfor
  endif

  % Accumulation block: process the payload in transmission order with the
  % reflected lookup recurrence used by the firmware implementation.
  crc = uint32 (hex2dec ("FFFFFFFF"));
  bytes = uint8 (bytes(:));
  for index = 1:numel (bytes)
    lookup = bitand (bitxor (crc, uint32 (bytes(index))), uint32 (255));
    crc = bitxor (bitshift (crc, -8), table(double (lookup) + 1));
  endfor

  % Finalization block: complement the accumulator to obtain CRC-32/IEEE.
  crc = bitxor (crc, uint32 (hex2dec ("FFFFFFFF")));
endfunction
