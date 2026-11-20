# A6-EC absolute linear mode vendor email

**Subject:** A6-EC mode 1 multi-turn behavior and migration guidance

Hello,

We are preparing to change an A6-EC robot joint from absolute rotation mode to absolute position linear mode.

Our current posture is:

- `C00.07 = 4` (absolute rotation mode)
- `C10.18/C10.19` set to the true mechanical ratio
- CSP operation (`6060h = 8`)

Our desired posture is:

- `C00.07 = 1` (absolute position linear mode)
- the host software applies the 100:1 mechanical ratio
- J6 has a bounded travel range of `-720°` to `+720°`
- normal operation remains CSP

The manual says mode 1 is for a fixed travel range without multi-turn data overflow. It also describes a 16-bit multi-turn count, `ErA0.1`, and `F31.10 = 4`, but we need the exact mode-1 behavior and migration procedure confirmed:

1. In mode 1, does `6064h` remain continuous and unique across motor and output-shaft turns, without the single-revolution wrap used by mode 4?
2. How does the 16-bit encoder turn counter behave at its boundary in mode 1? Does it wrap, saturate, or raise `ErA0.1` before wrapping? Please give the exact positive and negative limits and explain the mode-1 rule for `ErA0.1` and `C10.15`.
3. What is the valid range of `607Ch` in mode 1? Is it the full signed 32-bit reference-unit range, constrained by `607Dh`, or subject to another mode-specific range?
4. When changing `C00.07` from 4 to 1, is a drive power cycle sufficient, or must we also execute `F31.10`? If `F31.10` is required, which value should be used, what data does it clear, and must HM35 then be performed again?
5. Can mode 1 operate without an encoder battery box? If not, will an absent battery always produce `Er20.8`, and is mode 3 the only supported battery-less absolute mode?
6. What are the factory defaults of `6091h:01` (motor revolutions) and `6091h:02` (shaft revolutions)? For mode 1 with the 100:1 conversion performed in host software, should both be explicitly set to `1`, and does changing either value require a power cycle, reset, or new HM35 operation?

Please also provide the exact panel parameter and CoE object/subindex, data type, valid range, and default for every setting needed for this mode-4-to-mode-1 change. For each setting, please state whether it takes effect immediately or only after a power cycle, and whether the change requires a software reset, `F31.10`, or HM35. A recommended safe ordered procedure would be helpful.

Regards,  
Advanced Metal Research
