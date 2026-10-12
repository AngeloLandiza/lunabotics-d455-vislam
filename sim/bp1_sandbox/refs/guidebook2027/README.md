# NASA Lunabotics Guidebook 2026-2027: BP-1 reflectivity (Figure 7)

Source: "Lunabotics Guidebook 2027 - 10-5-27.pdf", NASA Lunabotics Challenge Guidebook 2026-2027 (78 pages),
https://static1.squarespace.com/static/5ab00c731aef1d58278394c4/t/6ac3c6b5cb12c867df9b1f82/1791215285548/Lunabotics+Guidebook+2027+-+10-5-27.pdf
(downloaded 2026-10-12).

- `figure7_bp1_reflectivity.jpg`: Figure 7 "LIDAR/Laser Reflectivity" as embedded in the PDF (PDF page 34, printed
  page 27), extracted losslessly with `pdfimages -j`: 803 x 533 px, sRGB JPEG, 144 ppi. Three photos side by side:
  camera at 10, 16 and 21 degrees to the surface; a red laser sheet (vertical) drapes over a BP-1 mound (bottom) and
  a standing target with light-grey and mid-grey bands on black (top).
- `page34_200dpi.png`: the whole PDF page rendered at 200 dpi with `pdftoppm`, for context.
- `section_11_1.txt`: section 11.1 Arena Composition as plain text (`pdftotext -layout`), including item 3
  (laser phase angle 15 deg; camera 1/8 s, f/4.5, manual).

Pixel notes (2026-10-12): laser-line pixels (R - G > 60, R > 120) peak at R ~ 190-240; only 2 of 6137 are at 255,
so the line is near but mostly not at saturation. The image is a small, JPEG-compressed, gamma-encoded composite, so
any brightness ratio read from it is a rough estimate (Inferred), not a calibrated reflectance.

## Measured line brightness (`measure_fig7.py`, results in `fig7_measure.json`)

Red excess of the laser line in linear light, summed across the line, median over the rows of each surface
(arbitrary units; rows at patch edges left out):

| Photo | Target black | Target mid grey | Target light grey | BP-1 | BP-1 as a patch, x light grey |
| --- | --- | --- | --- | --- | --- |
| 10 deg | 3.07 | 4.06 | 4.72 | 3.97 | 0.19 |
| 16 deg | 2.73 | 3.53 | 4.39 | 3.42 | 0.16 |
| 21 deg | 2.66 | 3.67 | 4.62 | 3.26 | 0.08 |

What it shows (Inferred):
- In all three photos the line on BP-1 is a little dimmer than on the target's mid-grey patch, and brighter than on black.
- The photos compress brightness heavily. The light-grey patch is 33-47x brighter than black in room light, but the
  laser line on it is only 1.5-1.7x brighter. The line's core is near saturation and JPEG blurs the red channel.
  Because of this, BP-1 is placed on a log scale between the two nearest patches. That gives the last column: BP-1
  matches a patch 0.08-0.19x as bright as the light-grey one.
- The guidebook gives no reflectance for the target. If the light-grey patch is the "white" (0.5-0.9), BP-1 comes
  out at roughly 0.04-0.17 at the laser's red wavelength.
- That figure is before any correction for the mound being seen at 10-21 degrees, which makes a matte surface look
  darker. The correction would raise it, but it is not applied: the trend across the three photos runs the wrong way
  for a simple matte surface.
- The guidebook laser is red, while the L1 runs at 905 nm. Basalt is usually no darker in the near infrared than in
  red (Assumed).

Use in the sandbox: params.BP1 lidar_reflectivity stays at 0.10 (Assumed), which this range supports. With the L1
model, flat ground stops returning at a slant range of (reflectivity x 0.986 m sensor height / 0.001)^(1/3), where
0.001 is the L1's 10 m at 10% threshold and the sensor sees a matte floor. That is about 3.5, 4.5 and 5.7 m ahead
along the floor for reflectivity 0.05, 0.10 and 0.20 (Inferred from sensors.py).
