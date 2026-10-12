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
