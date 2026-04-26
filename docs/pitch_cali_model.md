# Pitch Compensation Model

## Conclusion

The built-in pitch compensation fallback no longer uses the raw 10 measured points directly.
It now uses a denser 33-point table, generated offline from the 10 measured points with a
shape-preserving cubic Hermite model.

This keeps two useful properties at the same time:

- The model is smooth enough to sound good on paper.
- The generated curve still stays close to the measured points and does not invent wild overshoot.

## Human Explanation

The idea is simple:

1. Measure a small number of reliable pitch points on the real mechanism.
2. Build a smooth curve through those points.
3. Sample that curve into a denser built-in table.
4. Let runtime continue using the existing lightweight table interpolation.

So runtime stays cheap, while the default built-in compensation looks more like a continuous model.

## Model Form

For each interval `[x_i, x_{i+1}]`, use a cubic Hermite segment:

```text
t = (x - x_i) / (x_{i+1} - x_i)

h00 =  2 t^3 - 3 t^2 + 1
h10 =      t^3 - 2 t^2 + t
h01 = -2 t^3 + 3 t^2
h11 =      t^3 -     t^2

y(x) = h00 * y_i
     + h10 * (x_{i+1} - x_i) * m_i
     + h01 * y_{i+1}
     + h11 * (x_{i+1} - x_i) * m_{i+1}
```

The slopes `m_i` are computed with the Fritsch-Carlson shape-preserving rule.

That rule is the part worth saying out loud:

- it keeps the curve smooth,
- but it suppresses the ugly overshoot that ordinary cubic splines often create.

## Source Measured Points

Angle sample points, in rad:

```text
-0.40, -0.29, -0.18, -0.07, 0.04, 0.15, 0.26, 0.37, 0.48, 0.59
```

Hold current sample points:

```text
-234, 572, 2275, -115, -103, 1013, 865, -108, 1031, -157
```

Kick-up sample points:

```text
2800, 2650, 750, 3500, 3300, 1000, 2100, 3050, 1750, 3650
```

Kick-down sample points:

```text
700, 1300, 350, 450, 500, 1650, 1750, 550, 1900, 650
```

## Generated Built-in Table

The firmware stores a 33-point dense table generated from the model above.

Current built-in angle grid:

```text
-0.400000, -0.369062, -0.338125, -0.307187, -0.276250, -0.245312,
-0.214375, -0.183438, -0.152500, -0.121563, -0.090625, -0.059688,
-0.028750,  0.002187,  0.033125,  0.064062,  0.095000,  0.125937,
 0.156875,  0.187812,  0.218750,  0.249687,  0.280625,  0.311562,
 0.342500,  0.373437,  0.404375,  0.435312,  0.466250,  0.497187,
 0.528125,  0.559062,  0.590000
```

## Why This Is Reasonable

- It is still anchored by real measurement.
- It is smoother than direct sparse-point linear interpolation.
- It does not require extra runtime fitting or heavy math.
- If the TF card is missing, the mechanism still keeps a stable feel.

## Firmware Behavior

Boot behavior is now:

1. If the firmware contains a built-in dense table, use it first.
2. Only when firmware has no built-in default table, try `pitch_cali.bin` on the TF card.

So removing the TF card will not remove pitch compensation, and manual post-processing of the built-in table stays authoritative.
