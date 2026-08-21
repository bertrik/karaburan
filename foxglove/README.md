# Karaburan location metrics in Foxglove

## Import the complete layout

Import [`karaburan-layout.json`](./karaburan-layout.json) from the Foxglove
**Layouts** menu using **Import from file**. The layout already contains both
User Scripts, the `mapMetric` variable, a route map, a depth plot, and a plot
with the sonar temperature and all three DS18B20 temperature sensors.

Open all related MCAP files together after selecting the layout. There is no
need to add the two TypeScript files separately when using this layout.

[`karaburan_location_metrics.ts`](./karaburan_location_metrics.ts) is a
Foxglove User Script for the MCAP files in `20260818-logfiles/mcap`. It joins
each sonar measurement to the latest GNSS fix and exposes the following values
in the map-point tooltip:

- depth;
- sonar temperature;
- temperature sensor 1 (`28.C23646D48524`);
- temperature sensor 2 (`28.5CD456B5013C`);
- temperature sensor 3 (`28.F95856B5013C`).

## Use the script

1. Open the related MCAP files together in Foxglove.
2. Open **User Scripts** in the right sidebar and create a script.
3. Replace the generated example with the contents of
   `karaburan_location_metrics.ts`, then save it.
4. Add a **Map** panel and select `/karaburan/location_metrics`.
5. Set the topic time range to **All** to show the full measured route.
6. Hover over a route point to inspect all five measurements at that location.

The default point color represents depth over a 0–5 m scale. Add the string
layout variable `mapMetric` to select another color metric:

| `mapMetric` value | Point color represents |
| --- | --- |
| `depth` | Depth, 0–5 m |
| `sonarTemperature` | Sonar temperature, 18–25 °C |
| `temperature1` | Temperature sensor 1, 18–25 °C |
| `temperature2` | Temperature sensor 2, 18–25 °C |
| `temperature3` | Temperature sensor 3, 18–25 °C |

Low values are blue, high values are red, and unavailable values are grey.
The source logs identify the temperature sensors only by number and hardware
ID; update the labels in the script if their physical names become known.

## Plot the measurements

Add [`karaburan_sensor_metrics.ts`](./karaburan_sensor_metrics.ts) as a second
User Script. It publishes numeric values on `/karaburan/sensor_metrics`, which
can be used by Plot, Gauge, and similar panels.

Use these Y-value paths in a Plot panel:

| Measurement | Y-value path |
| --- | --- |
| Depth | `/karaburan/sensor_metrics.depth` |
| Sonar temperature | `/karaburan/sensor_metrics.sonarTemperature` |
| Temperature sensor 1 | `/karaburan/sensor_metrics.temperature1` |
| Temperature sensor 2 | `/karaburan/sensor_metrics.temperature2` |
| Temperature sensor 3 | `/karaburan/sensor_metrics.temperature3` |

Keep the X-axis set to **Timestamp** for time-series plots. The output also has
numeric `latitude` and `longitude` fields so each plotted sample retains its
associated position. Unavailable numeric measurements are emitted as `NaN`,
which produces a gap rather than a false value in a plot.
