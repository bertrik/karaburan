import type { Input, Message } from "./types.ts";

type InputTopic =
  | "karaburan/sensors/gnss/rtkgps/measurement"
  | "karaburan/sensors/sonar/D3:01:01:02:2F:C6/measurement"
  | "karaburan/sensors/temperature/28.C23646D48524/measurement"
  | "karaburan/sensors/temperature/28.5CD456B5013C/measurement"
  | "karaburan/sensors/temperature/28.F95856B5013C/measurement";

type StringMessage = Message<"std_msgs/msg/String">;
type LocationFix = Message<"foxglove.LocationFix">;

type LayoutVariables = {
  mapMetric?: string;
};

type GnssData = {
  lat?: number;
  lon?: number;
  altHAE?: number;
  altMSL?: number;
  mode?: number;
  track?: number;
  speed?: number;
};

type SonarData = {
  depth?: number;
  temperature?: number | null;
  battery?: number;
};

type Payload = {
  time?: number;
  type?: string;
  id?: string;
  data?: number | null | GnssData | SonarData;
};

type Rgb = { r: number; g: number; b: number; a: number };

export const inputs: InputTopic[] = [
  "karaburan/sensors/gnss/rtkgps/measurement",
  "karaburan/sensors/sonar/D3:01:01:02:2F:C6/measurement",
  "karaburan/sensors/temperature/28.C23646D48524/measurement",
  "karaburan/sensors/temperature/28.5CD456B5013C/measurement",
  "karaburan/sensors/temperature/28.F95856B5013C/measurement",
];

export const output = "/karaburan/location_metrics";

const GNSS_TOPIC = "karaburan/sensors/gnss/rtkgps/measurement";
const SONAR_TOPIC = "karaburan/sensors/sonar/D3:01:01:02:2F:C6/measurement";

const temperatureTopics: Record<string, keyof TemperatureState> = {
  "karaburan/sensors/temperature/28.C23646D48524/measurement": "temperature1",
  "karaburan/sensors/temperature/28.5CD456B5013C/measurement": "temperature2",
  "karaburan/sensors/temperature/28.F95856B5013C/measurement": "temperature3",
};

type TemperatureState = {
  temperature1?: number;
  temperature2?: number;
  temperature3?: number;
};

let gnss: GnssData | undefined;
const temperatures: TemperatureState = {};

function parsePayload(data: string): Payload | undefined {
  try {
    // Python's JSON encoder wrote a few unavailable sonar temperatures as NaN.
    return JSON.parse(data.replace(/\bNaN\b/g, "null")) as Payload;
  } catch {
    return undefined;
  }
}

function isFiniteNumber(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}

function display(value: number | null | undefined, unit: string): string {
  return isFiniteNumber(value) ? `${value.toFixed(2)} ${unit}` : "unavailable";
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}

function interpolateColor(value: number | undefined, minimum: number, maximum: number): Rgb {
  if (!isFiniteNumber(value)) {
    return { r: 0.55, g: 0.55, b: 0.55, a: 0.9 };
  }

  const ratio = clamp((value - minimum) / (maximum - minimum), 0, 1);
  return {
    r: 0.1 + ratio * 0.85,
    g: 0.4 - ratio * 0.25,
    b: 0.95 - ratio * 0.85,
    a: 0.9,
  };
}

function selectedColor(metric: string, sonar: SonarData): Rgb {
  switch (metric) {
    case "sonarTemperature":
      return interpolateColor(sonar.temperature ?? undefined, 18, 25);
    case "temperature1":
      return interpolateColor(temperatures.temperature1, 18, 25);
    case "temperature2":
      return interpolateColor(temperatures.temperature2, 18, 25);
    case "temperature3":
      return interpolateColor(temperatures.temperature3, 18, 25);
    case "depth":
    default:
      return interpolateColor(sonar.depth, 0, 5);
  }
}

function selectedMetric(metric: string, sonar: SonarData): string {
  switch (metric) {
    case "sonarTemperature":
      return `Sonar temperature: ${display(sonar.temperature, "°C")}`;
    case "temperature1":
      return `Temperature sensor 1: ${display(temperatures.temperature1, "°C")}`;
    case "temperature2":
      return `Temperature sensor 2: ${display(temperatures.temperature2, "°C")}`;
    case "temperature3":
      return `Temperature sensor 3: ${display(temperatures.temperature3, "°C")}`;
    case "depth":
    default:
      return `Depth: ${display(sonar.depth, "m")}`;
  }
}

export default function script(
  event: Input<InputTopic>,
  globalVariables: LayoutVariables,
): LocationFix | undefined {
  const payload = parsePayload((event.message as StringMessage).data);
  if (payload == undefined) {
    return undefined;
  }

  if (event.topic === GNSS_TOPIC) {
    const data = payload.data as GnssData;
    if (isFiniteNumber(data?.lat) && isFiniteNumber(data?.lon)) {
      gnss = data;
    }
    return undefined;
  }

  const temperatureKey = temperatureTopics[event.topic];
  if (temperatureKey != undefined) {
    if (isFiniteNumber(payload.data)) {
      temperatures[temperatureKey] = payload.data;
    }
    return undefined;
  }

  if (event.topic !== SONAR_TOPIC || gnss == undefined) {
    return undefined;
  }

  const sonar = payload.data as SonarData;
  if (!isFiniteNumber(gnss.lat) || !isFiniteNumber(gnss.lon) || !isFiniteNumber(sonar?.depth)) {
    return undefined;
  }

  const metric = globalVariables.mapMetric ?? "depth";
  const heading = isFiniteNumber(gnss.track) ? (gnss.track * Math.PI) / 180 : undefined;
  const speed = gnss.speed;

  return {
    timestamp: event.receiveTime,
    frame_id: "karaburan_gnss",
    latitude: gnss.lat,
    longitude: gnss.lon,
    altitude: gnss.altMSL ?? gnss.altHAE ?? 0,
    position_covariance: new Float64Array(9),
    position_covariance_type: 0,
    heading,
    velocity:
      isFiniteNumber(speed) && heading != undefined
        ? {
            x: speed * Math.sin(heading),
            y: speed * Math.cos(heading),
            z: 0,
          }
        : undefined,
    color: selectedColor(metric, sonar),
    metadata: [
      { key: "Map color", value: selectedMetric(metric, sonar) },
      { key: "Depth", value: display(sonar.depth, "m") },
      { key: "Sonar temperature", value: display(sonar.temperature, "°C") },
      { key: "Temperature sensor 1 (28.C23646D48524)", value: display(temperatures.temperature1, "°C") },
      { key: "Temperature sensor 2 (28.5CD456B5013C)", value: display(temperatures.temperature2, "°C") },
      { key: "Temperature sensor 3 (28.F95856B5013C)", value: display(temperatures.temperature3, "°C") },
      { key: "Sonar battery", value: display(sonar.battery, "%") },
      { key: "GNSS mode", value: gnss.mode?.toString() ?? "unavailable" },
    ],
  };
}
