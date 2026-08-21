import type { Input, Message } from "./types.ts";

type InputTopic =
  | "karaburan/sensors/gnss/rtkgps/measurement"
  | "karaburan/sensors/sonar/D3:01:01:02:2F:C6/measurement"
  | "karaburan/sensors/temperature/28.C23646D48524/measurement"
  | "karaburan/sensors/temperature/28.5CD456B5013C/measurement"
  | "karaburan/sensors/temperature/28.F95856B5013C/measurement";

type StringMessage = Message<"std_msgs/msg/String">;

type GnssData = {
  lat?: number;
  lon?: number;
};

type SonarData = {
  depth?: number;
  temperature?: number | null;
};

type Payload = {
  time?: number;
  data?: number | null | GnssData | SonarData;
};

type Output = {
  measurementTime: number;
  latitude: number;
  longitude: number;
  depth: number;
  sonarTemperature: number;
  temperature1: number;
  temperature2: number;
  temperature3: number;
};

type TemperatureState = {
  temperature1?: number;
  temperature2?: number;
  temperature3?: number;
};

export const inputs: InputTopic[] = [
  "karaburan/sensors/gnss/rtkgps/measurement",
  "karaburan/sensors/sonar/D3:01:01:02:2F:C6/measurement",
  "karaburan/sensors/temperature/28.C23646D48524/measurement",
  "karaburan/sensors/temperature/28.5CD456B5013C/measurement",
  "karaburan/sensors/temperature/28.F95856B5013C/measurement",
];

export const output = "/karaburan/sensor_metrics";

const GNSS_TOPIC = "karaburan/sensors/gnss/rtkgps/measurement";
const SONAR_TOPIC = "karaburan/sensors/sonar/D3:01:01:02:2F:C6/measurement";

const temperatureTopics: Record<string, keyof TemperatureState> = {
  "karaburan/sensors/temperature/28.C23646D48524/measurement": "temperature1",
  "karaburan/sensors/temperature/28.5CD456B5013C/measurement": "temperature2",
  "karaburan/sensors/temperature/28.F95856B5013C/measurement": "temperature3",
};

let gnss: GnssData | undefined;
const temperatures: TemperatureState = {};

// Foxglove treats an undefined runtime result as "do not publish", while some
// versions reject an explicit Output | undefined union for custom schemas.
const NO_OUTPUT = undefined as unknown as Output;

function parsePayload(data: string): Payload | undefined {
  try {
    return JSON.parse(data.replace(/\bNaN\b/g, "null")) as Payload;
  } catch {
    return undefined;
  }
}

function isFiniteNumber(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}

export default function script(event: Input<InputTopic>): Output {
  const payload = parsePayload((event.message as StringMessage).data);
  if (payload == undefined) {
    return NO_OUTPUT;
  }

  if (event.topic === GNSS_TOPIC) {
    const data = payload.data as GnssData;
    if (isFiniteNumber(data?.lat) && isFiniteNumber(data?.lon)) {
      gnss = data;
    }
    return NO_OUTPUT;
  }

  const temperatureKey = temperatureTopics[event.topic];
  if (temperatureKey != undefined) {
    if (isFiniteNumber(payload.data)) {
      temperatures[temperatureKey] = payload.data;
    }
    return NO_OUTPUT;
  }

  if (event.topic !== SONAR_TOPIC || gnss == undefined) {
    return NO_OUTPUT;
  }

  const sonar = payload.data as SonarData;
  if (!isFiniteNumber(gnss.lat) || !isFiniteNumber(gnss.lon) || !isFiniteNumber(sonar?.depth)) {
    return NO_OUTPUT;
  }

  return {
    measurementTime:
      isFiniteNumber(payload.time)
        ? payload.time
        : event.receiveTime.sec + event.receiveTime.nsec / 1_000_000_000,
    latitude: gnss.lat,
    longitude: gnss.lon,
    depth: sonar.depth,
    sonarTemperature: isFiniteNumber(sonar.temperature) ? sonar.temperature : Number.NaN,
    temperature1: temperatures.temperature1 ?? Number.NaN,
    temperature2: temperatures.temperature2 ?? Number.NaN,
    temperature3: temperatures.temperature3 ?? Number.NaN,
  };
}
