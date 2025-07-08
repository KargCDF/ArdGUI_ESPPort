//  proto.js  – pack/unpack the 8-byte frame used by the ESP32
export const SYNC = 0xAA;
export const CMD  = {
  SET:    0x01,
  TOGGLE: 0x02,
  GET:    0x03,
  ACK:    0x81,
  SAVE_NVS:   0x20,
  LOAD_NVS:   0x21,
  SAVE_PRESET: 0x30,  // NEW
  LOAD_PRESET: 0x31,  // NEW
  LIST_PRESETS: 0x32, // NEW
  DELETE_PRESET: 0x33  // NEW
};

export function buildFrame(cmd, id, value = 0) {
  const buf = new Uint8Array(8);
  buf[0] = SYNC;
  buf[1] = cmd;
  buf[2] = id;
  const dv = new DataView(buf.buffer, 3, 4);
  dv.setFloat32(0, value, true);        // little-endian
  let crc = cmd ^ id;
  for (let i = 3; i < 7; ++i) crc ^= buf[i];
  buf[7] = crc;
  return buf;
}

export function parseFrame(buf) {
  if (buf.length !== 8 || buf[0] !== SYNC) return null;
  let crc = buf[1] ^ buf[2];
  for (let i = 3; i < 7; ++i) crc ^= buf[i];
  if (crc !== buf[7]) return null;

  const dv = new DataView(buf.buffer, 3, 4);
  return {
    cmd:   buf[1],
    id:    buf[2],
    value: dv.getFloat32(0, true)
  };
}
