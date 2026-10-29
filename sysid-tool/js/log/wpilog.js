/**
 * WPILib DataLog (.wpilog) binary reader/writer.
 *
 * Implements the "WPILib Data Log File Format Specification, Version 1.0":
 * https://github.com/wpilibsuite/allwpilib/blob/main/datalog/doc/datalog.adoc
 */

const HEADER_MAGIC = "WPILOG";

function readU16(view, offset) {
  return view.getUint16(offset, true);
}
function readI32(view, offset) {
  return view.getInt32(offset, true);
}

/**
 * Parses a .wpilog file.
 *
 * @param {ArrayBuffer} buffer
 * @returns {{extraHeader: object, entries: Array<{id, name, type, metadata, values}>, numRecords: number, numEntries: number}}
 */
export function parseWpilog(buffer) {
  const bytes = new Uint8Array(buffer);
  const view = new DataView(buffer);

  if (bytes.length < 12) throw new Error("Not a valid datalog file (too short)");
  const magic = String.fromCharCode(...bytes.slice(0, 6));
  if (magic !== HEADER_MAGIC) throw new Error("Not a valid datalog file (bad magic)");

  const major = readU16(view, 6) >> 8;
  const extraLen = readI32(view, 8);
  let offset = 12;
  let extraHeader = {};
  if (extraLen > 0) {
    const extra = new TextDecoder().decode(bytes.slice(offset, offset + extraLen));
    try {
      extraHeader = JSON.parse(extra);
    } catch {
      extraHeader = { raw: extra };
    }
    offset += extraLen;
  }
  if (major > 1) throw new Error(`Unsupported datalog major version ${major}`);

  const entriesById = new Map();
  const entries = [];
  let numRecords = 0;

  const decoder = new TextDecoder();

  while (offset < bytes.length) {
    const hdr = bytes[offset++];
    const idLen = (hdr & 0x03) + 1;
    const sizeLen = ((hdr >> 2) & 0x03) + 1;
    const tsLen = ((hdr >> 4) & 0x07) + 1;

    let entryId = 0;
    for (let i = 0; i < idLen; i++) entryId |= bytes[offset + i] << (8 * i);
    offset += idLen;

    let payloadSize = 0;
    for (let i = 0; i < sizeLen; i++) payloadSize |= bytes[offset + i] << (8 * i);
    offset += sizeLen;

    let timestamp = 0;
    for (let i = 0; i < tsLen; i++) timestamp += bytes[offset + i] * 2 ** (8 * i);
    offset += tsLen;

    const payload = bytes.subarray(offset, offset + payloadSize);
    offset += payloadSize;
    numRecords++;

    if (entryId === 0) {
      // Control record
      const pv = new DataView(buffer, offset - payloadSize, payloadSize);
      const controlType = payload[0];
      if (controlType === 0) {
        // Start
        const id = pv.getUint32(1, true);
        const nameLen = pv.getUint32(5, true);
        const name = decoder.decode(payload.subarray(9, 9 + nameLen));
        const typeLen = pv.getUint32(9 + nameLen, true);
        const type = decoder.decode(
          payload.subarray(13 + nameLen, 13 + nameLen + typeLen)
        );
        const metaLen = pv.getUint32(13 + nameLen + typeLen, true);
        const metadata = decoder.decode(
          payload.subarray(17 + nameLen + typeLen, 17 + nameLen + typeLen + metaLen)
        );
        const entry = { id, name, type, metadata, values: [] };
        entriesById.set(id, entry);
        entries.push(entry);
      } else if (controlType === 1) {
        // Finish
        const id = pv.getUint32(1, true);
        const entry = entriesById.get(id);
        if (entry) entry.finished = true;
      } else if (controlType === 2) {
        // Set metadata
        const id = pv.getUint32(1, true);
        const metaLen = pv.getUint32(5, true);
        const metadata = decoder.decode(payload.subarray(9, 9 + metaLen));
        const entry = entriesById.get(id);
        if (entry) entry.metadata = metadata;
      }
      continue;
    }

    const entry = entriesById.get(entryId);
    if (!entry) continue;

    const t = timestamp * 1e-6; // seconds
    const value = decodeValue(entry.type, payload, buffer, offset - payloadSize, decoder);
    entry.values.push({ t, v: value });
  }

  return { extraHeader, entries, numRecords, numEntries: entries.length };
}

function decodeValue(type, payload, buffer, payloadOffset, decoder) {
  const view = new DataView(buffer, payloadOffset, payload.length);
  switch (type) {
    case "boolean":
      return payload[0] !== 0;
    case "int64":
      return Number(view.getBigInt64(0, true));
    case "float":
      return view.getFloat32(0, true);
    case "double":
      return view.getFloat64(0, true);
    case "string":
      return decoder.decode(payload);
    case "boolean[]": {
      const count = view.getUint32(0, true);
      return Array.from({ length: count }, (_, i) => payload[4 + i] !== 0);
    }
    case "int64[]": {
      const count = view.getUint32(0, true);
      return Array.from({ length: count }, (_, i) =>
        Number(view.getBigInt64(4 + i * 8, true))
      );
    }
    case "float[]": {
      const count = view.getUint32(0, true);
      return Array.from({ length: count }, (_, i) => view.getFloat32(4 + i * 4, true));
    }
    case "double[]": {
      const count = view.getUint32(0, true);
      return Array.from({ length: count }, (_, i) => view.getFloat64(4 + i * 8, true));
    }
    case "string[]": {
      const count = view.getUint32(0, true);
      const out = [];
      let pos = 4;
      for (let i = 0; i < count; i++) {
        const len = view.getUint32(pos, true);
        pos += 4;
        out.push(decoder.decode(payload.subarray(pos, pos + len)));
        pos += len;
      }
      return out;
    }
    default:
      return null;
  }
}

// ---------------------------------------------------------------------------
// Writer (used for demo export and round-trip tests)
// ---------------------------------------------------------------------------

function minimalLength(value) {
  if (value < 0x100) return 1;
  if (value < 0x10000) return 2;
  if (value < 0x1000000) return 3;
  return 4;
}

function writeVarLen(out, value, len) {
  for (let i = 0; i < len; i++) out.push(Math.floor(value / 2 ** (8 * i)) & 0xff);
}

function writeRecord(chunks, entryId, timestampUs, payload) {
  const idLen = minimalLength(entryId);
  const sizeLen = minimalLength(payload.length);
  const tsLen = timestampUs < 0x100 ? 1 : timestampUs < 0x10000 ? 2 : timestampUs < 0x1000000 ? 3 : timestampUs < 0x100000000 ? 4 : timestampUs < 0x1000000000000 ? 6 : 8;
  chunks.push(Uint8Array.from([(idLen - 1) | ((sizeLen - 1) << 2) | ((tsLen - 1) << 4)]));
  const head = [];
  writeVarLen(head, entryId, idLen);
  writeVarLen(head, payload.length, sizeLen);
  writeVarLen(head, timestampUs, tsLen);
  chunks.push(Uint8Array.from(head));
  chunks.push(payload);
}

function encodeValue(type, value, encoder) {
  switch (type) {
    case "boolean":
      return Uint8Array.from([value ? 1 : 0]);
    case "int64": {
      const buf = new ArrayBuffer(8);
      new DataView(buf).setBigInt64(0, BigInt(Math.round(value)), true);
      return new Uint8Array(buf);
    }
    case "float": {
      const buf = new ArrayBuffer(4);
      new DataView(buf).setFloat32(0, value, true);
      return new Uint8Array(buf);
    }
    case "double": {
      const buf = new ArrayBuffer(8);
      new DataView(buf).setFloat64(0, value, true);
      return new Uint8Array(buf);
    }
    case "string":
      return encoder.encode(value);
    default:
      throw new Error(`Unsupported write type ${type}`);
  }
}

/**
 * Writes entries to a .wpilog ArrayBuffer.
 *
 * @param {Array<{name, type, metadata?, values: Array<{t, v}>}>} entries
 * @param {object} [extraHeader]
 */
export function writeWpilog(entries, extraHeader = {}) {
  const encoder = new TextEncoder();
  const chunks = [];

  const extra = encoder.encode(JSON.stringify(extraHeader));
  const header = [];
  header.push(...encoder.encode(HEADER_MAGIC));
  header.push(0x00, 0x01); // version 1.0 LE (0x0100)
  const extraLen = [];
  writeVarLen(extraLen, extra.length, 4);
  header.push(...extraLen, ...extra);
  chunks.push(Uint8Array.from(header));

  entries.forEach((entry, index) => {
    const id = index + 1;
    const nameBytes = encoder.encode(entry.name);
    const typeBytes = encoder.encode(entry.type);
    const metaBytes = encoder.encode(entry.metadata ?? "");
    const payload = [];
    payload.push(0); // Start control record
    writeVarLen(payload, id, 4);
    writeVarLen(payload, nameBytes.length, 4);
    payload.push(...nameBytes);
    writeVarLen(payload, typeBytes.length, 4);
    payload.push(...typeBytes);
    writeVarLen(payload, metaBytes.length, 4);
    payload.push(...metaBytes);
    writeRecord(chunks, 0, 0, Uint8Array.from(payload));

    for (const { t, v } of entry.values) {
      writeRecord(chunks, id, Math.round(t * 1e6), encodeValue(entry.type, v, encoder));
    }

    const finish = [];
    finish.push(1); // Finish control record
    writeVarLen(finish, id, 4);
    writeRecord(chunks, 0, 0, Uint8Array.from(finish));
  });

  const total = chunks.reduce((n, c) => n + c.length, 0);
  const out = new Uint8Array(total);
  let pos = 0;
  for (const c of chunks) {
    out.set(c, pos);
    pos += c.length;
  }
  return out.buffer;
}
