// frontend/js/ParameterModel.js
// Mirrors the Python dataclass 1-for-1 (v0.5, June 2025)

export class ParameterModel {
  constructor(init = {}) {
    // fallback hard‐coded defaults
    this.feedRate = 600.0;
    this.loopLength = 4.0;
    this.loopHeightInput = 25.0;
    this.retractLength = 4.0;
    this.minAngle = 23.0;
    this.degSec = 0.002;
    this.riseTime = 116.48734593882432;
    this.fallTime = 116.48734593882432;
    this.yokeLength = 17.0;
    this.muSteps = 0.0625;
    this.degStep = 0.9;
    this.driveDiameter = 64.0;
    this.servoPwmMin = 650;
    this.servoPwmMax = 2610;

    // override from JSON if present
    for (let [k, v] of Object.entries(init)) {
      if (k in this && typeof this[k] === "number") {
        this[k] = v;
      }
    }
    this._validateTimes();
  }

  /**
   * Fetches your new parameterModel.json and merges it.
   */
  static async loadFromJSON() {
    let init = {};
    try {
      const resp = await fetch("/parameterModel.json");
      if (resp.ok) {
        init = await resp.json();
      } else {
        console.warn("parameterModel.json not found:", resp.status);
      }
    } catch (err) {
      console.warn("Error fetching parameterModel.json:", err);
    }
    return new ParameterModel(init);
  }

  /* ───────── helpers ───────── */
  _deg(rad) {
    return (rad * 180) / Math.PI;
  }
  _rad(deg) {
    return (deg * Math.PI) / 180;
  }

  /* ───────── derived values ───────── */
  get maxAngle() {
    const r = Math.max(
      -1,
      Math.min(1, (this.yokeLength - this.loopHeightInput) / this.yokeLength)
    );
    return this._deg(Math.acos(r));
  }
  get thetaRad() {
    return this._rad(this.maxAngle - this.minAngle);
  }
  get computedLoopHeight() {
    return this.yokeLength * (1 - Math.cos(this.thetaRad));
  }
  get loopHeight() {
    return this.computedLoopHeight;
  }
  get travelSpeed_mm_ms() {
    return this.feedRate / 60000;
  }
  get deg_ms() {
    return this.degSec * 1000;
  }
  get minTime() {
    return this.deg_ms * (this.maxAngle - this.minAngle);
  }
  get riseLength() {
    return this.riseTime * this.travelSpeed_mm_ms;
  }
  get fallLength() {
    return this.fallTime * this.travelSpeed_mm_ms;
  }
  get holdHigh() {
    return Math.max(
      0,
      (this.loopLength - this.riseLength - this.fallLength) /
        this.travelSpeed_mm_ms
    );
  }
  get holdLow() {
    return this.retractLength / this.travelSpeed_mm_ms;
  }
  get stepsPerRev() {
    return 360 / this.degStep / this.muSteps;
  }
  get stepMM() {
    return (Math.PI * this.driveDiameter) / this.stepsPerRev;
  }
  get stepsSec() {
    return this.feedRate / 60 / this.stepMM;
  }

  /* ───────── constraint helper ───────── */
  _validateTimes() {
    const min = this.minTime;
    if (this.riseTime < min) this.riseTime = min;
    if (this.fallTime < min) this.fallTime = min;
  }

  /* ───────── public mutator ───────── */
  update(name, value) {
    if (!(name in this)) throw new Error(`Unknown field ${name}`);
    this[name] = value;
    this._validateTimes();
  }

  /* ───────── trimmed dict for MCU ───────── */
  toSerialDict() {
    return {
      Steps_Per_Second: this.stepsSec,
      riseTime: this.riseTime,
      holdHigh: this.holdHigh,
      fallTime: this.fallTime,
      holdLow: this.holdLow,
      minAngle: this.minAngle,
      maxAngle: this.maxAngle,
      servoPwmMin: this.servoPwmMin,
      servoPwmMax: this.servoPwmMax,
    };
  }
}
