/*  ParameterModel.ts – clean rebuild (July 2025)
    • MCU-visible parameters end with “Raw” (riseTimeRaw, …)
    • Derived helpers keep the original names (maxAngle, loopHeight, …)
    • Maths copied verbatim from gui_mk5.py / OldParameterModel.js
    • Emits "change" & "preset" events
*/
function clone(o) {
    return typeof structuredClone === "function"
        ? structuredClone(o)
        : JSON.parse(JSON.stringify(o));
}
/* ───── MCU-side table (unchanged) ───── */
export var ParamID;
(function (ParamID) {
    ParamID[ParamID["riseTimeRaw"] = 1] = "riseTimeRaw";
    ParamID[ParamID["fallTimeRaw"] = 2] = "fallTimeRaw";
    ParamID[ParamID["holdHighRaw"] = 3] = "holdHighRaw";
    ParamID[ParamID["holdLowRaw"] = 4] = "holdLowRaw";
    ParamID[ParamID["minAngleRaw"] = 5] = "minAngleRaw";
    ParamID[ParamID["maxAngleRaw"] = 6] = "maxAngleRaw";
    ParamID[ParamID["stepsPerSecondRaw"] = 7] = "stepsPerSecondRaw";
    ParamID[ParamID["servoPwmMin"] = 8] = "servoPwmMin";
    ParamID[ParamID["servoPwmMax"] = 9] = "servoPwmMax";
    ParamID[ParamID["runYoke"] = 16] = "runYoke";
    ParamID[ParamID["runConveyor"] = 17] = "runConveyor";
})(ParamID || (ParamID = {}));
/* factory defaults (same numbers as Python script) */
const D = {
    riseTimeRaw: 120.48734593882432, fallTimeRaw: 120.48734593882432,
    holdHighRaw: 159.0, holdLowRaw: 715.0,
    minAngleRaw: 23.0, maxAngleRaw: 83.24, // will be overridden by derived maxAngle if you prefer
    stepsPerSecondRaw: 39.78873577297383,
    servoPwmMin: 650, servoPwmMax: 2610, runYoke: 0, runConveyor: 0,
    feedRate: 600, loopLength: 4, loopHeightInput: 15, retractLength: 7.15,
    degSec: 0.002, yokeLength: 17, muSteps: 0.0625, degStep: 0.9, driveDiameter: 64
};
/* ───── Name ⇄ ID maps (MCU only) ───── */
export const NameToID = {
    riseTimeRaw: 0x01, fallTimeRaw: 0x02, holdHighRaw: 0x03, holdLowRaw: 0x04,
    minAngleRaw: 0x05, maxAngleRaw: 0x06, stepsPerSecondRaw: 0x07,
    servoPwmMin: 0x08, servoPwmMax: 0x09, runYoke: 0x10, runConveyor: 0x11
};
export const IDToName = Object.fromEntries(Object.entries(NameToID).map(([k, v]) => [v, k]));
/* ══════════  CLASS  ══════════════════════════════════════ */
export class ParameterModel extends EventTarget {
    constructor(init = {}) { super(); Object.assign(this, clone(D), init); }
    /* ─── presets ─── */
    static async loadPresets() {
        try {
            const r = await fetch("/parameterPresets.json");
            if (r.ok)
                return r.json();
        }
        catch { }
        return { Default: clone(D) };
    }
    static async fromPreset(name = "Default") {
        const p = await this.loadPresets();
        return new ParameterModel(ParameterModel._sanitize(p[name] ?? D));
    }
    async applyPreset(name, presets) {
        const p = presets ?? await ParameterModel.loadPresets();
        if (!(name in p))
            throw Error(`Preset '${name}' not found`);
        Object.assign(this, ParameterModel._sanitize(p[name]));
        this.dispatchEvent(new CustomEvent("preset", { detail: { name } }));
    }
    /** migrate old-format objects (maxAngle → maxAngleRaw, …) */
    static _sanitize(obj) {
        const map = {
            maxAngle: "maxAngleRaw",
            minAngle: "minAngleRaw",
            stepsPerSecond: "stepsPerSecondRaw",
            riseTime: "riseTimeRaw",
            fallTime: "fallTimeRaw",
            holdHigh: "holdHighRaw",
            holdLow: "holdLowRaw"
        };
        const o = {};
        for (const [k, v] of Object.entries(obj))
            o[map[k] ?? k] = v;
        return { ...clone(D), ...o };
    }
    /** returns the JSON string you can store (NVS, localStorage, etc.) */
    savePreset() {
        /* only store the browser-editable fields (your list #2a) */
        const editable = {
            feedRate: this.feedRate, loopHeightInput: this.loopHeightInput,
            loopLength: this.loopLength, retractLength: this.retractLength,
            minAngleRaw: this.minAngleRaw, degSec: this.degSec,
            yokeLength: this.yokeLength, muSteps: this.muSteps,
            degStep: this.degStep, driveDiameter: this.driveDiameter,
            servoPwmMin: this.servoPwmMin, servoPwmMax: this.servoPwmMax,
            riseTimeRaw: this.riseTimeRaw, fallTimeRaw: this.fallTimeRaw
        };
        return JSON.stringify(editable, null, 2);
    }
    /* ─── MCU helpers ─── */
    toSerialDict() { const o = {}; for (const [k, id] of Object.entries(NameToID))
        o[id] = this[k]; return o; }
    applySerialDict(dict) {
        for (const [id, v] of Object.entries(dict)) {
            const n = IDToName[Number(id)];
            if (n)
                this[n] = v;
        }
        this.dispatchEvent(new Event("change"));
    }
    /* ─── generic update ─── */
    update(f, v) { this[f] = v; this.dispatchEvent(new CustomEvent("change", { detail: { field: f, value: v } })); }
    /* ─── math helpers (ported 1:1) ─── */
    _deg(r) { return r * 180 / Math.PI; }
    _rad(d) { return d * Math.PI / 180; }
    /* derived geometry */
    get maxAngle() {
        const r = (this.yokeLength - this.loopHeightInput) / this.yokeLength;
        return this._deg(Math.acos(Math.min(1, Math.max(-1, r))));
    }
    get thetaRad() { return this._rad(this.maxAngle - this.minAngleRaw); }
    get loopHeight() { return this.yokeLength * (1 - Math.cos(this.thetaRad)); }
    /* timing */
    get travelSpeed_mm_ms() { return this.feedRate / 60000; }
    get deg_ms() { return this.degSec * 1000; }
    get minTime() { return this.deg_ms * (this.maxAngle - this.minAngleRaw); }
    get riseLength() { return this.riseTimeRaw * this.travelSpeed_mm_ms; }
    get fallLength() { return this.fallTimeRaw * this.travelSpeed_mm_ms; }
    get holdHighCalc() { return Math.max(0, (this.loopLength - this.riseLength - this.fallLength) / this.travelSpeed_mm_ms); }
    get holdLowCalc() { return this.retractLength / this.travelSpeed_mm_ms; }
    /* stepper */
    get stepsPerRev() { return 360 / this.degStep / this.muSteps; }
    get stepMM() { return Math.PI * this.driveDiameter / this.stepsPerRev; }
    get stepsSecCalc() { return this.feedRate / 60 / this.stepMM; }
    /* JSON helper */
    toJSON() { return { ...this }; }
}
