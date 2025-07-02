/*  ParameterModel.ts – clean rebuild (July 2025)
    • MCU-visible parameters end with “Raw” (riseTimeRaw, …)
    • Derived helpers keep the original names (maxAngle, loopHeight, …)
    • Maths copied verbatim from gui_mk5.py / OldParameterModel.js
    • Emits "change" & "preset" events
*/

function clone<T>(o: T): T {
  return typeof structuredClone === "function"
    ? structuredClone(o)
    : JSON.parse(JSON.stringify(o));
}

/* ───── MCU-side table (unchanged) ───── */
export enum ParamID {
  riseTimeRaw       = 0x01,
  fallTimeRaw       = 0x02,
  holdHighRaw       = 0x03,
  holdLowRaw        = 0x04,
  minAngleRaw       = 0x05,
  maxAngleRaw       = 0x06,
  stepsPerSecondRaw = 0x07,
  servoPwmMin       = 0x08,
  servoPwmMax       = 0x09,
  runYoke           = 0x10,
  runConveyor       = 0x11
}

/* ───── full data shape ───── */
export interface ParameterShape {
  /* MCU fields (must match IDs above) */
  riseTimeRaw:number;  fallTimeRaw:number; holdHighRaw:number; holdLowRaw:number;
  minAngleRaw:number;  maxAngleRaw:number; stepsPerSecondRaw:number;
  servoPwmMin:number;  servoPwmMax:number; runYoke:number; runConveyor:number;

  /* UI-only inputs */
  feedRate:number; loopLength:number; loopHeightInput:number; retractLength:number;
  degSec:number; yokeLength:number; muSteps:number; degStep:number; driveDiameter:number;
}

/* factory defaults (same numbers as Python script) */
const D: ParameterShape = {
  riseTimeRaw: 120.48734593882432,  fallTimeRaw: 120.48734593882432,
  holdHighRaw: 159.0,               holdLowRaw:  715.0,
  minAngleRaw: 23.0,                maxAngleRaw: 83.24,      // will be overridden by derived maxAngle if you prefer
  stepsPerSecondRaw: 39.78873577297383,
  servoPwmMin: 650,  servoPwmMax: 2610,  runYoke: 0, runConveyor: 0,
  feedRate: 600, loopLength: 4, loopHeightInput: 15, retractLength: 7.15,
  degSec: 0.002, yokeLength: 17, muSteps: 0.0625, degStep: 0.9, driveDiameter: 64
};

/* ───── Name ⇄ ID maps (MCU only) ───── */
export const NameToID = {
  riseTimeRaw:0x01, fallTimeRaw:0x02, holdHighRaw:0x03, holdLowRaw:0x04,
  minAngleRaw:0x05, maxAngleRaw:0x06, stepsPerSecondRaw:0x07,
  servoPwmMin:0x08, servoPwmMax:0x09, runYoke:0x10, runConveyor:0x11
} as const;

export const IDToName = Object.fromEntries(
  Object.entries(NameToID).map(([k,v])=>[v,k])
) as Record<ParamID, keyof typeof NameToID>;

/* event payloads */
interface Chg { field:keyof ParameterShape; value:number }
interface Pre  { name:string }

/* ══════════  CLASS  ══════════════════════════════════════ */
export class ParameterModel extends EventTarget {
  /* raw fields (strict) */
  riseTimeRaw!:number;  fallTimeRaw!:number; holdHighRaw!:number; holdLowRaw!:number;
  minAngleRaw!:number;  maxAngleRaw!:number; stepsPerSecondRaw!:number;
  servoPwmMin!:number;  servoPwmMax!:number; runYoke!:number; runConveyor!:number;
  feedRate!:number; loopLength!:number; loopHeightInput!:number; retractLength!:number;
  degSec!:number; yokeLength!:number; muSteps!:number; degStep!:number; driveDiameter!:number;

  constructor(init:Partial<ParameterShape>={}) { super(); Object.assign(this, clone(D), init); }

  /* ─── presets ─── */
  static async loadPresets(){
    try{const r=await fetch("/parameterPresets.json"); if(r.ok) return r.json();}catch{}
    return {Default: clone(D)};
  }
  static async fromPreset(name="Default"){
    const p = await this.loadPresets();
    return new ParameterModel( ParameterModel._sanitize(p[name] ?? D) );
  }
  async applyPreset(name:string, presets?:Record<string,any>){
    const p = presets ?? await ParameterModel.loadPresets();
    if (!(name in p)) throw Error(`Preset '${name}' not found`);
    Object.assign(this, ParameterModel._sanitize(p[name]) );
    this.dispatchEvent(new CustomEvent<Pre>("preset",{detail:{name}}));
  }

  /** migrate old-format objects (maxAngle → maxAngleRaw, …) */
  private static _sanitize(obj:any): ParameterShape {
    const map:Record<string, keyof ParameterShape> = {
      maxAngle : "maxAngleRaw",
      minAngle : "minAngleRaw",
      stepsPerSecond : "stepsPerSecondRaw",
      riseTime : "riseTimeRaw",
      fallTime : "fallTimeRaw",
      holdHigh : "holdHighRaw",
      holdLow  : "holdLowRaw"
    };
    const o:any = {};
    for (const [k,v] of Object.entries(obj)) o[ map[k] ?? k ] = v;
    return { ...clone(D), ...o } as ParameterShape;
  }

  /** returns the JSON string you can store (NVS, localStorage, etc.) */
  savePreset(): string {
    /* only store the browser-editable fields (your list #2a) */
    const editable: Partial<ParameterShape> = {
      feedRate:this.feedRate, loopHeightInput:this.loopHeightInput,
      loopLength:this.loopLength, retractLength:this.retractLength,
      minAngleRaw:this.minAngleRaw, degSec:this.degSec,
      yokeLength:this.yokeLength, muSteps:this.muSteps,
      degStep:this.degStep, driveDiameter:this.driveDiameter,
      servoPwmMin:this.servoPwmMin, servoPwmMax:this.servoPwmMax,
      riseTimeRaw:this.riseTimeRaw, fallTimeRaw:this.fallTimeRaw
    };
    return JSON.stringify(editable, null, 2);
  }

  /* ─── MCU helpers ─── */
  toSerialDict(){const o:any={}; for(const [k,id] of Object.entries(NameToID))o[id]=(this as any)[k]; return o}
  applySerialDict(dict:Record<ParamID,number>){
    for(const [id,v] of Object.entries(dict)){const n=IDToName[Number(id) as ParamID]; if(n)(this as any)[n]=v;}
    this.dispatchEvent(new Event("change"));
  }

  /* ─── generic update ─── */
  update<K extends keyof ParameterShape>(f:K,v:number){(this as any)[f]=v; this.dispatchEvent(new CustomEvent<Chg>("change",{detail:{field:f,value:v}}));}

  /* ─── math helpers (ported 1:1) ─── */
  private _deg(r:number){return r*180/Math.PI}
  private _rad(d:number){return d*Math.PI/180}

  /* derived geometry */
  get maxAngle():number{
    const r=(this.yokeLength-this.loopHeightInput)/this.yokeLength;
    return this._deg(Math.acos(Math.min(1,Math.max(-1,r))));
  }
  get thetaRad(){return this._rad(this.maxAngle-this.minAngleRaw)}
  get loopHeight(){return this.yokeLength*(1-Math.cos(this.thetaRad))}

  /* timing */
  get travelSpeed_mm_ms(){return this.feedRate/60000}
  get deg_ms(){return this.degSec*1000}
  get minTime(){return this.deg_ms*(this.maxAngle-this.minAngleRaw)}
  get riseLength(){return this.riseTimeRaw*this.travelSpeed_mm_ms}
  get fallLength(){return this.fallTimeRaw*this.travelSpeed_mm_ms}
  get holdHighCalc(){return Math.max(0,(this.loopLength-this.riseLength-this.fallLength)/this.travelSpeed_mm_ms)}
  get holdLowCalc(){return this.retractLength/this.travelSpeed_mm_ms}

  /* stepper */
  get stepsPerRev(){return 360/this.degStep/this.muSteps}
  get stepMM(){return Math.PI*this.driveDiameter/this.stepsPerRev}
  get stepsSecCalc(){return this.feedRate/60/this.stepMM}

  /* JSON helper */
  toJSON():ParameterShape { return {...this}; }
}
