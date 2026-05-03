# SOLAR-TRACKING-DEVICE
import { useState, useEffect, useRef, useCallback } from "react";

// ─── Constants ────────────────────────────────────────────────────────────────
const SUN_RADIUS = 38;
const PANEL_W = 120;
const PANEL_H = 80;
const LDR_THRESHOLD = 12; // px diff before servos move
const SERVO_SPEED = 0.8; // deg per frame

// ─── Utilities ────────────────────────────────────────────────────────────────
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
function lerp(a, b, t) { return a + (b - a) * t; }
function toRad(d) { return (d * Math.PI) / 180; }
function toDeg(r) { return (r * 180) / Math.PI; }

// Map sun canvas position → servo angles
function sunPosToAngles(sx, sy, cw, ch) {
  const pan  = clamp(((sx / cw) - 0.5) * 180, -90, 90);   // left-right
  const tilt = clamp(((1 - sy / ch) - 0.5) * 180, -90, 90); // up-down
  return { pan, tilt };
}

// Efficiency: how well panel faces sun (0–100 %)
function calcEfficiency(panAngle, tiltAngle, targetPan, targetTilt) {
  const dp = toRad(panAngle - targetPan);
  const dt = toRad(tiltAngle - targetTilt);
  const dot = Math.cos(dp) * Math.cos(dt);
  return Math.max(0, dot * 100);
}

// ─── Serial-like log messages ─────────────────────────────────────────────────
let logId = 0;
function makeLog(msg, type = "info") {
  return { id: logId++, time: new Date().toLocaleTimeString("en", { hour12: false }), msg, type };
}

// ─── LDR Visualizer ──────────────────────────────────────────────────────────
function LDRGauge({ label, value }) {
  const pct = clamp(value, 0, 100);
  const color = pct > 60 ? "#f5c518" : pct > 30 ? "#f59518" : "#555";
  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: 4 }}>
      <div style={{ fontSize: 10, color: "#aaa", fontFamily: "monospace", letterSpacing: 1 }}>{label}</div>
      <div style={{
        width: 36, height: 36, borderRadius: "50%",
        border: `3px solid ${color}`,
        boxShadow: `0 0 ${pct * 0.3}px ${color}`,
        background: `radial-gradient(circle, ${color}${Math.round(pct * 2.55).toString(16).padStart(2,"0")} 0%, transparent 70%)`,
        display: "flex", alignItems: "center", justifyContent: "center",
        fontSize: 9, color: "#fff", fontFamily: "monospace"
      }}>{Math.round(pct)}</div>
    </div>
  );
}

// ─── Servo Dial ───────────────────────────────────────────────────────────────
function ServoDial({ label, angle, target }) {
  const r = 28;
  const cx = 34; const cy = 34;
  const needleX = cx + r * Math.sin(toRad(angle));
  const needleY = cy - r * Math.cos(toRad(angle));
  const targetX = cx + r * Math.sin(toRad(target));
  const targetY = cy - r * Math.cos(toRad(target));
  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: 2 }}>
      <div style={{ fontSize: 10, color: "#aaa", fontFamily: "monospace", letterSpacing: 1 }}>{label}</div>
      <svg width={68} height={68}>
        <circle cx={cx} cy={cy} r={r} fill="none" stroke="#333" strokeWidth={2} />
        {[-90,-60,-30,0,30,60,90].map(a => (
          <line key={a}
            x1={cx + (r-6)*Math.sin(toRad(a))} y1={cy - (r-6)*Math.cos(toRad(a))}
            x2={cx + r*Math.sin(toRad(a))}     y2={cy - r*Math.cos(toRad(a))}
            stroke="#444" strokeWidth={1}/>
        ))}
        {/* target ghost */}
        <line x1={cx} y1={cy} x2={targetX} y2={targetY} stroke="#f5c51833" strokeWidth={2} strokeDasharray="3 2" />
        {/* actual needle */}
        <line x1={cx} y1={cy} x2={needleX} y2={needleY} stroke="#f5c518" strokeWidth={2.5} strokeLinecap="round" />
        <circle cx={cx} cy={cy} r={3} fill="#f5c518" />
        <text x={cx} y={cy+r+10} textAnchor="middle" fill="#fff" fontSize={9} fontFamily="monospace">
          {Math.round(angle)}°
        </text>
      </svg>
    </div>
  );
}

// ─── Arduino Code Modal ───────────────────────────────────────────────────────
const ARDUINO_CODE = `// ═══════════════════════════════════════════════════════
//  Dual-Axis Solar Tracker — Arduino Code
//  Hardware: 2× SG90 Servo, 4× LDR, 4× 10kΩ resistor
// ═══════════════════════════════════════════════════════
#include <Servo.h>

Servo panServo;   // Horizontal (left-right)
Servo tiltServo;  // Vertical   (up-down)

// LDR analog pins
const int LDR_TL = A0;  // Top-Left
const int LDR_TR = A1;  // Top-Right
const int LDR_BL = A2;  // Bottom-Left
const int LDR_BR = A3;  // Bottom-Right

// Servo output pins
const int PAN_PIN  = 9;
const int TILT_PIN = 10;

// Tuning
const int TOLERANCE = 10;   // deadband (0–1023)
const int STEP      = 1;    // degrees per update
const int DELAY_MS  = 15;

int panAngle  = 90;  // 0–180 (90 = centre)
int tiltAngle = 90;

void setup() {
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
  Serial.begin(9600);
  Serial.println("Solar Tracker Initialised");
}

void loop() {
  int tl = analogRead(LDR_TL);
  int tr = analogRead(LDR_TR);
  int bl = analogRead(LDR_BL);
  int br = analogRead(LDR_BR);

  // Average pairs
  int top    = (tl + tr) / 2;
  int bottom = (bl + br) / 2;
  int left   = (tl + bl) / 2;
  int right  = (tr + br) / 2;

  // Tilt (vertical)
  int dv = top - bottom;
  if (abs(dv) > TOLERANCE) {
    tiltAngle += (dv > 0) ? STEP : -STEP;
    tiltAngle = constrain(tiltAngle, 0, 180);
    tiltServo.write(tiltAngle);
  }

  // Pan (horizontal)
  int dh = left - right;
  if (abs(dh) > TOLERANCE) {
    panAngle += (dh > 0) ? -STEP : STEP;
    panAngle = constrain(panAngle, 0, 180);
    panServo.write(panAngle);
  }

  // Serial telemetry
  Serial.print("PAN:");  Serial.print(panAngle);
  Serial.print(" TILT:"); Serial.print(tiltAngle);
  Serial.print(" TL:");  Serial.print(tl);
  Serial.print(" TR:");  Serial.print(tr);
  Serial.print(" BL:");  Serial.print(bl);
  Serial.print(" BR:");  Serial.println(br);

  delay(DELAY_MS);
}`;

function CodeModal({ onClose }) {
  const [copied, setCopied] = useState(false);
  return (
    <div style={{
      position:"fixed", inset:0, background:"rgba(0,0,0,0.85)", zIndex:100,
      display:"flex", alignItems:"center", justifyContent:"center", padding:16
    }} onClick={onClose}>
      <div style={{
        background:"#0d1117", border:"1px solid #30363d",
        borderRadius:12, maxWidth:720, width:"100%", maxHeight:"85vh",
        display:"flex", flexDirection:"column", overflow:"hidden"
      }} onClick={e=>e.stopPropagation()}>
        <div style={{
          display:"flex", alignItems:"center", justifyContent:"space-between",
          padding:"12px 16px", borderBottom:"1px solid #21262d"
        }}>
          <span style={{ fontFamily:"monospace", color:"#58a6ff", fontSize:13 }}>
            📄 solar_tracker.ino
          </span>
          <div style={{ display:"flex", gap:8 }}>
            <button onClick={()=>{ navigator.clipboard.writeText(ARDUINO_CODE); setCopied(true); setTimeout(()=>setCopied(false),2000); }}
              style={{ background:"#21262d", border:"1px solid #30363d", color:"#c9d1d9",
                borderRadius:6, padding:"4px 12px", cursor:"pointer", fontSize:12, fontFamily:"monospace" }}>
              {copied ? "✓ Copied!" : "Copy"}
            </button>
            <button onClick={onClose}
              style={{ background:"transparent", border:"none", color:"#6e7681", cursor:"pointer", fontSize:18 }}>✕</button>
          </div>
        </div>
        <pre style={{
          margin:0, padding:16, overflow:"auto", fontSize:12, lineHeight:1.6,
          fontFamily:"'Courier New', monospace", color:"#e6edf3",
          background:"#0d1117", flexGrow:1
        }}>{ARDUINO_CODE}</pre>
      </div>
    </div>
  );
}

// ─── Main App ─────────────────────────────────────────────────────────────────
export default function SolarTrackerSim() {
  const canvasRef = useRef(null);
  const animRef   = useRef(null);

  // Sun position on the sky canvas
  const [sunPos, setSunPos] = useState({ x: 300, y: 80 });
  const [dragging, setDragging] = useState(false);
  const [autoMode, setAutoMode] = useState(true);
  const autoAngle = useRef(0);

  // Servo state
  const [panAngle,  setPanAngle]  = useState(0);
  const [tiltAngle, setTiltAngle] = useState(0);
  const panTarget  = useRef(0);
  const tiltTarget = useRef(0);
  const panCurrent  = useRef(0);
  const tiltCurrent = useRef(0);

  // LDR readings (0–100 light level)
  const [ldrs, setLdrs] = useState({ TL:50, TR:50, BL:50, BR:50 });

  // Serial log
  const [logs, setLogs] = useState([makeLog("Solar Tracker Simulation Ready", "sys")]);
  const addLog = useCallback((msg, type) => {
    setLogs(prev => [...prev.slice(-60), makeLog(msg, type)]);
  }, []);

  // Efficiency
  const [efficiency, setEfficiency] = useState(100);
  const [totalEnergy, setTotalEnergy] = useState(0);

  // Code modal
  const [showCode, setShowCode] = useState(false);

  // Sky canvas dimensions
  const SKY_W = 600;
  const SKY_H = 220;

  // ── Auto sun movement ──────────────────────────────────────────────────────
  useEffect(() => {
    if (!autoMode) return;
    const id = setInterval(() => {
      autoAngle.current += 0.5;
      const a = toRad(autoAngle.current);
      const x = SKY_W * 0.5 + (SKY_W * 0.38) * Math.cos(a);
      const y = SKY_H * 0.5 - (SKY_H * 0.38) * Math.sin(a);
      setSunPos({ x: clamp(x, SUN_RADIUS, SKY_W-SUN_RADIUS), y: clamp(y, SUN_RADIUS, SKY_H-SUN_RADIUS) });
    }, 60);
    return () => clearInterval(id);
  }, [autoMode]);

  // ── Servo tracking loop ────────────────────────────────────────────────────
  useEffect(() => {
    const tick = () => {
      const { pan, tilt } = sunPosToAngles(sunPos.x, sunPos.y, SKY_W, SKY_H);
      panTarget.current  = pan;
      tiltTarget.current = tilt;

      // Simulate servo slew
      const dp = panTarget.current  - panCurrent.current;
      const dt = tiltTarget.current - tiltCurrent.current;

      if (Math.abs(dp) > 0.2) panCurrent.current  += Math.sign(dp) * Math.min(SERVO_SPEED, Math.abs(dp));
      if (Math.abs(dt) > 0.2) tiltCurrent.current += Math.sign(dt) * Math.min(SERVO_SPEED, Math.abs(dt));

      setPanAngle(panCurrent.current);
      setTiltAngle(tiltCurrent.current);

      // LDR simulation: based on how far sun is from each corner of the panel
      const cx = SKY_W * 0.5 + panCurrent.current * (SKY_W / 180);
      const cy = SKY_H * 0.5 - tiltCurrent.current * (SKY_H / 180);
      const spread = 80;
      const dist = (ox, oy) => Math.sqrt((sunPos.x-(cx+ox))**2+(sunPos.y-(cy+oy))**2);
      const ldrVal = d => clamp(100 - d * 0.6, 0, 100);
      setLdrs({
        TL: ldrVal(dist(-spread/2, -spread/2)),
        TR: ldrVal(dist( spread/2, -spread/2)),
        BL: ldrVal(dist(-spread/2,  spread/2)),
        BR: ldrVal(dist( spread/2,  spread/2)),
      });

      // Efficiency
      const eff = calcEfficiency(panCurrent.current, tiltCurrent.current, pan, tilt);
      setEfficiency(eff);
      setTotalEnergy(e => e + eff * 0.0001);

      animRef.current = requestAnimationFrame(tick);
    };
    animRef.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animRef.current);
  }, [sunPos]);

  // ── Serial log drip ────────────────────────────────────────────────────────
  useEffect(() => {
    const id = setInterval(() => {
      addLog(
        `PAN:${panAngle.toFixed(1)}° TILT:${tiltAngle.toFixed(1)}° EFF:${efficiency.toFixed(1)}%`,
        efficiency > 90 ? "good" : efficiency > 50 ? "warn" : "err"
      );
    }, 800);
    return () => clearInterval(id);
  }, [panAngle, tiltAngle, efficiency, addLog]);

  // ── Mouse / touch drag on sky canvas ──────────────────────────────────────
  function getPos(e, rect) {
    const src = e.touches ? e.touches[0] : e;
    return {
      x: clamp(src.clientX - rect.left, 0, SKY_W),
      y: clamp(src.clientY - rect.top,  0, SKY_H)
    };
  }
  function onDown(e) {
    if (autoMode) return;
    e.preventDefault();
    setDragging(true);
    const rect = e.currentTarget.getBoundingClientRect();
    setSunPos(getPos(e, rect));
  }
  function onMove(e) {
    if (!dragging || autoMode) return;
    e.preventDefault();
    const rect = e.currentTarget.getBoundingClientRect();
    setSunPos(getPos(e, rect));
  }
  function onUp() { setDragging(false); }

  // ── Panel 3D projection ────────────────────────────────────────────────────
  const panRad  = toRad(panCurrent.current  * 0.6);
  const tiltRad = toRad(tiltCurrent.current * 0.6);
  const pw = PANEL_W, ph = PANEL_H;
  // Four corners of panel, rotated by pan+tilt
  function rotCorner(lx, ly) {
    // tilt (rotate around x): y → y*cos - z*sin
    const y1 = ly * Math.cos(tiltRad);
    const z1 = ly * Math.sin(tiltRad);
    // pan (rotate around y): x → x*cos + z*sin
    const x2 = lx * Math.cos(panRad) + z1 * Math.sin(panRad);
    const y2 = y1;
    // Isometric-ish projection
    return { x: x2, y: y2 - z1 * 0.3 };
  }
  const corners = [[-pw/2,-ph/2],[pw/2,-ph/2],[pw/2,ph/2],[-pw/2,ph/2]].map(([x,y])=>rotCorner(x,y));
  const cx0 = 160, cy0 = 130;
  const polyPts = corners.map(c=>`${cx0+c.x},${cy0+c.y}`).join(" ");

  // ── Color theme ───────────────────────────────────────────────────────────
  const BG    = "#080c10";
  const CARD  = "#0e1420";
  const BORD  = "#1e2d40";
  const GOLD  = "#f5c518";
  const CYAN  = "#00d4ff";
  const GREEN = "#3ddc84";

  return (
    <div style={{
      minHeight:"100vh", background: BG, color:"#e0e8f0",
      fontFamily:"'Courier New', monospace",
      padding:"0 0 32px",
      backgroundImage:"radial-gradient(ellipse at 20% 0%, #0a1a2e 0%, transparent 60%)"
    }}>
      {showCode && <CodeModal onClose={()=>setShowCode(false)} />}

      {/* ── Header ─────────────────────────────────────────────────────── */}
      <div style={{
        background:"#060a0e", borderBottom:`1px solid ${BORD}`,
        padding:"14px 24px", display:"flex", alignItems:"center",
        justifyContent:"space-between", flexWrap:"wrap", gap:10
      }}>
        <div>
          <div style={{ fontSize:18, fontWeight:700, color: GOLD, letterSpacing:2 }}>
            ☀ SOLAR TRACKER SIM
          </div>
          <div style={{ fontSize:11, color:"#5a7a9a", letterSpacing:1 }}>
            DUAL-AXIS · ARDUINO UNO · 4-LDR FEEDBACK
          </div>
        </div>
        <div style={{ display:"flex", gap:10, flexWrap:"wrap" }}>
          <button onClick={()=>setAutoMode(m=>!m)} style={{
            background: autoMode ? GOLD+"22" : "#1a2535",
            border:`1px solid ${autoMode ? GOLD : BORD}`,
            color: autoMode ? GOLD : "#7090a0",
            borderRadius:6, padding:"6px 14px", cursor:"pointer",
            fontSize:11, letterSpacing:1, fontFamily:"monospace"
          }}>
            {autoMode ? "⏸ AUTO" : "▶ MANUAL"}
          </button>
          <button onClick={()=>setShowCode(true)} style={{
            background:"#0d2035", border:`1px solid ${CYAN}44`,
            color: CYAN, borderRadius:6, padding:"6px 14px",
            cursor:"pointer", fontSize:11, letterSpacing:1, fontFamily:"monospace"
          }}>
            {"</>  ARDUINO CODE"}
          </button>
        </div>
      </div>

      {/* ── Main Grid ──────────────────────────────────────────────────── */}
      <div style={{
        display:"grid",
        gridTemplateColumns:"minmax(300px,1fr) minmax(260px,360px)",
        gap:16, maxWidth:1100, margin:"20px auto 0", padding:"0 16px"
      }}>

        {/* LEFT COLUMN */}
        <div style={{ display:"flex", flexDirection:"column", gap:16 }}>

          {/* Sky Canvas */}
          <div style={{ background: CARD, border:`1px solid ${BORD}`, borderRadius:10, overflow:"hidden" }}>
            <div style={{ padding:"10px 16px", borderBottom:`1px solid ${BORD}`, display:"flex", justifyContent:"space-between", alignItems:"center" }}>
              <span style={{ fontSize:11, color:"#5a7a9a", letterSpacing:1 }}>SKY CANVAS</span>
              {!autoMode && <span style={{ fontSize:10, color: GOLD, animation:"pulse 1.5s infinite" }}>DRAG SUN ↕↔</span>}
            </div>
            <svg width="100%" viewBox={`0 0 ${SKY_W} ${SKY_H}`}
              style={{ display:"block", cursor: autoMode?"default":"crosshair", touchAction:"none", userSelect:"none" }}
              onMouseDown={onDown} onMouseMove={onMove} onMouseUp={onUp} onMouseLeave={onUp}
              onTouchStart={onDown} onTouchMove={onMove} onTouchEnd={onUp}>
              {/* Sky gradient */}
              <defs>
                <radialGradient id="skyGrad" cx="50%" cy="0%" r="100%">
                  <stop offset="0%" stopColor="#1a3a5c"/>
                  <stop offset="100%" stopColor="#040810"/>
                </radialGradient>
                <radialGradient id="sunGlow" cx="50%" cy="50%" r="50%">
                  <stop offset="0%" stopColor="#fff9c4" stopOpacity="1"/>
                  <stop offset="40%" stopColor={GOLD} stopOpacity="0.9"/>
                  <stop offset="100%" stopColor={GOLD} stopOpacity="0"/>
                </radialGradient>
                <filter id="blur4"><feGaussianBlur stdDeviation="4"/></filter>
              </defs>
              <rect width={SKY_W} height={SKY_H} fill="url(#skyGrad)"/>
              {/* Horizon */}
              <line x1={0} y1={SKY_H-2} x2={SKY_W} y2={SKY_H-2} stroke="#1a3a5c" strokeWidth={1}/>
              {/* Sun glow */}
              <circle cx={sunPos.x} cy={sunPos.y} r={SUN_RADIUS*2.5} fill="url(#sunGlow)" filter="url(#blur4)"/>
              {/* Sun body */}
              <circle cx={sunPos.x} cy={sunPos.y} r={SUN_RADIUS} fill={GOLD} opacity={0.95}/>
              <circle cx={sunPos.x} cy={sunPos.y} r={SUN_RADIUS*0.65} fill="#fff9c4"/>
              {/* Target crosshair — where panel is pointing */}
              {(() => {
                const tx = SKY_W*0.5 + panCurrent.current*(SKY_W/180);
                const ty = SKY_H*0.5 - tiltCurrent.current*(SKY_H/180);
                return <>
                  <line x1={tx-14} y1={ty} x2={tx+14} y2={ty} stroke={CYAN} strokeWidth={1.5} opacity={0.7}/>
                  <line x1={tx} y1={ty-14} x2={tx} y2={ty+14} stroke={CYAN} strokeWidth={1.5} opacity={0.7}/>
                  <circle cx={tx} cy={ty} r={6} fill="none" stroke={CYAN} strokeWidth={1.5} opacity={0.7}/>
                </>;
              })()}
              {/* Corner coords */}
              <text x={8} y={16} fill="#2a4a6a" fontSize={10}>W</text>
              <text x={SKY_W-18} y={16} fill="#2a4a6a" fontSize={10}>E</text>
            </svg>
          </div>

          {/* Panel 3D view */}
          <div style={{ background: CARD, border:`1px solid ${BORD}`, borderRadius:10, overflow:"hidden" }}>
            <div style={{ padding:"10px 16px", borderBottom:`1px solid ${BORD}` }}>
              <span style={{ fontSize:11, color:"#5a7a9a", letterSpacing:1 }}>PANEL ORIENTATION</span>
            </div>
            <svg width="100%" viewBox="0 0 320 200" style={{ display:"block" }}>
              <defs>
                <linearGradient id="panelGrad" x1="0" y1="0" x2="1" y2="1">
                  <stop offset="0%" stopColor="#1a3a6c"/>
                  <stop offset="100%" stopColor="#0a1a3c"/>
                </linearGradient>
              </defs>
              {/* Stand */}
              <rect x={150} y={160} width={20} height={30} rx={3} fill="#1a2535"/>
              <ellipse cx={160} cy={192} rx={28} ry={6} fill="#12202e"/>
              {/* Panel */}
              <polygon points={polyPts} fill="url(#panelGrad)" stroke={CYAN} strokeWidth={1.5}/>
              {/* Cell grid */}
              {[1,2,3,4].map(i =>
                <line key={`h${i}`}
                  x1={cx0+corners[0].x + i*(corners[1].x-corners[0].x)/5}
                  y1={cy0+corners[0].y + i*(corners[1].y-corners[0].y)/5}
                  x2={cx0+corners[3].x + i*(corners[2].x-corners[3].x)/5}
                  y2={cy0+corners[3].y + i*(corners[2].y-corners[3].y)/5}
                  stroke={CYAN} strokeWidth={0.5} opacity={0.4}/>
              )}
              {[1,2].map(i =>
                <line key={`v${i}`}
                  x1={cx0+corners[0].x + i*(corners[3].x-corners[0].x)/3}
                  y1={cy0+corners[0].y + i*(corners[3].y-corners[0].y)/3}
                  x2={cx0+corners[1].x + i*(corners[2].x-corners[1].x)/3}
                  y2={cy0+corners[1].y + i*(corners[2].y-corners[1].y)/3}
                  stroke={CYAN} strokeWidth={0.5} opacity={0.4}/>
              )}
              {/* Efficiency glow on panel */}
              <polygon points={polyPts}
                fill={GOLD}
                opacity={efficiency/600}
              />
              {/* Efficiency text */}
              <text x={230} y={100} fill={efficiency>80?GREEN:efficiency>40?GOLD:"#e05050"}
                fontSize={28} fontWeight="bold" textAnchor="middle">
                {efficiency.toFixed(0)}%
              </text>
              <text x={230} y={116} fill="#4a6a8a" fontSize={10} textAnchor="middle">EFFICIENCY</text>
              <text x={230} y={145} fill="#4a6a8a" fontSize={10} textAnchor="middle">
                {totalEnergy.toFixed(3)} Wh
              </text>
              <text x={230} y={158} fill="#2a4a6a" fontSize={9} textAnchor="middle">CUMULATIVE</text>
            </svg>
          </div>
        </div>

        {/* RIGHT COLUMN */}
        <div style={{ display:"flex", flexDirection:"column", gap:16 }}>

          {/* Servo readout */}
          <div style={{ background: CARD, border:`1px solid ${BORD}`, borderRadius:10, overflow:"hidden" }}>
            <div style={{ padding:"10px 16px", borderBottom:`1px solid ${BORD}` }}>
              <span style={{ fontSize:11, color:"#5a7a9a", letterSpacing:1 }}>SERVO ANGLES</span>
            </div>
            <div style={{ padding:16, display:"flex", justifyContent:"space-around" }}>
              <ServoDial label="PAN  (SG90 #1)" angle={panAngle}  target={panTarget.current} />
              <ServoDial label="TILT (SG90 #2)" angle={tiltAngle} target={tiltTarget.current} />
            </div>
          </div>

          {/* LDR readings */}
          <div style={{ background: CARD, border:`1px solid ${BORD}`, borderRadius:10, overflow:"hidden" }}>
            <div style={{ padding:"10px 16px", borderBottom:`1px solid ${BORD}` }}>
              <span style={{ fontSize:11, color:"#5a7a9a", letterSpacing:1 }}>LDR SENSORS (A0–A3)</span>
            </div>
            <div style={{ padding:16 }}>
              <div style={{ display:"grid", gridTemplateColumns:"1fr 1fr", gap:"12px 24px" }}>
                <LDRGauge label="A0 · TOP-LEFT"    value={ldrs.TL}/>
                <LDRGauge label="A1 · TOP-RIGHT"   value={ldrs.TR}/>
                <LDRGauge label="A2 · BOT-LEFT"    value={ldrs.BL}/>
                <LDRGauge label="A3 · BOT-RIGHT"   value={ldrs.BR}/>
              </div>
              <div style={{ marginTop:14, fontSize:10, color:"#3a5a7a", lineHeight:1.6 }}>
                <div>ΔV (top−bot) = {(ldrs.TL+ldrs.TR-ldrs.BL-ldrs.BR).toFixed(0)}</div>
                <div>ΔH (lft−rgt) = {(ldrs.TL+ldrs.BL-ldrs.TR-ldrs.BR).toFixed(0)}</div>
              </div>
            </div>
          </div>

          {/* Serial Monitor */}
          <div style={{ background: CARD, border:`1px solid ${BORD}`, borderRadius:10, overflow:"hidden", flexGrow:1 }}>
            <div style={{ padding:"10px 16px", borderBottom:`1px solid ${BORD}`, display:"flex", alignItems:"center", gap:8 }}>
              <div style={{ width:8, height:8, borderRadius:"50%", background: GREEN, boxShadow:`0 0 6px ${GREEN}` }}/>
              <span style={{ fontSize:11, color:"#5a7a9a", letterSpacing:1 }}>SERIAL MONITOR · 9600 BAUD</span>
            </div>
            <div style={{
              height:220, overflowY:"auto", padding:"8px 12px",
              display:"flex", flexDirection:"column", gap:1,
              background:"#050810"
            }}>
              {logs.map(l => (
                <div key={l.id} style={{
                  fontSize:10, lineHeight:1.5, fontFamily:"monospace",
                  color: l.type==="sys" ? CYAN : l.type==="good" ? GREEN : l.type==="warn" ? GOLD : l.type==="err" ? "#e05050" : "#8aabb0"
                }}>
                  <span style={{ color:"#2a4a6a" }}>[{l.time}]</span> {l.msg}
                </div>
              ))}
            </div>
          </div>

        </div>
      </div>

      {/* ── Bill of Materials strip ──────────────────────────────────────── */}
      <div style={{
        maxWidth:1100, margin:"16px auto 0", padding:"0 16px"
      }}>
        <div style={{ background: CARD, border:`1px solid ${BORD}`, borderRadius:10, overflow:"hidden" }}>
          <div style={{ padding:"10px 16px", borderBottom:`1px solid ${BORD}` }}>
            <span style={{ fontSize:11, color:"#5a7a9a", letterSpacing:1 }}>BILL OF MATERIALS</span>
          </div>
          <div style={{
            display:"grid",
            gridTemplateColumns:"repeat(auto-fill, minmax(155px, 1fr))",
            gap:1, background: BORD
          }}>
            {[
              ["Arduino Uno R3","×1"],["SG90 Servo","×2"],["LDR (GL5528)","×4"],
              ["10kΩ Resistor","×4"],["Mini Solar Panel","×1"],["Breadboard","×1"],
              ["3D-Printed Frame","×1"],["Jumper Wires","×20"],
            ].map(([name, qty]) => (
              <div key={name} style={{
                padding:"10px 14px", background: CARD,
                fontSize:11, display:"flex", justifyContent:"space-between"
              }}>
                <span style={{ color:"#8aabb0" }}>{name}</span>
                <span style={{ color: GOLD }}>{qty}</span>
              </div>
            ))}
          </div>
        </div>
      </div>

      <style>{`
        @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.3} }
      `}</style>
    </div>
  );
}
