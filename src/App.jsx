import React, { useMemo, useRef, useState, useEffect } from "react";
import { motion } from "framer-motion";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer, Label } from "recharts";

// === i18n helper ===
const STR = {
  en: {
    title: "PID Control Playground",
    madeBy: "Created with GPT-5 Thinking",
    subtitle: "Now with a DC motor plant and an animated visualization. Use the controls on the right; plots and animation update automatically.",
    presetZN: "Ziegler–Nichols-ish",
    plant: "Plant",
    first: "1st order",
    second: "Mass–spring–damper",
    dcMotor: "DC motor",
    responseTitleY: "Response y(t) vs reference r(t)",
    responseTitleW: "Speed ω(t) vs reference ω_ref(t)",
    time: "time [s]",
    yAxisY: "y",
    yAxisW: "ω [rad/s]",
    refR: "reference r",
    refW: "reference ω_ref",
    outY: "output y",
    outW: "output ω",
    controlError: "Control input u(t) and error e(t)",
    yAxisUE: "u, e",
    yAxisUEW: "u [V], e [rad/s]",
    viz: "Visualization & Recording",
    play: "Play ▶",
    record: "Record ▼",
    stop: "Stop ■",
    download: "Download video",
    videoNote: "The video is exported as WebM. If you specifically need a GIF, I can add a GIF encoder.",
    gains: "PID Gains",
    kp: "Kp",
    ki: "Ki",
    kd: "Kd",
    n: "Derivative filter N",
    aw: "Anti-windup (conditional integration)",
    umin: "u min",
    umax: "u max",
    setup: "Experiment setup",
    setpointAmp: "setpoint amplitude",
    noiseStd: "noise std",
    dt: "dt (s)",
    T: "horizon T (s)",
    tips: "Tips",
    tip1: "DC motor: controlling speed (ω) can be stiff—start with modest Kp and add Ki slowly.",
    tip2: "Derivative filtering (N) helps reduce noise amplification from Kd.",
    tip3: "Actuator saturation can cause windup—keep anti-windup on when Ki > 0.",
    tau: "τ (time constant)",
    m: "m",
    c: "c",
    k: "k",
    J: "J",
    b: "b",
    Kt: "Kt",
    Ke: "Ke",
    R: "R",
    L: "L",
    TL: "Load torque Tₗ",
    langBtn: "日本語",
  },
  ja: {
    title: "PID制御プレイグラウンド",
    madeBy: "GPT-5 Thinking によって作成",
    subtitle: "DCモータモデルとアニメーション可視化付き。右側のコントロールを調整すると、プロットとアニメが自動更新されます。",
    presetZN: "ジーグラ・ニコルス風",
    plant: "プラント",
    first: "1次遅れ系",
    second: "質量-ばね-ダンパ",
    dcMotor: "DCモータ",
    responseTitleY: "応答 y(t) と基準 r(t)",
    responseTitleW: "角速度 ω(t) と基準 ω_ref(t)",
    time: "時間 [s]",
    yAxisY: "y",
    yAxisW: "ω [rad/s]",
    refR: "基準 r",
    refW: "基準 ω_ref",
    outY: "出力 y",
    outW: "出力 ω",
    controlError: "入力 u(t) と偏差 e(t)",
    yAxisUE: "u, e",
    yAxisUEW: "u [V], e [rad/s]",
    viz: "可視化と録画",
    play: "再生 ▶",
    record: "録画 ▼",
    stop: "停止 ■",
    download: "動画をダウンロード",
    videoNote: "動画は WebM で出力されます。GIF が必要なら追加できます。",
    gains: "PIDゲイン",
    kp: "Kp",
    ki: "Ki",
    kd: "Kd",
    n: "微分フィルタ N",
    aw: "アンチワインドアップ（条件付き積分）",
    umin: "u 最小",
    umax: "u 最大",
    setup: "実験設定",
    setpointAmp: "目標値の大きさ",
    noiseStd: "ノイズ標準偏差",
    dt: "dt (s)",
    T: "地平 T (s)",
    tips: "ヒント",
    tip1: "DCモータの速度制御はシビアになりがち。まず Kp を控えめに、Ki は少しずつ。",
    tip2: "微分フィルタ N を上げるほど速いがノイズに弱くなる。",
    tip3: "アクチュエータ飽和でワインドアップが起きるので、Ki>0 のときは対策を。",
    tau: "τ（時定数）",
    m: "m",
    c: "c",
    k: "k",
    J: "J",
    b: "b",
    Kt: "Kt",
    Ke: "Ke",
    R: "R",
    L: "L",
    TL: "負荷トルク Tₗ",
    langBtn: "EN",
  }
};

// --- Utility helpers ---
function rk4Step(f, state, u, dt) {
  const k1 = f(state, u);
  const s2 = state.map((v, i) => v + 0.5 * dt * k1[i]);
  const k2 = f(s2, u);
  const s3 = state.map((v, i) => v + 0.5 * dt * k2[i]);
  const k3 = f(s3, u);
  const s4 = state.map((v, i) => v + dt * k3[i]);
  const k4 = f(s4, u);
  return state.map((v, i) => v + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
}

// --- Plant models ---
function makePlant(plantType, params) {
  // Returns {x0, f, y} where f(x,u)-> xdot, y(x)
  if (plantType === "first") {
    const tau = Math.max(1e-6, params.tau ?? 1.0);
    return { x0: [0], f: (x, u) => [-(1 / tau) * x[0] + (1 / tau) * u], y: (x) => x[0] };
  }
  if (plantType === "second") {
    const m = Math.max(1e-6, params.m ?? 1.0);
    const c = params.c ?? 0.8;
    const k = params.k ?? 1.0;
    return { x0: [0, 0], f: (x, u) => [x[1], (u - c * x[1] - k * x[0]) / m], y: (x) => x[0] };
  }
  if (plantType === "dcMotor") {
    // Armature-controlled DC motor, output is speed ω
    // Electrical: L di/dt + R i + Ke * ω = u
    // Mechanical: J dω/dt + b ω = Kt * i - T_L
    // State: [θ, ω, i], y = ω
    const J = Math.max(1e-6, params.J ?? 0.01);
    const b = params.b ?? 0.1;
    const Kt = params.Kt ?? 0.02;
    const Ke = params.Ke ?? 0.02;
    const R = Math.max(1e-6, params.R ?? 1.0);
    const L = Math.max(1e-6, params.L ?? 0.5);
    const TL = params.TL ?? 0.0; // constant load torque (disturbance)
    return {
      x0: [0, 0, 0],
      f: (x, u) => {
        const [th, om, i] = x;
        const dth = om;
        const dom = (Kt * i - b * om - TL) / J;
        const di = (u - R * i - Ke * om) / L;
        return [dth, dom, di];
      },
      y: (x) => x[1],
    };
  }
  return makePlant("first", params);
}

// --- PID Controller ---
function pidControllerFactory({ Kp, Ki, Kd, dt, N = 50, aw = true, uMin = -Infinity, uMax = Infinity }) {
  let integ = 0;
  let dFilt = 0;
  let lastMeas = 0;
  return function step(setpoint, meas) {
    const e = setpoint - meas;
    const dm = (meas - lastMeas) / dt; // derivative on measurement
    dFilt = dFilt + dt * (N * (-dFilt + dm));
    lastMeas = meas;
    const u_unsat = Kp * e + Ki * integ - Kd * dFilt;
    let u = Math.max(uMin, Math.min(uMax, u_unsat));
    if (aw) {
      const gate = (u === u_unsat) || ((u === uMax) && e < 0) || ((u === uMin) && e > 0);
      if (gate) integ += e * dt;
    } else {
      integ += e * dt;
    }
    return { u, e };
  };
}

// --- Metrics ---
function computeStepMetrics(data, setpoint = 1.0) {
  if (!data.length) return {};
  const y = data.map((d) => d.y);
  const t = data.map((d) => d.t);
  const yFinal = y[y.length - 1];
  const yMax = Math.max(...y);
  const overshoot = ((yMax - setpoint) / Math.max(1e-9, setpoint)) * 100;
  const y10 = 0.1 * setpoint, y90 = 0.9 * setpoint;
  let t10 = null, t90 = null;
  for (let i = 1; i < y.length; i++) {
    if (t10 === null && y[i - 1] < y10 && y[i] >= y10) t10 = t[i];
    if (t90 === null && y[i - 1] < y90 && y[i] >= y90) { t90 = t[i]; break; }
  }
  const riseTime = t10 !== null && t90 !== null ? t90 - t10 : null;
  const tol = 0.02 * Math.max(1, Math.abs(setpoint));
  let settlingTime = null;
  for (let i = 0; i < y.length; i++) {
    if (y.slice(i).every((yy) => Math.abs(yy - setpoint) <= tol)) { settlingTime = t[i]; break; }
  }
  return { overshoot: Number.isFinite(overshoot) ? overshoot : null, riseTime, settlingTime, yFinal };
}

// --- Main Component ---
export default function PIDPlayground() {
  const [lang, setLang] = useState('en');
  const t = (k) => STR[lang][k];

  const [plantType, setPlantType] = useState("second");
  const [plantParams, setPlantParams] = useState({ tau: 1, m: 1, c: 0.8, k: 1, J: 0.01, b: 0.1, Kt: 0.02, Ke: 0.02, R: 1.0, L: 0.5, TL: 0.0 });

  const [Kp, setKp] = useState(2.0);
  const [Ki, setKi] = useState(0.5);
  const [Kd, setKd] = useState(0.2);
  const [N, setN] = useState(50);
  const [aw, setAw] = useState(true);
  const [uMin, setUMin] = useState(-5);
  const [uMax, setUMax] = useState(5);

  const [dt, setDt] = useState(0.01);
  const [T, setT] = useState(10);
  const [setpoint, setSetpoint] = useState(1);
  const [noiseStd, setNoiseStd] = useState(0);

  const [inputType, setInputType] = useState("step");
  const [sineFreq, setSineFreq] = useState(0.5);

  const [data, setData] = useState([]);
  const [extra, setExtra] = useState([]);

  // Visualization & recording
  const canvasRef = useRef(null);
  const [playing, setPlaying] = useState(false);
  const [downloadURL, setDownloadURL] = useState("");
  const mediaRecorderRef = useRef(null);
  const recordedChunksRef = useRef([]);

  // Recording capability & MIME selection (iOS/Safari friendly)
  const [canRecord, setCanRecord] = useState(false);
  const [recMime, setRecMime] = useState('');
  useEffect(() => {
    try {
      if (!('MediaRecorder' in window)) { setCanRecord(false); return; }
      const prefs = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm', 'video/mp4'];
      const pick = prefs.find(m => window.MediaRecorder.isTypeSupported && MediaRecorder.isTypeSupported(m));
      if (pick) { setRecMime(pick); setCanRecord(true); } else { setCanRecord(false); }
    } catch (e) { setCanRecord(false); }
  }, []);

  // keep canvas pixel size in sync with CSS width for crisp rendering and layout fit
  useEffect(() => {
    const c = canvasRef.current;
    if (!c) return;
    const resize = () => {
      const rect = c.getBoundingClientRect();
      c.width = Math.max(600, Math.floor(rect.width));
      c.height = 320;
    };
    resize();
    window.addEventListener('resize', resize);
    return () => window.removeEventListener('resize', resize);
  }, []);

  const runSim = () => {
    const plant = makePlant(plantType, plantParams);
    const pid = pidControllerFactory({ Kp, Ki, Kd, dt, N, aw, uMin, uMax });
    let x = plant.x0.slice();
    let tNow = 0;
    const steps = Math.floor(T / dt);
    const out = [], more = [];
    for (let i = 0; i <= steps; i++) {
      const yTrue = plant.y(x);
      const meas = yTrue + (noiseStd > 0 ? randn() * noiseStd : 0);
      const r = inputType === "step" ? setpoint : inputType === "sine" ? setpoint * Math.sin(2 * Math.PI * sineFreq * tNow) : setpoint * (Math.sign(Math.sin(2 * Math.PI * sineFreq * tNow)));
      const { u, e } = pid(r, meas);
      x = rk4Step(plant.f, x, u, dt);
      out.push({ t: tNow, y: yTrue, r });
      more.push({ t: tNow, u, e });
      tNow += dt;
    }
    setData(out);
    setExtra(more);
  };

  useEffect(() => { runSim(); }, [plantType, plantParams, Kp, Ki, Kd, N, aw, uMin, uMax, dt, T, setpoint, noiseStd, inputType, sineFreq]);

  const metrics = useMemo(() => computeStepMetrics(data, inputType === "step" ? setpoint : 1), [data, setpoint, inputType]);

  const presetZN = () => {
    if (plantType === "second") { setKp(3.2); setKi(2.4); setKd(0.4); setN(60); }
    if (plantType === "first") { setKp(1.2); setKi(1.0); setKd(0.0); setN(50); }
    if (plantType === "dcMotor") { setKp(6.0); setKi(3.0); setKd(0.2); setN(60); }
  };

  // --- Animation ---
  function drawFrame(ctx, w, h, tIndex) {
    ctx.clearRect(0, 0, w, h);
    ctx.save();

    const point = data[Math.min(tIndex, data.length-1)] || { y:0, r:0 };
    const y = point.y, r = point.r ?? setpoint;

    if (plantType === 'dcMotor') {
      // Speed gauge (ω) with reference needle
      const cx = w/2, cy = h/2 + 40, rad = Math.min(w,h)*0.28;
      // dial
      ctx.fillStyle = "#e2e8f0"; ctx.beginPath(); ctx.arc(cx, cy, rad, Math.PI, 2*Math.PI); ctx.fill();
      ctx.strokeStyle = "#1f2937"; ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(cx, cy, rad, Math.PI, 2*Math.PI); ctx.stroke();
      // map speed (y,r) to dial angle [-π..0]
      const map = (val) => {
        const clamped = Math.max(-10, Math.min(10, val));
        const frac = (clamped + 10) / 20; // 0..1
        return Math.PI - frac * Math.PI; // π..0
      };
      const ay = map(y);
      const ar = map(r);
      // reference needle (thin, red)
      ctx.strokeStyle = "#ef4444"; ctx.lineWidth = 2; ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + (rad-16)*Math.cos(ar), cy + (rad-16)*Math.sin(ar));
      ctx.stroke();
      // actual needle (thick, green)
      ctx.strokeStyle = "#10b981"; ctx.lineWidth = 5; ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + (rad-24)*Math.cos(ay), cy + (rad-24)*Math.sin(ay));
      ctx.stroke();
      // center hub
      ctx.fillStyle = "#1f2937"; ctx.beginPath(); ctx.arc(cx, cy, 6, 0, 2*Math.PI); ctx.fill();
      // labels
      ctx.fillStyle = "#334155"; ctx.font = "12px sans-serif";
      ctx.fillText("-10", cx - rad + 6, cy);
      ctx.fillText("0", cx - 6, cy - rad + 16);
      ctx.fillText("+10", cx + rad - 28, cy);
      ctx.fillText("ω (rad/s)", cx - 34, cy + 22);
    } else if (plantType === 'second') {
      // mass-spring visualization
      const left = 60, right = w-40, midY = h/2;
      const trackLen = right - left - 80;
      const x = y; // displacement
      const px = left + 40 + trackLen * (0.5 + 0.15 * Math.tanh(x));
      // wall
      ctx.fillStyle = "#94a3b8"; ctx.fillRect(left-10, midY-40, 10, 80);
      // spring
      ctx.strokeStyle = "#64748b"; ctx.lineWidth = 2; ctx.beginPath();
      const coils = 10; const startX = left; const endX = px-40; const amp = 12;
      for (let i=0;i<=coils;i++){
        const xx = startX + (i/coils)*(endX-startX);
        const yy = midY + (i%2===0? -amp: amp);
        if (i===0) ctx.moveTo(xx, midY); else ctx.lineTo(xx, yy);
      }
      ctx.lineTo(endX, midY); ctx.stroke();
      // mass
      ctx.fillStyle = "#1f2937"; ctx.fillRect(px-40, midY-30, 80, 60);
      // reference bar
      ctx.strokeStyle = "#ef4444"; ctx.setLineDash([6,4]); ctx.beginPath();
      const rx = left + 40 + trackLen * (0.5 + 0.15 * Math.tanh(r));
      ctx.moveTo(rx, midY-50); ctx.lineTo(rx, midY+50); ctx.stroke(); ctx.setLineDash([]);
    } else {
      // first-order: gauge bar
      const gx = 80, gy = h/2-15, gw=w-140, gh=30;
      ctx.fillStyle = "#e5e7eb"; ctx.fillRect(gx, gy, gw, gh);
      const frac = 0.5 + 0.25*Math.tanh(y);
      ctx.fillStyle = "#10b981"; ctx.fillRect(gx, gy, gw*frac, gh);
      // reference marker
      const rfrac = 0.5 + 0.25*Math.tanh(r);
      ctx.strokeStyle = "#ef4444"; ctx.lineWidth = 2; ctx.beginPath();
      ctx.moveTo(gx+gw*rfrac, gy-10); ctx.lineTo(gx+gw*rfrac, gy+gh+10); ctx.stroke();
    }

    ctx.restore();
  }

  function playAnimation(record = false) {
    const canvas = canvasRef.current; if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    let idx = 0;
    setPlaying(true);

    let rec;
    if (record) {
      recordedChunksRef.current = [];
      const stream = canvas.captureStream(60);
      rec = new MediaRecorder(stream, recMime ? { mimeType: recMime } : undefined);
      mediaRecorderRef.current = rec;
      rec.ondataavailable = (e) => { if (e.data && e.data.size) recordedChunksRef.current.push(e.data); };
      rec.onstop = () => {
        const blob = new Blob(recordedChunksRef.current, { type: 'video/webm' });
        const url = URL.createObjectURL(blob);
        setDownloadURL(url);
      };
      rec.start();
    }

    function loop() {
      if (idx >= data.length) {
        setPlaying(false);
        if (record && mediaRecorderRef.current && mediaRecorderRef.current.state !== 'inactive') mediaRecorderRef.current.stop();
        return;
      }
      drawFrame(ctx, w, h, idx);
      idx++;
      requestAnimationFrame(loop);
    }
    requestAnimationFrame(loop);
  }

  function stopPlayback() {
    setPlaying(false);
    // no-op; loop stops at end
  }

  function randn() {
    let u = 0, v = 0; while (u === 0) u = Math.random(); while (v === 0) v = Math.random();
    return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
  }

  const card = "rounded-2xl shadow-lg p-4 bg-white/70 backdrop-blur border border-gray-100";
  const label = "text-sm text-gray-600";
  const slider = "w-full";

  return (
    <div className="min-h-screen w-full bg-gradient-to-br from-slate-50 to-slate-200 text-gray-900 p-4 sm:p-6 md:p-8">
      <div className="max-w-7xl mx-auto grid grid-cols-1 lg:grid-cols-3 gap-6 items-start">
        <motion.div initial={{ opacity: 0, y: 8 }} animate={{ opacity: 1, y: 0 }} className="lg:col-span-2">
          <div className={`${card}`}>
            <div className="flex items-center justify-between gap-4 mb-2">
              <div>
                <h1 className="text-2xl font-bold">{t('title')}</h1>
                <div className="text-xs text-gray-500 mt-1">{t('madeBy')}</div>
              </div>
              <div className="flex gap-2">
                <button onClick={() => setLang(lang === 'en' ? 'ja' : 'en')} className="px-3 py-1.5 rounded-xl bg-slate-800 text-white text-sm">{t('langBtn')}</button>
                <button onClick={presetZN} className="px-3 py-1.5 rounded-xl bg-indigo-600 text-white text-sm">{t('presetZN')}</button>
              </div>
            </div>
            <p className="text-sm text-gray-600 mb-4">{t('subtitle')}</p>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              <div className={`${card}`}>
                <div className="flex items-center justify-between">
                  <h2 className="font-semibold">{plantType === 'dcMotor' ? t('responseTitleW') : t('responseTitleY')}</h2>
                  <span className="text-xs text-gray-500">dt = {dt.toFixed(3)} s, T = {T}s</span>
                </div>
                <ResponsiveContainer width="100%" height={320}>
                  <LineChart data={data} margin={{ top: 5, right: 20, bottom: 5, left: 0 }}>
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis dataKey="t" type="number" domain={[0, 'dataMax']} tickFormatter={(v) => v.toFixed(1)}>
                      <Label value={t('time')} position="insideBottomRight" offset={-5} />
                    </XAxis>
                    <YAxis tickFormatter={(v) => v.toFixed(1)}>
                      <Label angle={-90} position="insideLeft" style={{ textAnchor: 'middle' }} value={plantType === 'dcMotor' ? t('yAxisW') : t('yAxisY')} />
                    </YAxis>
                    <Tooltip formatter={(v) => (typeof v === 'number' ? v.toFixed(3) : v)} />
                    <Legend />
                    <Line type="monotone" dataKey="r" strokeWidth={2} dot={false} name={plantType === 'dcMotor' ? t('refW') : t('refR')} />
                    <Line type="monotone" dataKey="y" strokeWidth={2} dot={false} name={plantType === 'dcMotor' ? t('outW') : t('outY')} />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              <div className={`${card}`}>
                <h2 className="font-semibold">{t('controlError')}</h2>
                <ResponsiveContainer width="100%" height={320}>
                  <LineChart data={extra} margin={{ top: 5, right: 20, bottom: 5, left: 0 }}>
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis dataKey="t" type="number" domain={[0, 'dataMax']} tickFormatter={(v) => v.toFixed(1)}>
                      <Label value={t('time')} position="insideBottomRight" offset={-5} />
                    </XAxis>
                    <YAxis tickFormatter={(v) => v.toFixed(1)}>
                      <Label angle={-90} position="insideLeft" style={{ textAnchor: 'middle' }} value={plantType === 'dcMotor' ? t('yAxisUEW') : t('yAxisUE')} />
                    </YAxis>
                    <Tooltip formatter={(v) => (typeof v === 'number' ? v.toFixed(3) : v)} />
                    <Legend />
                    <Line type="monotone" dataKey="u" strokeWidth={2} dot={false} name="u" />
                    <Line type="monotone" dataKey="e" strokeWidth={1.5} dot={false} name="e" />
                  </LineChart>
                </ResponsiveContainer>
              </div>
            </div>

            <div className="mt-4 grid grid-cols-1 gap-4">
              <div className={`${card}`}>
                <div className="flex items-center justify-between mb-2">
                  <h2 className="font-semibold">{t('viz')}</h2>
                  <div className="flex gap-2">
                    {!playing && <button onClick={() => playAnimation(false)} className="px-3 py-1.5 rounded-xl bg-emerald-600 text-white text-sm">{t('play')}</button>}
                    {!playing && canRecord && <button onClick={() => playAnimation(true)} className="px-3 py-1.5 rounded-xl bg-rose-600 text-white text-sm">{t('record')}</button>}
                    {playing && <button onClick={stopPlayback} className="px-3 py-1.5 rounded-xl bg-slate-700 text-white text-sm">{t('stop')}</button>}
                    {downloadURL && <a className="px-3 py-1.5 rounded-xl bg-slate-900 text-white text-sm" href={downloadURL} download="pid-visualization.webm">{t('download')}</a>}
                  </div>
                </div>
                <canvas ref={canvasRef} className="w-full rounded-xl bg-white border border-slate-200" />
                <p className="text-xs text-gray-500 mt-2">{t('videoNote')}</p>
              </div>
            </div>
          </div>
        </motion.div>

        <motion.div initial={{ opacity: 0, y: 8 }} animate={{ opacity: 1, y: 0 }} className="space-y-4">
          <div className={`${card}`}>
            <h2 className="font-semibold mb-2">{t('plant')}</h2>
            <div className="flex gap-2 flex-wrap mb-3">
              <button onClick={() => setPlantType("first")} className={`px-3 py-1.5 rounded-xl text-sm ${plantType==='first' ? 'bg-slate-900 text-white' : 'bg-slate-100'}`}>{t('first')}</button>
              <button onClick={() => setPlantType("second")} className={`px-3 py-1.5 rounded-xl text-sm ${plantType==='second' ? 'bg-slate-900 text-white' : 'bg-slate-100'}`}>{t('second')}</button>
              <button onClick={() => setPlantType("dcMotor")} className={`px-3 py-1.5 rounded-xl text-sm ${plantType==='dcMotor' ? 'bg-slate-900 text-white' : 'bg-slate-100'}`}>{t('dcMotor')}</button>
            </div>

            {plantType === 'first' && (
              <div className="space-y-2">
                <label className={label}>{t('tau')}
                  <input type="range" min={0.05} max={5} step={0.05} value={plantParams.tau} onChange={(e)=>setPlantParams(p=>({...p, tau: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.tau.toFixed(2)}</div>
                </label>
              </div>
            )}

            {plantType === 'second' && (
              <div className="grid grid-cols-2 gap-2">
                <label className={label}>{t('m')}
                  <input type="range" min={0.2} max={5} step={0.1} value={plantParams.m} onChange={(e)=>setPlantParams(p=>({...p, m: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.m.toFixed(2)}</div>
                </label>
                <label className={label}>{t('c')}
                  <input type="range" min={0} max={5} step={0.05} value={plantParams.c} onChange={(e)=>setPlantParams(p=>({...p, c: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.c.toFixed(2)}</div>
                </label>
                <label className={label}>{t('k')}
                  <input type="range" min={0.1} max={10} step={0.1} value={plantParams.k} onChange={(e)=>setPlantParams(p=>({...p, k: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.k.toFixed(2)}</div>
                </label>
              </div>
            )}

            {plantType === 'dcMotor' && (
              <div className="grid grid-cols-2 gap-2">
                <label className={label}>{t('J')}
                  <input type="range" min={0.001} max={0.2} step={0.001} value={plantParams.J} onChange={(e)=>setPlantParams(p=>({...p, J: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.J.toFixed(3)}</div>
                </label>
                <label className={label}>{t('b')}
                  <input type="range" min={0} max={0.5} step={0.001} value={plantParams.b} onChange={(e)=>setPlantParams(p=>({...p, b: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.b.toFixed(3)}</div>
                </label>
                <label className={label}>{t('Kt')}
                  <input type="range" min={0.001} max={0.2} step={0.001} value={plantParams.Kt} onChange={(e)=>setPlantParams(p=>({...p, Kt: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.Kt.toFixed(3)}</div>
                </label>
                <label className={label}>{t('Ke')}
                  <input type="range" min={0.001} max={0.2} step={0.001} value={plantParams.Ke} onChange={(e)=>setPlantParams(p=>({...p, Ke: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.Ke.toFixed(3)}</div>
                </label>
                <label className={label}>{t('R')}
                  <input type="range" min={0.1} max={10} step={0.1} value={plantParams.R} onChange={(e)=>setPlantParams(p=>({...p, R: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.R.toFixed(1)}</div>
                </label>
                <label className={label}>{t('L')}
                  <input type="range" min={0.01} max={2} step={0.01} value={plantParams.L} onChange={(e)=>setPlantParams(p=>({...p, L: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.L.toFixed(2)}</div>
                </label>
                <label className={label}>{t('TL')}
                  <input type="range" min={-0.2} max={0.2} step={0.005} value={plantParams.TL} onChange={(e)=>setPlantParams(p=>({...p, TL: parseFloat(e.target.value)}))} className={slider} />
                  <div className="text-xs">{plantParams.TL.toFixed(3)} N·m</div>
                </label>
              </div>
            )}
          </div>

          <div className={`${card}`}>
            <h2 className="font-semibold mb-2">{t('gains')}</h2>
            <div className="space-y-3">
              <label className={label}>{t('kp')}
                <input type="range" min={0} max={20} step={0.05} value={Kp} onChange={(e)=>setKp(parseFloat(e.target.value))} className={slider} />
                <div className="text-xs">{Kp.toFixed(2)}</div>
              </label>
              <label className={label}>{t('ki')}
                <input type="range" min={0} max={10} step={0.02} value={Ki} onChange={(e)=>setKi(parseFloat(e.target.value))} className={slider} />
                <div className="text-xs">{Ki.toFixed(2)}</div>
              </label>
              <label className={label}>{t('kd')}
                <input type="range" min={0} max={10} step={0.02} value={Kd} onChange={(e)=>setKd(parseFloat(e.target.value))} className={slider} />
                <div className="text-xs">{Kd.toFixed(2)}</div>
              </label>
              <label className={label}>{t('n')}
                <input type="range" min={1} max={200} step={1} value={N} onChange={(e)=>setN(parseInt(e.target.value))} className={slider} />
                <div className="text-xs">{N}</div>
              </label>
              <div className="flex items-center gap-2 text-sm">
                <input id="aw" type="checkbox" checked={aw} onChange={(e)=>setAw(e.target.checked)} />
                <label htmlFor="aw">{t('aw')}</label>
              </div>
              <div className="grid grid-cols-2 gap-2">
                <label className={label}>{t('umin')}
                  <input type="range" min={-20} max={0} step={0.5} value={uMin} onChange={(e)=>setUMin(parseFloat(e.target.value))} className={slider} />
                  <div className="text-xs">{uMin.toFixed(1)}</div>
                </label>
                <label className={label}>{t('umax')}
                  <input type="range" min={0} max={20} step={0.5} value={uMax} onChange={(e)=>setUMax(parseFloat(e.target.value))} className={slider} />
                  <div className="text-xs">{uMax.toFixed(1)}</div>
                </label>
              </div>
            </div>
          </div>

          <div className={`${card}`}>
            <h2 className="font-semibold mb-2">{t('setup')}</h2>
            <div className="grid grid-cols-2 gap-2">
              <label className={label}>{t('setpointAmp')}
                <input type="range" min={-2} max={2} step={0.05} value={setpoint} onChange={(e)=>setSetpoint(parseFloat(e.target.value))} className={slider} />
                <div className="text-xs">{setpoint.toFixed(2)}</div>
              </label>
              <label className={label}>{t('noiseStd')}
                <input type="range" min={0} max={0.5} step={0.005} value={noiseStd} onChange={(e)=>setNoiseStd(parseFloat(e.target.value))} className={slider} />
                <div className="text-xs">{noiseStd.toFixed(3)}</div>
              </label>
              <label className={label}>{t('dt')}
                <input type="range" min={0.001} max={0.05} step={0.001} value={dt} onChange={(e)=>setDt(parseFloat(e.target.value))} className={slider} />
                <div className="text-xs">{dt.toFixed(3)}</div>
              </label>
              <label className={label}>{t('T')}
                <input type="range" min={2} max={40} step={1} value={T} onChange={(e)=>setT(parseFloat(e.target.value))} className={slider} />
                <div className="text-xs">{T.toFixed(0)}</div>
              </label>
            </div>
          </div>

          <div className={`${card}`}>
            <h2 className="font-semibold mb-2">{t('tips')}</h2>
            <ul className="list-disc pl-5 text-sm text-gray-700 space-y-1">
              <li>{t('tip1')}</li>
              <li>{t('tip2')}</li>
              <li>{t('tip3')}</li>
            </ul>
          </div>
        </motion.div>
      </div>
    </div>
  );
}
