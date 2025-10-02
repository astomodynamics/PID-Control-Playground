### Built with GPT-5 Thinking (ChatGPT)

<video controls width="720" poster="./demo.png">
  <source src="./demo.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

# PID Control Playground — Usage Guide

Interactive web app to build intuition for PID control on simple plants.

## Run locally

```bash
npm install
npm run dev
```

Open the printed local URL (usually http://localhost:5173).

## UI at a glance
- Left: Output vs Reference, and Control/Error plots (responsive)
- Visualization & Recording: animated plant view, optional WebM recording (hidden if unsupported)
- Right: Plant selection & parameters, PID gains, and experiment setup

## Plants
- 1st order, Mass–spring–damper, DC motor (speed). DC motor supports constant load torque Tₗ.

## Inputs
- Step, Sine, Square. Set amplitude, noise std, dt, and horizon T.

## Controller
- PID with derivative-on-measurement and first-order filter N, conditional anti-windup, and actuator saturation.

## iOS note
- Recording support depends on browser codecs. The app detects support; the Record button appears only when safe.

Enjoy!
