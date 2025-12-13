# Unitree B2 - 3D Viewer

A modern 3D visualization application for the Unitree B2 robot using Three.js and URDF.

## Features

- 🎨 Modern Three.js implementation
- 🖱️ Interactive camera controls (orbit, zoom, pan)
- 💡 Realistic lighting with shadows
- 🤖 URDF robot model loading with joint control
- 🚶 Realistic trot gait walking animation
- ⚡ Fast development with Vite

## Getting Started

### Installation

```bash
npm install
```

### Development

```bash
npm run dev
```

The application will open automatically in your browser at `http://localhost:3000`

### Build

```bash
npm run build
```

### Preview Production Build

```bash
npm run preview
```

## Controls

- **Left Click + Drag**: Rotate camera around the model
- **Right Click + Drag**: Pan the camera
- **Scroll Wheel**: Zoom in/out

## Setup

1. Place your Unitree B2 URDF file in `Assets/b2_description/urdf/b2_description.urdf`
2. Ensure all mesh files referenced in the URDF are accessible (relative paths from URDF location)
3. Install dependencies: `npm install`
4. Run: `npm run dev`

## Technologies

- **Three.js**: 3D graphics library
- **urdf-loader**: URDF file loading and joint control
- **Vite**: Modern build tool and dev server
- **ES6 Modules**: Modern JavaScript

## URDF File Structure

The application expects a URDF file with joints named using Unitree conventions:
- `FL_*`, `FR_*`, `HL_*`, `HR_*` for front-left, front-right, hind-left, hind-right
- Joints like `hip`, `hipy`, `hipx`, `thigh`, `calf` for each leg

If your URDF uses different naming, check the console output and adjust the joint mapping in `src/main.js`.

