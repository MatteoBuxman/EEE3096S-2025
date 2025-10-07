// generate_luts.js
// Run with: node generate_luts.js
// Generates three files: Sine.c, Saw.c, Triangle.c

const fs = require("fs");

const NS = 128;          // number of samples per cycle
const MAX = 4095;        // 12-bit resolution max value
const HALF = MAX / 2;    // center point for sine

function generateSine() {
  let arr = [];
  for (let i = 0; i < NS; i++) {
    let angle = (2 * Math.PI * i) / NS;
    let val = Math.round(HALF * (1 + Math.sin(angle))); // scale 0–4095
    arr.push(val);
  }
  return arr;
}

function generateSaw() {
  let arr = [];
  for (let i = 0; i < NS; i++) {
    let val = Math.round((MAX * i) / (NS - 1)); // linear ramp
    arr.push(val);
  }
  return arr;
}

function generateTriangle() {
  let arr = [];
  for (let i = 0; i < NS; i++) {
    let phase = (i / NS) * 2; // 0–2
    let val = phase < 1
      ? Math.round(MAX * phase)            // rising slope
      : Math.round(MAX * (2 - phase));     // falling slope
    arr.push(val);
  }
  return arr;
}

function formatCArray(name, arr) {
  let lines = [];
  lines.push(`#include <stdint.h>`);
  lines.push(`const uint16_t ${name}[${NS}] = {`);
  for (let i = 0; i < arr.length; i++) {
    let entry = arr[i] + (i < arr.length - 1 ? "," : "");
    if (i % 16 === 0) lines.push("    " + entry);
    else lines[lines.length - 1] += " " + entry;
  }
  lines.push("};\n");
  return lines.join("\n");
}

function writeFile(filename, name, arr) {
  const content = formatCArray(name, arr);
  fs.writeFileSync(filename, content, "utf8");
  console.log(`✅ Wrote ${filename}`);
}

// Generate LUTs
writeFile("Sine.c", "Sine_LUT", generateSine());
writeFile("Saw.c", "Saw_LUT", generateSaw());
writeFile("Triangle.c", "Triangle_LUT", generateTriangle());
