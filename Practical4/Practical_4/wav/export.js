const fs = require("fs");
const wav = require("wav-decoder");

// Extract evenly spaced samples
async function extractSamples(filename, targetCount) {
  const buffer = fs.readFileSync(filename);
  const decoded = await wav.decode(buffer);

  const samples = decoded.channelData[0]; // Float32Array [-1, 1]
  const total = samples.length;

  if (targetCount >= total) return samples;

  const step = total / targetCount;
  const result = new Float32Array(targetCount);

  for (let i = 0; i < targetCount; i++) {
    const index = Math.floor(i * step);
    result[i] = samples[index];
  }
  return result;
}

// Write to .inc file as int16_t array
async function writeSamplesToInc(inputWav, outputInc, arrayName, targetCount) {
  const samples = await extractSamples(inputWav, targetCount);

  let cCode = `const int16_t ${arrayName}[${samples.length}] = {\n  `;

  for (let i = 0; i < samples.length; i++) {
    // Scale float [-1,1] → int16_t [-32768,32767]
    const val = Math.max(-32768, Math.min(32767, Math.round(samples[i] * 32767)));
    cCode += val;

    if (i < samples.length - 1) cCode += ", ";
    if ((i + 1) % 10 === 0) cCode += "\n  ";
  }

  cCode += "\n};\n";

  fs.writeFileSync(outputInc, cCode);
  console.log(`✅ Wrote ${samples.length} samples to ${outputInc} as int16_t array "${arrayName}"`);
}

// --- CLI Entry Point ---
(async () => {
  const [,, inputWav, outputInc, arrayName, sampleCountStr] = process.argv;

  if (!inputWav || !outputInc || !arrayName || !sampleCountStr) {
    console.error("Usage: node export.js <input.wav> <output.inc> <arrayName> <numSamples>");
    process.exit(1);
  }

  const targetCount = parseInt(sampleCountStr, 10);
  if (isNaN(targetCount) || targetCount <= 0) {
    console.error("❌ numSamples must be a positive integer");
    process.exit(1);
  }

  await writeSamplesToInc(inputWav, outputInc, arrayName, targetCount);
})();
