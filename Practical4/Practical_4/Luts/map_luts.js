// Node.js script
// Maps all LUTs in .inc files from 0–4095 range to CCR values for a given ARR.
// Overwrites the original files, keeping the same array names.
//
// Usage: node map_all_luts_to_ccr.js
// Example: node map_all_luts_to_ccr.js 399

const fs = require("fs");
const path = require("path");

const ARR = Number(process.argv[2]) || 399; // Default to 399 if not provided

console.log(`Mapping all .inc LUTs to CCR values (ARR = ${ARR})\n`);

const files = fs.readdirSync(".").filter(f => f.endsWith(".inc"));

if (files.length === 0) {
  console.log("No .inc files found in current directory.");
  process.exit(0);
}

for (const file of files) {
  console.log(`Processing: ${file}`);

  try {
    const text = fs.readFileSync(file, "utf8");

    // Extract all integers
    const matches = text.match(/\b\d+\b/g);
    if (!matches) {
      console.warn(`  ⚠️  No integers found in ${file}, skipping.`);
      continue;
    }

    const samples = matches.map(Number);

    // Map to CCR values
    const ccrValues = samples.map(v => Math.round((v / 4095) * ARR));

    // Try to keep the array name if it exists
    const nameMatch = text.match(/const\s+\w+\s+(\w+)\s*\[/);
    const arrayName = nameMatch ? nameMatch[1] : path.basename(file, ".inc");

    // Reconstruct the file
    let output = `#include <stdint.h>\n\n`;
    output += `// Auto-generated CCR LUT for ARR=${ARR}\n`;
    output += `const uint16_t ${arrayName}[${ccrValues.length}] = {\n`;

    for (let i = 0; i < ccrValues.length; i++) {
      output += ccrValues[i].toString().padStart(4, " ");
      output += (i < ccrValues.length - 1 ? "," : "");
      if ((i + 1) % 16 === 0) output += "\n";
      else output += " ";
    }

    output += `\n};\n`;

    // Overwrite the file
    fs.writeFileSync(file, output, "utf8");

    console.log(`  ✅ Updated ${file} (${ccrValues.length} samples)`);

  } catch (err) {
    console.error(`  ❌ Failed to process ${file}: ${err.message}`);
  }
}

console.log("\nAll .inc LUTs mapped successfully!");
