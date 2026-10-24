import fs from "node:fs/promises";
import path from "node:path";
import { spawnSync } from "node:child_process";

import {
  ensureArtifactToolWorkspace,
  importArtifactTool,
  padSlideNumber,
  readImageBlob,
  saveBlobToFile,
} from "/Users/sota/.codex/plugins/cache/openai-primary-runtime/presentations/26.601.10930/skills/presentations/scripts/artifact_tool_utils.mjs";

const WORKSPACE =
  "/Users/sota/projects/FomulaTMU/sim_overall/outputs/019e78b3-3998-7cd3-8588-eb5bab0bc90f/presentations/ttc-midterm-azusa3";
const STARTER_PPTX = path.join(WORKSPACE, "template-starter.pptx");
const OUTPUT_PPTX = path.join(WORKSPACE, "output", "ttc_midterm_report_azusa3_en.pptx");
const FINAL_PPTX =
  "/Users/sota/projects/FomulaTMU/sim_overall/データ解析特論/ttc_midterm_report_azusa3_en.pptx";
const PREVIEW_DIR = path.join(WORKSPACE, "preview_en");
const LAYOUT_DIR = path.join(WORKSPACE, "layout_en");
const CONTACT_SHEET = path.join(PREVIEW_DIR, "contact-sheet.png");
const CONTACT_SCRIPT =
  "/Users/sota/.codex/plugins/cache/openai-primary-runtime/presentations/26.601.10930/skills/presentations/scripts/make_contact_sheet.py";
const ASSET_DIR = path.join(WORKSPACE, "assets");

const COLORS = {
  bg: "#191A1C",
  panel: "#232528",
  panelAlt: "#2A2D31",
  border: "#5D6CEC",
  text: "#FCFDF2",
  muted: "#C8C7BA",
  green: "#5AE08E",
  blue: "#7ABBFF",
  pink: "#FC68C5",
};

function solid(color) {
  return { type: "solid", color };
}

function noneFill() {
  return { type: "none" };
}

function line(color, width = 1) {
  return { style: "solid", width, fill: solid(color) };
}

function noLine() {
  return { style: "solid", width: 0, fill: noneFill() };
}

function setFrame(shape, left, top, width, height) {
  shape.frame = { left, top, width, height };
}

function baseShapes(slide) {
  const [title, subtitle, footer, connector, note, button, clickLabel] = slide.shapes.items;
  return { title, subtitle, footer, connector, note, button, clickLabel };
}

function styleText(shape, options) {
  const {
    text,
    left,
    top,
    width,
    height,
    fontSize,
    typeface,
    color = COLORS.text,
    bold = false,
    align = "left",
    valign = "top",
  } = options;
  setFrame(shape, left, top, width, height);
  shape.text = text;
  shape.fill = noneFill();
  shape.line = noLine();
  shape.text.fontSize = fontSize;
  shape.text.typeface = typeface;
  shape.text.color = color;
  shape.text.bold = bold;
  shape.text.alignment = align;
  shape.text.verticalAlignment = valign;
  shape.text.insets = { left: 4, right: 4, top: 2, bottom: 2 };
}

function addRect(slide, { left, top, width, height, fill = COLORS.panel, lineColor = COLORS.border, lineWidth = 1 }) {
  return slide.shapes.add({
    geometry: "rect",
    position: { left, top, width, height },
    fill: solid(fill),
    line: lineColor ? line(lineColor, lineWidth) : noLine(),
  });
}

function addText(slide, options) {
  const shape = slide.shapes.add({
    geometry: "rect",
    position: {
      left: options.left,
      top: options.top,
      width: options.width,
      height: options.height,
    },
    fill: options.fill ? solid(options.fill) : noneFill(),
    line: options.border ? line(options.border, options.borderWidth ?? 1) : noLine(),
  });
  shape.text = options.text ?? "";
  shape.text.fontSize = options.fontSize ?? 14;
  shape.text.typeface = options.typeface ?? "Aptos";
  shape.text.color = options.color ?? COLORS.text;
  shape.text.bold = Boolean(options.bold);
  shape.text.alignment = options.align ?? "left";
  shape.text.verticalAlignment = options.valign ?? "top";
  shape.text.insets = options.insets ?? { left: 8, right: 8, top: 6, bottom: 6 };
  return shape;
}

async function addImage(slide, imageName, frame) {
  const blob = await readImageBlob(path.join(ASSET_DIR, imageName));
  const image = slide.images.add({ blob, fit: "contain", alt: imageName });
  image.position = frame;
  return image;
}

function addAccentLine(slide, left, top, width) {
  slide.shapes.add({
    geometry: "rect",
    position: { left, top, width, height: 4 },
    fill: solid(COLORS.green),
    line: noLine(),
  });
}

function addPageTag(slide, pageText) {
  addText(slide, {
    left: 798,
    top: 38,
    width: 120,
    height: 24,
    text: pageText,
    fontSize: 10,
    typeface: "Montserrat",
    color: COLORS.muted,
    align: "right",
    valign: "middle",
  });
}

function titleLineBreak(text) {
  return text.replace(": ", ":\n");
}

function cleanupTemplateHelpers(slide) {
  const { connector, note, button, clickLabel } = baseShapes(slide);
  connector.delete();
  note.delete();
  button.delete();
  clickLabel.delete();
}

function setupContentSlide(slide, pageText, titleText, subtitleText, footerText) {
  const { title, subtitle, footer } = baseShapes(slide);
  cleanupTemplateHelpers(slide);
  slide.background.fill = COLORS.bg;
  styleText(title, {
    text: titleText,
    left: 48,
    top: 34,
    width: 740,
    height: 64,
    fontSize: 28,
    typeface: "Montserrat ExtraBold",
    color: COLORS.text,
    bold: true,
    valign: "bottom",
  });
  styleText(subtitle, {
    text: subtitleText,
    left: 48,
    top: 104,
    width: 760,
    height: 56,
    fontSize: 13,
    typeface: "Aptos",
    color: COLORS.muted,
    valign: "top",
  });
  styleText(footer, {
    text: footerText,
    left: 48,
    top: 506,
    width: 780,
    height: 18,
    fontSize: 8,
    typeface: "Work Sans",
    color: COLORS.muted,
    valign: "middle",
  });
  addPageTag(slide, pageText);
  addAccentLine(slide, 48, 170, 140);
}

function buildCover(slide) {
  const { title, subtitle, footer } = baseShapes(slide);
  cleanupTemplateHelpers(slide);
  slide.background.fill = COLORS.bg;
  styleText(title, {
    text: "Multiple Regression Analysis of Longitudinal Tire Force\nUsing TTC Tire Data",
    left: 48,
    top: 64,
    width: 820,
    height: 170,
    fontSize: 34,
    typeface: "Montserrat ExtraBold",
    color: COLORS.text,
    bold: true,
    valign: "bottom",
  });
  styleText(subtitle, {
    text: "Fx main analysis with Fy comparison for Hoosier_18p0X6p0-10_R20_Rim6",
    left: 48,
    top: 246,
    width: 820,
    height: 50,
    fontSize: 18,
    typeface: "Aptos",
    color: COLORS.muted,
  });
  styleText(footer, {
    text: "Data Analysis Special Lecture  /  Formula TMU  /  sota  /  2026-06-02",
    left: 48,
    top: 500,
    width: 820,
    height: 18,
    fontSize: 9,
    typeface: "Work Sans",
    color: COLORS.muted,
  });
  addText(slide, {
    left: 48,
    top: 336,
    width: 360,
    height: 88,
    text: "Main target: Fx\nComparison target: Fy\nOutliers removed by Mahalanobis distance",
    fontSize: 15,
    typeface: "Aptos",
    color: COLORS.text,
    fill: COLORS.panel,
    border: COLORS.border,
  });
  addText(slide, {
    left: 430,
    top: 336,
    width: 278,
    height: 88,
    text: "Template: Azusa 3\nTone: English, research presentation\nExtension: PINN in the second half",
    fontSize: 14,
    typeface: "Aptos",
    color: COLORS.text,
    fill: COLORS.panel,
    border: COLORS.border,
  });
  addPageTag(slide, "01 / 11");
}

function buildContext(slide) {
  setupContentSlide(
    slide,
    "02 / 11",
    "Can longitudinal tire force Fx be explained by a multivariate linear model?",
    "Using TTC measurements, this report tests whether slip, load, camber, and temperature explain the main trends relevant to Formula SAE traction and braking.",
    "Goal: build an interpretable Fx baseline, compare it with Fy, and clarify where a PINN becomes necessary."
  );
  addRect(slide, { left: 48, top: 196, width: 412, height: 250, fill: COLORS.panel, lineColor: COLORS.border });
  addRect(slide, { left: 500, top: 196, width: 412, height: 250, fill: COLORS.panelAlt, lineColor: COLORS.border });
  addText(slide, {
    left: 64,
    top: 210,
    width: 380,
    height: 220,
    text:
      "Background\n\n" +
      "- In Formula SAE, tire forces directly affect lap time.\n" +
      "- Fx directly governs acceleration, braking, and traction.\n" +
      "- TTC data provide measured forces together with load, slip, temperature, and camber.",
    fontSize: 15,
    typeface: "Aptos",
    fill: COLORS.panel,
    border: null,
  });
  addText(slide, {
    left: 516,
    top: 210,
    width: 380,
    height: 220,
    text:
      "Questions and hypothesis\n\n" +
      "1. How much of Fx can be explained by SR, Fz, IA, and TSTC?\n" +
      "2. Compared with Fy, which factors are shared and which are dominant?\n" +
      "3. Where does a linear baseline stop being sufficient?\n\n" +
      "Hypothesis: Fx is governed mainly by SR and Fz, while IA and TSTC act as secondary factors.",
    fontSize: 14,
    typeface: "Aptos",
    fill: COLORS.panelAlt,
    border: null,
  });
}

function buildData(slide) {
  setupContentSlide(
    slide,
    "03 / 11",
    "The dataset is TTC Hoosier 18x6.0-10 R20 Rim6, which includes both drive/brake and cornering tests.",
    "The same tire can therefore be used for both Fx and Fy, and the variables connect directly to the existing tire_pinn pipeline.",
    "Dataset: Hoosier_18p0X6p0-10_R20_Rim6  |  Source: Tire Testing Consortium (www.fsaettc.org)"
  );
  addRect(slide, { left: 48, top: 196, width: 200, height: 116, fill: COLORS.panel, lineColor: COLORS.border });
  addRect(slide, { left: 262, top: 196, width: 200, height: 116, fill: COLORS.panel, lineColor: COLORS.border });
  addRect(slide, { left: 476, top: 196, width: 200, height: 116, fill: COLORS.panel, lineColor: COLORS.border });
  addText(slide, {
    left: 62, top: 206, width: 172, height: 94,
    text: "Total samples\n\n329600",
    fontSize: 20, typeface: "Montserrat", bold: true, fill: COLORS.panel, border: null, valign: "middle"
  });
  addText(slide, {
    left: 276, top: 206, width: 172, height: 94,
    text: "DriveBrake\n\n162647",
    fontSize: 20, typeface: "Montserrat", bold: true, fill: COLORS.panel, border: null, valign: "middle"
  });
  addText(slide, {
    left: 490, top: 206, width: 172, height: 94,
    text: "Cornering\n\n166953",
    fontSize: 20, typeface: "Montserrat", bold: true, fill: COLORS.panel, border: null, valign: "middle"
  });
  addText(slide, {
    left: 700,
    top: 198,
    width: 212,
    height: 114,
    text: "Why this tire?\n\n- Same-tire comparison for Fx and Fy\n- Both DriveBrake and Cornering are available\n- Directly compatible with the existing PINN workflow",
    fontSize: 13,
    typeface: "Aptos",
    fill: COLORS.panelAlt,
    border: COLORS.border,
  });
  addText(slide, {
    left: 58,
    top: 338,
    width: 848,
    height: 96,
    text:
      "Inputs      SR   /   SA   /   Fz   /   IA   /   TSTC\n\n" +
      "Outputs     Fx   /   Fy   /   Mx   /   Mz\n\n" +
      "Fx: longitudinal force   |   Fy: lateral force",
    fontSize: 18,
    typeface: "Montserrat",
    align: "center",
    valign: "middle",
    fill: COLORS.panel,
    border: COLORS.border,
  });
}

function buildMethod(slide) {
  setupContentSlide(
    slide,
    "04 / 11",
    "The main analysis extracts near-pure longitudinal DriveBrake data and explains Fx with four variables.",
    "The same OLS pipeline is also applied to Fy so that the dominant factors for longitudinal and lateral force can be compared.",
    "Pure longitudinal and pure cornering conditions are cut out separately and evaluated with the same regression workflow."
  );
  addRect(slide, { left: 48, top: 196, width: 412, height: 250, fill: COLORS.panel, lineColor: COLORS.border });
  addRect(slide, { left: 500, top: 196, width: 412, height: 250, fill: COLORS.panelAlt, lineColor: COLORS.border });
  addText(slide, {
    left: 64, top: 210, width: 380, height: 220,
    text:
      "Main analysis: Fx\n\n" +
      "Objective variable: Fx (longitudinal force)\n" +
      "Explanatory variables: SR, Fz, IA, TSTC\n\n" +
      "Filters\n- DriveBrake data only\n- |SA| <= 1.0 deg\n- Fz >= 300 N\n- 35 <= TSTC <= 85 C\n\n" +
      "Samples after filtering: 41059",
    fontSize: 14, typeface: "Aptos", fill: COLORS.panel, border: null
  });
  addText(slide, {
    left: 516, top: 210, width: 380, height: 220,
    text:
      "Comparison analysis: Fy\n\n" +
      "Objective variable: Fy (lateral force)\n" +
      "Explanatory variables: SA, Fz, IA, TSTC\n\n" +
      "Filters\n- Cornering data only\n- |SR| <= 0.02\n- Fz >= 300 N\n- 35 <= TSTC <= 85 C\n\n" +
      "Samples after filtering: 17153",
    fontSize: 14, typeface: "Aptos", fill: COLORS.panelAlt, border: null
  });
}

function buildOutlier(slide) {
  setupContentSlide(
    slide,
    "05 / 11",
    "No strong univariate outliers appeared; outliers emerged mainly as multivariate combinations.",
    "Final regression uses samples after removing points above the 95% Mahalanobis threshold in explanatory-variable space.",
    "Outlier rule: remove Mahalanobis distance > chi-square(0.95, df=4)"
  );
  addText(slide, {
    left: 48, top: 200, width: 864, height: 46,
    text: "Univariate outliers: 0 cases for both Fx and Fy under the 1.96 SD and 1.5 IQR checks",
    fontSize: 15, typeface: "Aptos", fill: COLORS.panel, border: COLORS.border, valign: "middle"
  });
  addRect(slide, { left: 48, top: 264, width: 420, height: 170, fill: COLORS.panel, lineColor: COLORS.border });
  addRect(slide, { left: 492, top: 264, width: 420, height: 170, fill: COLORS.panelAlt, lineColor: COLORS.border });
  addText(slide, {
    left: 62, top: 278, width: 392, height: 146,
    text:
      "Fx main analysis\n\n" +
      "- Samples after filtering: 41059\n" +
      "- Mahalanobis outliers: 1269\n" +
      "- Samples after removal: 39790\n\n" +
      "Takeaway: the dominant anomalies were multivariate, not univariate.",
    fontSize: 14, typeface: "Aptos", fill: COLORS.panel, border: null
  });
  addText(slide, {
    left: 506, top: 278, width: 392, height: 146,
    text:
      "Fy comparison analysis\n\n" +
      "- Samples after filtering: 17153\n" +
      "- Mahalanobis outliers: 1214\n" +
      "- Samples after removal: 15939\n\n" +
      "Following the lecture requirement, the final regression uses the cleaned dataset.",
    fontSize: 14, typeface: "Aptos", fill: COLORS.panelAlt, border: null
  });
}

function buildFxCoefficients(slide) {
  setupContentSlide(
    slide,
    "06 / 11",
    "In the Fx model, collinearity is weak and SR is the dominant explanatory variable.",
    "All VIF values stay near 1, so coefficient instability from multicollinearity is limited under these filtered conditions.",
    "Model: Fx = a0 + a1*SR + a2*Fz + a3*IA + a4*TSTC"
  );
  addRect(slide, { left: 48, top: 196, width: 420, height: 250, fill: COLORS.panel, lineColor: COLORS.border });
  addRect(slide, { left: 492, top: 196, width: 420, height: 250, fill: COLORS.panelAlt, lineColor: COLORS.border });
  addText(slide, {
    left: 62, top: 208, width: 392, height: 224,
    text:
      "Coef., standardized beta, t-value, p-value\n\n" +
      "Variable    Coef.      Beta     t        p\n" +
      "Intercept  -1097.62      -     -21.52   <1e-100\n" +
      "SR         16393.42   0.9494   537.54   <1e-300\n" +
      "Fz             0.1937 0.0212    12.26   1.78e-34\n" +
      "IA            21.39   0.0195    11.09   1.50e-28\n" +
      "TSTC          21.76   0.0507    28.27   4.25e-174\n\n" +
      "R^2 = 0.8830\nAdjusted R^2 = 0.8830",
    fontSize: 12.5, typeface: "Source Code Pro", fill: COLORS.panel, border: null
  });
  addText(slide, {
    left: 506, top: 208, width: 392, height: 224,
    text:
      "VIF\n\n" +
      "SR    1.06\n" +
      "Fz    1.01\n" +
      "IA    1.05\n" +
      "TSTC  1.09\n\n" +
      "Interpretation\n\n" +
      "- SR is the primary driver of Fx.\n" +
      "- Fz, IA, and TSTC are also statistically significant.\n" +
      "- Severe multicollinearity is not observed.",
    fontSize: 14, typeface: "Aptos", fill: COLORS.panelAlt, border: null
  });
}

async function buildFxFit(slide) {
  setupContentSlide(
    slide,
    "07 / 11",
    "The linear Fx model reaches R^2 = 0.883 and works as a strong baseline.",
    "Observed-vs-predicted and residual views show high explanatory power, while also revealing the limit of a purely linear approximation.",
    "Panels: observed vs predicted and residuals after outlier removal"
  );
  await addImage(slide, "fx_regression_panels.png", { left: 48, top: 198, width: 560, height: 250 });
  addText(slide, {
    left: 634, top: 198, width: 278, height: 250,
    text:
      "Key reading\n\n" +
      "- Points gather close to the diagonal, so the linear baseline is strong.\n" +
      "- Residuals stay near zero on average but widen in saturation regions.\n" +
      "- Therefore, linear regression is a valid first step, not the final model.",
    fontSize: 14, typeface: "Aptos", fill: COLORS.panel, border: COLORS.border
  });
}

async function buildComparison(slide) {
  setupContentSlide(
    slide,
    "08 / 11",
    "Comparing Fy shows that load, temperature, and camber matter for both longitudinal and lateral force.",
    "Fx is driven mainly by SR and Fy by SA, but Fz, IA, and TSTC remain shared explanatory factors in both models.",
    "Comparison: standardized coefficients and Fy regression summary"
  );
  await addImage(slide, "coefficient_comparison.png", { left: 48, top: 198, width: 520, height: 248 });
  addText(slide, {
    left: 590, top: 198, width: 322, height: 248,
    text:
      "Fy summary\n\n" +
      "R^2 = 0.8937\nAdjusted R^2 = 0.8937\n\n" +
      "Std. beta / t-value\n" +
      "SA    -0.9274 / -349.42\n" +
      "Fz     0.0481 /   17.85\n" +
      "IA     0.0217 /    7.93\n" +
      "TSTC   0.0545 /   20.94\n\n" +
      "Takeaway\n- Fx: SR-driven\n- Fy: SA-driven\n- Fz, IA, TSTC are shared factors",
    fontSize: 12.5, typeface: "Aptos", fill: COLORS.panelAlt, border: COLORS.border
  });
}

async function buildLimits(slide) {
  setupContentSlide(
    slide,
    "09 / 11",
    "Linear regression is a useful baseline, but it cannot capture the intrinsic nonlinearities of tire behavior.",
    "The Fx-SR scatter by load level shows saturation, load-dependent slope changes, and combined-slip structure beyond a linear approximation.",
    "Limits of the linear model: saturation, friction-circle effects, combined slip, temperature dependence"
  );
  await addImage(slide, "fx_nonlinearity.png", { left: 48, top: 198, width: 520, height: 248 });
  addText(slide, {
    left: 590, top: 198, width: 322, height: 248,
    text:
      "What linear regression misses\n\n" +
      "- Saturation at high slip ratios\n" +
      "- Load-dependent slope changes\n" +
      "- Coupled limits between Fx and Fy\n" +
      "- Nonlinear temperature effects\n\n" +
      "For that reason, the next step needs a nonlinear model with physical constraints.",
    fontSize: 14, typeface: "Aptos", fill: COLORS.panel, border: COLORS.border
  });
}

async function buildPinn(slide) {
  setupContentSlide(
    slide,
    "10 / 11",
    "PINN extends the analysis to a nonlinear model with physical constraints.",
    "The regression stage identifies influential variables and provides an interpretable baseline; PINN then handles saturation, load dependence, and combined-slip nonlinearities.",
    "Figure: decoupled Db/Cr quicklook for Hoosier_18p0X6p0-10_LCO_C2000_Rim7"
  );
  await addImage(slide, "pinn_decoupled_quicklook_fx_fy.png", {
    left: 48,
    top: 192,
    width: 380,
    height: 276,
  });
  addRect(slide, { left: 452, top: 194, width: 460, height: 126, fill: COLORS.panelAlt, lineColor: COLORS.border });
  addRect(slide, { left: 452, top: 332, width: 460, height: 136, fill: COLORS.panel, lineColor: COLORS.border });
  addText(slide, {
    left: 468, top: 208, width: 428, height: 98,
    text:
      "What the existing PINN quicklook shows\n\n" +
      "- Fx slices: R^2 = 0.9921, RMSE = 97.6 N\n" +
      "- Fy slices: R^2 = 0.9847, RMSE = 117.1 N\n" +
      "- The model reproduces smooth load-dependent section shapes.",
    fontSize: 14, typeface: "Aptos", fill: COLORS.panelAlt, border: null
  });
  addText(slide, {
    left: 468, top: 344, width: 428, height: 112,
    text:
      "Why it connects to this report\n\n" +
      "- Reuse SR, SA, Fz, IA, and TSTC.\n" +
      "- PINN adds saturation and load dependence.\n" +
      "- Rim6 is the regression case; Rim7 is the reference implementation.",
    fontSize: 12.5, typeface: "Aptos", fill: COLORS.panel, border: null
  });
}

function buildSummary(slide) {
  setupContentSlide(
    slide,
    "11 / 11",
    "The Fx main analysis and Fy comparison reveal the multivariate structure of tire forces in TTC data.",
    "Multiple regression works as an interpretable first step, and PINN becomes a natural extension when nonlinear structure must be modeled explicitly.",
    "Conclusion: regression and PINN are complementary stages rather than competing approaches."
  );
  addRect(slide, { left: 48, top: 196, width: 490, height: 250, fill: COLORS.panel, lineColor: COLORS.border });
  addRect(slide, { left: 566, top: 196, width: 346, height: 250, fill: COLORS.panelAlt, lineColor: COLORS.border });
  addText(slide, {
    left: 62, top: 210, width: 462, height: 222,
    text:
      "Key findings\n\n" +
      "- Fx is explained mainly by SR with R^2 = 0.8830.\n" +
      "- Fy is explained mainly by SA with R^2 = 0.8937.\n" +
      "- Fz, IA, and TSTC are significant in both models.\n" +
      "- Collinearity is weak, and the relevant outliers are multivariate.\n" +
      "- Linear regression is interpretable and useful, but limited in saturation regions.",
    fontSize: 14, typeface: "Aptos", fill: COLORS.panel, border: null
  });
  addText(slide, {
    left: 580, top: 210, width: 318, height: 222,
    text:
      "Next step\n\n" +
      "1. Add nonlinear interaction terms or piecewise models.\n" +
      "2. Extend the same variable set to a PINN.\n" +
      "3. Validate the model on combined-slip TTC sections.\n\n" +
      "In one sentence:\nMultiple regression is a valid and interpretable first step for TTC tire-force modeling.",
    fontSize: 14, typeface: "Aptos", fill: COLORS.panelAlt, border: null
  });
}

async function renderArtifacts(presentation) {
  await fs.mkdir(PREVIEW_DIR, { recursive: true });
  await fs.mkdir(LAYOUT_DIR, { recursive: true });
  const previewPaths = [];
  for (let index = 0; index < presentation.slides.items.length; index += 1) {
    const slide = presentation.slides.items[index];
    const num = padSlideNumber(index + 1);
    const previewPath = path.join(PREVIEW_DIR, `slide-${num}.png`);
    const layoutPath = path.join(LAYOUT_DIR, `slide-${num}.layout.json`);
    const preview = await presentation.export({ slide, format: "png", scale: 2 });
    const layout = await presentation.export({ slide, format: "layout" });
    await saveBlobToFile(preview, previewPath);
    await saveBlobToFile(layout, layoutPath);
    previewPaths.push(previewPath);
  }
  spawnSync("python3", [CONTACT_SCRIPT, ...previewPaths, "--output", CONTACT_SHEET, "--cols", "3"], {
    stdio: "inherit",
  });
}

async function main() {
  await ensureArtifactToolWorkspace(WORKSPACE);
  const { FileBlob, PresentationFile } = await importArtifactTool(WORKSPACE);
  const presentation = await PresentationFile.importPptx(await FileBlob.load(STARTER_PPTX));
  const slides = presentation.slides.items;

  buildCover(slides[0]);
  buildContext(slides[1]);
  buildData(slides[2]);
  buildMethod(slides[3]);
  buildOutlier(slides[4]);
  buildFxCoefficients(slides[5]);
  await buildFxFit(slides[6]);
  await buildComparison(slides[7]);
  await buildLimits(slides[8]);
  await buildPinn(slides[9]);
  buildSummary(slides[10]);

  await renderArtifacts(presentation);

  await fs.mkdir(path.dirname(OUTPUT_PPTX), { recursive: true });
  const pptx = await PresentationFile.exportPptx(presentation);
  await pptx.save(OUTPUT_PPTX);
  await fs.copyFile(OUTPUT_PPTX, FINAL_PPTX);
  console.log(JSON.stringify({ output: OUTPUT_PPTX, final: FINAL_PPTX, slides: slides.length, previewDir: PREVIEW_DIR }, null, 2));
}

main().catch((error) => {
  console.error(error.stack || error.message || String(error));
  process.exit(1);
});
