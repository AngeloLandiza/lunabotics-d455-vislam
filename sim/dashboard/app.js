/* Sandbox dashboard front end. Vanilla JS; every chart is inline SVG drawn here. Data comes from server.py. */
"use strict";
(() => {
  // ------------------------------------------------------------------------------------------------------------
  // DOM helpers (all text goes in through textContent / text nodes)
  // ------------------------------------------------------------------------------------------------------------
  const SVGNS = "http://www.w3.org/2000/svg";
  const $ = (sel, root = document) => root.querySelector(sel);

  function setAttrs(e, attrs) {
    if (!attrs) return;
    for (const [k, v] of Object.entries(attrs)) {
      if (v == null || v === false) continue;
      if (k === "text") e.textContent = v;
      else if (k === "class") e.setAttribute("class", v);
      else if (k.startsWith("on") && typeof v === "function") e.addEventListener(k.slice(2), v);
      else if (k === "style" && typeof v === "object") Object.assign(e.style, v);
      else e.setAttribute(k, v === true ? "" : String(v));
    }
  }
  function append(e, kids) {
    for (const k of kids.flat(Infinity)) {
      if (k == null || k === false) continue;
      e.append(k instanceof Node ? k : document.createTextNode(String(k)));
    }
  }
  function el(tag, attrs, ...kids) {
    const e = document.createElement(tag);
    setAttrs(e, attrs);
    append(e, kids);
    return e;
  }
  function sv(tag, attrs, ...kids) {
    const e = document.createElementNS(SVGNS, tag);
    setAttrs(e, attrs);
    append(e, kids);
    return e;
  }
  function debounce(fn, ms) {
    let t = null;
    return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); };
  }

  // ------------------------------------------------------------------------------------------------------------
  // formatting
  // ------------------------------------------------------------------------------------------------------------
  const DASH = "—";
  const isNum = (x) => typeof x === "number" && Number.isFinite(x);
  const clamp01 = (x) => (isNum(x) ? Math.max(0, Math.min(1, x)) : 0);
  function fmt(x, d = 2) { return isNum(x) ? x.toFixed(d) : DASH; }
  function trimNum(x) {
    if (!isNum(x)) return String(x);
    if (Number.isInteger(x)) return String(x);
    const a = Math.abs(x);
    if (a !== 0 && (a < 1e-3 || a >= 1e6)) return x.toExponential(2).replace(/\.?0+e/, "e");
    return String(+x.toPrecision(6));
  }
  function pct(x, d = 0) { return isNum(x) ? (100 * x).toFixed(d) + "%" : DASH; }
  function bytes(b, d = 1, base = 1000) {
    if (!isNum(b)) return DASH;
    const u = ["B", "KB", "MB", "GB", "TB"];
    let i = 0;
    while (Math.abs(b) >= base && i < u.length - 1) { b /= base; i++; }
    return (i === 0 ? String(Math.round(b)) : b.toFixed(d)) + " " + u[i];
  }
  const ram = (b) => bytes(b, 1, 1024);   // memory in 1024-based units, as system monitors show it
  const speedText = (v) => (isNum(v) ? `${v.toFixed(2).replace(/0$/, "")} m/s` : DASH);
  /** A path with line-break chances after each slash, so long paths wrap at folder boundaries. */
  function pathNode(p) {
    const parts = String(p).split("/");
    return el("code", null, parts.map((x, i) => (i < parts.length - 1 ? [x + "/", el("wbr")] : x)));
  }
  function dur(s) {
    if (!isNum(s)) return DASH;
    s = Math.max(0, s);
    if (s < 60) return Math.round(s) + " s";
    if (s < 3600) { const m = Math.floor(s / 60), r = Math.round(s % 60); return r && m < 10 ? `${m} min ${r} s` : `${Math.round(s / 60)} min`; }
    if (s < 86400) { const hr = Math.floor(s / 3600), m = Math.round((s % 3600) / 60); return `${hr} h ${String(m).padStart(2, "0")} min`; }
    const dd = Math.floor(s / 86400), hr = Math.round((s % 86400) / 3600);
    return `${dd} d ${hr} h`;
  }
  const ago = (s) => (isNum(s) ? dur(s) + " ago" : DASH);
  function when(epoch) {
    if (!isNum(epoch)) return DASH;
    return new Date(epoch * 1000).toLocaleString([], { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" });
  }
  function clock(epoch) {
    return new Date(epoch * 1000).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" });
  }
  function fmtValue(v) {
    if (v == null) return DASH;
    if (isNum(v)) return trimNum(v);
    if (Array.isArray(v)) return "(" + v.map(fmtValue).join(", ") + ")";
    if (typeof v === "object") return Object.entries(v).map(([k, x]) => `${k}: ${fmtValue(x)}`).join(", ");
    return String(v);
  }
  function linkify(text) {
    const out = [];
    const re = /https?:\/\/[^\s)]+/g;
    let last = 0, m;
    text = String(text || "");
    while ((m = re.exec(text))) {
      let url = m[0].replace(/[.,;]+$/, "");
      out.push(text.slice(last, m.index));
      out.push(el("a", { href: url, target: "_blank", rel: "noopener noreferrer", text: url }));
      last = m.index + url.length;
    }
    out.push(text.slice(last));
    return out;
  }
  const code = (t) => el("code", { text: t });
  function sourceLine(path, mtime, extra) {
    return el("p", { class: "source" }, "Source: ", pathNode(path), isNum(mtime) ? ` · modified ${when(mtime)}` : "", extra || "");
  }

  // ------------------------------------------------------------------------------------------------------------
  // shared vocabulary
  // ------------------------------------------------------------------------------------------------------------
  const SENSORS = {
    spin16: { label: "16-beam LiDAR", cls: "lidar", shape: "circle" },
    l1: { label: "Unitree L1", cls: "lidar", shape: "circle" },
    d455: { label: "D455", cls: "d455", shape: "square" },
    fused: { label: "LiDAR or D455 (fused)", cls: "fused", shape: "diamond" },
  };
  const sensorInfo = (s) => SENSORS[s] || { label: s, cls: "lidar", shape: "circle" };

  function shape(kind, cx, cy, size, attrs) {
    const r = size / 2;
    if (kind === "square") return sv("rect", { x: cx - r * 0.9, y: cy - r * 0.9, width: r * 1.8, height: r * 1.8, rx: 1.5, ...attrs });
    if (kind === "diamond") {
      const q = r * 1.25;
      return sv("polygon", { points: `${cx},${cy - q} ${cx + q},${cy} ${cx},${cy + q} ${cx - q},${cy}`, ...attrs });
    }
    return sv("circle", { cx, cy, r, ...attrs });
  }
  function sensorKey(s) {
    const info = sensorInfo(s);
    const svg = sv("svg", { width: 12, height: 12, viewBox: "0 0 12 12", "aria-hidden": "true" },
      shape(info.shape, 6, 6, 9, { class: "f-" + info.cls }));
    return el("span", { class: "sensor-key" }, svg, info.label);
  }
  function sensorLegend(list) {
    return el("div", { class: "legend", "aria-label": "Sensors" }, list.map((s) => el("span", { class: "item" }, sensorKey(s))));
  }

  const RUN_STATUS = {
    running: ["info", "●", "Running"],
    done: ["good", "✓", "Finished"],
    error: ["bad", "✕", "Failed"],
    stale: ["warn", "!", "No updates"],
    stopped: ["bad", "■", "Stopped without finishing"],
    unreadable: ["neutral", "?", "Unreadable"],
  };
  function chip(kind, icon, text, title) {
    return el("span", { class: "chip " + kind, title: title || null }, el("i", { "aria-hidden": "true", text: icon }), text);
  }
  function runChip(r) {
    const [k, i, t] = RUN_STATUS[r.status] || RUN_STATUS.unreadable;
    if (r.status === "stale") return chip(k, i, `No update for ${dur(r.age_s)}`, "No new line for over 10 minutes and no end line");
    return chip(k, i, t);
  }
  const PARAM_STATUS = {
    Verified: ["good", "✓"],
    Inferred: ["info", "≈"],
    Assumed: ["warn", "?"],
    Unlabelled: ["neutral", "–"],
  };
  function paramChip(s) {
    const [k, i] = PARAM_STATUS[s] || PARAM_STATUS.Unlabelled;
    return chip(k, i, s);
  }

  // ------------------------------------------------------------------------------------------------------------
  // tooltip
  // ------------------------------------------------------------------------------------------------------------
  const tip = $("#tip");
  function showTip(x, y, nodes) {
    tip.replaceChildren(...nodes);
    tip.hidden = false;
    const r = tip.getBoundingClientRect();
    let left = x + 14, top = y + 14;
    if (left + r.width > window.innerWidth - 8) left = x - r.width - 14;
    if (top + r.height > window.innerHeight - 8) top = y - r.height - 14;
    tip.style.left = Math.max(8, left) + "px";
    tip.style.top = Math.max(8, top) + "px";
  }
  function hideTip() { tip.hidden = true; }
  function tipRows(value, label, extra) {
    return [el("div", { class: "tv", text: value }), el("div", { class: "tl", text: label }), extra ? el("div", { class: "tl", text: extra }) : null].filter(Boolean);
  }
  function bindTip(node, content) {
    node.addEventListener("pointermove", (ev) => showTip(ev.clientX, ev.clientY, content()));
    node.addEventListener("pointerleave", hideTip);
    node.addEventListener("focus", () => { const b = node.getBoundingClientRect(); showTip(b.right, b.top, content()); });
    node.addEventListener("blur", hideTip);
  }

  // ------------------------------------------------------------------------------------------------------------
  // scales
  // ------------------------------------------------------------------------------------------------------------
  function niceStep(range, count) {
    const raw = range / Math.max(1, count);
    const p = Math.pow(10, Math.floor(Math.log10(raw)));
    const m = raw / p;
    return (m >= 7.5 ? 10 : m >= 3.5 ? 5 : m >= 1.5 ? 2 : 1) * p;
  }
  function niceTicks(min, max, count = 5) {
    if (!(max > min)) max = min + 1;
    const step = niceStep(max - min, count);
    const lo = Math.floor(min / step + 1e-9) * step, hi = Math.ceil(max / step - 1e-9) * step;
    const ticks = [];
    for (let v = lo; v <= hi + step * 1e-6; v += step) ticks.push(+v.toFixed(10));
    return { lo, hi, step, ticks };
  }
  function tickText(v, step) {
    if (Math.abs(v) >= 1000 && step >= 100) return trimNum(v / 1000) + "K";
    const d = Math.max(0, Math.min(6, -Math.floor(Math.log10(step) + 1e-9)));
    return v.toFixed(d);
  }

  // ------------------------------------------------------------------------------------------------------------
  // charts
  // ------------------------------------------------------------------------------------------------------------
  /** Line chart with hairline grid, one y axis, 2px line, end dot, and a crosshair + tooltip. */
  function lineChart({ points, width, height = 150, xName, yName, xFmt, yFmt }) {
    const m = { l: 48, r: 14, t: 10, b: 26 };
    const W = Math.max(220, Math.floor(width)), H = height;
    const iw = W - m.l - m.r, ih = H - m.t - m.b;
    const svg = sv("svg", { width: W, height: H, viewBox: `0 0 ${W} ${H}`, role: "img", "aria-label": `${yName} over ${xName}` });
    if (!points.length) return svg;
    const xs = points.map((p) => p[0]), ys = points.map((p) => p[1]);
    let x0 = Math.min(...xs), x1 = Math.max(...xs);
    if (x1 === x0) x1 = x0 + 1;
    let ymin = Math.min(...ys), ymax = Math.max(...ys);
    if (ymin >= 0 && ymin <= 0.5 * ymax) ymin = 0;
    if (ymax === ymin) { const p = Math.abs(ymin) * 0.1 || 1; ymax += p; if (ymin !== 0) ymin -= p; }
    const yt = niceTicks(ymin, ymax, 4);
    const X = (v) => m.l + ((v - x0) / (x1 - x0)) * iw;
    const Y = (v) => m.t + ih - ((v - yt.lo) / (yt.hi - yt.lo)) * ih;
    for (const t of yt.ticks) {
      svg.append(sv("line", { class: "grid-line", x1: m.l, x2: W - m.r, y1: Y(t), y2: Y(t) }));
      svg.append(sv("text", { x: m.l - 6, y: Y(t) + 3.5, "text-anchor": "end", text: (yFmt ? yFmt(t, yt.step) : tickText(t, yt.step)) }));
    }
    const xt = niceTicks(x0, x1, Math.max(2, Math.floor(iw / 90)));
    for (const t of xt.ticks) {
      if (t < x0 - 1e-9 || t > x1 + 1e-9) continue;
      svg.append(sv("text", { x: X(t), y: H - 8, "text-anchor": "middle", text: xFmt ? xFmt(t, xt.step) : tickText(t, xt.step) }));
    }
    svg.append(sv("line", { class: "axis-line", x1: m.l, x2: W - m.r, y1: m.t + ih, y2: m.t + ih }));
    const d = points.map((p, i) => `${i ? "L" : "M"}${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join("");
    if (points.length > 1) {
      const base = Y(Math.max(yt.lo, Math.min(0, yt.hi)));
      svg.append(sv("path", { class: "area", d: `${d}L${X(points[points.length - 1][0]).toFixed(1)},${base}L${X(points[0][0]).toFixed(1)},${base}Z` }));
      svg.append(sv("path", { class: "series", d }));
    }
    const last = points[points.length - 1];
    svg.append(sv("circle", { class: "end-dot", cx: X(last[0]), cy: Y(last[1]), r: 4 }));
    // crosshair
    const cross = sv("line", { class: "crosshair", y1: m.t, y2: m.t + ih, visibility: "hidden" });
    const cdot = sv("circle", { class: "end-dot", r: 4, visibility: "hidden" });
    svg.append(cross, cdot);
    const hit = sv("rect", { class: "hit", x: m.l, y: m.t, width: iw, height: ih + 6 });
    svg.append(hit);
    hit.addEventListener("pointermove", (ev) => {
      const box = svg.getBoundingClientRect();
      const xv = x0 + ((ev.clientX - box.left - m.l) / iw) * (x1 - x0);
      let lo = 0, hi = points.length - 1;
      while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (points[mid][0] < xv) lo = mid; else hi = mid; }
      const p = Math.abs(points[lo][0] - xv) <= Math.abs(points[hi][0] - xv) ? points[lo] : points[hi];
      cross.setAttribute("x1", X(p[0])); cross.setAttribute("x2", X(p[0])); cross.setAttribute("visibility", "visible");
      cdot.setAttribute("cx", X(p[0])); cdot.setAttribute("cy", Y(p[1])); cdot.setAttribute("visibility", "visible");
      showTip(ev.clientX, ev.clientY, tipRows(trimNum(+p[1].toPrecision(6)), yName, `${xName} ${xFmt ? xFmt(p[0]) : trimNum(+p[0].toPrecision(4))}`));
    });
    hit.addEventListener("pointerleave", () => { cross.setAttribute("visibility", "hidden"); cdot.setAttribute("visibility", "hidden"); hideTip(); });
    return svg;
  }

  /** Small progress sparkline: items done over time. */
  function sparkline(points, total, width = 240, height = 44) {
    const svg = sv("svg", { width, height, viewBox: `0 0 ${width} ${height}`, role: "img", "aria-label": "Items done over time" });
    if (!points || points.length < 2) {
      svg.append(sv("text", { x: 0, y: height / 2 + 4, text: "Waiting for progress lines" }));
      return svg;
    }
    const t0 = points[0][0], t1 = Math.max(points[points.length - 1][0], t0 + 1);
    const ymax = Math.max(isNum(total) ? total : 0, ...points.map((p) => p[1]), 1);
    const X = (t) => 2 + ((t - t0) / (t1 - t0)) * (width - 8);
    const Y = (v) => height - 3 - (v / ymax) * (height - 8);
    const d = points.map((p, i) => `${i ? "L" : "M"}${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join("");
    svg.append(sv("line", { class: "axis-line", x1: 2, x2: width - 6, y1: height - 3, y2: height - 3 }));
    if (isNum(total)) svg.append(sv("line", { class: "grid-line", x1: 2, x2: width - 6, y1: Y(total), y2: Y(total) }));
    svg.append(sv("path", { class: "area", d: `${d}L${X(points[points.length - 1][0])},${height - 3}L${X(t0)},${height - 3}Z` }));
    svg.append(sv("path", { class: "series", d }));
    const last = points[points.length - 1];
    svg.append(sv("circle", { class: "end-dot", cx: X(last[0]), cy: Y(last[1]), r: 4 }));
    return svg;
  }

  /** Horizontal 100% stacked bar (HTML) for research check categories. */
  function stackBar(row, cats, labels, name) {
    const total = cats.reduce((a, c) => a + (isNum(row[c]) ? row[c] : 0), 0);
    const bar = el("div", { class: "stack", role: "img", "aria-label": `${name}: ` + cats.filter((c) => isNum(row[c]) && row[c] > 0).map((c) => `${labels[c]} ${row[c]}`).join(", ") });
    if (!total) return bar;
    for (const c of cats) {
      const v = row[c];
      if (!isNum(v) || v <= 0) continue;
      const seg = el("span", { class: "bg-" + c, style: { flex: `${v} 1 0px` }, tabindex: "0" });
      bindTip(seg, () => tipRows(`${v} rows (${pct(v / total, 1)})`, `${labels[c]} · ${name}`));
      bar.append(seg);
    }
    return bar;
  }

  /** Meter card: label, value, bar, detail. Severity changes the fill and adds a text level, never color alone. */
  function meterCard({ label, value, unit, frac, detail, warnAt = 0.7, highAt = 0.9, idle, words = ["Busy", "High"] }) {
    const f = clamp01(frac);
    const lvl = idle ? "" : f >= highAt ? "high" : f >= warnAt ? "warn" : "";
    const word = lvl === "high" ? "\u25b2 " + words[1] : lvl === "warn" ? "\u25b2 " + words[0] : "";
    return el("div", { class: "card meter-card" },
      el("div", { class: "label" }, el("span", { text: label }), word ? el("span", { class: "level " + lvl, text: word }) : null),
      el("div", { class: "value" }, value, unit ? el("small", { text: " " + unit }) : null),
      el("div", { class: "meter " + lvl, role: "meter", "aria-label": label, "aria-valuemin": "0", "aria-valuemax": "100", "aria-valuenow": String(Math.round(f * 100)) },
        el("span", { style: { width: (f * 100).toFixed(1) + "%" } })),
      detail ? el("div", { class: "detail", text: detail }) : null);
  }

  // ------------------------------------------------------------------------------------------------------------
  // data loading, connection state, tabs
  // ------------------------------------------------------------------------------------------------------------
  class ApiError extends Error {}
  async function getJSON(url) {
    const r = await fetch(url, { cache: "no-store" });
    const text = await r.text();
    let data;
    try { data = JSON.parse(text); } catch (e) { throw new ApiError(`${url} did not return JSON (HTTP ${r.status})`); }
    if (!r.ok) throw new ApiError(data && data.error ? `${url}: ${data.error}` : `${url}: HTTP ${r.status}`);
    return { data, text };
  }
  function setConn(ok, msg) {
    const c = $("#conn");
    c.classList.toggle("bad", !ok);
    $("#conn-text").textContent = ok ? `Updated ${new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" })}` : msg;
  }
  function errorBox(e) {
    return el("div", { class: "error-box", role: "alert" }, el("strong", { text: "Could not load this section. " }), e.message || String(e));
  }

  const state = { runSel: null, detail: null, paramStatus: null, paramText: "", texts: {} };
  const TABS = {
    live: { load: loadLive, render: renderLive, every: 2000 },
    runs: { load: loadRuns, render: renderRuns, every: 5000 },
    results: { load: loadResults, render: renderResults, every: 30000 },
    planner: { load: loadPlanner, render: renderPlanner, every: 30000 },
    params: { load: loadParams, render: renderParams, every: 60000 },
    research: { load: loadResearch, render: renderResearch, every: 60000 },
  };
  let current = null, timer = null, gen = 0;

  function route() {
    const hash = decodeURIComponent(location.hash.replace(/^#/, ""));
    const [name, ...rest] = hash.split("/");
    const tab = TABS[name] ? name : "live";
    const sel = tab === "runs" && rest.length ? rest.join("/") : null;
    if (sel !== state.runSel) { state.runSel = sel; state.detail = null; }
    for (const a of document.querySelectorAll(".tabs a")) {
      if (a.dataset.tab === tab) a.setAttribute("aria-current", "page"); else a.removeAttribute("aria-current");
    }
    for (const k of Object.keys(TABS)) document.getElementById("tab-" + k).hidden = k !== tab;
    current = tab;
    hideTip();
    schedule();
  }
  function schedule() {
    clearTimeout(timer);
    const my = ++gen;
    const run = async () => {
      if (my !== gen) return;
      if (!document.hidden) {
        try {
          await TABS[current].load();
          setConn(true);
        } catch (e) {
          if (e instanceof ApiError) setConn(true);
          else setConn(false, "Cannot reach the dashboard server");
          if (!(e instanceof ApiError)) console.warn(e);
        }
      }
      if (my === gen) timer = setTimeout(run, TABS[current].every);
    };
    run();
  }

  // ------------------------------------------------------------------------------------------------------------
  // Live
  // ------------------------------------------------------------------------------------------------------------
  async function loadLive() {
    let sys, runs;
    try {
      [sys, runs] = await Promise.all([getJSON("/api/system"), getJSON("/api/runs")]);
    } catch (e) {
      if (e instanceof ApiError) $("#live-meters").replaceChildren(errorBox(e));
      throw e;
    }
    state.sys = sys.data;
    state.runs = runs.data;
    renderLive();
  }

  function renderLive() {
    const s = state.sys;
    if (!s) return;
    $("#host").textContent = s.hostname || "";
    $("#live-sub").textContent = `${s.hostname} · ${s.cpu_count} CPU cores · refreshes every 2 s`;
    const cards = [];
    const cores = s.cpu_count || 1;
    if (s.load) {
      const [l1, l5, l15] = s.load;
      cards.push(meterCard({ label: "CPU load (1 min average)", value: fmt(l1, 2), unit: `of ${cores} cores`, frac: l1 / cores,
        detail: `1 / 5 / 15 min: ${fmt(l1)} · ${fmt(l5)} · ${fmt(l15)}${l1 > cores ? " · more work than cores" : ""}` }));
    }
    cards.push(meterCard({ label: "CPU busy", value: isNum(s.cpu_busy) ? pct(s.cpu_busy) : "measuring", frac: s.cpu_busy || 0, idle: !isNum(s.cpu_busy),
      detail: "share of CPU time in use since the last refresh" }));
    if (s.memory) {
      const m = s.memory;
      cards.push(meterCard({ label: "Memory (RAM)", value: ram(m.used), unit: `of ${ram(m.total)}`, frac: m.used / m.total, warnAt: 0.75, words: ["Filling up", "Nearly full"],
        detail: `${ram(m.available)} available${m.swap_total ? ` · swap ${ram(m.swap_used)} used` : ""}` }));
    }
    if (s.disk) {
      cards.push(meterCard({ label: "Disk (repo drive)", value: bytes(s.disk.free), unit: "free", frac: 1 - s.disk.free / s.disk.total, warnAt: 0.85, highAt: 0.95, words: ["Filling up", "Nearly full"],
        detail: `of ${bytes(s.disk.total)}; the bar shows the share not free` }));
    }
    if (s.gpu && s.gpu.length) {
      s.gpu.forEach((g, i) => {
        const tag = s.gpu.length > 1 ? ` ${i}` : "";
        cards.push(meterCard({ label: `GPU${tag} use`, value: isNum(g.util_pct) ? `${Math.round(g.util_pct)}%` : DASH, frac: (g.util_pct || 0) / 100, idle: true, detail: g.name }));
        const mem = isNum(g.mem_used_mib) && isNum(g.mem_total_mib);
        cards.push(meterCard({ label: `GPU${tag} memory`, value: mem ? ram(g.mem_used_mib * 1048576) : DASH, unit: mem ? `of ${ram(g.mem_total_mib * 1048576)}` : "",
          frac: mem ? g.mem_used_mib / g.mem_total_mib : 0, warnAt: 0.8, words: ["Filling up", "Nearly full"], detail: mem ? `${ram((g.mem_total_mib - g.mem_used_mib) * 1048576)} free` : "not reported" }));
        cards.push(meterCard({ label: `GPU${tag} temperature`, value: isNum(g.temp_c) ? `${Math.round(g.temp_c)} °C` : DASH, frac: (g.temp_c || 0) / 100, warnAt: 0.8, highAt: 0.88, words: ["Warm", "Hot"],
          detail: isNum(g.power_w) ? `drawing ${fmt(g.power_w, 0)} W` : "power not reported" }));
      });
    } else {
      cards.push(el("div", { class: "card meter-card" },
        el("div", { class: "label" }, el("span", { text: "GPU" })),
        el("div", { class: "value" }, el("small", { text: "No NVIDIA GPU readout" })),
        el("div", { class: "detail", text: "nvidia-smi was not found or did not answer within 2 s." })));
    }
    $("#live-meters").replaceChildren(...cards);

    // runs
    const runs = (state.runs && state.runs.runs) || [];
    const active = runs.filter((r) => ["running", "stale", "stopped"].includes(r.status) ||
      (["done", "error"].includes(r.status) && r.age_s < 900));
    const box = $("#live-runs");
    if (!active.length) {
      box.replaceChildren(el("div", { class: "empty" }, el("strong", { text: "No runs in progress. " }),
        "Programs that report progress with telemetry.py appear here while they work. ",
        runs.length ? `${runs.length} earlier run${runs.length > 1 ? "s are" : " is"} listed under Runs.` : `Their progress files go to ${state.runs ? state.runs.dir : "sim/telemetry"}.`));
    } else {
      box.replaceChildren(el("div", { class: "grid cols-2" }, active.map(runCard)));
    }

    // processes
    const procs = s.processes || [];
    const pbox = $("#live-procs");
    if (!procs.length) {
      pbox.replaceChildren(el("div", { class: "empty", text: "None running right now." }));
    } else {
      pbox.replaceChildren(el("div", { class: "card" }, el("div", { class: "table-wrap" }, el("table", null,
        el("thead", null, el("tr", null, el("th", { text: "Program" }), el("th", { class: "num", text: "PID" }), el("th", { class: "num", text: "Worker processes" }),
          el("th", { class: "num", text: "CPU time" }), el("th", { class: "num", text: "Average cores used" }), el("th", { class: "num", text: "Memory" }), el("th", { class: "num", text: "Running for" }))),
        el("tbody", null, procs.map((p) => el("tr", null,
          el("td", { class: "mono wrap-any", text: p.cmd }),
          el("td", { class: "num", text: p.pid }),
          el("td", { class: "num", text: p.workers || DASH }),
          el("td", { class: "num", text: dur((p.cpu_s || 0) + (p.workers_cpu_s || 0)) }),
          el("td", { class: "num", text: isNum(p.cpu_pct) ? fmt(p.cpu_pct / 100, 1) : DASH }),
          el("td", { class: "num", text: ram((p.rss || 0) + (p.workers_rss || 0)) }),
          el("td", { class: "num", text: dur(p.age_s) }))))))));
    }
  }

  function metricsGrid(metrics) {
    const entries = Object.entries(metrics || {});
    if (!entries.length) return el("p", { class: "note", text: "No metrics reported yet." });
    return el("div", { class: "metrics" }, entries.map(([k, v]) =>
      el("div", { title: `${k}: ${fmtValue(v)}` }, el("span", { text: k }), el("span", { text: fmtValue(v) }))));
  }

  function runTitle(r) { return r.label || r.kind || r.run; }

  function runCard(r) {
    const prog = r.progress;
    const count = isNum(r.done) ? (isNum(r.total) ? `${r.done} / ${r.total}` : `${r.done} done`) : DASH;
    const facts = [];
    if (r.status === "running") facts.push(el("span", null, "Time left ", el("b", { text: isNum(r.eta_now_s) ? dur(r.eta_now_s) : "estimating" })));
    facts.push(el("span", null, "Last update ", el("b", { text: ago(r.age_s) })));
    facts.push(el("span", null, r.status === "running" ? "Running for " : "Ran for ", el("b", { text: dur(r.status === "running" ? r.duration_s + r.age_s : r.duration_s) })));
    if (isNum(r.rate_per_s) && r.rate_per_s > 0) facts.push(el("span", null, "Pace ", el("b", { text: r.rate_per_s >= 1 ? `${fmt(r.rate_per_s, 1)} per s` : `${fmt(r.rate_per_s * 60, 2)} per min` })));
    facts.push(el("span", null, "On ", el("b", { text: `${r.host || "?"}` }), ` (PID ${r.pid ?? "?"})`));
    const sparkPts = (r.spark || []).filter((p) => isNum(p[0]) && isNum(p[1]));
    return el("div", { class: "card run-card" },
      el("div", { class: "row1" }, runChip(r), el("span", { class: "title", text: runTitle(r) }), r.label ? el("span", { class: "muted", text: r.kind }) : null,
        el("a", { class: "id", href: "#runs/" + encodeURIComponent(r.run), title: "Open this run under Runs", text: r.run })),
      el("div", { class: "bar-line" },
        el("div", { class: "meter", role: "progressbar", "aria-label": "Progress", "aria-valuemin": "0", "aria-valuemax": "100", "aria-valuenow": String(Math.round(clamp01(prog) * 100)) },
          el("span", { style: { width: (clamp01(prog) * 100).toFixed(1) + "%" } })),
        el("span", { class: "count", text: count + (isNum(prog) ? ` · ${pct(prog)}` : "") })),
      el("div", { class: "facts" }, facts),
      r.status === "error" && r.msg ? el("div", { class: "error-box", text: r.msg }) : null,
      el("div", { class: "run-body" },
        el("div", null, el("div", { class: "spark-label", text: "Items done over time" }), sparkline(sparkPts, r.total)),
        el("div", null, el("div", { class: "spark-label", text: "Latest numbers" }), metricsGrid(r.metrics))));
  }

  // ------------------------------------------------------------------------------------------------------------
  // Runs
  // ------------------------------------------------------------------------------------------------------------
  async function loadRuns() {
    let runs;
    try {
      runs = await getJSON("/api/runs");
    } catch (e) {
      if (e instanceof ApiError) $("#runs-table").replaceChildren(errorBox(e));
      throw e;
    }
    state.runs = runs.data;
    if (state.runSel) {
      const sum = state.runs.runs.find((r) => r.run === state.runSel);
      const have = state.detail && state.detail.run === state.runSel ? state.detail : null;
      if (!have || (sum && sum.n_lines !== have.lines.length)) {
        try {
          state.detail = (await getJSON("/api/run/" + encodeURIComponent(state.runSel))).data;
        } catch (e) {
          if (!(e instanceof ApiError)) throw e;
          state.detail = { run: state.runSel, error: e.message, lines: [] };
        }
      }
    }
    renderRuns();
  }

  function renderRuns() {
    const data = state.runs;
    if (!data) return;
    const all = data.runs;
    $("#runs-sub").textContent = `${all.length} run${all.length === 1 ? "" : "s"} in ${data.dir}`;
    const q = $("#runs-filter").value.trim().toLowerCase();
    const st = $("#runs-status").value;
    const list = all.filter((r) => (!st || r.status === st) &&
      (!q || [r.kind, r.label, r.host, r.run].some((x) => String(x || "").toLowerCase().includes(q))));
    const box = $("#runs-table");
    if (!all.length) {
      box.replaceChildren(el("div", { class: "empty" }, el("strong", { text: "No runs recorded yet. " }),
        "A program records a run by creating a Telemetry object (sim/bp1_sandbox/telemetry.py); its progress file appears in ", code(data.dir), "."));
    } else {
      const rows = list.map((r) => {
        const open = () => { location.hash = "#runs/" + encodeURIComponent(r.run); };
        return el("tr", { class: "clickable" + (r.run === state.runSel ? " selected" : ""), tabindex: "0", onclick: open,
          onkeydown: (ev) => { if (ev.key === "Enter" || ev.key === " ") { ev.preventDefault(); open(); } } },
          el("td", { text: r.kind || DASH }),
          el("td", { class: "wrap-any", text: r.label || DASH }),
          el("td", { class: "nowrap", text: when(r.started) }),
          el("td", { class: "num", text: dur(r.duration_s) }),
          el("td", null, runChip(r)),
          el("td", { class: "num", text: isNum(r.done) ? (isNum(r.total) ? `${r.done} / ${r.total}` : String(r.done)) : DASH }),
          el("td", { class: "wrap-any", text: r.host || DASH }));
      });
      box.replaceChildren(el("div", { class: "card" }, el("div", { class: "table-wrap" }, el("table", null,
        el("thead", null, el("tr", null, ["Kind", "Label", "Started", "Duration", "Status", "Done", "Computer"].map((t, i) =>
          el("th", { class: i === 3 || i === 5 ? "num" : null, text: t })))),
        el("tbody", null, rows.length ? rows : el("tr", null, el("td", { colspan: "7", class: "muted", text: "No runs match the filter." })))))));
    }
    renderRunDetail();
  }

  function renderRunDetail() {
    const box = $("#run-detail");
    if (!state.runSel) { box.replaceChildren(); return; }
    const det = state.detail;
    const close = el("button", { class: "btn", type: "button", onclick: () => { location.hash = "#runs"; }, text: "Close" });
    if (!det) { box.replaceChildren(el("div", { class: "card" }, el("div", { class: "detail-head" }, el("h2", { text: state.runSel }), close), el("p", { class: "note", text: "Loading..." }))); return; }
    if (det.error) { box.replaceChildren(el("div", { class: "card" }, el("div", { class: "detail-head" }, el("h2", { text: state.runSel }), close), errorBox(new Error(det.error)))); return; }
    const sum = (state.runs.runs.find((r) => r.run === det.run)) || det.summary || {};
    const lines = det.lines || [];
    const t0 = lines.length ? lines[0].t : 0;
    const span = lines.length ? lines[lines.length - 1].t - t0 : 0;
    const useMin = span > 180;
    const xName = useMin ? "minutes since start" : "seconds since start";
    const xOf = (t) => (t - t0) / (useMin ? 60 : 1);
    const xFmt = (v) => trimNum(+(+v).toPrecision(3)) + (useMin ? " min" : " s");

    const card = el("div", { class: "card" });
    box.replaceChildren(card);
    card.append(el("div", { class: "detail-head" }, runChip(sum), el("h2", { text: runTitle(sum) }), el("span", { class: "muted", text: sum.kind || "" }), close));
    card.append(el("div", { class: "facts" },
      el("span", null, "Started ", el("b", { text: when(sum.started) })),
      el("span", null, "Duration ", el("b", { text: dur(sum.duration_s) })),
      el("span", null, "Done ", el("b", { text: isNum(sum.done) ? `${sum.done}${isNum(sum.total) ? " / " + sum.total : ""}` : DASH })),
      el("span", null, "Computer ", el("b", { text: sum.host || DASH }), ` (PID ${sum.pid ?? "?"})`),
      el("span", null, "Lines ", el("b", { text: String(lines.length) }))));
    if (sum.msg) card.append(el("div", { class: sum.status === "error" ? "error-box" : "note", text: sum.msg }));
    const args = sum.args || (lines[0] && lines[0].args);
    if (args && Object.keys(args).length) {
      card.append(el("details", { open: Object.keys(args).length <= 12 },
        el("summary", { text: "Settings the run started with" }),
        el("dl", { class: "kv" }, Object.entries(args).map(([k, v]) => [el("dt", { text: k }), el("dd", { text: fmtValue(v) })]))));
    }
    // charts: items done + every numeric metric
    const series = [];
    const donePts = lines.filter((l) => isNum(l.done) && isNum(l.t)).map((l) => [xOf(l.t), l.done]);
    if (donePts.length) series.push(["Items done", donePts]);
    const keys = [];
    for (const l of lines) for (const [k, v] of Object.entries(l.metrics || {})) if (isNum(v) && !keys.includes(k)) keys.push(k);
    for (const k of keys) {
      const pts = lines.filter((l) => l.metrics && isNum(l.metrics[k]) && isNum(l.t)).map((l) => [xOf(l.t), l.metrics[k]]);
      if (pts.length) series.push([k, pts]);
    }
    card.append(el("h3", { text: "Numbers over time" }));
    const grid = el("div", { class: "grid charts" });
    card.append(grid);
    if (!series.length) grid.append(el("p", { class: "note", text: "No numbers reported yet." }));
    for (const [name, pts] of series) {
      const c = el("div", { class: "card" }, el("div", { class: "chart-title", text: name }), el("div", { class: "chart-sub", text: `${pts.length} point${pts.length > 1 ? "s" : ""} · x: ${xName}` }));
      grid.append(c);
      c.append(lineChart({ points: pts, width: c.clientWidth - 34, height: 150, xName, yName: name, xFmt }));
    }
    card.append(sourceLine(det.file || sum.file || ""));
    // the lines themselves, newest first
    const shown = lines.slice(-500).reverse();
    card.append(el("h3", { text: `Progress lines (newest first${lines.length > 500 ? ", last 500" : ""})` }));
    card.append(el("div", { class: "log" }, el("table", null,
      el("thead", null, el("tr", null, ["Time", "Event", "Done", "Time left", "Numbers", "Message"].map((t, i) => el("th", { class: i === 2 || i === 3 ? "num" : null, text: t })))),
      el("tbody", null, shown.map((l) => el("tr", null,
        el("td", { class: "nowrap", text: isNum(l.t) ? clock(l.t) : DASH }),
        el("td", { text: l.event || DASH }),
        el("td", { class: "num", text: isNum(l.done) ? (isNum(l.total) ? `${l.done} / ${l.total}` : String(l.done)) : DASH }),
        el("td", { class: "num", text: isNum(l.eta_s) ? dur(l.eta_s) : DASH }),
        el("td", { class: "small wrap-any", text: Object.entries(l.metrics || {}).map(([k, v]) => `${k} ${fmtValue(v)}`).join(" · ") }),
        el("td", { class: "small wrap-any", text: l.msg || "" })))))));
  }

  $("#runs-filter").addEventListener("input", () => renderRuns());
  $("#runs-status").addEventListener("change", () => renderRuns());

  // ------------------------------------------------------------------------------------------------------------
  // Results
  // ------------------------------------------------------------------------------------------------------------
  async function loadResults() {
    let r;
    try {
      r = await getJSON("/api/results");
    } catch (e) {
      if (e instanceof ApiError) $("#results-body").replaceChildren(errorBox(e));
      throw e;
    }
    $("#results-sub").textContent = `${r.data.items.length} files · checked ${clock(r.data.time)}`;
    if (r.text === state.texts.results) return;
    state.texts.results = r.text;
    state.results = r.data;
    renderResults();
  }

  function speedKey(row, speed) {
    return Object.keys(row).find((k) => /^v[0-9.]+$/.test(k) && Math.abs(parseFloat(k.slice(1)) - speed) < 1e-9);
  }
  const missed = (x) => (x && isNum(x.n) ? `${x.missed}/${x.n}` : DASH);
  const missedPct = (x) => (x && isNum(x.n) ? `${x.missed}/${x.n}${isNum(x.rate) ? ` (${pct(x.rate)})` : ""}` : DASH);
  const air = (dust) => (dust ? "dust plume" : "clear air");

  function renderResults() {
    const data = state.results;
    if (!data) return;
    const body = $("#results-body");
    body.replaceChildren();
    const by = (t) => data.items.filter((x) => x.type === t && !x.error);
    const errs = data.items.filter((x) => x.error);
    if (!data.items.length) {
      body.append(el("div", { class: "empty" }, el("strong", { text: "No result files found. " }),
        "Run the simulations (results land in sim/bp1_sandbox/results*), or copy the cloud summaries into the repo with ",
        code("python3 sim/dashboard/collect_summaries.py"), " and pull them on this computer."));
      return;
    }
    for (const it of by("robustness")) robustnessBlock(it, body);
    for (const it of by("speed_gate")) body.append(speedGateBlock(it));
    for (const it of by("selfmask_eval")) body.append(selfmaskBlock(it));
    const all96 = by("all96_summary");
    for (const it of all96) body.append(all96Block(it));
    const metas = by("approaches_meta");
    if (metas.length) body.append(approachesBlock(metas, data.roots));
    for (const it of by("isaac_tags")) body.append(isaacTagsBlock(it));
    const drift = by("odometry_drift"), isaacVo = by("isaac_vo");
    if (drift.length || isaacVo.length) body.append(odometrySection(drift, isaacVo));
    const recs = by("records");
    if (recs.length) body.append(recordsBlock(recs));
    for (const it of by("autonomy_runs")) body.append(autonomyRunsBlock(it));
    const locReplay = by("localizer_replay");
    if (locReplay.length) body.append(localizerReplaySection(locReplay));
    if (errs.length) {
      body.append(el("div", { class: "card" }, el("h3", { text: "Files that could not be read" }),
        el("ul", null, errs.map((x) => el("li", null, code(x.path), ` — ${x.error}`)))));
    }
  }

  function robustnessBlock(it, parent) {
    const c = it.content || {};
    const rows = c.rows || [];
    const speeds = c.speeds || [0.75, 1.0];
    const thr = c.thresholds || {};
    const sec = el("div");
    parent.append(sec);   // in the page first, so the chart can measure its width
    sec.append(el("div", { class: "section-head" }, el("h2", { text: "Floor roughness, floor waves and attitude error" }),
      el("span", { class: "sub", text: `${new Set(rows.map((r) => r.case)).size} cases · thresholds: 16-beam LiDAR ${fmtValue(thr.spin16)}, D455 ${fmtValue(thr.d455)} · constant speed from rest` })));
    // dot chart: phantom stops per 10 m at the first speed (0.75 m/s)
    const sp = speeds.includes(0.75) ? 0.75 : speeds[0];
    const chartCard = el("div", { class: "card" });
    sec.append(chartCard);
    chartCard.append(el("div", { class: "chart-title", text: `Phantom stops per 10 m at ${speedText(sp)}, by case` }),
      el("div", { class: "chart-sub", text: "A phantom stop is a frame where something that is not there gets confirmed inside the stopping distance. Lower is better." }),
      sensorLegend(["spin16", "d455", "fused"]));
    chartCard.append(dotChart(rows, sp, chartCard.clientWidth - 34));
    chartCard.append(sourceLine(it.path, it.mtime));
    // a table per case
    const grid = el("div", { class: "stack-list", style: { marginTop: "12px" } });
    sec.append(grid);
    const cases = [...new Set(rows.map((r) => r.case))];
    for (const cs of cases) {
      const rs = rows.filter((r) => r.case === cs);
      const n = rs[0] && rs[0].n_arenas;
      const dir = rs[0] && rs[0].dir;
      grid.append(el("div", { class: "card" },
        el("h3", { text: cs }),
        el("div", { class: "table-wrap" }, el("table", null,
          el("thead", null,
            el("tr", null, el("th", { rowspan: "2", text: "Sensor" }), el("th", { colspan: "2", class: "grp", text: "False hazards left in the map, per frame" }),
              speeds.map((v) => el("th", { colspan: "2", class: "grp", text: `At ${speedText(v)}` }))),
            el("tr", null, el("th", { class: "num", text: "Boulders" }), el("th", { class: "num", text: "Craters" }),
              speeds.map(() => [el("th", { class: "num", text: "Missed craters · boulders" }), el("th", { class: "num", text: "Phantom stops per 10 m" })]))),
          el("tbody", null, rs.map((r) => el("tr", null,
            el("td", null, sensorKey(r.sensor)),
            el("td", { class: "num", text: fmt(r.map_fa_pos) }),
            el("td", { class: "num", text: fmt(r.map_fa_neg) }),
            speeds.map((v) => {
              const x = r[speedKey(r, v)] || {};
              return [el("td", { class: "num", text: `${missed(x.craters)} · ${missed(x.boulders)}` }), el("td", { class: "num", text: fmt(x.phantom_per_10m) })];
            })))))),
        sourceLine(it.path, it.mtime, ` · ${n ?? "?"} arenas from ${dir ? shortDir(dir) : "?"}`)));
    }
    return sec;
  }
  function shortDir(d) {
    const i = String(d).indexOf("bp1_sandbox/");
    return i >= 0 ? d.slice(i + "bp1_sandbox/".length) : d;
  }

  /** Horizontal dot chart: one row per case, one marker per sensor (color + shape), dodged so equal values stay visible. */
  function dotChart(rows, speed, width) {
    const cases = [...new Set(rows.map((r) => r.case))];
    const sensors = ["spin16", "d455", "fused"].filter((s) => rows.some((r) => r.sensor === s));
    const val = (r) => { const x = r[speedKey(r, speed)]; return x ? x.phantom_per_10m : null; };
    const vals = rows.map(val).filter(isNum);
    const t = niceTicks(0, Math.max(1, ...vals), Math.max(3, Math.floor(width / 140)));
    const W = Math.max(320, Math.floor(width));
    const labelW = Math.max(180, Math.round(W * 0.34));
    const tw = W - labelW - 12;
    const pad = 10;
    const X = (v) => pad + (v / t.hi) * (tw - 2 * pad);
    const rowH = 30;
    const off = { spin16: -8, d455: 0, fused: 8 };
    const wrap = el("div", { class: "dotchart", style: { gridTemplateColumns: `${labelW}px ${tw}px` } });
    for (const cs of cases) {
      wrap.append(el("div", { class: "case", text: cs }));
      const svg = sv("svg", { width: tw, height: rowH, viewBox: `0 0 ${tw} ${rowH}` });
      for (const tk of t.ticks) svg.append(sv("line", { class: "grid-line", x1: X(tk), x2: X(tk), y1: 0, y2: rowH }));
      for (const s of sensors) {
        const r = rows.find((x) => x.case === cs && x.sensor === s);
        if (!r) continue;
        const v = val(r);
        if (!isNum(v)) continue;
        const info = sensorInfo(s);
        const cy = rowH / 2 + off[s];
        const g = sv("g", { class: "mark", tabindex: "0", role: "img", "aria-label": `${cs}, ${info.label}: ${fmt(v)} phantom stops per 10 m` },
          sv("circle", { class: "hit", cx: X(v), cy, r: 10 }),
          shape(info.shape, X(v), cy, 9, { class: "dot f-" + info.cls }));
        bindTip(g, () => tipRows(`${fmt(v)} per 10 m`, info.label, cs));
        svg.append(g);
      }
      wrap.append(el("div", { class: "track" }, svg));
    }
    const ax = sv("svg", { width: tw, height: 22, viewBox: `0 0 ${tw} 22` });
    ax.append(sv("line", { class: "axis-line", x1: 0, x2: tw, y1: 0.5, y2: 0.5 }));
    for (const tk of t.ticks) ax.append(sv("text", { x: X(tk), y: 15, "text-anchor": "middle", text: tickText(tk, t.step) }));
    wrap.append(el("div"), el("div", { class: "axis" }, ax));
    wrap.append(el("div", { class: "axis-label", text: `Phantom stops per 10 m driven at ${speedText(speed)}` }));
    return wrap;
  }

  function speedGateBlock(it) {
    const c = it.content || {};
    const a = c.assumed || {};
    const sec = el("div");
    const tu = c.tune_seeds, te = c.test_seeds;
    sec.append(el("div", { class: "section-head" }, el("h2", { text: "Speed and confirmation thresholds" }),
      el("span", { class: "sub", text: te ? `tested on seeds ${te[0]}–${te[1]} (${te[2]} arenas)` : "" })));
    const top = el("div", { class: "card" });
    sec.append(top);
    top.append(el("div", { class: "chart-title", text: "Chosen confirmation thresholds" }));
    top.append(el("p", { class: "note", text: `How much summed evidence a candidate needs before it counts as a hazard. Picked as the lowest value with at most ${a.PHANTOM_LIMIT ?? "?"} phantom stops per 10 m at ${a.TUNE_SPEED ?? "?"} m/s in dust${tu ? `, on seeds ${tu[0]}–${tu[1]} (${tu[2]} arenas)` : ""}.` }));
    top.append(el("div", { class: "chosen" }, Object.entries(c.chosen || {}).map(([s, v]) =>
      el("div", { class: "tile" }, el("div", { class: "t-label" }, sensorKey(s)), el("div", { class: "t-value", text: fmtValue(v) }), el("div", { class: "t-sub", text: "confirmation threshold" })))));
    if (Object.keys(a).length) {
      top.append(el("p", { class: "note", text: `Assumed robot numbers: reaction time ${a.T_LAT} s, braking ${a.DECEL} m/s², speeding up ${a.ACCEL} m/s², ${a.MARGIN} m kept from the hazard, slow speed ${a.V_SLOW} m/s.` }));
    }
    top.append(sourceLine(it.path, it.mtime));

    // constant-speed table
    const groups = [];
    for (const r of c.constant || []) {
      const key = `${r.sensor}|${r.dust}`;
      let g = groups.find((x) => x.key === key);
      if (!g) { g = { key, sensor: r.sensor, dust: r.dust, rows: [] }; groups.push(g); }
      g.rows.push(r);
    }
    const tbl = el("div", { class: "card" });
    sec.append(tbl);
    tbl.append(el("div", { class: "chart-title", text: "Constant speed from rest: missed hazards and phantom stops" }),
      el("p", { class: "note", text: "A hazard is missed when it is confirmed closer than the stopping distance, or never. Counts are missed / hazards on the path." }));
    tbl.append(el("div", { class: "table-wrap" }, el("table", null,
      el("thead", null, el("tr", null, ["Speed", "Stopping distance", "Mean speed", "Missed craters", "Missed boulders", "Phantom stops per 10 m"].map((t) => el("th", { class: "num", text: t })))),
      groups.map((g) => el("tbody", null,
        el("tr", { class: "group" }, el("td", { colspan: "6" }, sensorKey(g.sensor), ` · ${air(g.dust)}`)),
        g.rows.map((r) => el("tr", null,
          el("td", { class: "num", text: speedText(r.speed) }),
          el("td", { class: "num", text: `${fmt(r.d_stop)} m` }),
          el("td", { class: "num", text: `${fmt(r.mean_speed)} m/s` }),
          el("td", { class: "num", text: missedPct(r.craters) }),
          el("td", { class: "num", text: missedPct(r.boulders) }),
          el("td", { class: "num", text: fmt(r.phantom_per_10m) }))))))));
    tbl.append(sourceLine(it.path, it.mtime));

    if ((c.gate || []).length) {
      const det = el("details", { class: "card" }, el("summary", { text: "Speed gate: drive fast while nothing is unresolved ahead, else slow" }));
      det.append(el("div", { class: "table-wrap" }, el("table", null,
        el("thead", null, el("tr", null, el("th", { text: "Sensor" }), el("th", { text: "Air" }), ["Fast speed", "Mean speed", "Distance driven slow", "Missed hazards (95% range)", "Phantom stops per 10 m"].map((t) => el("th", { class: "num", text: t })))),
        el("tbody", null, c.gate.map((r) => el("tr", null,
          el("td", null, sensorKey(r.sensor)), el("td", { text: air(r.dust) }),
          el("td", { class: "num", text: speedText(r.v_fast) }),
          el("td", { class: "num", text: `${fmt(r.mean_speed)} m/s` }),
          el("td", { class: "num", text: r.slow_share ? `${pct(r.slow_share.any)} (unseen ground ${pct(r.slow_share.unseen)}, unconfirmed ${pct(r.slow_share.candidate)})` : DASH }),
          el("td", { class: "num", text: r.all ? `${r.all.missed}/${r.all.n} (${pct(r.all.lo)}–${pct(r.all.hi)})` : DASH }),
          el("td", { class: "num", text: fmt(r.phantom_per_10m) })))))));
      det.append(sourceLine(it.path, it.mtime));
      sec.append(det);
    }
    for (const n of sec.children) if (n !== sec.firstChild) n.style.marginTop = "12px";
    return sec;
  }

  function selfmaskBlock(it) {
    const c = it.content || {};
    const rows = c.rows || [];
    const m = (x) => (isNum(x) ? `${fmt(x)} m` : DASH);
    const card = el("div", { class: "card" });
    card.append(el("div", { class: "section-head" }, el("h2", { text: "Self-mask, calibration error and ruts" }),
      el("span", { class: "sub", text: `thresholds: ${Object.entries(c.chosen || {}).map(([s, v]) => `${sensorInfo(s).label} ${fmtValue(v)}`).join(", ")} · seeds from ${c.test_from ?? "?"} up` })));
    card.append(el("div", { class: "table-wrap" }, el("table", null,
      el("thead", null, el("tr", null, el("th", { text: "Case" }), el("th", { text: "Sensor" }), el("th", { text: "Air" }),
        ["Approaches", "Body points per frame", "Ground removed per frame", "Blind ahead (median / 90th pct)", "Frames with a phantom on the robot",
          "Phantoms left on path", "Missed at 1.0 m/s", "Phantom stops per 10 m", "Gate mean speed"].map((t) => el("th", { class: "num", text: t })))),
      el("tbody", null, rows.map((r) => el("tr", null,
        el("td", { text: r.case }), el("td", null, sensorKey(r.sensor)), el("td", { text: air(r.dust) }),
        el("td", { class: "num", text: r.n }), el("td", { class: "num", text: fmt(r.body_per_frame) }),
        el("td", { class: "num", text: fmt(r.lost_per_frame) }),
        el("td", { class: "num", text: `${m(r.blind_median)} / ${m(r.blind_p90)}` }),
        el("td", { class: "num", text: pct(r.footprint_phantom_share) }), el("td", { class: "num", text: fmt(r.left_on_path) }),
        el("td", { class: "num", text: missedPct(r.v1) }), el("td", { class: "num", text: fmt(r.v1 && r.v1.phantom_per_10m) }),
        el("td", { class: "num", text: r.gate1 ? `${fmt(r.gate1.mean_speed)} m/s` : DASH })))))));
    card.append(sourceLine(it.path, it.mtime));
    return card;
  }

  function all96Block(it) {
    const rows = Array.isArray(it.content) ? it.content : [];
    const det = el("details", { class: "card" }, el("summary", { text: `Summary over all arenas, constant speed and speed gate (${rows.length} rows)` }));
    det.append(el("div", { class: "table-wrap" }, el("table", null,
      el("thead", null, el("tr", null, el("th", { text: "Sensor" }), el("th", { text: "Air" }), el("th", { text: "Driving" }),
        ["Speed", "Mean speed", "Missed craters", "Missed boulders", "Phantom stops per 10 m", "Driven slow"].map((t) => el("th", { class: "num", text: t })))),
      el("tbody", null, rows.map((r) => el("tr", null,
        el("td", null, sensorKey(r.sensor)), el("td", { text: air(r.dust) }), el("td", { text: r.policy === "gate" ? "speed gate" : "constant" }),
        el("td", { class: "num", text: speedText(r.speed) }), el("td", { class: "num", text: `${fmt(r.mean)} m/s` }),
        el("td", { class: "num", text: `${r.cr}/${r.ncr}` }), el("td", { class: "num", text: `${r.bo}/${r.nbo}` }),
        el("td", { class: "num", text: fmt(r.ph) }), el("td", { class: "num", text: isNum(r.slow) ? pct(r.slow) : DASH })))))));
    det.append(sourceLine(it.path, it.mtime));
    return el("div", null, det);
  }

  function groundText(m) {
    if (!m.ground_fit) return DASH;
    if (m.ground_fit === "plane") return "one plane";
    let s = `local, ${trimNum(100 * (m.ground_sigma || 0))} cm window`;
    if (m.ground_spread) s += `, cells under ${trimNum(100 * m.ground_spread)} cm spread`;
    return s;
  }
  const cm = (x) => (isNum(x) ? `${trimNum(+(100 * x).toFixed(3))} cm` : DASH);

  function approachesBlock(items, roots) {
    const card = el("div", { class: "card" });
    card.append(el("div", { class: "section-head" }, el("h2", { text: "Approach runs" }), el("span", { class: "sub", text: "settings stored with each approaches.json" })));
    card.append(el("div", { class: "table-wrap" }, el("table", null,
      el("thead", null, el("tr", null, el("th", { text: "Source" }), ["Arenas", "Seeds", "Run time"].map((t) => el("th", { class: "num", text: t })),
        ["Floor roughness", "Floor waves", "Pitch error", "Ground fit", "Dust runs"].map((t) => el("th", { text: t })))),
      el("tbody", null, items.map((it) => {
        const m = (it.content && it.content.meta) || {};
        const s0 = m.seed0, n = m.n_arenas;
        return el("tr", null,
          el("td", { class: "small" }, pathNode(it.path)),
          el("td", { class: "num", text: n ?? DASH }),
          el("td", { class: "num", text: isNum(s0) && isNum(n) ? `${s0}–${s0 + n - 1}` : DASH }),
          el("td", { class: "num", text: dur(m.seconds) }),
          el("td", { text: cm(m.surface_rms) }),
          el("td", { text: isNum(m.undulation) ? (m.undulation ? cm(m.undulation) : "none") : DASH }),
          el("td", { text: isNum(m.pitch_err) ? (m.pitch_err ? `${trimNum(m.pitch_err)}°` : "none") : DASH }),
          el("td", { text: groundText(m) }),
          el("td", { text: m.clear_only === true ? "no, clear air only" : m.clear_only === false ? "yes" : DASH }));
      })))));
    card.append(el("p", { class: "source" }, "Source: the ", code("meta"), " part of each file listed in the first column (", code("approaches_meta.json"),
      " is the copy kept in the repo). A dash means the run predates that setting."));
    return card;
  }

  function isaacTagsBlock(it) {
    const c = it.content || {};
    const m = c.meta || {};
    const card = el("div", { class: "card" });
    card.append(el("div", { class: "section-head" }, el("h2", { text: "Start-zone tags in Isaac Sim" }),
      el("span", { class: "sub", text: `${m.layout ?? DASH} layout · ${c.n_poses ?? DASH} start poses · tags ${fmt((m.tag_size ?? NaN) * 100, 2)} cm, ${fmt((m.tag_height ?? NaN) * 100, 0)} cm high · modified ${when(it.mtime)}` })));
    const cm = (x) => (isNum(x) ? fmt(x * 100, 1) : DASH);
    const rows = [];
    for (const [cam, v] of Object.entries(c.cameras || {})) {
      for (const [key, label, turn] of [["isaac_placement_error", "Isaac (tag placement error)", "turn_isaac"],
                                         ["isaac_exact_map", "Isaac (exact tag map)", null],
                                         ["python_prediction_same_poses", "Python prediction", "turn_python"]]) {
        const s = v[key] || {};
        const t = turn ? v[turn] || {} : {};
        rows.push(el("tr", null,
          el("td", { text: `${cam} (${fmt(v.hfov_rad, 1)} rad, ${v.pitch_deg_up > 0 ? "+" : ""}${v.pitch_deg_up} deg)` }),
          el("td", { text: label }),
          el("td", { class: "num", text: isNum(s.seen_frac) ? pct(s.seen_frac, 0) : DASH }),
          el("td", { class: "num", text: `${cm(s.e_xy_median)} / ${cm(s.e_xy_p95)}` }),
          el("td", { class: "num", text: `${fmt(s.e_yaw_median, 1)} / ${fmt(s.e_yaw_p95, 1)}` }),
          el("td", { class: "num", text: s.heading_flips ?? DASH }),
          el("td", { class: "num", text: turn ? `${fmt(t.turn_median_deg, 0)} / ${isNum(t.never_frac) ? pct(t.never_frac, 0) : DASH}` : "" })));
      }
    }
    card.append(el("div", { class: "table-wrap" }, el("table", null,
      el("thead", null, el("tr", null, el("th", { text: "Camera" }), el("th", { text: "Source" }),
        ["Seen at start heading", "xy error median / p95 (cm)", "Heading error median / p95 (deg)", "Heading flips (> 5 deg)",
          "Turn to a tag median (deg) / never"].map((h) => el("th", { class: "num", text: h })))),
      el("tbody", null, rows))));
    card.append(el("p", { class: "source" }, "Source: ", pathNode(it.path), ` (sim/fiducial/isaac_tags_eval.py; detector ${c.detector ?? DASH})`));
    return card;
  }

  // Odometry drift (sim/odometry/results/drift_*/summary.json; isaac_vo_* summaries are shown as their keys)
  function odometrySection(drift, isaacVo) {
    const wrap = el("div", null);
    wrap.append(el("div", { class: "section-head" }, el("h2", { text: "Odometry drift" }),
      el("span", { class: "sub", text: "position and heading error of each estimator, newest results first" })));
    const newest = (a, b) => (b.mtime || 0) - (a.mtime || 0);
    for (const it of [...drift].sort(newest)) wrap.append(odometryDriftBlock(it));
    for (const it of [...isaacVo].sort(newest)) wrap.append(isaacVoBlock(it));
    return wrap;
  }

  function odometryDriftBlock(it) {
    const c = it.content || {};
    const m = c.meta || {};
    const rows = Array.isArray(c.summary) ? c.summary.filter((r) => r && typeof r === "object") : [];
    const folder = String(it.path).split("/").slice(-2, -1)[0] || it.path;
    const seeds = Array.isArray(m.seeds) && m.seeds.length === 2 ? `seeds ${m.seeds[0]}\u2013${m.seeds[1]}` : null;
    const card = el("div", { class: "card" });
    card.append(el("div", { class: "section-head" }, el("h3", { text: folder }),
      el("span", { class: "sub", text: [seeds, m.mount ? `mount ${m.mount}` : null, `modified ${when(it.mtime)}`].filter(Boolean).join(" \u00b7 ") })));
    if (!rows.length) {
      card.append(el("p", { class: "empty", text: "No rows in summary." }));
    } else {
      // group rows by slip, then point, keeping the order they were written in
      const groups = new Map();
      for (const r of rows) {
        const key = `${r.slip ?? DASH}\u0000${r.point ?? DASH}`;
        if (!groups.has(key)) groups.set(key, { slip: r.slip ?? DASH, point: r.point ?? DASH, rows: [] });
        groups.get(key).rows.push(r);
      }
      const tbody = el("tbody", null);
      for (const g of groups.values()) {
        tbody.append(el("tr", { class: "group" }, el("td", { colspan: "6", text: `${g.slip} slip \u00b7 ${g.point}` })));
        for (const r of g.rows) {
          const ci = Array.isArray(r.xy_ci_cm) ? `[${fmt(r.xy_ci_cm[0], 1)}, ${fmt(r.xy_ci_cm[1], 1)}]` : "";
          tbody.append(el("tr", null,
            el("td", { text: r.estimator ?? DASH }),
            el("td", { class: "num", text: r.n ?? DASH }),
            el("td", { class: "num", text: isNum(r.dist_median_m) ? `${fmt(r.dist_median_m, 1)} m` : DASH }),
            el("td", { class: "num", text: `${fmt(r.xy_median_cm, 1)} ${ci}`.trim() }),
            el("td", { class: "num", text: fmt(r.xy_p95_cm, 1) }),
            el("td", { class: "num", text: `${fmt(r.yaw_median_deg, 1)} / ${fmt(r.yaw_p95_deg, 1)}` })));
        }
      }
      card.append(el("div", { class: "table-wrap" }, el("table", null,
        el("thead", null, el("tr", null, el("th", { text: "Estimator" }),
          ["Runs", "Distance", "Position error median [95% CI] (cm)", "Position error p95 (cm)", "Heading median / p95 (deg)"]
            .map((h) => el("th", { class: "num", text: h })))),
        tbody)));
    }
    card.append(sourceLine(it.path, it.mtime, " · values as written in the file: xy_median_cm with its xy_ci_cm, xy_p95_cm, yaw_median_deg, yaw_p95_deg, dist_median_m"));
    return card;
  }

  function isaacVoBlock(it) {
    const c = it.content || {};
    const m = c.meta || {};
    const card = el("div", { class: "card" });
    card.append(el("div", { class: "section-head" }, el("h2", { text: "D455 stereo visual odometry in Isaac Sim" }),
      el("span", { class: "sub", text: `${Object.keys(c.drives || {}).length} drives · ${(m.image || []).join(" x ")}, ${fmt(m.hfov_deg, 0)} deg, baseline ${fmt((m.baseline_m ?? NaN) * 1000, 0)} mm · 15 Hz · modified ${when(it.mtime)}` })));
    const legs = ["dig1", "dump1", "dig2", "end"];
    const cell = (v) => (v ? `${fmt(v.e_xy * 100, 1)} cm / ${fmt(v.e_yaw_deg, 2)}°` : DASH);
    const rows = [];
    for (const [drive, d] of Object.entries(c.drives || {})) {
      for (const [name, est] of Object.entries(d.estimators || {})) {
        rows.push(el("tr", null,
          el("td", { text: drive.replace("_", " · ") }),
          el("td", { text: name }),
          legs.map((l) => el("td", { class: "num", text: cell(est[l]) })),
          el("td", { class: "num", text: `${fmt(d.features_tracked_median, 0)} (min ${d.features_tracked_min ?? DASH})` }),
          el("td", { class: "num", text: `${d.lost_frames ?? DASH} / ${isNum(d.frames) ? d.frames - 1 : DASH}` }),
          el("td", { class: "num", text: fmt(d.path_m, 1) })));
      }
    }
    card.append(el("div", { class: "table-wrap" }, el("table", null,
      el("thead", null, el("tr", null, el("th", { text: "Drive" }), el("th", { text: "Estimator" }),
        ["Dig 1", "Dump 1", "Dig 2", "End", "Tracked features median", "Lost frames", "Path (m)"].map((h) => el("th", { class: "num", text: h })))),
      el("tbody", null, rows))));
    card.append(el("p", { class: "source" }, "Position / heading error at each leg end. vo: camera only; vo_gyro: gyro heading, VO translation. Source: ", pathNode(it.path), " (sim/odometry/vo_eval.py)"));
    return card;
  }

  function recordsBlock(items) {
    const card = el("div", { class: "card" });
    card.append(el("div", { class: "section-head" }, el("h2", { text: "Detection Monte Carlo runs" }), el("span", { class: "sub", text: "records.json from evaluate.py" })));
    card.append(el("div", { class: "table-wrap" }, el("table", null,
      el("thead", null, el("tr", null, el("th", { text: "Source" }), ["Trials", "First seed", "Run time"].map((t) => el("th", { class: "num", text: t })))),
      el("tbody", null, items.map((it) => el("tr", null,
        el("td", { class: "small" }, pathNode(it.path)),
        el("td", { class: "num", text: it.content.n_trials ?? DASH }),
        el("td", { class: "num", text: it.content.seed0 ?? DASH }),
        el("td", { class: "num", text: dur(it.content.seconds) })))))));
    card.append(el("p", { class: "source", text: "Source: the file in the first column (only its counts are read; the records themselves are skipped)." }));
    return card;
  }

  function autonomyRunsBlock(it) {
    const c = it.content || {};
    const what = c.what || DASH;
    const truth = c.truth || {};
    const runs = c.runs || [];
    const card = el("div", { class: "card" });
    const truthStr = Object.entries(truth).map(([k, v]) => `${k}: ${v}`).join(", ") || "(none)";
    card.append(el("div", { class: "section-head" }, el("h2", { text: "Autonomy runs in arena.sh" }),
      el("span", { class: "sub", text: `${what} · Ground truth: ${truthStr}` })));
    const rows = [];
    for (const r of runs) {
      rows.push(el("tr", null,
        el("td", { text: r.run ?? DASH }),
        el("td", { text: r.commit ?? DASH }),
        el("td", { text: r.ended ?? DASH }),
        el("td", { class: "num", text: `${r.goals_reached ?? DASH} / ${r.goals_skipped ?? DASH}` }),
        el("td", { class: "num", text: `${fmt(r.loc_xy_median_cm, 1)} / ${fmt(r.loc_xy_max_cm, 1)}` }),
        el("td", { class: "num", text: fmt(r.true_path_m, 2) }),
        el("td", { class: "num", text: `${r.matched ?? DASH} / ${r.missed ?? DASH} / ${r.false ?? DASH}` }),
        el("td", { class: "num", text: fmt(r.largest_radius_m, 2) }),
        el("td", { class: "num", text: r.detector_ms ?? DASH }),
        el("td", { text: r.note ?? DASH })));
    }
    card.append(el("div", { class: "table-wrap" }, el("table", null,
      el("thead", null, el("tr", null, ["Run", "Commit", "Ended", "Goals reached/skipped", "Localizer xy median/max (cm)", "True path (m)", "Hazards matched/missed/false", "Largest radius (m)", "Detector ms", "Note"].map((h) => el("th", { class: h === "Run" || h === "Commit" || h === "Ended" || h === "Note" ? "" : "num", text: h })))),
      el("tbody", null, rows))));
    card.append(sourceLine(it.path, it.mtime));
    return card;
  }

  function localizerReplaySection(items) {
    const wrap = el("div", null);
    wrap.append(el("div", { class: "section-head" }, el("h2", { text: "Localizer replays (sim/autonomy_diag)" }),
      el("span", { class: "sub", text: "diagnostics from localizer variants, newest files first" })));
    const newest = (a, b) => (b.mtime || 0) - (a.mtime || 0);
    for (const it of [...items].sort(newest)) wrap.append(localizerReplayBlock(it));
    return wrap;
  }

  function localizerReplayBlock(it) {
    const c = it.content || {};
    const card = el("div", { class: "card" });
    const rows = [];

    // Build table rows: one per variant
    for (const [variant_name, variant_data] of Object.entries(c)) {
      if (!variant_data || typeof variant_data !== "object") continue;

      // Format counts as compact "k: v" list
      const counts = variant_data.counts || {};
      const countsStr = Object.entries(counts)
        .map(([k, v]) => `${k}: ${v}`)
        .join(", ") || DASH;

      // Count worsening_fixes
      const n_fixes = (Array.isArray(variant_data.worsening_fixes) ? variant_data.worsening_fixes.length : 0);

      rows.push(el("tr", null,
        el("td", { text: variant_name }),
        el("td", { class: "num", text: fmt(variant_data.median_cm, 2) }),
        el("td", { class: "num", text: fmt(variant_data.p95_cm, 2) }),
        el("td", { class: "num", text: fmt(variant_data.max_cm, 2) }),
        el("td", { class: "small", text: countsStr }),
        el("td", { class: "num", text: n_fixes > 0 ? String(n_fixes) : DASH })));
    }

    card.append(el("div", { class: "section-head" },
      el("h3", null, pathNode(it.path)),
      el("span", { class: "sub", text: `modified ${when(it.mtime)}` })));

    if (rows.length) {
      card.append(el("div", { class: "table-wrap" }, el("table", null,
        el("thead", null, el("tr", null, ["Variant", "Median (cm)", "P95 (cm)", "Max (cm)", "Counts", "Worsening fixes"].map((h) => el("th", { class: h === "Variant" || h === "Counts" ? "" : "num", text: h })))),
        el("tbody", null, rows))));
    }
    card.append(sourceLine(it.path, it.mtime));
    return card;
  }

  // ------------------------------------------------------------------------------------------------------------
  // Planner benchmarks
  // ------------------------------------------------------------------------------------------------------------
  async function loadPlanner() {
    let r;
    try {
      r = await getJSON("/api/planner");
    } catch (e) {
      if (e instanceof ApiError) $("#planner-body").replaceChildren(errorBox(e));
      throw e;
    }
    $("#planner-sub").textContent = `${r.data.items.length} results · checked ${clock(r.data.time)}`;
    if (r.text === state.texts.planner) return;
    state.texts.planner = r.text;
    state.planner = r.data;
    renderPlanner();
  }

  function renderPlanner() {
    const data = state.planner;
    if (!data) return;
    const body = $("#planner-body");
    body.replaceChildren();
    if (!data.items.length) {
      body.append(el("div", { class: "empty" }, "No planner results found. Run compare.py or nav2 benchmarks."));
      return;
    }
    // Group items by (planner, slip) for summary table
    const by_key = {};
    const errors = [];
    for (const it of data.items) {
      if (it.error) {
        errors.push(it);
        continue;
      }
      const key = `${it.planner}|${it.slip}`;
      if (!by_key[key]) by_key[key] = [];
      by_key[key].push(it);
    }
    // Sort by mtime descending (newest first)
    const sorted_keys = Object.keys(by_key).sort((a, b) => {
      const mt_a = Math.max(...by_key[a].map((x) => x.mtime));
      const mt_b = Math.max(...by_key[b].map((x) => x.mtime));
      return mt_b - mt_a;
    });
    // Summary table
    const card = el("div", { class: "card" });
    body.append(card);
    card.append(el("div", { class: "section-head" }, el("h3", { text: "Benchmark summary by planner and slip level" })));
    const summ_rows = [];
    for (const key of sorted_keys) {
      const items_for_key = by_key[key].sort((a, b) => b.mtime - a.mtime);
      const it = items_for_key[0];  // Use the newest item for this key
      summ_rows.push({
        planner: it.planner,
        slip: it.slip,
        missions: it.missions,
        complete: it.complete,
        time_med: it.mission_time_median,
        time_p95: it.mission_time_p95,
        contact: it.legs_with_contact,
        type: it.type,
        items: items_for_key
      });
    }
    // Per-file detail rows (per run for Nav2 files) sit right under their summary row, hidden until it is clicked
    function detailRow(it) {
      const detail_card = el("div", { class: "card" },
        el("div", { class: "section-head" },
          el("h4", { text: it.path }),
          el("span", { class: "sub", text: `Modified ${when(it.mtime)}` })));
      if (it.type === "nav2" && it.runs && it.runs.length) {
        detail_card.append(el("p", { text: `Missions so far: ${it.runs.length}` }));
        detail_card.append(el("div", { class: "table-wrap" }, el("table", null,
          el("thead", null,
            el("tr", null,
              el("th", { text: "Seed" }),
              el("th", { text: "Complete" }),
              el("th", { class: "num", text: "Time (s)" }),
              el("th", { text: "Legs (status, time s, e_xy m, e_yaw deg)" }))),
          el("tbody", null, it.runs.map((run) =>
            el("tr", null,
              el("td", { text: run.seed }),
              el("td", { text: run.complete ? "Yes" : "No", class: run.complete ? "" : "bad" }),
              el("td", { class: "num", text: fmt(run.time, 1) }),
              el("td", { text: (run.legs || []).map((l) =>
                `${l.status ?? "no plan"} ${fmt(l.time, 0)}/${fmt(l.e_xy, 2)}/${fmt(l.e_yaw, 0)}`).join(" · ") })))))));
      } else {
        detail_card.append(el("dl", null,
          it.plan_s_median != null ? [el("dt", { text: "Plan time median (s)" }), el("dd", { class: "num", text: fmt(it.plan_s_median, 2) })] : [],
          it.plan_s_p95 != null ? [el("dt", { text: "Plan time 95th pct (s)" }), el("dd", { class: "num", text: fmt(it.plan_s_p95, 2) })] : [],
          it.dump_on_target != null ? [el("dt", { text: "Dumps on target" }), el("dd", { class: "num", text: pct(it.dump_on_target) })] : []));
      }
      return el("tr", { class: "detail-row", hidden: true }, el("td", { colspan: "8" }, detail_card));
    }
    const toggle = (e) => {
      for (let d = e.currentTarget.nextElementSibling; d && d.classList.contains("detail-row"); d = d.nextElementSibling)
        d.hidden = !d.hidden;
    };
    card.append(el("div", { class: "table-wrap" }, el("table", null,
      el("thead", null,
        el("tr", null,
          el("th", { text: "Planner" }),
          el("th", { text: "Slip" }),
          el("th", { class: "num", text: "Missions" }),
          el("th", { class: "num", text: "Complete" }),
          el("th", { class: "num", text: "Success %" }),
          el("th", { class: "num", text: "Median time (s)" }),
          el("th", { class: "num", text: "95th pct time (s)" }),
          el("th", { class: "num", text: "Legs with contact" }))),
      el("tbody", null, summ_rows.map((r) => [
        el("tr", { class: "clickable", tabindex: "0", onclick: toggle },
          el("td", { text: r.planner }),
          el("td", { text: r.slip }),
          el("td", { class: "num", text: r.missions }),
          el("td", { class: "num", text: r.complete }),
          el("td", { class: "num", text: r.missions ? pct(r.complete / r.missions, 0) : DASH }),
          el("td", { class: "num", text: fmt(r.time_med, 1) }),
          el("td", { class: "num", text: fmt(r.time_p95, 1) }),
          el("td", { class: "num", text: r.contact ?? DASH })),
        r.items.map(detailRow)])))));
    if (errors.length) {
      body.append(el("div", { class: "card" }, el("h3", { text: "Files that could not be read" }),
        el("ul", null, errors.map((x) => el("li", null, code(x.path), ` — ${x.error}`)))));
    }
  }

  // ------------------------------------------------------------------------------------------------------------
  // Parameters
  // ------------------------------------------------------------------------------------------------------------
  const TABLE_NAMES = {
    ARENA: "Arena", OBSTACLES: "Boulders and craters", BP1: "BP-1 regolith", MOUNTS: "Sensor mounts", D455: "D455 depth camera",
    LIDAR_GENERIC: "LiDAR (generic)", LIDAR_PROFILES: "LiDAR scan patterns", DUST: "Dust plume", DETECT: "Hazard detector",
  };
  async function loadParams() {
    let r;
    try {
      r = await getJSON("/api/params");
    } catch (e) {
      if (e instanceof ApiError) $("#params-body").replaceChildren(errorBox(e));
      throw e;
    }
    if (r.text === state.texts.params) return;
    state.texts.params = r.text;
    state.params = r.data;
    renderParams();
  }
  function renderParams() {
    const p = state.params;
    if (!p) return;
    $("#params-sub").textContent = `${p.total} parameters in ${p.tables.length} tables`;
    const statuses = Object.keys(p.counts).filter((s) => p.counts[s] > 0 || ["Verified", "Inferred", "Assumed"].includes(s));
    const tile = (s, label, n, icon) => el("button", {
      class: "tile", type: "button", "aria-pressed": String(state.paramStatus === s),
      onclick: () => { state.paramStatus = state.paramStatus === s ? null : s; renderParams(); },
    }, el("div", { class: "t-label" }, icon, label), el("div", { class: "t-value", text: String(n) }), el("div", { class: "t-sub", text: isNum(n) && p.total ? pct(n / p.total) + " of all" : "" }));
    $("#params-tiles").replaceChildren(
      tile(null, "All parameters", p.total, null),
      ...statuses.map((s) => tile(s, "", p.counts[s], paramChip(s))));
    const q = state.paramText.trim().toLowerCase();
    const match = (e, tname) => (!state.paramStatus || e.status === state.paramStatus) &&
      (!q || [e.name, fmtValue(e.value), e.unit, e.source, e.status, tname, TABLE_NAMES[tname]].some((x) => String(x || "").toLowerCase().includes(q)));
    let shown = 0;
    const bodies = [];
    for (const t of p.tables) {
      const es = t.entries.filter((e) => match(e, t.name));
      if (!es.length) continue;
      shown += es.length;
      bodies.push(el("tbody", null,
        el("tr", { class: "group" }, el("td", { colspan: "5" }, TABLE_NAMES[t.name] || t.name, " ", el("span", { class: "muted mono", text: t.name }))),
        es.map((e) => el("tr", null,
          el("td", { class: "mono", text: e.name }),
          el("td", { class: "mono wrap-any", text: fmtValue(e.value) }),
          el("td", { class: "nowrap", text: e.unit || "" }),
          el("td", null, paramChip(e.status)),
          el("td", { class: "wrap-any small" }, linkify(e.source))))));
    }
    const body = $("#params-body");
    body.replaceChildren(el("div", { class: "card" },
      el("p", { class: "note", text: `Showing ${shown} of ${p.total}` + (state.paramStatus ? ` · status ${state.paramStatus}` : "") + (q ? ` · matching “${state.paramText.trim()}”` : "") }),
      el("div", { class: "table-wrap" }, el("table", null,
        el("colgroup", null, el("col", { style: { width: "16%" } }), el("col", { style: { width: "16%" } }), el("col", { style: { width: "9%" } }), el("col", { style: { width: "11%" } }), el("col")),
        el("thead", null, el("tr", null, ["Name", "Value", "Unit", "Status", "Source"].map((t) => el("th", { text: t })))),
        bodies.length ? bodies : el("tbody", null, el("tr", null, el("td", { colspan: "5", class: "muted", text: "Nothing matches the filter." }))))),
      sourceLine(p.file, p.mtime)));
  }
  $("#params-filter").addEventListener("input", debounce((ev) => { state.paramText = ev.target.value; renderParams(); }, 120));

  // ------------------------------------------------------------------------------------------------------------
  // Research
  // ------------------------------------------------------------------------------------------------------------
  async function loadResearch() {
    let r;
    try {
      r = await getJSON("/api/research");
    } catch (e) {
      if (e instanceof ApiError) $("#research-body").replaceChildren(errorBox(e));
      throw e;
    }
    if (r.text === state.texts.research) return;
    state.texts.research = r.text;
    state.research = r.data;
    renderResearch();
  }
  function renderResearch() {
    const d = state.research;
    if (!d) return;
    const body = $("#research-body");
    body.replaceChildren();
    if (!d.checks || !d.checks.length) {
      body.append(el("div", { class: "empty" }, el("strong", { text: "No research index yet. " }), "Build it with ", code("python3 sim/dashboard/make_research_index.py"), "."));
      return;
    }
    $("#research-sub").textContent = `${d.files.length} research files · ${d.checks.length} check files · index built ${d.generated ? d.generated.replace("T", " ").slice(0, 16) : "?"}`;
    const labels = d.labels || {};
    const allCats = d.categories || [];
    const used = allCats.filter((c) => (d.totals.overall || {})[c] > 0 || ["confirmed", "mismatch", "not_found", "could_not_open", "not_checked"].includes(c));
    const ov = d.totals.overall || {};
    // tiles
    body.append(el("div", { class: "tiles" },
      el("div", { class: "tile" }, el("div", { class: "t-label", text: "Claim rows checked" }), el("div", { class: "t-value", text: String(ov.rows ?? DASH) }), el("div", { class: "t-sub", text: `in ${ov.files ?? "?"} research files` })),
      used.map((c) => el("div", { class: "tile" }, el("div", { class: "t-label" }, el("span", { class: "sw bg-" + c, style: { width: "10px", height: "10px", borderRadius: "3px", display: "inline-block" } }), labels[c] || c),
        el("div", { class: "t-value", text: String(ov[c] ?? 0) }), el("div", { class: "t-sub", text: ov.rows ? pct((ov[c] || 0) / ov.rows, 1) + " of rows" : "" })))));
    const legend = el("div", { class: "legend" }, used.map((c) => el("span", { class: "item" }, el("span", { class: "sw bg-" + c }), labels[c] || c)));
    const numCells = (row) => used.map((c) => el("td", { class: "num", text: isNum(row[c]) ? String(row[c]) : DASH, title: isNum(row[c]) ? null : "not reported in this check file" }));
    const subtotal = (c) => {   // a check file's totals, with a dash where none of its rows report that column
      const t = Object.assign({}, c.totals);
      for (const k of used) if (c.rows.every((r) => !isNum(r[k]))) t[k] = null;
      return t;
    };
    const head = () => el("thead", null, el("tr", null, el("th", { text: "File" }), el("th", { text: "Share of rows" }), el("th", { class: "num", text: "Rows" }), used.map((c) => el("th", { class: "num", text: labels[c] || c }))));

    // totals
    const scopes = Object.keys(d.totals).filter((k) => k !== "overall").sort();
    const totalsCard = el("div", { class: "card" }, el("div", { class: "chart-title", text: "Totals per step and overall" }), legend.cloneNode(true),
      el("div", { class: "table-wrap" }, el("table", null, head(),
        el("tbody", null, [...scopes, "overall"].map((k) => {
          const t = d.totals[k];
          const name = k === "overall" ? "All steps" : k.replace(/^step/, "Step ");
          return el("tr", { class: k === "overall" ? "total" : null }, el("td", { class: "nowrap", text: name }), el("td", { class: "bar-cell" }, stackBar(t, used, labels, name)), el("td", { class: "num", text: String(t.rows) }), numCells(t));
        })))),
      el("p", { class: "source" }, "Source: ", code(d.index_file || "sim/dashboard/research_index.json"), ", built from ", code(d.root || "?")));
    body.append(totalsCard);
    // rebind tooltips on the cloned legend is not needed (legend has none)

    // per check file
    const perCard = el("div", { class: "card", style: { marginTop: "12px" } }, el("div", { class: "chart-title", text: "Per check file" }), legend,
      el("div", { class: "table-wrap" }, el("table", null, head(),
        d.checks.map((c) => el("tbody", null,
          el("tr", { class: "group" }, el("td", null, el("span", { class: "mono", text: c.file }), el("div", { class: "small muted", text: [c.step.replace(/^step/, "Step "), c.title].filter(Boolean).join(" \u00b7 ") })),
            el("td", { class: "bar-cell" }, c.totals ? stackBar(c.totals, used, labels, c.file) : null),
            el("td", { class: "num", text: c.totals ? String(c.totals.rows) : DASH }), c.totals ? numCells(subtotal(c)) : used.map(() => el("td"))),
          c.rows.map((r) => el("tr", null,
            el("td", { class: "mono small", text: r.research_file ? r.research_file.split("/").pop() : r.file }),
            el("td", { class: "bar-cell" }, stackBar(r, used, labels, r.file)),
            el("td", { class: "num", text: String(r.rows) }), numCells(r))),
          (c.warnings || []).length ? el("tr", null, el("td", { colspan: String(3 + used.length), class: "note", text: "Note: " + c.warnings.join("; ") })) : null)))),
      el("p", { class: "source" }, "Source: the “Result counts” table of each check file named above, under ", code(d.root || "?"), ". A dash means that check file's table has no such column."));
    body.append(perCard);

    // research files
    const byFile = {};
    for (const c of d.checks) for (const r of c.rows) if (r.research_file) byFile[r.research_file] = r;
    const filesCard = el("div", { class: "card", style: { marginTop: "12px" } }, el("div", { class: "chart-title", text: "Research files" }),
      el("div", { class: "table-wrap" }, el("table", null,
        el("thead", null, el("tr", null, el("th", { text: "Step" }), el("th", { text: "File and title" }), el("th", { class: "num", text: "Size" }), el("th", { text: "Check result" }))),
        el("tbody", null, d.files.map((f) => {
          const r = byFile[f.path];
          return el("tr", null,
            el("td", { class: "nowrap", text: f.step.replace(/^step/, "Step ") }),
            el("td", null, el("div", { class: "mono small", text: f.file }), el("div", { text: f.title || "(no title)" }),
              f.check_note ? el("div", { class: "note", text: f.check_note }) : el("div", { class: "note", text: "No check note at the top of this file." })),
            el("td", { class: "num", text: bytes(f.size, 1) }),
            el("td", { class: "small" }, r ? [el("div", null, `${r.confirmed ?? "?"} of ${r.rows} rows confirmed`), el("div", { class: "muted" }, `${r.mismatch ?? 0} mismatch, ${r.not_found ?? 0} not found`)] : el("span", { class: "muted", text: "not in a check file" }),
              f.check_file ? el("div", { class: "mono muted small", text: f.check_file.split("/").pop() }) : null));
        })))),
      el("p", { class: "source" }, "Source: ", code(d.index_file || "sim/dashboard/research_index.json"), " (first heading, size and first-line check note of each file)."));
    body.append(filesCard);
  }

  // ------------------------------------------------------------------------------------------------------------
  // start
  // ------------------------------------------------------------------------------------------------------------
  window.addEventListener("hashchange", route);
  document.addEventListener("visibilitychange", () => { if (!document.hidden) schedule(); });
  window.addEventListener("resize", debounce(() => {
    if (current === "results") renderResults();
    else if (current === "runs") renderRunDetail();
  }, 200));
  route();
})();
