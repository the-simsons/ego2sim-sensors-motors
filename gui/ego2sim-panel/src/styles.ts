// Panel styles. Colors are CSS variables so the panel follows Foxglove's light/dark theme.
export const PANEL_CSS = `
.e2s {
  --bg: #15151a; --card: #1f1f27; --border: #34343f; --text: #e6e6ea; --muted: #9a9aa6;
  --accent: #6f8cff; --ok: #4cc38a; --warn: #e5a93a; --error: #f06a6a; --input: #2a2a34;
  box-sizing: border-box; height: 100%; overflow: auto; padding: 12px;
  background: var(--bg); color: var(--text);
  font: 13px/1.4 system-ui, -apple-system, "Segoe UI", Ubuntu, sans-serif;
}
.e2s-light {
  --bg: #f6f6f8; --card: #ffffff; --border: #dcdce3; --text: #1d1d24; --muted: #6b6b78;
  --accent: #3f5fe0; --ok: #1f8f5a; --warn: #a86a00; --error: #c93a3a; --input: #f0f0f4;
}
.e2s *, .e2s *::before, .e2s *::after { box-sizing: border-box; }
.e2s-header { display: flex; flex-wrap: wrap; gap: 12px; align-items: center;
  justify-content: space-between; margin-bottom: 12px; }
.e2s-title { font-size: 16px; font-weight: 600; }
.e2s-mode { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
.e2s-segmented { display: inline-flex; border: 1px solid var(--border); border-radius: 6px;
  overflow: hidden; }
.e2s-segmented button { border: 0; border-radius: 0; padding: 6px 14px; }
.e2s-segmented button.active { background: var(--accent); color: #fff; }
.e2s-banner { padding: 8px 10px; border-radius: 6px; margin-bottom: 10px; border: 1px solid; }
.e2s-banner.error { border-color: var(--error); color: var(--error); }
.e2s-banner.warn { border-color: var(--warn); color: var(--warn); }
.e2s-sensors { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
  gap: 8px; margin-bottom: 10px; }
.e2s-motors { display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr));
  gap: 8px; margin-bottom: 10px; }
.e2s-lower { display: grid; grid-template-columns: repeat(auto-fill, minmax(320px, 1fr));
  gap: 8px; }
.e2s-card { background: var(--card); border: 1px solid var(--border); border-radius: 8px;
  padding: 10px; min-width: 0; }
.e2s-card-title { font-weight: 600; }
.e2s-subtitle { margin-top: 8px; font-weight: 600; color: var(--muted); }
.e2s-sensor-head, .e2s-motor-head { display: flex; flex-wrap: wrap; gap: 6px;
  align-items: center; justify-content: space-between; }
.e2s-motor-head { justify-content: flex-start; }
.e2s-motor-head .e2s-card-title { margin-right: auto; }
.e2s-sensor-value { font-size: 24px; font-weight: 600; font-variant-numeric: tabular-nums;
  margin: 4px 0 2px; }
.e2s-unit { font-size: 13px; font-weight: 400; margin-left: 4px; color: var(--muted); }
.e2s-readout { display: grid; grid-template-columns: auto 1fr; gap: 2px 10px; margin: 8px 0; }
.e2s-readout dt { color: var(--muted); }
.e2s-readout dd { margin: 0; font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }
.e2s-controls { display: flex; flex-direction: column; gap: 6px; padding: 8px 0;
  border-top: 1px solid var(--border); }
.e2s-row { display: flex; flex-wrap: wrap; gap: 6px; align-items: center; }
.e2s-field { display: flex; gap: 6px; align-items: center; flex: 1 1 auto; }
.e2s-field input { width: 80px; }
.e2s-slider { width: 100%; accent-color: var(--accent); }
.e2s button { background: var(--input); color: var(--text); border: 1px solid var(--border);
  border-radius: 6px; padding: 4px 10px; font: inherit; cursor: pointer; }
.e2s button:hover:not(:disabled) { border-color: var(--accent); }
.e2s button:disabled { opacity: 0.45; cursor: not-allowed; }
.e2s input[type="number"] { background: var(--input); color: var(--text);
  border: 1px solid var(--border); border-radius: 6px; padding: 4px 6px; font: inherit; }
.e2s .e2s-stop { width: 100%; margin-top: 8px; padding: 8px; font-weight: 700;
  background: var(--error); border-color: var(--error); color: #fff; }
.e2s-result { padding: 6px 8px; margin-bottom: 4px; border-radius: 6px; background: var(--input); }
.e2s-pill { display: inline-block; padding: 1px 7px; border-radius: 999px; font-size: 11px;
  border: 1px solid var(--border); color: var(--muted); }
.e2s-level-0 { color: var(--ok); border-color: var(--ok); }
.e2s-level-1 { color: var(--warn); border-color: var(--warn); }
.e2s-level-2 { color: var(--error); border-color: var(--error); }
.e2s-level-3 { color: var(--muted); }
.e2s-tone-ok { color: var(--ok); }
.e2s-tone-warn { color: var(--warn); }
.e2s-tone-error { color: var(--error); }
.e2s-tone-missing { color: var(--muted); }
.e2s-muted { color: var(--muted); }
.e2s-small { font-size: 12px; }
.e2s-mono { font-family: ui-monospace, "SF Mono", "Ubuntu Mono", monospace; }
.e2s-table { width: 100%; border-collapse: collapse; margin-top: 6px; }
.e2s-table td { padding: 3px 6px 3px 0; border-top: 1px solid var(--border); }
.e2s-diag { display: flex; flex-wrap: wrap; gap: 6px; align-items: baseline; padding: 3px 0; }
.e2s-diag-name { font-weight: 600; }
.e2s-kill { display: flex; gap: 12px; align-items: center; justify-content: space-between;
  padding: 10px 12px; margin-bottom: 10px; border-radius: 8px; border: 2px solid; }
.e2s-kill.armed { border-color: var(--ok); }
.e2s-kill.killed { border-color: var(--error); background: color-mix(in srgb, var(--error) 15%, transparent); }
.e2s-kill-state { font-size: 16px; font-weight: 700; }
.e2s-kill.armed .e2s-kill-state { color: var(--ok); }
.e2s-kill.killed .e2s-kill-state { color: var(--error); }
.e2s .e2s-kill-btn { padding: 10px 22px; font-weight: 800; font-size: 15px; color: #fff; }
.e2s .e2s-kill-btn.kill { background: var(--error); border-color: var(--error); }
.e2s .e2s-kill-btn.release { background: var(--ok); border-color: var(--ok); }
`;
