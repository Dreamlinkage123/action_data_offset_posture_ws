/**
 * 前端占位：拉取 /api/state 填充展示；按钮调用占位接口。
 * 后续与真实标定逻辑对接时替换。
 */

const STEP_SPIN = 0.5;
const STEP_MAX = 10;
const STEP_MIN = 0;
/** 步进值未下发或与后端约定时的默认展示 */
const STEP_DEFAULT = "1";

/** 姿态示教角度（度）：0.1°～10°，按钮微调 0.1° */
const ROT_STEP_SPIN = 0.1;
const ROT_STEP_MAX = 10;
const ROT_STEP_MIN = 0.1;
const ROT_STEP_DEFAULT = "0.1";

function clampStep(n) {
  return Math.min(STEP_MAX, Math.max(STEP_MIN, n));
}

/** 步进值展示：最大 10，大于 10 时显示 10 */
function formatStepDisplay(n) {
  const c = clampStep(n);
  if (c >= STEP_MAX) return String(STEP_MAX);
  if (c === 0) return "0";
  return c % 1 === 0 ? String(c) : c.toFixed(1);
}

function clampRotStepDeg(n) {
  const c = Math.min(ROT_STEP_MAX, Math.max(ROT_STEP_MIN, n));
  return Math.round(c * 10) / 10;
}

function formatRotStepDisplay(n) {
  const c = clampRotStepDeg(Number.isNaN(n) ? parseFloat(ROT_STEP_DEFAULT) : n);
  return c % 1 === 0 ? String(c) : c.toFixed(1);
}

function parseStepInputString(s) {
  const t = String(s ?? "").trim().replace(",", ".");
  const n = parseFloat(t);
  return Number.isNaN(n) ? NaN : n;
}

/** 页面居中弹窗，替代原生 alert / confirm */
const AppModal = (function () {
  let onResult = null;
  let modalMode = "alert";
  let keyHandler = null;

  function teardown(confirmValue) {
    const root = document.getElementById("app-modal-root");
    if (root) root.hidden = true;
    if (keyHandler) {
      document.removeEventListener("keydown", keyHandler);
      keyHandler = null;
    }
    const fn = onResult;
    const m = modalMode;
    onResult = null;
    if (!fn) return;
    if (m === "confirm") fn(confirmValue === true);
    else fn();
  }

  function init() {
    const ok = document.getElementById("app-modal-ok");
    const cancel = document.getElementById("app-modal-cancel");
    const backdrop = document.querySelector("#app-modal-root .app-modal-backdrop");
    if (!ok || !cancel || !backdrop) return;
    ok.addEventListener("click", () => {
      if (modalMode === "confirm") teardown(true);
      else teardown();
    });
    cancel.addEventListener("click", () => teardown(false));
    backdrop.addEventListener("click", () => {
      if (modalMode === "confirm") teardown(false);
      else teardown();
    });
  }

  function alert(message) {
    return new Promise((resolve) => {
      modalMode = "alert";
      onResult = resolve;
      const root = document.getElementById("app-modal-root");
      const msgEl = document.getElementById("app-modal-message");
      const cancelBtn = document.getElementById("app-modal-cancel");
      const okBtn = document.getElementById("app-modal-ok");
      if (!root || !msgEl || !cancelBtn || !okBtn) {
        resolve();
        return;
      }
      msgEl.textContent = String(message ?? "");
      cancelBtn.hidden = true;
      okBtn.textContent = "确定";
      root.hidden = false;
      keyHandler = (e) => {
        if (e.key === "Enter" || e.key === "Escape") {
          e.preventDefault();
          teardown();
        }
      };
      document.addEventListener("keydown", keyHandler);
    });
  }

  function confirm(message) {
    return new Promise((resolve) => {
      modalMode = "confirm";
      onResult = resolve;
      const root = document.getElementById("app-modal-root");
      const msgEl = document.getElementById("app-modal-message");
      const cancelBtn = document.getElementById("app-modal-cancel");
      const okBtn = document.getElementById("app-modal-ok");
      if (!root || !msgEl || !cancelBtn || !okBtn) {
        resolve(false);
        return;
      }
      msgEl.textContent = String(message ?? "");
      cancelBtn.hidden = false;
      okBtn.textContent = "确定";
      root.hidden = false;
      keyHandler = (e) => {
        if (e.key === "Escape") {
          e.preventDefault();
          teardown(false);
        } else if (e.key === "Enter") {
          e.preventDefault();
          teardown(true);
        }
      };
      document.addEventListener("keydown", keyHandler);
    });
  }

  return { init, alert, confirm };
})();

async function modalAlert(message) {
  return AppModal.alert(message);
}

async function modalConfirm(message) {
  return AppModal.confirm(message);
}

function setPanelValues(arm, axis, row) {
  const panel = document.querySelector(`.arm-panel[data-arm="${arm}"]`);
  if (!panel) return;
  const block = panel.querySelector(`.axis-block[data-axis="${axis}"]`);
  if (!block) return;
  if (!row || typeof row !== "object") return;
  // 步进仅客户端维护，勿用 /api/state 的 step 覆盖（轮询会重置为 1）
  block.querySelector(".inp-current").textContent = row.current ?? "";
  block.querySelector(".inp-initial").textContent = row.initial ?? "";
  block.querySelector(".inp-delta").textContent = row.delta ?? "";
}

function setRotAxisValues(arm, axis, row) {
  const panel = document.querySelector(`.arm-panel[data-arm="${arm}"]`);
  if (!panel) return;
  const block = panel.querySelector(`.axis-block[data-rot-axis="${axis}"]`);
  if (!block) return;
  if (!row || typeof row !== "object") return;
  const cur = block.querySelector(".inp-current");
  const ini = block.querySelector(".inp-initial");
  const del = block.querySelector(".inp-delta");
  if (cur) cur.textContent = row.current ?? "";
  if (ini) ini.textContent = row.initial ?? "";
  if (del) del.textContent = row.delta ?? "";
}

function wireStepControls() {
  document.querySelectorAll(".step-input-wrap").forEach((wrap) => {
    const inp = wrap.querySelector(".inp-step");
    const dec = wrap.querySelector(".btn-step-dec");
    const inc = wrap.querySelector(".btn-step-inc");
    if (!inp || !dec || !inc) return;

    const isDeg = wrap.classList.contains("step-input-wrap--deg");
    const spin = (() => {
      if (isDeg) {
        const raw = parseFloat(wrap.getAttribute("data-step-spin") || "");
        return Number.isNaN(raw) ? ROT_STEP_SPIN : raw;
      }
      return STEP_SPIN;
    })();

    const applyFromInput = () => {
      const v = parseStepInputString(inp.value);
      if (Number.isNaN(v)) {
        inp.value = isDeg ? ROT_STEP_DEFAULT : STEP_DEFAULT;
        return;
      }
      inp.value = isDeg ? formatRotStepDisplay(v) : formatStepDisplay(v);
    };

    const applyDelta = (delta) => {
      const v = parseStepInputString(inp.value);
      const defNum = isDeg ? parseFloat(ROT_STEP_DEFAULT) || 0.1 : parseFloat(STEP_DEFAULT) || 1;
      const base = Number.isNaN(v) ? defNum : v;
      inp.value = isDeg ? formatRotStepDisplay(base + delta) : formatStepDisplay(base + delta);
    };

    dec.addEventListener("click", () => applyDelta(-spin));
    inc.addEventListener("click", () => applyDelta(spin));
    inp.addEventListener("blur", applyFromInput);
    inp.addEventListener("change", applyFromInput);
    inp.addEventListener("keydown", (e) => {
      if (e.key === "Enter") {
        e.preventDefault();
        inp.blur();
      }
    });
  });
}

function updateEulerOffsetBanner(_d) {
  /* 顶部外旋欧拉角 banner 已移除；保留空壳以避免调用点空引用 */
}

async function refreshState() {
  try {
    const r = await fetch("/api/state");
    const j = await r.json();
    if (!j.ok || !j.data) return;
    const d = j.data;
    updateEulerOffsetBanner(d);
    ["left", "right"].forEach((arm) => {
      ["x", "y", "z"].forEach((ax) => {
        setPanelValues(arm, ax, d[arm][ax]);
      });
      ["rx", "ry", "rz"].forEach((ax) => {
        setRotAxisValues(arm, ax, d[arm][ax]);
      });
    });
  } catch (e) {
    console.warn("refreshState failed", e);
  }
}

function wireRotAdjustButtons() {
  document.querySelectorAll(".arm-panel").forEach((panel) => {
    const arm = panel.getAttribute("data-arm");
    panel.querySelectorAll(".axis-block[data-rot-axis]").forEach((block) => {
      const axis = block.getAttribute("data-rot-axis");
      if (!axis) return;
      block.querySelectorAll(".btn-inc").forEach((btn) => {
        btn.addEventListener("click", async () => {
          const direction = btn.getAttribute("data-dir");
          const stepInp = block.querySelector(".inp-step-deg");
          let step = 0.1;
          if (stepInp) {
            const v = parseStepInputString(stepInp.value);
            step = Number.isNaN(v) ? 0.1 : clampRotStepDeg(v);
          }
          try {
            const r = await fetch(`/api/arm/${arm}/rotate/${axis}`, {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({ direction, step }),
            });
            const j = await r.json().catch(() => ({}));
            if (!r.ok || j.ok === false) {
              throw new Error(j.message || r.statusText || "姿态示教失败");
            }
            await refreshState();
          } catch (e) {
            console.warn("rotate adjust failed", e);
            await modalAlert(e.message || String(e));
          }
        });
      });
    });
  });
}

function wireAdjustButtons() {
  document.querySelectorAll(".arm-panel").forEach((panel) => {
    const arm = panel.getAttribute("data-arm");
    panel.querySelectorAll(".axis-block[data-axis]").forEach((block) => {
      const axis = block.getAttribute("data-axis");
      block.querySelectorAll(".btn-inc").forEach((btn) => {
        btn.addEventListener("click", async () => {
          const direction = btn.getAttribute("data-dir");
          const stepInp = block.querySelector(".inp-step");
          let step = 1;
          if (stepInp) {
            const v = parseStepInputString(stepInp.value);
            step = Number.isNaN(v) ? 1 : clampStep(v);
          }
          try {
            const r = await fetch(`/api/arm/${arm}/axis/${axis}/adjust`, {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({ direction, step }),
            });
            const j = await r.json().catch(() => ({}));
            if (!r.ok || j.ok === false) {
              throw new Error(j.message || r.statusText || "示教失败");
            }
            await refreshState();
          } catch (e) {
            console.warn("adjust failed", e);
            await modalAlert(e.message || String(e));
          }
        });
      });
    });
  });
}

const INSTRUMENT_LABELS = {
  drum: "架子鼓",
  bass: "贝斯",
  guitar: "吉他",
  keyboard: "电子琴",
};

/** 后端仍使用 instruments 数组，单选时长度为 0 或 1 */
function getSelectedInstruments() {
  const root = document.getElementById("instrument-dropdown");
  if (!root) return [];
  const el = root.querySelector('input[name="instrument"]:checked');
  return el ? [el.value] : [];
}

function updateInstrumentDropdownLabel() {
  const span = document.getElementById("instrument-dropdown-label");
  if (!span) return;
  const selected = getSelectedInstruments();
  if (selected.length === 0) {
    span.textContent = "请选择一种乐器";
    return;
  }
  span.textContent = INSTRUMENT_LABELS[selected[0]] || selected[0];
}

function wireInstrumentDropdown() {
  const root = document.getElementById("instrument-dropdown");
  const toggle = document.getElementById("instrument-dropdown-toggle");
  const panel = document.getElementById("instrument-dropdown-panel");
  if (!root || !toggle || !panel) return;

  const setOpen = (open) => {
    root.classList.toggle("is-open", open);
    panel.hidden = !open;
    toggle.setAttribute("aria-expanded", open ? "true" : "false");
  };

  toggle.addEventListener("click", (e) => {
    e.stopPropagation();
    setOpen(panel.hidden);
  });

  document.addEventListener("click", (e) => {
    if (!root.contains(e.target)) setOpen(false);
  });

  root.querySelectorAll('input[name="instrument"]').forEach((inp) => {
    inp.addEventListener("change", () => {
      updateInstrumentDropdownLabel();
      setOpen(false);
    });
  });

  updateInstrumentDropdownLabel();
}

async function postJson(url, body) {
  const r = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const j = await r.json().catch(() => ({}));
  if (!r.ok || j.ok === false) {
    const msg = j.message || r.statusText || "请求失败";
    throw new Error(msg);
  }
  return j;
}

/** 轮询直到标定/播放轨迹空闲（用于「结束标定」播完后恢复按钮状态） */
async function waitForCalibrationIdle(maxMs = 600000, intervalMs = 300) {
  await new Promise((res) => setTimeout(res, 100));
  const t0 = Date.now();
  for (;;) {
    const r = await fetch("/api/calibration/status");
    const j = await r.json().catch(() => ({}));
    if (j.ok === true && j.playing === false) return;
    if (Date.now() - t0 > maxMs) {
      throw new Error("等待轨迹播放结束超时，请检查机器人或网络后重试");
    }
    await new Promise((res) => setTimeout(res, intervalMs));
  }
}

function setCalibrationButtonsDisabled(playing) {
  const start = document.getElementById("btn-calib-start");
  const end = document.getElementById("btn-calib-end");
  const debugOff = document.getElementById("btn-upper-body-debug-off");
  const hint = playing ? "标定轨迹播放中，请等待播完后再点" : "";
  if (start) {
    start.disabled = playing;
    start.title = hint;
  }
  if (end) {
    end.disabled = playing;
    end.title = hint;
  }
  if (debugOff) {
    debugOff.disabled = playing;
    debugOff.title = hint;
  }
}

async function refreshCalibrationPlaybackStatus() {
  try {
    const r = await fetch("/api/calibration/status");
    const j = await r.json();
    if (j.ok && typeof j.playing === "boolean") {
      setCalibrationButtonsDisabled(j.playing);
    }
  } catch (_) {
    /* 忽略轮询失败 */
  }
}

document.getElementById("btn-calib-start")?.addEventListener("click", async () => {
  const instruments = getSelectedInstruments();
  if (instruments.length === 0) {
    await modalAlert("请先选择一种乐器");
    return;
  }
  if (
    !(await modalConfirm(
      "将先开启「上半身调试模式」，再加载并播放「开始标定」轨迹（100Hz）。\n\n" +
        "点击「确定」执行；「取消」则不执行。"
    ))
  ) {
    return;
  }
  let debugEnabled = false;
  try {
    await postJson("/api/upper_body_debug", { enable: true });
    debugEnabled = true;
    await postJson("/api/calibration/start", { instruments });
    setCalibrationButtonsDisabled(true);
  } catch (e) {
    console.warn(e);
    if (debugEnabled) {
      try {
        await postJson("/api/upper_body_debug", { enable: false });
      } catch (e2) {
        console.warn("回滚关闭上半身调试失败", e2);
      }
    }
  }
});

document.getElementById("btn-calib-end")?.addEventListener("click", async () => {
  const instruments = getSelectedInstruments();
  if (instruments.length === 0) {
    await modalAlert("请先选择一种乐器");
    return;
  }
  if (
    !(await modalConfirm(
      "将加载并播放「结束标定」轨迹（100Hz）。\n\n" +
        "不会自动关闭上半身调试；播完后如需退出调试，请再点「关闭上半身调试模式」。\n\n" +
        "点击「确定」执行；「取消」则不执行。"
    ))
  ) {
    return;
  }
  try {
    await postJson("/api/calibration/end", { instruments });
    setCalibrationButtonsDisabled(true);
    await waitForCalibrationIdle();
  } catch (e) {
    console.warn(e);
  } finally {
    await refreshCalibrationPlaybackStatus();
  }
});

document.getElementById("btn-upper-body-debug-off")?.addEventListener("click", async () => {
  if (
    !(await modalConfirm(
      "确定关闭「上半身调试模式」？\n\n将调用服务 /motion/upper_body_debug (false)。"
    ))
  ) {
    return;
  }
  try {
    await postJson("/api/upper_body_debug", { enable: false });
  } catch (e) {
    console.warn(e);
  }
});

/** 轨迹 API 请求体：按当前「数据来源」下可见控件取值 */
function buildTrajectoryPayloadForPlay() {
  const modeEl = document.getElementById("trajectory-mode");
  const mode = modeEl ? modeEl.value : "resource_start";
  const instruments = getSelectedInstruments();
  const rowAct = document.getElementById("row-action-data");
  const rowOff = document.getElementById("row-offset-data");
  const rowCust = document.getElementById("row-custom-path");
  const actionSel = document.getElementById("action-data-select");
  const offsetSel = document.getElementById("offset-data-select");
  const customInp = document.getElementById("custom-trajectory-path");
  const outBase = document.getElementById("output-basename");
  return {
    trajectory_mode: mode,
    instruments,
    action_data_basename: rowAct && !rowAct.hidden && actionSel ? actionSel.value : "",
    offset_data_relative: rowOff && !rowOff.hidden && offsetSel ? offsetSel.value : "",
    custom_path: rowCust && !rowCust.hidden && customInp ? customInp.value.trim() : "",
    output_basename: outBase ? outBase.value.trim() : "",
  };
}

function syncTrajectoryModeRows() {
  const modeEl = document.getElementById("trajectory-mode");
  const mode = modeEl ? modeEl.value : "";
  const show = (id, on) => {
    const el = document.getElementById(id);
    if (el) el.hidden = !on;
  };
  show("row-action-data", mode === "action_data");
  show("row-offset-data", mode === "offset_data");
  show("row-custom-path", mode === "custom");
}

/**
 * 刷新轨迹文件下拉框。
 * @param {{ selectValue?: string }} [options] selectValue：生成偏移后传入 ``new_offset_data/...`` 以便选中。
 */
async function loadTrajectoryFileLists(options = {}) {
  const selectValue = typeof options.selectValue === "string" ? options.selectValue : "";
  try {
    const ar = await fetch("/api/trajectory/action_data_files");
    const aj = await ar.json();
    const sel = document.getElementById("action-data-select");
    if (sel && aj.ok && Array.isArray(aj.files)) {
      sel.innerHTML = "";
      aj.files.forEach((f) => {
        const o = document.createElement("option");
        o.value = f;
        o.textContent = f;
        sel.appendChild(o);
      });
    }
  } catch (e) {
    console.warn("load action_data list failed", e);
  }
  try {
    const r = await fetch("/api/trajectory/offset_data_files");
    const j = await r.json();
    const rRecent = await fetch("/api/trajectory/recent_generated_offset_files");
    const jRecent = await rRecent.json().catch(() => ({}));
    const sel = document.getElementById("offset-data-select");
    if (sel && j.ok && Array.isArray(j.files)) {
      const disk = j.files;
      const recent =
        jRecent.ok && Array.isArray(jRecent.files) ? jRecent.files : [];
      const seen = new Set();
      sel.innerHTML = "";
      const addOpt = (f) => {
        if (seen.has(f)) return;
        seen.add(f);
        const o = document.createElement("option");
        o.value = f;
        o.textContent = f;
        sel.appendChild(o);
      };
      for (const f of recent) addOpt(f);
      for (const f of disk) addOpt(f);
      if (selectValue) {
        const has = [...sel.options].some((o) => o.value === selectValue);
        if (has) {
          sel.value = selectValue;
        } else {
          const o = document.createElement("option");
          o.value = selectValue;
          o.textContent = selectValue;
          sel.insertBefore(o, sel.firstChild);
          sel.value = selectValue;
        }
      }
    }
  } catch (e) {
    console.warn("load offset_data list failed", e);
  }
}

document.getElementById("trajectory-mode")?.addEventListener("change", syncTrajectoryModeRows);

function formatOffsetSummary(side, o) {
  if (!o) return `${side}: —`;
  const rad2deg = 180 / Math.PI;
  const mm = (v) => (typeof v === "number" ? v.toFixed(2) : "—");
  const deg = (v) => (typeof v === "number" ? (v * rad2deg).toFixed(3) : "—");
  return (
    `${side}: X=${mm(o.x)}mm Y=${mm(o.y)}mm Z=${mm(o.z)}mm | ` +
    `RX=${deg(o.rx)}° RY=${deg(o.ry)}° RZ=${deg(o.rz)}°`
  );
}

document.getElementById("btn-save")?.addEventListener("click", async () => {
  if (!(await modalConfirm("保存当前左右臂末端 6D 偏移量（X/Y/Z mm + RX/RY/RZ rad 外旋 XYZ）到 web_saved_offsets.json？"))) {
    return;
  }
  try {
    const j = await postJson("/api/save", {});
    const lines = [
      j.message || "已保存",
      j.saved_path ? `落地: ${j.saved_path}` : "",
      formatOffsetSummary("左臂", j.left),
      formatOffsetSummary("右臂", j.right),
    ].filter(Boolean);
    await modalAlert(lines.join("\n"));
    await refreshState();
  } catch (e) {
    console.warn("save failed", e);
    await modalAlert(e.message || String(e));
  }
});

function setOffsetProduceProgress(visible) {
  const wrap = document.getElementById("offset-produce-progress");
  const btn = document.getElementById("btn-offset-produce");
  if (wrap) wrap.hidden = !visible;
  if (btn) btn.disabled = !!visible;
}

document.getElementById("btn-offset-produce")?.addEventListener("click", async () => {
  const instruments = getSelectedInstruments();
  if (instruments.length === 0) {
    await modalAlert("请先在乐器下拉框中选择一种乐器（与「开始标定」同源）");
    return;
  }
  if (!(await modalConfirm("请先保存位姿偏移量，确定生成新偏移数据？"))) {
    return;
  }
  setOffsetProduceProgress(true);
  try {
    const produceBody = {
      trajectory_mode: "resource_start",
      instruments,
      action_data_basename: "",
      offset_data_relative: "",
      custom_path: "",
      output_basename: "",
    };
    const j = await postJson("/api/offset_data_produce", produceBody);
    const copies =
      Array.isArray(j.output_copies) && j.output_copies.length
        ? j.output_copies
        : j.output_copy
        ? [j.output_copy]
        : [];
    const pick = typeof j.offset_data_relative === "string" ? j.offset_data_relative : "";
    const nameOnly = pick ? pick.split("/").pop() : "";
    const successLines = [
      "生成成功",
      "",
      `文件名：${nameOnly || "-"}`,
      j.output_primary ? `文件路径：${j.output_primary}` : "",
      copies.length ? `副本路径：\n  ${copies.join("\n  ")}` : "",
    ].filter(Boolean);
    await modalAlert(successLines.join("\n"));
    await loadTrajectoryFileLists({ selectValue: pick });
    const modeEl = document.getElementById("trajectory-mode");
    if (modeEl && pick) {
      modeEl.value = "offset_data";
      syncTrajectoryModeRows();
    }
  } catch (e) {
    console.warn("offset_data_produce failed", e);
    await modalAlert(`失败：${e.message || String(e)}`);
  } finally {
    setOffsetProduceProgress(false);
  }
});

document.getElementById("btn-play-selected")?.addEventListener("click", async () => {
  const body = buildTrajectoryPayloadForPlay();
  if (
    !(await modalConfirm(
      "将先开启「上半身调试模式」，再按当前「数据来源」加载并播放所选轨迹（100Hz）。\n\n" +
        "点击「确定」执行；「取消」则不执行。"
    ))
  ) {
    return;
  }
  let debugEnabled = false;
  try {
    await postJson("/api/upper_body_debug", { enable: true });
    debugEnabled = true;
    await postJson("/api/calibration/play_selected", body);
    setCalibrationButtonsDisabled(true);
  } catch (e) {
    console.warn(e);
    if (debugEnabled) {
      try {
        await postJson("/api/upper_body_debug", { enable: false });
      } catch (e2) {
        console.warn("回滚关闭上半身调试失败", e2);
      }
    }
  }
});

/* ================== 乐队多片段数据偏移生成播放 ================== */
/**
 * "已生成 / 待播放"列表里按点击顺序维护的绝对路径数组：
 * - 点击未选中的条目 → 追加到末尾，并显示序号
 * - 点击已选中的条目 → 移除，其余后续条目序号自动前移
 * 播放时以此数组为序调用 ``/api/band/play_sequence``。
 */
const bandOrder = {
  paths: /** @type {string[]} */ ([]),
  items: /** @type {{path: string, name: string}[]} */ ([]),
};

/**
 * 「列出来源」后的动作文件列表；paths 为点击顺序（与 bandOrder 交互一致）。
 */
const bandSourcePick = {
  items: /** @type {{path: string, name: string}[]} */ ([]),
  paths: /** @type {string[]} */ ([]),
};

function renderBandSourceList() {
  const host = document.getElementById("band-source-list");
  if (!host) return;
  host.innerHTML = "";
  if (!bandSourcePick.items.length) {
    const p = document.createElement("p");
    p.className = "band-generated-empty";
    p.textContent = "请先点「列出来源」";
    host.appendChild(p);
    return;
  }
  bandSourcePick.items.forEach((it) => {
    const row = document.createElement("div");
    row.className = "band-generated-item";
    row.setAttribute("data-path", it.path);
    const orderIdx = bandSourcePick.paths.indexOf(it.path);
    const selected = orderIdx >= 0;
    if (selected) row.classList.add("is-selected");
    const ordinal = document.createElement("span");
    ordinal.className = "band-ordinal";
    ordinal.textContent = selected ? String(orderIdx + 1) : "";
    const name = document.createElement("span");
    name.className = "band-item-name";
    name.textContent = it.name;
    const path = document.createElement("span");
    path.className = "band-item-path";
    path.textContent = it.path;
    row.appendChild(ordinal);
    row.appendChild(name);
    row.appendChild(path);
    row.addEventListener("click", () => {
      const idx = bandSourcePick.paths.indexOf(it.path);
      if (idx >= 0) bandSourcePick.paths.splice(idx, 1);
      else bandSourcePick.paths.push(it.path);
      renderBandSourceList();
    });
    host.appendChild(row);
  });
}

/** @returns {string[]} 绝对路径列表（来源点击顺序） */
function buildBandRawPlayPaths() {
  return bandSourcePick.paths.slice();
}

function basenameOnly(p) {
  const s = p.replace(/\\/g, "/");
  const i = s.lastIndexOf("/");
  return i >= 0 ? s.slice(i + 1) : s;
}

function setBandProduceProgress(visible) {
  const wrap = document.getElementById("band-produce-progress");
  const btn = document.getElementById("btn-band-produce");
  if (wrap) wrap.hidden = !visible;
  if (btn) btn.disabled = !!visible;
}

function getBandSaveDirOrEmpty() {
  const el = document.getElementById("band-save-dir");
  return el ? el.value.trim() : "";
}

function getBandSourceDirOrEmpty() {
  const el = document.getElementById("band-source-dir");
  return el ? el.value.trim() : "";
}

function renderBandGeneratedList() {
  const host = document.getElementById("band-generated-list");
  if (!host) return;
  host.innerHTML = "";
  if (!bandOrder.items.length) {
    const p = document.createElement("p");
    p.className = "band-generated-empty";
    p.textContent = '尚无条目；先"批量生成"或点"扫描保存目录"。';
    host.appendChild(p);
    return;
  }
  bandOrder.items.forEach((it) => {
    const row = document.createElement("div");
    row.className = "band-generated-item";
    row.setAttribute("data-path", it.path);
    const orderIdx = bandOrder.paths.indexOf(it.path);
    const selected = orderIdx >= 0;
    if (selected) row.classList.add("is-selected");
    const ordinal = document.createElement("span");
    ordinal.className = "band-ordinal";
    ordinal.textContent = selected ? String(orderIdx + 1) : "";
    const name = document.createElement("span");
    name.className = "band-item-name";
    name.textContent = it.name;
    const path = document.createElement("span");
    path.className = "band-item-path";
    path.textContent = it.path;
    row.appendChild(ordinal);
    row.appendChild(name);
    row.appendChild(path);
    row.addEventListener("click", () => {
      const idx = bandOrder.paths.indexOf(it.path);
      if (idx >= 0) bandOrder.paths.splice(idx, 1);
      else bandOrder.paths.push(it.path);
      renderBandGeneratedList();
    });
    host.appendChild(row);
  });
}

/** 合并一批新条目到 ``bandOrder.items``，不重复；保持原有顺序在前。 */
function mergeBandItems(newItems) {
  const seen = new Set(bandOrder.items.map((x) => x.path));
  for (const it of newItems) {
    if (!it || !it.path) continue;
    if (seen.has(it.path)) continue;
    seen.add(it.path);
    bandOrder.items.push({ path: it.path, name: it.name || it.path.split("/").pop() || it.path });
  }
}

document.getElementById("btn-band-list-source")?.addEventListener("click", async () => {
  const dir = getBandSourceDirOrEmpty();
  if (!dir) {
    await modalAlert("请先填写数据来源目录（绝对路径）");
    return;
  }
  try {
    const j = await postJson("/api/band/list_dir", { dir });
    const row = document.getElementById("band-source-row");
    const base = String(j.dir || dir)
      .trim()
      .replace(/\/+$/, "");
    bandSourcePick.items = (j.files || []).map((f) => ({
      name: f,
      path: `${base}/${f}`.replace(/\\/g, "/"),
    }));
    bandSourcePick.paths = [];
    renderBandSourceList();
    if (row) row.hidden = false;
    if (!(j.files || []).length) {
      await modalAlert(`目录内未发现 .data / .csv 文件：\n${j.dir || dir}`);
    }
  } catch (e) {
    console.warn("band list_dir (source) failed", e);
    await modalAlert(`列出来源失败：${e.message || String(e)}`);
  }
});

document.getElementById("btn-band-list-generated")?.addEventListener("click", async () => {
  const dir = getBandSaveDirOrEmpty();
  if (!dir) {
    await modalAlert("请先填写保存目录（绝对路径），或先通过「批量生成」自动定位");
    return;
  }
  try {
    const j = await postJson("/api/band/list_dir", { dir });
    const items = (j.files || []).map((name) => ({
      name,
      path: `${j.dir || dir}/${name}`.replace(/\\/g, "/"),
    }));
    if (!items.length) {
      bandOrder.items = [];
      bandOrder.paths = [];
      renderBandGeneratedList();
      await modalAlert(`目录内未发现 .data / .csv 文件：\n${j.dir || dir}`);
      return;
    }
    mergeBandItems(items);
    renderBandGeneratedList();
  } catch (e) {
    console.warn("band list_dir (save) failed", e);
    await modalAlert(`扫描保存目录失败：${e.message || String(e)}`);
  }
});

document.getElementById("btn-band-clear-source-order")?.addEventListener("click", () => {
  bandSourcePick.paths = [];
  renderBandSourceList();
});

document.getElementById("btn-band-clear-order")?.addEventListener("click", () => {
  bandOrder.paths = [];
  renderBandGeneratedList();
});

document.getElementById("btn-band-produce")?.addEventListener("click", async () => {
  const sourceDir = getBandSourceDirOrEmpty();
  const saveDir = getBandSaveDirOrEmpty();
  const filenames = bandSourcePick.paths.map((p) => basenameOnly(p));
  if (!sourceDir) {
    await modalAlert("请先填写数据来源目录（绝对路径）");
    return;
  }
  if (!filenames.length) {
    await modalAlert('请先在「来源动作数据」列表中按顺序点选至少一个文件（点击后出现序号，同下方「待播放偏移」操作）');
    return;
  }
  const tailTarget = saveDir || "默认 new_offset_data 目录（首个可写位置）";
  if (
    !(await modalConfirm(
      "请先保存位姿偏移量，确定批量生成偏移数据？\n\n" +
        `来源：${sourceDir}\n` +
        `保存：${tailTarget}\n` +
        `文件数：${filenames.length}`
    ))
  ) {
    return;
  }
  setBandProduceProgress(true);
  try {
    const j = await postJson("/api/band/produce", {
      source_dir: sourceDir,
      filenames,
      save_dir: saveDir,
    });
    const results = Array.isArray(j.results) ? j.results : [];
    const okRows = results.filter((r) => r && r.ok);
    const failRows = results.filter((r) => r && !r.ok);
    const newItems = okRows.map((r) => ({
      name: r.output_name || (r.output_path || "").split("/").pop(),
      path: r.output_path,
    }));
    mergeBandItems(newItems);
    renderBandGeneratedList();
    const lines = [
      j.ok ? "批量生成成功" : "批量生成部分失败",
      "",
      `保存目录：${j.save_dir || saveDir || "-"}`,
      `成功：${okRows.length}，失败：${failRows.length}`,
    ];
    if (failRows.length) {
      lines.push("");
      lines.push("失败条目：");
      failRows.slice(0, 20).forEach((r) => {
        lines.push(`  - ${r.input}: ${r.message || "未知错误"}`);
      });
      if (failRows.length > 20) lines.push(`  …（共 ${failRows.length} 条，仅显示前 20 条）`);
    }
    await modalAlert(lines.join("\n"));
    // 同步刷新单文件下拉（若保存目录正好是 new_offset_data 之一，会显示在那里）
    await loadTrajectoryFileLists();
  } catch (e) {
    console.warn("band produce failed", e);
    await modalAlert(`批量生成失败：${e.message || String(e)}`);
  } finally {
    setBandProduceProgress(false);
  }
});

document.getElementById("btn-band-play")?.addEventListener("click", async () => {
  const modeEl = document.querySelector('input[name="band-play-mode"]:checked');
  const mode = modeEl ? modeEl.value : "offset";
  /** @type {string[]} */
  let files = [];
  if (mode === "raw") {
    files = buildBandRawPlayPaths();
    if (!files.length) {
      await modalAlert(
        "请先在「数据来源目录」点「列出来源」，并在「来源动作数据」列表中按顺序点选至少一个文件"
      );
      return;
    }
  } else {
    files = bandOrder.paths.slice();
    if (!files.length) {
      await modalAlert("请先在「已生成 / 待播放偏移数据」列表里按顺序点选至少一个文件");
      return;
    }
  }
  const modeLabel = mode === "raw" ? "来源目录原数据（未做偏移）" : "已生成偏移数据";
  if (
    !(await modalConfirm(
      "将先开启「上半身调试模式」，再按所选顺序串接播放（100Hz）。\n\n" +
        `内容：${modeLabel}\n` +
        `片段数：${files.length}\n\n` +
        "播放结束后将自动关闭上半身调试模式。\n\n" +
        "点击「确定」执行；「取消」则不执行。"
    ))
  ) {
    return;
  }
  let debugEnabled = false;
  try {
    await postJson("/api/upper_body_debug", { enable: true });
    debugEnabled = true;
    await postJson("/api/band/play_sequence", { files });
    setCalibrationButtonsDisabled(true);
    await waitForCalibrationIdle();
    await refreshCalibrationPlaybackStatus();
  } catch (e) {
    console.warn("band play failed", e);
    if (debugEnabled) {
      try {
        await postJson("/api/upper_body_debug", { enable: false });
      } catch (e2) {
        console.warn("回滚关闭上半身调试失败", e2);
      }
    }
    await modalAlert(`播放失败：${e.message || String(e)}`);
  }
});

function wireRefreshInitialButtons() {
  document.querySelectorAll(".btn-refresh-initial").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const arm = btn.getAttribute("data-arm");
      if (!arm) return;
      const prevLabel = btn.textContent;
      btn.disabled = true;
      btn.textContent = "刷新中...";
      try {
        const j = await postJson(`/api/arm/${arm}/initial/refresh`, {});
        await refreshState();
        const p = j && j.initial_ee_m;
        const r = j && j.initial_rpy_deg;
        let hint = `已更新${arm === "left" ? "左" : "右"}臂初始值`;
        if (p) {
          hint +=
            `：XYZ m x=${Number(p.x).toFixed(6)}, y=${Number(p.y).toFixed(6)}, z=${Number(p.z).toFixed(6)}`;
        }
        if (r) {
          hint += `；RX/RY/RZ ° ${Number(r.rx).toFixed(3)} / ${Number(r.ry).toFixed(3)} / ${Number(r.rz).toFixed(3)}`;
        }
        console.info(hint);
      } catch (e) {
        console.warn("refresh initial failed", e);
        await modalAlert(e.message || String(e));
      } finally {
        btn.disabled = false;
        btn.textContent = prevLabel;
      }
    });
  });
}

AppModal.init();
wireAdjustButtons();
wireRotAdjustButtons();
wireStepControls();
wireInstrumentDropdown();
wireRefreshInitialButtons();
syncTrajectoryModeRows();
loadTrajectoryFileLists();
refreshState();
setInterval(refreshState, 100);
setInterval(refreshCalibrationPlaybackStatus, 400);
refreshCalibrationPlaybackStatus();
