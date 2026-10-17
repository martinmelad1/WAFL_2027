// ==========================================================
// WAFL 2027 Dashboard - script.js
// Step 1: tabs + Auto/Manual mode (no backend yet)
// ==========================================================

// ---------- 1) Tabs ----------
const tabButtons = document.querySelectorAll(".tab-btn");
const tabPanels  = document.querySelectorAll(".tab-panel");

function openTab(name) {
  // turn every tab button and panel off...
  tabButtons.forEach(function (btn) {
    btn.classList.toggle("active", btn.dataset.tab === name);
  });
  tabPanels.forEach(function (panel) {
    panel.classList.toggle("active", panel.id === "tab-" + name);
  });
}

// when a tab button is clicked, open its tab
tabButtons.forEach(function (btn) {
  btn.addEventListener("click", function () {
    openTab(btn.dataset.tab);
  });
});

// ---------- 2) Mode: Auto / Manual ----------
const autoBtn      = document.getElementById("mode-auto");
const manualBtn    = document.getElementById("mode-manual");
const controlPanel = document.getElementById("control-panel");

function setMode(mode) {
  const isManual = (mode === "manual");

  autoBtn.classList.toggle("active", !isManual);
  manualBtn.classList.toggle("active", isManual);

  // the Control panel shows in Manual mode only
  controlPanel.classList.toggle("hidden", !isManual);

  // later: send the mode to the backend here
}

autoBtn.addEventListener("click",   function () { setMode("auto"); });
manualBtn.addEventListener("click", function () { setMode("manual"); });

// start in Auto mode
setMode("auto");