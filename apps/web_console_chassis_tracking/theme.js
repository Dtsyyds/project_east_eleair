/* Presentation tokens only. Read once; never resolve CSS in the frame decode path. */
"use strict";
(() => {
  const style = getComputedStyle(document.documentElement);
  const color = name => style.getPropertyValue("--" + name).trim();
  window.ConsoleTheme = Object.freeze({
    background: color("canvas"), grid: color("plot-grid"), text: color("muted"),
    good: color("good"), warning: color("warn"), danger: color("bad"), cyan: color("cyan"),
    channels: Object.freeze(Array.from({length:8}, (_, i) => color("chart-" + (i + 1))))
  });
})();
