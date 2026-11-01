(function () {
  "use strict";

  // The widget must load first. Keep the documentation usable if it is blocked.
  if (!window.WeKnora) return;

  window.WeKnora.on("ready", function () {
    var match = window.location.pathname.match(
      /\/(sf32lb(?:52|55|56|57|58)x)(?:\/|$)/i
    );

    // This is the page default, not an explicit chip choice by the user.
    window.WeKnora.setContext({
      page_url: window.location.href,
      page_title: document.title,
      page_chip: match ? match[1].toUpperCase() : "",
    });
  });
})();
