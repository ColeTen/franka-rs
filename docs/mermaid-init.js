document.addEventListener("DOMContentLoaded", function () {
  if (typeof mermaid !== "undefined") {
    var theme = "default";
    var body = document.body;
    if (body.classList.contains("ayu") || body.classList.contains("coal") || body.classList.contains("navy")) {
      theme = "dark";
    }
    mermaid.initialize({
      startOnLoad: false,
      theme: theme,
      securityLevel: "loose",
      flowchart: { useMaxWidth: true, htmlLabels: true },
      sequence: { useMaxWidth: true },
      mindmap: { useMaxWidth: true },
      themeVariables: {
        fontFamily: '"Source Code Pro", monospace',
        fontSize: "14px"
      }
    });
    document.querySelectorAll("code.language-mermaid").forEach(function (code) {
      var pre = code.parentElement;
      pre.classList.add("mermaid");
      pre.textContent = code.textContent;
    });
    mermaid.run({ querySelector: "pre.mermaid" });
  }
});
