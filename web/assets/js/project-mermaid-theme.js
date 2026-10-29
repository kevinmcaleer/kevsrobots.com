// The look of a project's mermaid diagrams, shared by the project editor
// (project-editor.js) and the published page (projects/view.html) so a
// diagram looks the same in both. Mermaid's default theme draws purple boxes
// with a drop shadow; this one uses the document's palette and font
// (project-content.css, after markdown-studio's editor tokens): white boxes
// with a hairline border on the parchment, muted lines, no shadow.
//
// Pass it as the options to mermaid-tools' renderMermaid(source, options)
// and spread it into mountEditor(element, { ...ProjectMermaidTheme, ... }).
(function () {
  const FONT = 'ui-sans-serif, system-ui, -apple-system, "Segoe UI", Roboto, sans-serif';
  const TEXT = '#141413';
  const MUTED = '#6b6a63';
  const BORDER = '#cfcabd';

  window.ProjectMermaidTheme = {
    theme: 'base',
    config: {
      fontFamily: FONT,
      themeVariables: {
        fontFamily: FONT,
        fontSize: '15px',
        background: '#faf9f5',
        primaryColor: '#ffffff',
        primaryTextColor: TEXT,
        primaryBorderColor: BORDER,
        secondaryColor: '#f0eee6',
        secondaryTextColor: TEXT,
        secondaryBorderColor: BORDER,
        tertiaryColor: '#f0eee6',
        tertiaryTextColor: TEXT,
        tertiaryBorderColor: BORDER,
        lineColor: MUTED,
        textColor: TEXT,
        mainBkg: '#ffffff',
        nodeBorder: BORDER,
        clusterBkg: '#f0eee6',
        clusterBorder: BORDER,
        titleColor: TEXT,
        edgeLabelBackground: '#faf9f5',
        noteBkgColor: '#fbe7a1',
        noteTextColor: TEXT,
        noteBorderColor: '#e0c66b',
        actorBkg: '#ffffff',
        actorBorder: BORDER,
        actorTextColor: TEXT,
        actorLineColor: MUTED,
        signalColor: MUTED,
        signalTextColor: TEXT,
        labelBoxBkgColor: '#ffffff',
        labelBoxBorderColor: BORDER,
        labelTextColor: TEXT,
        dropShadow: 'none',
        useGradient: false,
      },
    },
  };
})();
