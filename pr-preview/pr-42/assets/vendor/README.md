# Vendored front-end libraries

Served from fjp.at instead of a CDN, so visitors don't contact third parties. Pinned versions, each with its license.

| Library | Version | Source | Changes |
|---|---|---|---|
| KaTeX | 0.19.0 | npm `katex` (`dist/`: katex.min.js, katex.min.css, contrib/auto-render.min.js, woff2 fonts) | none |
| pseudocode.js | 2.4.1 | npm `pseudocode` (`build/`) | removed the `@import` of KaTeX 0.16.7 CSS from cdnjs in `pseudocode.min.css`; the site loads its own KaTeX CSS |
| Font Awesome Free | 7.3.1 | npm `@fortawesome/fontawesome-free` (`css/all.min.css`, woff2 webfonts) | none |

Used in `_includes/head/custom.html` (KaTeX, pseudocode.js) and `_includes/head.html` (Font Awesome).
