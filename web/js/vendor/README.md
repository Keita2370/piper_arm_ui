# 同梱ライブラリ

現場ではインターネットに繋がらない前提なので、CDN は使わずここに置いている。

| ファイル | 版 | ライセンス | 用途 |
|---|---|---|---|
| `three.min.js` | three.js r149 | MIT (`three.LICENSE`) | 3D ビュー（URDF の描画） |

r149 を選んでいる理由: r150 以降の `three.min.js` は
「deprecated、r160 で削除」の警告をコンソールに出し、r160 が UMD 最後の版。
`<script>` タグ 1 本で読めてコンソールが汚れない最後の版が r149。
使っている API（WebGLRenderer / BufferGeometry / MeshStandardMaterial /
ShadowMaterial / GridHelper など）はこの前後で変わっていない。

更新するときは UMD ビルド（グローバル `THREE` を定義するもの）を置くこと。
ES モジュール版 (`three.module.js`) は `<script>` タグから読めないので不可。

視点操作（OrbitControls）は同梱していない。r160 の配布物には UMD 版が無く、
また RViz と同じ操作感にしたいので `web/js/robotview.js` に自前で実装してある。
