#!/usr/bin/env python3
"""URDF とメッシュをブラウザ用に変換する.

RViz の画面を動画で送る代わりに、ブラウザ側で本物の URDF を描画するための下ごしらえ。

  - URDF を読んでリンク階層・関節の原点/軸/可動域を JSON にする
  - STL（バイナリ）を読んで軽量化し、コンパクトなバイナリにする

なぜ変換するか:
  piper_description の STL は合計 9.2MB / 18.4万三角形あり、そのままでは
  タブレットへ送るには重い。頂点クラスタリング（格子に丸めて頂点をまとめる）で
  1mm 格子なら 1.9MB / 10.7万三角形、形状誤差 0.06mm まで落ちる。
  UI の 600px ほどのパネルで見るぶんには見た目は変わらない。

依存は numpy だけ。外部のメッシュ処理ライブラリは要らない。
変換結果はキャッシュするので、負担がかかるのは起動後の最初の一度だけ。
"""

import hashlib
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

# バイナリ形状データのヘッダ
#   magic 'PARM' | version u32 | vertex count u32 | index count u32
#   → float32 * 3 * vcount（位置）→ uint32 * icount（三角形の頂点番号）
MESH_MAGIC = b'PARM'
MESH_VERSION = 1


def _floats(text, default):
    if not text:
        return list(default)
    try:
        return [float(v) for v in text.split()]
    except ValueError:
        return list(default)


def load_binary_stl(path):
    """バイナリ STL を (三角形数, 3, 3) の配列で読む."""
    with open(path, 'rb') as f:
        raw = f.read()
    if len(raw) < 84:
        raise ValueError('STL が短すぎます: %s' % path)
    if raw[:5].lower().lstrip().startswith(b'solid') and b'facet' in raw[:512]:
        raise ValueError('アスキー STL は未対応です: %s' % path)
    ntri = struct.unpack('<I', raw[80:84])[0]
    need = 84 + 50 * ntri
    if len(raw) < need:
        raise ValueError('STL が壊れています（三角形数と長さが合わない）: %s' % path)
    block = np.frombuffer(raw[84:need], dtype=np.uint8).reshape(ntri, 50)
    # 各三角形 50 バイト: 法線 12 + 頂点 36 + 属性 2。法線は使わず後で計算する
    verts = block[:, 12:48].copy().view(np.float32).reshape(ntri, 3, 3)
    return verts.astype(np.float64)


def simplify(tris, cell):
    """頂点クラスタリングでポリゴンを減らす.

    格子セルに入った頂点をその重心 1 点にまとめ、つぶれた三角形を捨てる。
    二次誤差計量ほどきれいではないが、依存ライブラリ無しで速く、
    機械部品の輪郭を保つには十分。cell <= 0 なら頂点の結合だけ行う。
    """
    v = tris.reshape(-1, 3)
    if cell and cell > 0:
        key = np.floor(v / cell).astype(np.int64)
    else:
        key = np.round(v * 1e6).astype(np.int64)
    uniq, inverse = np.unique(key, axis=0, return_inverse=True)

    rep = np.zeros((len(uniq), 3))
    count = np.zeros(len(uniq))
    np.add.at(rep, inverse, v)
    np.add.at(count, inverse, 1.0)
    rep /= count[:, None]

    faces = inverse.reshape(-1, 3)
    # 同じ代表点を2つ以上含む三角形は面積ゼロなので捨てる
    keep = ((faces[:, 0] != faces[:, 1])
            & (faces[:, 1] != faces[:, 2])
            & (faces[:, 0] != faces[:, 2]))
    faces = faces[keep]
    if len(faces) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint32)

    # 重複した三角形を除く（向きは後で法線を計算し直すので気にしない）
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    faces = faces[np.sort(first)]

    used, remap = np.unique(faces, return_inverse=True)
    return rep[used], remap.reshape(-1, 3).astype(np.uint32)


def pack_mesh(vertices, faces):
    """ブラウザがそのまま Float32Array / Uint32Array にできる形にまとめる."""
    head = MESH_MAGIC + struct.pack('<III', MESH_VERSION, len(vertices), faces.size)
    return (head
            + vertices.astype('<f4').tobytes()
            + faces.astype('<u4').ravel().tobytes())


def resolve_package_path(url, extra_roots=None):
    """package://<pkg>/<path> を実ファイルのパスに直す."""
    if not url.startswith('package://'):
        return url if os.path.isfile(url) else None
    rest = url[len('package://'):]
    pkg, _, rel = rest.partition('/')
    roots = []
    try:
        from ament_index_python.packages import get_package_share_directory
        roots.append(get_package_share_directory(pkg))
    except Exception:  # noqa: BLE001
        pass
    for root in (extra_roots or []):
        roots.append(os.path.join(root, pkg))
        roots.append(root)
    for root in roots:
        path = os.path.join(root, rel)
        if os.path.isfile(path):
            return path
    return None


class UrdfScene:
    """URDF を読み、ブラウザへ渡す形（JSON + メッシュのバイナリ）にする."""

    def __init__(self, urdf_path, cell=0.001, extra_roots=None, logger=None):
        self.urdf_path = urdf_path
        self.cell = float(cell)
        self.extra_roots = extra_roots or []
        self.logger = logger
        self.meshes = {}        # mesh id -> packed bytes
        self.scene = None       # JSON にする辞書
        self._build()

    def _log(self, level, text):
        if self.logger is not None:
            getattr(self.logger, level)(text)

    # ------------------------------------------------------------------ #
    def _build(self):
        root = ET.parse(self.urdf_path).getroot()

        links = {}
        for link in root.findall('link'):
            name = link.get('name')
            visuals = []
            for vis in link.findall('visual'):
                geom = vis.find('geometry')
                mesh_el = geom.find('mesh') if geom is not None else None
                if mesh_el is None:
                    continue          # box/cylinder 等は piper では使われていない
                origin = vis.find('origin')
                color = [0.8, 0.82, 0.93, 1.0]
                mat = vis.find('material')
                if mat is not None and mat.find('color') is not None:
                    color = _floats(mat.find('color').get('rgba'), color)
                mesh_id = self._add_mesh(mesh_el.get('filename'))
                if mesh_id is None:
                    continue
                visuals.append({
                    'mesh': mesh_id,
                    'xyz': _floats(origin.get('xyz') if origin is not None else None,
                                   (0, 0, 0)),
                    'rpy': _floats(origin.get('rpy') if origin is not None else None,
                                   (0, 0, 0)),
                    'scale': _floats(mesh_el.get('scale'), (1, 1, 1)),
                    'color': color,
                })
            links[name] = {'visuals': visuals}

        joints = []
        children = set()
        for j in root.findall('joint'):
            origin = j.find('origin')
            axis = j.find('axis')
            limit = j.find('limit')
            child = j.find('child').get('link')
            children.add(child)
            entry = {
                'name': j.get('name'),
                'type': j.get('type'),
                'parent': j.find('parent').get('link'),
                'child': child,
                'xyz': _floats(origin.get('xyz') if origin is not None else None,
                               (0, 0, 0)),
                'rpy': _floats(origin.get('rpy') if origin is not None else None,
                               (0, 0, 0)),
                'axis': _floats(axis.get('xyz') if axis is not None else None,
                                (0, 0, 1)),
            }
            if limit is not None:
                entry['lower'] = float(limit.get('lower', 0.0))
                entry['upper'] = float(limit.get('upper', 0.0))
            joints.append(entry)

        roots = [n for n in links if n not in children]
        self.scene = {
            'name': root.get('name', 'robot'),
            'root': roots[0] if roots else next(iter(links), ''),
            'links': links,
            'joints': joints,
            'source': os.path.basename(self.urdf_path),
            'simplify_cell': self.cell,
        }
        tri = sum(len(m) for m in self.meshes.values())
        self._log('info',
                  'URDF を読み込みました: %s（リンク %d / 関節 %d / メッシュ %d、計 %.2fMB）'
                  % (os.path.basename(self.urdf_path), len(links), len(joints),
                     len(self.meshes), tri / 1e6))

    def _add_mesh(self, url):
        if not url:
            return None
        mesh_id = hashlib.sha1(url.encode('utf-8')).hexdigest()[:16]
        if mesh_id in self.meshes:
            return mesh_id
        path = resolve_package_path(url, self.extra_roots)
        if path is None:
            self._log('warn', 'メッシュが見つかりません: %s' % url)
            return None
        try:
            tris = load_binary_stl(path)
            verts, faces = simplify(tris, self.cell)
            if len(faces) == 0:
                raise ValueError('簡略化の結果が空になりました')
            self.meshes[mesh_id] = pack_mesh(verts, faces)
        except Exception as exc:  # noqa: BLE001
            self._log('warn', 'メッシュを読めません (%s): %s' % (os.path.basename(path), exc))
            return None
        return mesh_id

    # ------------------------------------------------------------------ #
    def mesh_bytes(self, mesh_id):
        return self.meshes.get(mesh_id)

    def total_bytes(self):
        return sum(len(m) for m in self.meshes.values())
