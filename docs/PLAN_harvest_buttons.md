# 収穫シーケンスのボタンを動くようにする — 作業計画

いまの UI のボタンは `harvest_sequencer` の内部状態機械を動かすだけで、
チームの収穫システムを呼んでいない。そこを繋ぐための手順。

**この文書は計画。まだ何も修正していない。**

---

## 1. 方針：どこを直すか

**`harvest_sequencer` を「自分で動かすノード」から「チームのサービスを呼ぶノード」に変える。**
ブラウザ側はほとんど変えない。

```
いま:
  ブラウザ → /arm_ui/command → harvest_sequencer → 内部の状態機械
                                                 → Trajectory → /joint_states

これから:
  ブラウザ → /arm_ui/command → harvest_sequencer → /lemon_approach/approach
                                （呼ぶだけ）      → /lemon_grasp/grasp
                                                 → /arm_home/go_home …
                              ← /arm_ui/state ← 各ノードの status を集約
```

### なぜブラウザから直接サービスを呼ばないのか

rosbridge は `call_service` を中継できるので、ブラウザから直接呼ぶこともできる。
それでも間に `harvest_sequencer` を置くのは、次の4つのため。

| | |
|---|---|
| **UI の契約が保たれる** | `/arm_ui/command` と `/arm_ui/state` は変えない。ブラウザ側の変更が最小で済む |
| **遠隔アクセスの権限がそのまま効く** | `remote_access.py` は `/arm_ui/command` への publish を見て遮断している。ブラウザから直接サービスを呼ぶ経路にすると、**遠隔の人が実機を動かせる穴になる** |
| **長い呼び出しの面倒を1か所で見られる** | `grasp` は実機で最大40秒、`call_timeout` は120秒。失敗・タイムアウト・途中の状態表示をまとめて扱える |
| **テストできる** | `tests/mini_ros.py` でサービスを模擬すれば、ROS も実機も無しで確かめられる |

### 既存の仕組みは消さない

`backend` パラメータで切り替えられるようにする。

| `backend` | 動き | 使う場面 |
|---|---|---|
| `internal`（既定のまま） | いまどおり。自前の状態機械＋`Trajectory`＋`/joint_states` | 実機なしの開発用PC、UI だけの確認 |
| `lemon` | チームのサービスを呼ぶ | ミニPC（実機） |

**捨てずに並べる理由**: 実機なしで UI を触れる環境が残る。既存テスト（110項目）が
そのまま通る。うまくいかなければパラメータ1つで戻せる。

---

## 2. 先に決めること（1つだけ）

**把持対象を誰が選ぶか。** これだけは着手前に決める必要がある。残り3つは後で決めてよい。

| | 案A：自動選択に乗る | 案B：クリックで選ぶ |
|---|---|---|
| やること | `lemon_target` が選んだ対象を**表示するだけ**。「把持対象の選択」ボタンは「**次の候補へ**」（＝`/lemon_target/release`）に変える | `lemon_target` に「この位置のものを選べ」という口を足す |
| 相手のコード | **無改造** | **改造が要る**（相談が必要） |
| 作業量 | 小 | 中＋調整 |
| 使い勝手 | 選べない。気に入らなければ次へ送る | いまの UI と同じ感覚 |

**まず案Aで通すことを勧める。** 案Bは案Aが動いたあとに足せる（ボタンの意味が変わるだけで、
下回りは同じ）。最初から相手のコードに手を入れると、動かないときの切り分けが難しくなる。

### 後で決めればよいこと

- 収穫位置 ①②③ をどうするか（相手に対応する機能が無い。§6）
- 座標指定・関節ジョグを残すか（§8）
- UI がマネージャか監視役か → **実質決まっている。`auto_run:=false` で UI がマネージャ**

---

## 3. ★ 技術的に一番危ないところ

**いまの `harvest_sequencer` は単一スレッドで spin している。このままサービスを呼ぶと固まる。**

```python
# いま（main の中）
rclpy.spin(node)
```

rclpy では、コールバックの中から同期的にサービスの結果を待つと
**executor が止まって結果が永久に返らない**。

相手のコードは全部この対策をしている。同じ形にする。

```python
# lemon_harvest_manager / arm_home / lemon_approach がやっていること
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()

self.cb_group = ReentrantCallbackGroup()
self.client = self.create_client(Trigger, '...', callback_group=self.cb_group)

# 結果待ちは別スレッドで（moveit_helper.wait_future と同じ形）
future = client.call_async(Trigger.Request())
event = threading.Event()
future.add_done_callback(lambda _: event.set())
event.wait(timeout)
```

### 併せて守ること

| | |
|---|---|
| **50Hz の制御ループと 10Hz の状態配信を止めない** | 40秒の `grasp` を待っている間も、3Dビューと状態表示は動き続けないといけない。タイマーのコールバックグループをサービス呼び出しと分ける |
| **シーケンスは別スレッドで回す** | `lemon_harvest_manager` と同じ形。コマンドのコールバックの中で40秒待たない |
| **二重呼び出しを防ぐ** | 相手も `busy` を返すが、UI 側でもボタンを無効にする |

**ここを雑にやると「ボタンを押すと画面が固まる」という形で出る。**
段階3（下記）で、無害なサービス1つだけで配線を確かめてから先へ進む。

---

## 4. 段階（この順で。1段ずつ確かめる）

危険度が上がる順に並べてある。**各段が終わるたびに実機で確かめる。**

### 段階0 — 現状の確認（コードは変えない）

```bash
ros2 service list | grep -E "lemon|arm_home"
ros2 topic echo /lemon_harvest_manager/status     # auto_run:=true のとき
ros2 topic echo /lemon_approach/status
ros2 topic echo /lemon_target/target
ros2 service call /lemon_target/clear_exclusions std_srvs/srv/Trigger
```

**サービス名・トピック名・応答の形が、資料どおりか自分の目で確かめる。**
違っていたらそこから。

### 段階1 — 状態表示だけ（アームは動かない）

`/lemon_harvest_manager/status`、`/lemon_approach/status`、`/lemon_grasp/status`、
`/lemon_target/target` を購読して `/arm_ui/state` に載せ、画面に出す。

- **サービスは呼ばない。読むだけ。** 危険ゼロ
- 画面に「いま相手が何をしているか」が出るようになる
- これだけでも、サービスを手で呼びながら UI で経過を見られるので実用になる

**確認**: `auto_run:=true` で相手を自動で回し、UI の表示が追従するか見る。

### 段階2 — 検出アダプタ（アームは動かない）

`/lemon_ellipse/lemons`（`LemonEllipseArray`）→ `/arm_ui/detections`（JSON）に変換する
ノードを `piper_arm_ui` に1つ足す。新しいパッケージは要らない。

| UI が期待する | 変換元 | 注意 |
|---|---|---|
| `u`, `v` | `center_u`, `center_v` | **px なので camera_info の幅高で割る** |
| `w`, `h` | `major_radius`, `minor_radius` | 楕円。外接矩形にする |
| `distance` | `center.z` | カメラ光学座標系の z が奥行き |
| `confidence` | `score` | |
| `id` | `id` | **実機は `use_tracking:=False` なので空かもしれない。連番を振る** |

`/lemon_target/target` も購読して、**いま選ばれている対象に印を付ける**
（各検出の3D中心を base_link に変換して、一番近いものを選ぶ）。

**確認**: カメラ画面に検出枠が出て、選ばれている1つが強調されるか。

### 段階3 — サービス呼び出しの土台（アームは動かない）

**ここが §3 の山場。** executor とスレッドの作りを変えて、
**動かないサービス1つだけ**で配線を確かめる。

- `MultiThreadedExecutor` + `ReentrantCallbackGroup` に変更
- `backend` パラメータを足す
- `/lemon_target/clear_exclusions` を呼ぶボタンを1つ足す（**アームは動かない**）

**確認**:
- ボタンを押して除外が消えるか（応答の `Cleared N exclusions` をログに出す）
- **押している間も 3D ビューが止まらないか** ← ここが本当の確認項目
- 相手が居ないときに押して、固まらずタイムアウトするか

### 段階4 — 動きの小さいものから

順に置き換える。**1つ置き換えるごとに実機で確かめる。**

1. `/lemon_target/release` — アームは動かない。「次の候補へ」ボタン
2. `/arm_home/go_home` — **ここで初めてアームが動く**。既知の姿勢へ戻るだけなので一番安全
3. `/lemon_grasp/open` — グリッパだけ

**確認**: 各ボタンで実機が期待どおり動くか。失敗時のメッセージが画面に出るか。

### 段階5 — 収穫の本番

1. `/lemon_approach/approach` — pregrasp へ移動（**大きく動く**）
2. `/lemon_grasp/grasp` — 差し込んで掴む（**最大40秒・Servo で動く**）

**必ず物理的な非常停止を手元に置いて、1回ずつ確かめる。**

**確認**: 一連を UI だけで通せるか。`missed` が「失敗」ではなく
「掴めなかった」と出るか。

### 段階6 — 停止と後始末

- 停止ボタン → `/lemon_target/release`（差し込み中なら約0.5秒で止まる）
- **止まらない場面があることを画面に明示する**（戻り動作、cartesian 戻り）
- シーケンスの自動連続（manager と同じループ）を UI から回すかどうか

---

## 5. ボタンの対応

| 画面のボタン | いまのコマンド | これから |
|---|---|---|
| 把持対象の選択 | `start_selection` | **「次の候補へ」** → `/lemon_target/release`（案A） |
| 把持（把持姿勢の計算） | `plan_grasp` | `/lemon_approach/approach` |
| 把持 | `grasp` | `/lemon_grasp/grasp` |
| 収穫位置 ①②③ | `goto_basket` | **相手に無い。当面は隠す**（§6） |
| 開放（収穫完了） | `release` | `/lemon_grasp/open` → `/lemon_target/release` の2段 |
| ホームへ戻す | `home` | `/arm_home/go_home` |
| 停止（赤） | `abort` | `/lemon_target/release`（§制約あり） |
| **除外をクリア**（新規） | — | `/lemon_target/clear_exclusions` |

**`/arm_ui/command` のコマンド名は変えない。** 中身の動きだけを差し替える。
ブラウザ側で変わるのは、ボタンのラベルと `updateButtons()` の有効条件だけ。

### 「UI操作を有効化」トグルの意味

いまは「`/joint_states` に publish するか」。`backend: lemon` では publish しないので、
**「サービスを呼んでよいか」の元栓**に意味を変える。
OFF の間はボタンを押してもサービスを呼ばない（＝実機が動かない）。

安全弁（`_check_command_topic_owner`）は `backend: internal` のときだけ効けばよい。

---

## 6. 収穫位置 ①②③ をどうするか

相手の流れは **掴む → home へ戻る → 開く** で、かごへ運ぶ段が無い。
3つの選択肢がある。

| | |
|---|---|
| **隠す**（当面これを勧める） | ボタンを非表示にする。`backend: lemon` のときだけ隠せばよい |
| 残して別に作る | かご姿勢へ MoveIt で動かすノードを自分で足す。相手の流れに1段挟む形になる |
| 相手に足してもらう | かごへ運ぶ段を `lemon_harvest_manager` に入れてもらう |

**決まるまでは隠す。** 押しても何も起きないボタンが一番よくない。

---

## 7. 状態の対応

UI の12状態は残し、**遷移を「軌道の完了」から「サービスの応答」に変える**。

| 相手の status | UI の状態 | 画面表示 |
|---|---|---|
| （対象なし） | `IDLE` | 待機 |
| 対象が固定された | `SELECTED` | 対象を選択済み |
| `approaching` | `PLANNING` → `PLANNED` | 把持姿勢へ移動中 → 到達 |
| `grasping` / `inserting` / `closing` | `GRASPING` | 把持中（`/lemon_grasp/status` で細かく） |
| `grasp` 成功 | `GRASPED` | 把持完了 |
| `grasp` が `missed` | `IDLE` | **「掴めませんでした」**（異常ではない） |
| `going_home` | `MOVING_TO_BASKET` | ホームへ移動中 |
| `opening` | `RELEASING` | 開放中 |
| 一連の完了 | `DONE` | 収穫完了 |
| サービスが失敗 | `ERROR` | 失敗の message をそのまま |

**`missed` を `ERROR` にしないこと。** 動作は正常に終わっていて、掴めなかっただけ。

---

## 8. 座標指定・関節ジョグ

`backend: lemon` では `/joint_states` に publish できないので、**いまのままでは動かない**。

| | |
|---|---|
| **当面は隠す**（勧める） | `backend: lemon` のとき非表示。収穫シーケンスを先に通す |
| 後で作り直す | `/arm_controller/follow_joint_trajectory` に短い軌道を投げる。`moveit_helper.compute_ik` で IK を解く |

**収穫シーケンスが通ってから手を付ける。** 両方を同時に直すと切り分けができない。

---

## 9. テストの手当て

`tests/mini_ros.py` に**サービスの模擬が無い**。段階3で足す。

- `create_client` / `create_service` / `Trigger` の模擬
- 応答を仕込めるようにする（成功・失敗・タイムアウト・`busy`）
- `MultiThreadedExecutor` 相当（スタブなので実スレッドは要らない。
  呼び出しが即座に返る形でよい）

新しいテストで押さえること:

| | |
|---|---|
| ボタン1つがサービス1つを正しく呼ぶ | コマンド名とサービス名の対応 |
| 失敗の message が画面に出る | `missed` が `ERROR` にならないことを含む |
| サービスが居ないときに固まらない | タイムアウトして理由を出す |
| **呼び出し中も状態配信が止まらない** | §3 の本題 |
| `backend: internal` が壊れていない | 既存110項目がそのまま通ること |

---

## 10. まとめ：着手の順

1. **案A／案Bを決める**（§2）→ 案Aなら即着手できる
2. 段階0で現状を自分の目で確かめる
3. 段階1（状態表示）→ **ここまでで既に実用になる**
4. 段階2（検出アダプタ）
5. 段階3（土台）← **一番慎重に。3Dビューが止まらないことを必ず確認**
6. 段階4 → 5 → 6

**段階1〜3はアームが動かない。** そこまでは落ち着いて進められる。
段階4から実機が動くので、物理的な非常停止を手元に置くこと。
