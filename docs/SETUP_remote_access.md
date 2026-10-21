# 別ネットワークから UI を使う

同一 LAN の外（自宅、客先、出張先）からこの UI を開くための手順。

- **チーム内で使う** → 方法 A（Tailscale）。これが本命。
- **外部の人に見せる** → 方法 B（公開トンネル）。閲覧のみで渡す。

---

## 0. まず前提を 2 つ

### 0.1 この UI には認証がない

ログイン画面も、パスワードも、ユーザ管理もない。
**URL を知っている人 = 実機のアームを動かせる人**である。
LAN の中ならそれで問題なかったが、外に出す以上そのままでは危ない。

そこで v0.2.0 から、**「手元」と「遠隔」で権限を分ける**ようにした。

| | 見る | 動かす | 非常停止 |
|---|---|---|---|
| 手元（LAN / VPN） | ○ | ○ | ○ |
| 遠隔（トンネル経由） | ○ | **×**（手元が許可した間だけ ○） | ○ |

「動かす」を止めているのは**サーバ側**であって、画面のボタンではない。
ボタンを隠すだけでは意味がない（ブラウザの devtools から WebSocket に
直接 publish できてしまう）。遠隔からの `publish` は
`ui_server` の中継のところで捨てられるので、何を送っても ROS に届かない。

**非常停止だけは遠隔でも常に通る。**
「危ないと思った人が止められない」ほうが危険で、止める操作はアームを
より安全な側にしか動かさないため。中身まで見て、
`{"cmd":"abort"}` と `stop` そのものだけを通している。

### 0.2 通信路が 1 本にまとまった

以前はブラウザが 2 か所に繋いでいた（UI が 8080、rosbridge が 9090）。
v0.2.0 では rosbridge へも `ui_server` 経由（`/rosbridge`）で繋ぐ。

```
    ブラウザ ──┬── 8080 /              UI（HTML/CSS/JS）
               ├── 8080 /rosbridge  ──→ 127.0.0.1:9090 rosbridge
               └── 8080 /offer         WebRTC のシグナリング
```

これで、

- **外に開けるポートは 1 つだけ**（トンネル 1 本で済む）
- **https で開いても動く**（`wss://` になるので混在コンテンツで弾かれない）
- **権限判定が 1 か所に集まる**

rosbridge 自体は既定で `127.0.0.1` にしか開かない。
外から 9090 を直接叩けたら中継を素通りできてしまうため。

---

## 方法 A: Tailscale（チーム内。おすすめ）

ロボットと、見る側の端末を同じ仮想 LAN に入れる。

**コードの変更はいらない。** UI からは同一 LAN と区別がつかないので、
**手元扱いになりフル操作できる**（`100.64.0.0/10` を最初から手元に入れてある）。

### 手順

ロボット側：

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
# 表示された URL をブラウザで開いてログイン
tailscale ip -4        # 100.x.y.z が出る。これがロボットのアドレス
```

見る側（PC / タブレット / スマホ）：

1. Tailscale アプリを入れて同じアカウントでログイン
2. ブラウザで `http://100.x.y.z:8080/` を開く

以上。UI 側の設定変更は不要。

### 注意

- **無料枠は 6 ユーザまで**（端末数は無制限）。チーム内なら足りるはず。
- 見る人全員にアプリを入れてもらう必要がある。
  **外部の人に見せる用途には向かない。**
- ロボット側は再起動しても自動で繋がる（`tailscaled` が常駐する）。

### 映像も見たいとき

Tailscale は WireGuard の直結なので、たいていそのまま WebRTC も通る。
繋がらない場合だけ STUN を足す：

```bash
ros2 launch piper_arm_ui arm_ui.launch.py \
    ice_servers:=stun:stun.l.google.com:19302
```

---

## 方法 B: 公開トンネル（外部の人に見せる）

アプリを入れてもらえない相手に、URL だけで見せる。
**閲覧専用で渡すのが前提。**

### B-1. 公開用ポートを開けて立ち上げる

```bash
ros2 launch piper_arm_ui arm_ui.launch.py \
    public_port:=8081 \
    ice_servers:=stun:stun.l.google.com:19302
```

`public_port` を指定すると、UI が **8080（手元用）と 8081（公開用）の 2 口**で
待ち受けるようになる。中身は同じだが、

- **8081 に届いた接続は、送信元が何であろうと必ず遠隔扱い**
- 8080 はこれまでどおり

**トンネルは必ず 8081 に向けること。** これが一番確実な形。
なぜ「ヘッダを見る」だけにしないのかは §手元／遠隔の見分けかた に書いた。

さらに固くしたいなら、操作を完全に封じる：

```bash
ros2 launch piper_arm_ui arm_ui.launch.py \
    public_port:=8081 allow_remote_control:=false \
    ice_servers:=stun:stun.l.google.com:19302
```

`allow_remote_control:=false` にすると、**手元からも許可を出せなくなる**。
デモの最中に誤って開いてしまう事故が起きない。
あとから操作させる可能性があるなら既定の `true` のままでよい
（その場合も遠隔は最初は閲覧のみで、手元で許可を出した 10 分間だけ動く）。

### B-2. トンネルを張る

**cloudflared（推奨）**

```bash
# 入れる
curl -L https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64.deb -o cloudflared.deb
sudo dpkg -i cloudflared.deb

# 使い捨ての URL を発行する。**公開用ポート (8081) に向ける**
cloudflared tunnel --url http://localhost:8081
# → https://<ランダム>.trycloudflare.com が表示される
```

**ngrok**

```bash
ngrok http 8081
```

どちらも 1 本通すだけでよい。9090 は要らない。
**8080 のほうを間違えて公開しないこと**（そちらは手元用で、フル操作できてしまう）。

### B-3. 渡した相手の画面

上部に黄色い帯で「閲覧のみ」と出て、操作系のボタンが押せなくなる。
「操作を申請」ボタンを押すと、手元の画面に青い帯で申請が出る。

### B-4. 一時的に操作させる

手元（ロボットの隣、または VPN 越し）の画面で「10分だけ許可」を押す。

- 許可中は手元の帯が**黄色**に変わり、残り秒数が出る。
  外から実機が動く状態なので、目を離さないこと。
- **10 分で自動的に閉じる**。切り忘れても開きっぱなしにはならない。
- 「許可を取り消す」で即座に閉じる。
- 許可を出せるのは手元だけ。遠隔から `/api/control/grant` を叩いても 403。

秒数を変えるには：

```bash
ros2 launch piper_arm_ui arm_ui.launch.py control_grant_seconds:=120.0
```

### B-5. URL 自体に鍵をかけたいとき

上の構成では、**URL を知っていれば誰でも「閲覧」はできる**。
カメラ映像が見えるので、それも困る場合はトンネル側で認証をかける。

- **Cloudflare Access**（無料枠あり）: 名前付きトンネルにして、
  許可するメールアドレスを列挙する。相手はワンタイムコードでログインする。
- **ngrok**: `ngrok http 8081 --basic-auth "user:password"`

UI 側には手を入れない。認証はトンネルの層でやるのが素直。

### 注意

- **`trycloudflare.com` の URL は起動ごとに変わる。** 固定したいなら
  名前付きトンネル（要ドメイン）にする。
- 映像は WebRTC なのでトンネルを通らず**直接**繋ごうとする。
  NAT が厳しいと繋がらないことがある。その場合は TURN が要る（下記）。
- **Cloudflare の WebSocket は全プランで使えるが、既定で有効とは限らない。**
  `/rosbridge` が繋がらないときは Network 設定で WebSockets を確認する。

---

## 映像が出ないとき（WebRTC と ICE）

UI 本体と状態表示（3D ビュー含む）は WebSocket なのでトンネルを通る。
**カメラ映像だけは WebRTC** で、別経路を掘ろうとする。

| 状況 | 必要なもの |
|---|---|
| 同一 LAN | なし（既定） |
| Tailscale | たいてい なし。だめなら STUN |
| 公開トンネル | STUN（多くの場合これで足りる） |
| 相手が厳しい NAT / 企業網 | TURN |

```bash
# STUN
ros2 launch piper_arm_ui arm_ui.launch.py \
    ice_servers:=stun:stun.l.google.com:19302

# TURN も（カンマ区切り。user:pass@ の形で書ける）
ros2 launch piper_arm_ui arm_ui.launch.py \
    ice_servers:=stun:stun.l.google.com:19302,turn:user:pass@turn.example.com:3478
```

TURN は自前で建てるなら coturn。中継するので帯域を食う（1 視聴者あたり
映像 3 本 ≒ 2Mbps）。**映像が要らないなら 3D ビューだけで運用するほうが軽い**
（3D ビューは関節角しか流れないので数 kbps）。

---

## 起動オプション早見

| 引数 | 既定 | 意味 |
|---|---|---|
| `public_port` | `0`（無効） | 公開用の待ち受けポート。**ここに届いた接続は必ず遠隔**。トンネルはここへ |
| `allow_remote_control` | `true` | `false` で遠隔からの操作を完全に封じる（非常停止を除く） |
| `control_grant_seconds` | `600.0` | 1 回の許可で操作できる秒数 |
| `trusted_networks` | 空（＝下記の既定） | 「手元」とみなす送信元。カンマ区切り |
| `ice_servers` | 空 | WebRTC の STUN / TURN。カンマ区切り |
| `rosbridge_address` | `127.0.0.1` | rosbridge の待ち受け。**外に開かないこと** |

`trusted_networks` の既定：

```
127.0.0.0/8, ::1/128, 10.0.0.0/8, 172.16.0.0/12, 192.168.0.0/16,
fc00::/7, 100.64.0.0/10
```

最後の `100.64.0.0/10` が Tailscale。ここを外すと Tailscale 越しの端末も
遠隔扱い（閲覧のみ）になる。**逆に、その扱いにしたいときは外せばよい。**

---

## 手元／遠隔の見分けかた

`ui_server` は上から順に、確実なものから見ている。

1. **`public_port` に届いた接続** → 必ず遠隔。**これが一番確実**
2. `X-Forwarded-For` / `X-Real-IP` / `CF-Connecting-IP` / `Forwarded` の
   いずれかが付いていたら → 遠隔
3. どちらでもなければ、送信元アドレスが `trusted_networks` に入っているかで判定

2 が要る理由：トンネルはロボット上の `localhost` から `ui_server` に繋ぐので、
送信元アドレスだけ見ると `127.0.0.1` = 手元に見えてしまう。
トンネルはこの類のヘッダを付けるので、それを遠隔の印にしている。

**ただし 2 だけを当てにしてはいけない。** ヘッダを付けない中継もあり得るし、
**WebSocket の upgrade だけヘッダが落ちる**実装も実在する
（開発中に Chromium で確認した。extra_http_headers が WS に付かない）。
そこがちょうど権限判定の入口なので、落ちると静かに穴が開く。
**1 の方式にはその弱点がない**ので、外に出すときは `public_port` を使うこと。

**3 の副作用**：自前で nginx などのリバースプロキシを前段に置くと、
LAN からのアクセスも遠隔扱いになる。その構成にするなら
プロキシがこれらのヘッダを付けないようにするか、UI に直接繋ぐこと。

---

## 動作確認

```bash
python3 /root/work/e2e/test_remote_access.py    # サーバ側の遮断（25項目）
python3 /root/work/e2e/test_remote_browser.py   # 遠隔の人の画面（本物の Chromium）
```

前者は偽の rosbridge を立てて、遠隔からの publish が本当に届かないことを見る。
権限判定を消すとこのテストは落ちる（確認済み）。
後者は公開用ポートを実際に開き、本物のブラウザで
「押しても進まない → 手元が許可 → 進む → 取り消し → また進まない」を通す。

手動で確かめるなら、LAN の中からでもヘッダを付ければ遠隔のふりができる：

```bash
# 遠隔として見た /api/info
curl -H 'X-Forwarded-For: 203.0.113.9' http://localhost:8080/api/info
#   "trusted": false  が返れば正しい

# 遠隔として許可を出そうとする → 403
curl -X POST -H 'X-Forwarded-For: 203.0.113.9' http://localhost:8080/api/control/grant
```
