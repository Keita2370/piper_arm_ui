"""piper_arm_ui 共通定義.

PiPER の関節構成・可動範囲・UI とのやり取りに使うトピック名をここに集約する。
可動範囲は既存の joy_to_piper_jointstates.cpp と揃えてある。
"""

# piper_ros が /joint_states で受け付ける関節名（joint7/8 はグリッパ）
JOINT_NAMES = [
    'joint1', 'joint2', 'joint3', 'joint4',
    'joint5', 'joint6', 'joint7', 'joint8',
]

# (下限, 上限) [rad]。joint7/8 はグリッパ開度 [m]。
JOINT_LIMITS = {
    'joint1': (-2.618, 2.618),
    'joint2': (0.0, 3.14),
    'joint3': (-2.967, 0.0),
    'joint4': (-1.745, 1.745),
    'joint5': (-1.220, 1.220),
    'joint6': (-2.094, 2.094),
    'joint7': (0.0, 0.035),
    'joint8': (-0.035, 0.0),
}

# グリッパ開度
GRIPPER_OPEN = 0.035
GRIPPER_CLOSED = 0.0

# --- UI とのやり取り（rosbridge 経由）。JSON 文字列を std_msgs/String で運ぶ ---
TOPIC_UI_COMMAND = '/arm_ui/command'   # UI -> シーケンサ
TOPIC_UI_STATE = '/arm_ui/state'       # シーケンサ -> UI
TOPIC_UI_LOG = '/arm_ui/log'           # シーケンサ -> UI（イベントログ）
TOPIC_DETECTIONS = '/arm_ui/detections'  # 認識ノード -> UI/シーケンサ

# --- チームの収穫システム（~/lemon_ws）。段階1では購読して表示するだけ ---
# どれも std_msgs/String の素の文字列（JSON ではない）。
# manager は auto_run:=false では起動しないので、居ないのが普通の構成がある
TOPIC_TEAM_MANAGER_STATUS = '/lemon_harvest_manager/status'
TOPIC_TEAM_APPROACH_STATUS = '/lemon_approach/status'
TOPIC_TEAM_GRASP_STATUS = '/lemon_grasp/status'
# geometry_msgs/PointStamped（latched）。frame_id が空なら「対象なし」
TOPIC_TEAM_TARGET = '/lemon_target/target'

# 相手の status 文字列 → (種別, 画面表示)。
# 種別: idle / busy / ok / missed / fail。表に無い文字列は呼び出し側で fail 扱いにする
# （相手は失敗理由をそのまま status に流すので、列挙しきれない）。
# 文字列はソース（lemon_harvest_manager_node.py / lemon_approach_node.py /
# lemon_grasp_node.cpp）から拾ったもの。2026-10-30 時点。
TEAM_STATUS_LABELS = {
    'manager': {
        'waiting_target': ('idle', '対象を待っています'),
        'approaching': ('busy', '把持姿勢へ移動中'),
        'grasping': ('busy', '把持中'),
        'going_home': ('busy', 'ホームへ移動中'),
        'opening': ('busy', 'グリッパを開いています'),
        'releasing': ('busy', '対象を解除しています'),
        'error': ('fail', '停止（ホームへ戻れませんでした）'),
    },
    'approach': {
        'approaching': ('busy', '把持姿勢へ移動中'),
        'reached': ('ok', '把持姿勢に到達'),
    },
    'grasp': {
        # grasp の最初に出る。/lemon_grasp/open（開放）はこの status を出さない
        'opening': ('busy', 'グリッパを開いています（把持の準備）'),
        'inserting': ('busy', '差し込み中'),
        'closing': ('busy', 'グリッパを閉じています'),
        # 戻り動作は対象を解除しても止まらない（ワークスペースの CLAUDE.md §5）
        'retreating': ('busy', '戻り中（ソフト停止では止まりません）'),
        'grasped': ('ok', '把持成功'),
    },
}

# --- RViz 埋め込み ---
TOPIC_RVIZ_IMAGE = '/rviz_ui/image'     # rviz_bridge -> ui_server（画面キャプチャ）
TOPIC_RVIZ_INPUT = '/rviz_ui/input'     # UI -> rviz_bridge（マウス操作）

# シーケンサの状態
STATE_IDLE = 'IDLE'                 # 待機
STATE_SELECTING = 'SELECTING'       # 把持対象の選択中
STATE_SELECTED = 'SELECTED'         # 対象確定
STATE_PLANNING = 'PLANNING'         # 把持姿勢の計算中
STATE_PLANNED = 'PLANNED'           # 計画完了
STATE_GRASPING = 'GRASPING'         # 把持動作中
STATE_GRASPED = 'GRASPED'           # 把持完了
STATE_MOVING_TO_BASKET = 'MOVING_TO_BASKET'  # 収穫位置へ移動中
STATE_AT_BASKET = 'AT_BASKET'       # 収穫位置到達
STATE_RELEASING = 'RELEASING'       # 開放中
STATE_DONE = 'DONE'                 # 収穫完了
STATE_ERROR = 'ERROR'               # 異常

# UI 表示用の日本語ラベル
STATE_LABELS = {
    STATE_IDLE: '待機',
    STATE_SELECTING: '把持対象を選択中',
    STATE_SELECTED: '対象を選択済み',
    STATE_PLANNING: '把持姿勢を計算中',
    STATE_PLANNED: '把持姿勢の計算完了',
    STATE_GRASPING: '把持中',
    STATE_GRASPED: '把持完了',
    STATE_MOVING_TO_BASKET: '収穫位置へ移動中',
    STATE_AT_BASKET: '収穫位置に到達',
    STATE_RELEASING: '開放中',
    STATE_DONE: '収穫完了',
    STATE_ERROR: '異常',
}


def clamp_joint(name: str, value: float) -> float:
    """関節可動範囲でクランプする."""
    lo, hi = JOINT_LIMITS.get(name, (-3.15, 3.15))
    return max(lo, min(hi, value))


def clamp_pose(pose):
    """joint1..joint6 の配列をクランプして返す."""
    return [clamp_joint(JOINT_NAMES[i], float(v)) for i, v in enumerate(pose[:6])]
