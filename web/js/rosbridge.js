/*
 * 依存ゼロの rosbridge クライアント。
 * roslibjs を CDN から読み込む構成だと農場でオフラインのときに動かないため、
 * rosbridge の WebSocket プロトコル（JSON）だけを自前で実装している。
 *
 *   const ros = new RosBridge('ws://' + location.hostname + ':9090');
 *   ros.subscribe('/arm_ui/state', 'std_msgs/msg/String', msg => {...});
 *   ros.publish('/arm_ui/command', 'std_msgs/msg/String', {data: '{...}'});
 */
class RosBridge {
  constructor(url) {
    this.url = url;
    this.ws = null;
    this.connected = false;
    this.subscribers = new Map();   // topic -> {type, handlers:[]}
    this.advertised = new Set();
    this.serviceCallbacks = new Map();
    this.listeners = { connect: [], close: [], error: [] };
    this._serviceSeq = 0;
    this._reconnectDelay = 1000;
    this.connect();
  }

  on(event, handler) {
    if (this.listeners[event]) this.listeners[event].push(handler);
    return this;
  }

  _emit(event, arg) {
    (this.listeners[event] || []).forEach((h) => {
      try { h(arg); } catch (e) { console.error(e); }
    });
  }

  connect() {
    try {
      this.ws = new WebSocket(this.url);
    } catch (e) {
      this._scheduleReconnect();
      return;
    }

    this.ws.onopen = () => {
      this.connected = true;
      this._reconnectDelay = 1000;
      // 再接続時に購読／広告を張り直す
      this.subscribers.forEach((entry, topic) => this._sendSubscribe(topic, entry.type));
      this.advertised.forEach((key) => {
        const [topic, type] = key.split('|');
        this._send({ op: 'advertise', topic, type });
      });
      this._emit('connect');
    };

    this.ws.onclose = () => {
      this.connected = false;
      this._emit('close');
      this._scheduleReconnect();
    };

    this.ws.onerror = (e) => this._emit('error', e);

    this.ws.onmessage = (event) => {
      let msg;
      try { msg = JSON.parse(event.data); } catch (e) { return; }
      if (msg.op === 'publish') {
        const entry = this.subscribers.get(msg.topic);
        if (entry) entry.handlers.forEach((h) => h(msg.msg));
      } else if (msg.op === 'service_response') {
        const cb = this.serviceCallbacks.get(msg.id);
        if (cb) {
          this.serviceCallbacks.delete(msg.id);
          cb(msg.values, msg.result !== false);
        }
      }
    };
  }

  _scheduleReconnect() {
    setTimeout(() => this.connect(), this._reconnectDelay);
    this._reconnectDelay = Math.min(this._reconnectDelay * 1.5, 8000);
  }

  _send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj));
      return true;
    }
    return false;
  }

  _sendSubscribe(topic, type) {
    this._send({ op: 'subscribe', topic, type, throttle_rate: 0, queue_length: 1 });
  }

  subscribe(topic, type, handler) {
    let entry = this.subscribers.get(topic);
    if (!entry) {
      entry = { type, handlers: [] };
      this.subscribers.set(topic, entry);
      this._sendSubscribe(topic, type);
    }
    entry.handlers.push(handler);
  }

  advertise(topic, type) {
    const key = `${topic}|${type}`;
    if (this.advertised.has(key)) return;
    this.advertised.add(key);
    this._send({ op: 'advertise', topic, type });
  }

  publish(topic, type, msg) {
    this.advertise(topic, type);
    return this._send({ op: 'publish', topic, msg });
  }

  callService(name, type, args, callback) {
    const id = `call_${++this._serviceSeq}`;
    if (callback) this.serviceCallbacks.set(id, callback);
    this._send({ op: 'call_service', id, service: name, type, args: args || {} });
  }
}

window.RosBridge = RosBridge;
