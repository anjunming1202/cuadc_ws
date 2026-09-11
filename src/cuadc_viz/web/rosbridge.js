// 手写的极简 rosbridge v2 协议客户端，不引入 roslibjs——这个页面只需要
// subscribe/call_service 两种操作，几十行够用，换来的是前端零第三方
// 依赖（现场网络可能没有外网，不想还要考虑 vendoring 一个 JS 库的问题）。
// 协议本身是 rosbridge_suite 的标准 JSON over WebSocket，字段名不是这里
// 发明的。

export class RosBridge {
  constructor(url) {
    this.url = url;
    this.ws = null;
    this._subs = new Map(); // topic -> Set(callback)
    this._pending = new Map(); // service call id -> {resolve, reject}
    this._callId = 0;
    this._onStatus = null;
  }

  onStatus(cb) { this._onStatus = cb; }

  connect() {
    this.ws = new WebSocket(this.url);
    this.ws.onopen = () => {
      this._onStatus && this._onStatus('connected');
      // rosbridge 的订阅状态是连接级别的，重连是一个全新的 WebSocket——
      // 服务端不记得之前订阅过什么，必须重发一遍，不然断线重连之后
      // 页面会安安静静地再也收不到任何更新，界面卡在最后一帧不报错。
      for (const [topic, { type }] of this._subs) this._send({ op: 'subscribe', topic, type });
    };
    this.ws.onclose = () => {
      this._onStatus && this._onStatus('disconnected');
      // 断线时任何还没收到回应的 call_service 调用永远不会再有回应了
      // （旧连接已经死了）——不 reject 的话调用方的 Promise 会一直挂着，
      // 表现是"点了按钮没反应也不报错"。
      for (const { reject } of this._pending.values()) reject(new Error('rosbridge 连接已断开'));
      this._pending.clear();
      setTimeout(() => this.connect(), 2000); // 断线重连，监测工具不能一断就得手动刷新
    };
    this.ws.onerror = () => this.ws.close();
    this.ws.onmessage = (ev) => this._onMessage(JSON.parse(ev.data));
  }

  _send(obj) { this.ws.readyState === WebSocket.OPEN && this.ws.send(JSON.stringify(obj)); }

  subscribe(topic, type, cb) {
    if (!this._subs.has(topic)) {
      this._subs.set(topic, { type, callbacks: new Set() });
      this._send({ op: 'subscribe', topic, type });
    }
    this._subs.get(topic).callbacks.add(cb);
  }

  callService(service, type, args) {
    const id = `call_${this._callId++}`;
    this._send({ op: 'call_service', service, type, args, id });
    return new Promise((resolve, reject) => this._pending.set(id, { resolve, reject }));
  }

  _onMessage(msg) {
    if (msg.op === 'publish') {
      const sub = this._subs.get(msg.topic);
      sub && sub.callbacks.forEach((cb) => cb(msg.msg));
    } else if (msg.op === 'service_response') {
      const p = this._pending.get(msg.id);
      if (!p) return;
      this._pending.delete(msg.id);
      msg.result === false ? p.reject(msg.values) : p.resolve(msg.values);
    }
  }
}
