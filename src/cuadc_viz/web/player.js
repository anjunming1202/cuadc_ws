// 视图播放器：原始画面/调试图像/侦察图传三处共用的同一份逻辑。只关心
// "现在该不该播"这一个布尔输入，不关心是谁触发的（勾选框/点格子+排队/
// 任务阶段自动判断都是外层选择器的事，见各自的调用点）。四种状态：
// 未开启 / 已选中等待首帧 / 正在接收 / 已选中但未收到。

const RECEIVE_TIMEOUT_MS = 4000;

export function createViewPlayer(tileEl, topic, videoSrc) {
  const img = tileEl.querySelector('img');
  const status = tileEl.querySelector('.view-tile-status');
  let timeoutHandle = null;

  // MJPEG 是靠 <img> 直接拉 multipart 流，浏览器对这种流的 load 事件
  // 触发时机不完全一致（有的只在收到第一部分时触发一次）——这里只当
  // 它是"至少收到过一帧"的信号，不是持续的活体检测，够用，见
  // cuadc_viz/README.md 的说明。
  img.addEventListener('load', () => {
    clearTimeout(timeoutHandle);
    tileEl.classList.add('receiving');
  });
  img.addEventListener('error', () => {
    clearTimeout(timeoutHandle);
    tileEl.classList.remove('receiving');
    status.textContent = '已选中但未收到画面';
  });

  return {
    setActive(active) {
      clearTimeout(timeoutHandle);
      tileEl.classList.toggle('active', active);
      tileEl.classList.remove('receiving');
      if (!active) {
        img.src = '';
        status.textContent = '未开启';
        return;
      }
      status.textContent = '等待画面…';
      img.src = videoSrc(topic);
      timeoutHandle = setTimeout(() => {
        if (!tileEl.classList.contains('receiving')) status.textContent = '已选中但未收到画面';
      }, RECEIVE_TIMEOUT_MS);
    },
  };
}
