/* EMAT 只读展示：接收板端算好的同帧曲线，不在浏览器重复计算测厚。 */
"use strict";
(() => {
  const $=id=>document.getElementById(id), section=$("view-emat");
  $("workspace").appendChild(section);
  let socket=null, generation=0, retry=null, desired=true, frame=null, received=0;
  const fmt=(v,n=2)=>Number.isFinite(v)?v.toFixed(n):"—";
  const reasons={waiting_for_waveform:"等待原始波形",waiting_for_processed_waveform:"原始波形已到，等待本帧分析",
    acquisition_identity_unconfirmed:"尚未证明是新的独立采集，仅显示候选结果",algorithm_not_connected:"尚未配置解算声速",
    quality_not_calibrated:"质量门限尚未标定",analysis_failed:"本帧分析失败",thickness_unavailable:"没有可信厚度候选",
    processed_waveform_mismatch:"处理结果与原始波形不匹配",incomplete_waveform:"原始波形不完整",valid:"本帧测量检查通过"};
  function seriesOK(s,count) {
    return s===null||s===undefined||(Number.isInteger(s.count)&&s.count>0&&s.count<=65536&&
      Array.isArray(s.indices)&&Array.isArray(s.values)&&s.indices.length===s.values.length&&
      s.indices.length>0&&s.indices.length<=2048&&s.values.every(Number.isFinite)&&
      s.indices.every((v,i)=>Number.isInteger(v)&&v>=0&&v<count&&(!i||v>s.indices[i-1])));
  }
  function valid(p) {
    if(!p||typeof p!=="object"||typeof p.available!=="boolean")return false;
    if(!p.available)return true;
    const count=p.raw?.count;
    return Number.isInteger(count)&&count>=3&&count<=65536&&p.raw&&seriesOK(p.raw,count)&&
      seriesOK(p.signal,count)&&seriesOK(p.envelope,count)&&Array.isArray(p.echo_indices)&&
      p.echo_indices.length<=65536&&p.echo_indices.every(i=>Number.isInteger(i)&&i>=0&&i<count);
  }
  function palette(name){return getComputedStyle(document.documentElement).getPropertyValue(name).trim();}
  function draw(id,processed) {
    const canvas=$(id), box=canvas.getBoundingClientRect(), width=Math.max(320,box.width||800), height=box.height||280;
    const ratio=Math.min(devicePixelRatio||1,2), ctx=canvas.getContext("2d");
    canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);ctx.scale(ratio,ratio);
    ctx.fillStyle=palette("--canvas");ctx.fillRect(0,0,width,height);
    const curves=frame?.available?(processed?[frame.signal,frame.envelope]:[frame.raw]):[];
    const points=curves.filter(Boolean).reduce((sum,s)=>sum+s.values.length,0);
    canvas.dataset.points=String(points);
    ctx.font="12px sans-serif";ctx.fillStyle=palette("--muted");
    if(!points){ctx.fillText(processed?"暂无本帧处理结果":"等待原始波形",60,75);return;}
    const left=58,right=22,top=26,bottom=42, w=width-left-right,h=height-top-bottom;
    const values=curves.filter(Boolean).flatMap(s=>s.values);
    let low=processed?Math.min(0,...values):0, high=processed?Math.max(0,...values):255;
    if(high-low<1){low-=1;high+=1;}
    if(processed){const pad=(high-low)*.08;low-=pad;high+=pad;}
    const end=Math.max(1,frame.raw.count-1), x=i=>left+i/end*w, y=v=>top+(high-v)/(high-low)*h;
    const timeAxis=$("emat-axis").value==="time"&&Number.isFinite(frame.sample_rate_hz)&&frame.sample_rate_hz>0;
    ctx.strokeStyle=palette("--plot-grid");ctx.lineWidth=1;
    for(let i=0;i<=4;i++){
      const yy=top+i*h/4, xx=left+i*w/4;
      ctx.beginPath();ctx.moveTo(left,yy);ctx.lineTo(left+w,yy);ctx.stroke();
      ctx.beginPath();ctx.moveTo(xx,top);ctx.lineTo(xx,top+h);ctx.stroke();
      ctx.textAlign="right";ctx.fillText((high-i*(high-low)/4).toFixed(0),left-8,yy+4);
      ctx.textAlign="center";const sample=end*i/4;
      ctx.fillText(timeAxis?(sample/frame.sample_rate_hz*1e6).toFixed(1):Math.round(sample),xx,height-22);
    }
    ctx.textAlign="left";ctx.fillText("ADC 计数",left,16);ctx.textAlign="right";
    ctx.fillText(timeAxis?"相对采样时间 / μs":"原始采样点下标（从 0 开始）",width-right,height-5);
    curves.forEach((s,index)=>{
      if(!s)return;ctx.strokeStyle=palette(processed?(index?"--chart-3":"--chart-2"):"--chart-1");
      ctx.lineWidth=index?1.8:1.1;ctx.beginPath();
      s.values.forEach((v,i)=>{if(i)ctx.lineTo(x(s.indices[i]),y(v));else ctx.moveTo(x(s.indices[i]),y(v));});ctx.stroke();
    });
    if(processed){ctx.strokeStyle=palette("--chart-5");ctx.setLineDash([3,5]);
      for(const i of frame.echo_indices){ctx.beginPath();ctx.moveTo(x(i),top);ctx.lineTo(x(i),top+h);ctx.stroke();}ctx.setLineDash([]);}
  }
  function render() {
    const p=frame?.available?frame:null;
    $("emat-speed").textContent=p&&Number.isFinite(p.sound_speed_mps)?fmt(p.sound_speed_mps,1)+" m/s":"—";
    $("emat-thickness").textContent=p&&Number.isFinite(p.thickness_mm)?fmt(p.thickness_mm,4)+" mm":"—";
    $("emat-validity").textContent=p?.measurement_valid?"测量有效（不等于机械贴合确认）":"未通过测量有效性检查";
    $("emat-sampling").textContent=p?`${p.raw.count} 点 / ${Number.isFinite(p.sample_rate_hz)?fmt(p.sample_rate_hz/1e6,2)+" MHz":"未知"}`:"—";
    $("emat-source").textContent=p?(p.source==="file_replay"?"文件回放":p.source==="emat_serial"?"检测仪串口":String(p.source)):"—";
    $("emat-calibration").textContent=p?.quality_calibrated?"已配置标定："+p.quality_profile_id:"未标定，数值仅作候选";
    $("emat-reason").textContent=frame?(reasons[frame.reason]||String(frame.reason||"状态未提供")):"等待采集节点的原始波形及分析结果。";
    $("emat-frame").textContent=p?`会话 ${p.session_id} · 采集/读取 ${p.acquisition_id}`:"尚无采集标识";
    $("emat-raw-hint").textContent=p?`完整 ${p.raw.count} 点 · 预览 ${p.raw.values.length} 点 · ADC 基线 ${fmt(p.adc_zero,1)}`:"尚无原始数据";
    $("emat-processed-hint").textContent=p?.processing_valid?`从第 ${p.start_sample} 点开始处理 · 底波候选 ${p.echo_indices.length} 个 · 各曲线按原始下标对齐`:"尚无本帧处理结果，不显示上一帧曲线";
    draw("emat-raw",false);draw("emat-processed",true);age();
  }
  function age(){
    $("emat-age").textContent=frame?.available&&Number.isFinite(frame.waveform_age_s)?
      `距桥接收到原始波形 ${fmt(frame.waveform_age_s+(performance.now()-received)/1000,1)} s`:
      "未收到波形；等待单次采集完成";
  }
  function close() {
    generation++;clearTimeout(retry);retry=null;frame=null;
    if(socket){socket.close();socket=null;}render();
  }
  function connect(address) {
    close();desired=true;let endpoint;
    try {
      endpoint=new URL(address||window.RobotConnectionDefaults?.emat||window.RobotConnectionDefaults?.robot||
        `${location.protocol==="https:"?"wss:":"ws:"}//${location.hostname||"127.0.0.1"}:9001/ws/emat`);
      if(!["ws:","wss:"].includes(endpoint.protocol)||endpoint.username||endpoint.password)throw Error("无效协议或凭据");
      endpoint.pathname="/ws/emat";endpoint.search="";endpoint.hash="";
    } catch {$("emat-link").textContent="地址无效";return;}
    const mine=++generation, ws=socket=new WebSocket(endpoint.href);
    $("emat-link").textContent="只读连接中";
    $("emat-link").className="badge warn";
    ws.onopen=()=>{if(mine===generation){$("emat-link").textContent="已连接 · 只读";$("emat-link").className="badge good";}};
    ws.onmessage=event=>{
      if(mine!==generation||typeof event.data!=="string"||event.data.length>1048576)return;
      let packet;try{packet=JSON.parse(event.data);}catch{return;}
      if(packet.header?.robot_id!=="tank_inspection_robot"||packet.header?.msg_type!=="emat_waveform"||!valid(packet.payload))return;
      frame=packet.payload;received=performance.now();render();
    };
    ws.onerror=()=>{if(mine===generation)$("emat-link").textContent="只读通道连接失败";};
    ws.onclose=()=>{if(mine!==generation)return;socket=null;frame=null;render();$("emat-link").textContent=desired?"等待只读重连":"已断开";$("emat-link").className="badge warn";
      if(desired)retry=setTimeout(()=>connect(endpoint.href),2000);};
  }
  $("emat-axis").onchange=render;
  new ResizeObserver(()=>{draw("emat-raw",false);draw("emat-processed",true);}).observe(section);
  document.querySelector("nav [data-view=emat]").addEventListener("click",render);
  $("connect").addEventListener("click",()=>connect($("endpoint").value));
  $("disconnect").addEventListener("click",()=>{desired=false;close();$("emat-link").textContent="已断开";});
  window.addEventListener("pagehide",()=>{desired=false;close();});
  window.addEventListener("pageshow",event=>{if(event.persisted)connect();});
  setInterval(age,1000);render();connect();
})();
