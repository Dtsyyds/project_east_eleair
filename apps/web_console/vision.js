/* Passive D405 experiment controls; never sends chassis or recording commands. */
"use strict";
(() => {
  const $ = id => document.getElementById(id);
  const actions = ['vision.start', 'vision.capture_start', 'vision.stop', 'vision.save_sample', 'vision.select_candidate', 'vision.clear_selection'];
  const candidateColors = ['#e879f9','#22d3ee','#fb923c'];
  const candidateRequests = new Map(), pressedRequests = new Map();
  let selectionSession=null;
  const labels = {idle:'未启动', starting:'启动中', running:'运行中', stopping:'停止中', stopped:'已停止', failed:'进程异常', unavailable:'服务未配置'};
  let api, value=null, received=0, error='';
  function receive(packet) {
    const p=packet.payload;
    if(p.vision && !(p.vision.service_id===value?.service_id && p.vision.revision<value?.revision)) {
      value=p.vision;received=performance.now();
    }
    if(p.action?.startsWith('vision.') && p.status==='failed')error=p.error||'视觉巡迹操作失败';
    if(actions.includes(p.action) && p.status==='completed')error='';
  }
  function render(connected,replayBusy) {
    const fresh=connected&&value&&performance.now()-received<3500;
    const busy=actions.some(a=>api.inFlight(a))||value?.busy;
    const status=value?.status;
    $('vision-start').disabled=!fresh||!api.gate('vision.start').allowed||busy||replayBusy||['starting','running','stopping'].includes(status);
    $('vision-capture-start').disabled=!fresh||!api.gate('vision.capture_start').allowed||busy||replayBusy||['starting','running','stopping'].includes(status);
    $('vision-stop').disabled=!fresh||!api.gate('vision.stop').allowed||busy||!['starting','running','failed'].includes(status);
    $('vision-save').disabled=!fresh||!api.gate('vision.save_sample').allowed||busy||status!=='running'||!value?.diagnostics_fresh;
    $('vision-start').textContent=status==='starting'?'正在启动…':'启动视觉巡迹';
    $('vision-stop').textContent=status==='stopping'?'正在停止…':value?.capture_only?'停止图片采集':'停止视觉巡迹';
    $('vision-save').textContent=api.inFlight('vision.save_sample')?'正在保存…':value?.capture_only?'保存一组原图':'保存实验样本';
    const sample=value?.sample;
    let message=!connected?'连接断开；巡迹运行状态待确认':!api.gate('vision.start').allowed?'板端未提供视觉巡迹操作能力':!fresh?'等待当前巡迹运行状态':
      (labels[status]||'状态未知')+' · '+(value?.algorithm_label||'Color + Depth')+(value?.saved_count!=null?' · 已保存 '+value.saved_count+'/30 组':'');
    if(fresh&&status==='failed'&&value.message)message+='；'+value.message;
    if(fresh&&status==='running'&&!value.diagnostics_fresh)message+=' · 节点诊断已过期';
    if(replayBusy)message+=' · 回放期间不可启动';
    if(sample?.message)message+='；'+sample.message;
    $('vision-message').textContent=error||message;
    $('vision-message').className='reason '+(error||sample?.status==='failed'||status==='failed'?'bad':'muted');
    $('vision-directory').textContent=value?.directory||'启动后由板端生成实验目录';
    $('vision-sample-path').textContent=sample?.path||'尚无已确认保存的样本';
  }
  function init(options) {
    api=options;
    for(const [id,action] of [['vision-start','vision.start'],['vision-capture-start','vision.capture_start'],['vision-stop','vision.stop'],['vision-save','vision.save_sample']]) {
      $(id).onclick=()=>{error='';api.send(action,{});};
    }
    for(let i=1;i<=3;i++) {
      const button=$('vision-candidate-'+i);
      button.style.borderColor=candidateColors[i-1];
      button.onpointerdown=()=>{pressedRequests.set(i,candidateRequests.get(i));};
      button.onpointercancel=()=>pressedRequests.delete(i);
      button.onclick=event=>{
        const request=event.detail ? pressedRequests.get(i)||candidateRequests.get(i) : candidateRequests.get(i);
        pressedRequests.delete(i);
        if(request){error='';api.send('vision.select_candidate',request);}
      };
    }
    $('vision-clear-selection').onclick=()=>{
      if(selectionSession){error='';api.send('vision.clear_selection',{session_id:selectionSession});}
    };
  }
  function renderCandidates(resultRecord, usable) {
    const r=resultRecord?.data;
    const supported=value?.diagnostics?.candidate_selection_supported===true;
    const fresh=value?.status==='running'&&value.diagnostics_fresh&&performance.now()-received<3500;
    const busy=actions.some(a=>api.inFlight(a))||value?.busy;
    const candidates=r?.session_id===value?.session_id? r?.candidates||[] : [];
    const selection=usable?r?.selection:value?.diagnostics?.selection;
    const manual=selection&&selection.state!=='AUTO';
    selectionSession=value?.session_id||null;
    $('vision-candidates').hidden=!supported;
    candidateRequests.clear();
    for(let i=1;i<=3;i++) {
      const c=candidates.find(c=>String(c.candidate_id)===String(i));
      const button=$('vision-candidate-'+i);
      button.hidden=!c;
      button.disabled=!c||!fresh||!usable||busy||!api.gate('vision.select_candidate').allowed;
      button.textContent=c?`选择候选 ${i} · 评分 ${Number(c.score).toFixed(2)}`:`选择候选 ${i}`;
      if(!button.disabled)candidateRequests.set(i,{session_id:r.session_id,candidate_set_id:r.candidate_set_id,candidate_id:String(c.candidate_id)});
    }
    $('vision-clear-selection').disabled=!fresh||!manual||busy||!api.gate('vision.clear_selection').allowed;
    const names={ACQUIRING:'已选择，等待新图像确认',TRACKING:'正在沿选中目标跟踪',LOST:'选中目标暂时无可靠测量',RESELECT_REQUIRED:'选中目标已丢失，请重新选择'};
    $('vision-candidate-message').textContent=!fresh?'候选选择不可用：等待运行中的新鲜诊断':
      (names[selection?.state]||'自动选线')+(candidates.length?` · 当前有 ${candidates.length} 条疑似线，按画面编号选择`:
        !usable?' · 等待新鲜的同帧画面':r?.vision_2d_valid?'':' · 当前图像没有足够依据生成候选线');
  }
  function drawCandidates(context, result) {
    for(const c of (result.candidates||[]).slice(0,3)) {
      const line=c.centerline;
      if(!Array.isArray(line)||line.length<2||!line.every(p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite)))continue;
      context.save();context.strokeStyle=candidateColors[Number(c.rank)-1]||candidateColors[0];
      context.fillStyle=context.strokeStyle;context.lineWidth=3;context.setLineDash([7,5]);
      context.beginPath();context.moveTo(...line[0]);
      for(const p of line.slice(1))context.lineTo(...p);
      context.stroke();context.setLineDash([]);
      const p=line[Math.floor(line.length/2)];
      context.font='bold 16px sans-serif';context.fillText(String(c.candidate_id),Math.max(2,Math.min(context.canvas.width-18,p[0]+8)),p[1]);
      context.restore();
    }
  }
  function renderReference(shown, stale, visible) {
    const canvas=$('depth_camera-reference'), context=canvas.getContext('2d');
    const label=$('depth_camera-reference-display');
    const source=$('depth_camera-canvas');
    if(canvas.width!==source.width)canvas.width=source.width;
    if(canvas.height!==source.height)canvas.height=source.height;
    context.clearRect(0,0,canvas.width,canvas.height);
    const r=value?.diagnostics?.tracking_error_reference;
    if(!shown||stale||!value||performance.now()-received>=3500||value.status!=='running'||!value.diagnostics_fresh) {
      label.textContent='参考线未显示：等待新鲜画面和运行中的巡迹参考参数';return;
    }
    if(!r||!['image_width','image_height','reference_x','reference_y','reference_dx_dy','heading_y_min','heading_y_max'].every(k=>Number.isFinite(r[k]))||
       r.image_width<=0||r.image_height<=0||r.reference_x<0||r.reference_x>=r.image_width||
       !(0<=r.heading_y_min&&r.heading_y_min<r.reference_y&&r.reference_y<r.heading_y_max&&r.heading_y_max<r.image_height)) {
      label.textContent='参考线未显示：节点尚未提供有效参考参数';return;
    }
    // Match the bridge's pixel-center resize convention. Never assume 640x480.
    const sx=canvas.width/r.image_width, sy=canvas.height/r.image_height;
    if(Math.abs(canvas.width*r.image_height-canvas.height*r.image_width)>Math.max(r.image_width,r.image_height)) {
      label.textContent='参考线未显示：画面与参考配置的宽高比不一致';return;
    }
    const x=v=>(v+.5)*sx-.5, y=v=>(v+.5)*sy-.5;
    const lineX=v=>r.reference_x+r.reference_dx_dy*(v-r.reference_y);
    const angle=Math.atan(r.reference_dx_dy)*180/Math.PI;
    label.textContent=(visible?'黄色虚线：参考中线 · 十字：参考点 · 黄色带：方向拟合区':'参考叠加已隐藏')+
      ` · 原图参考点 (${r.reference_x}, ${r.reference_y}) · 向左偏角 ${angle.toFixed(1)}° · y=${r.heading_y_min}～${r.heading_y_max}`;
    if(!visible)return;
    context.save();
    context.fillStyle='rgba(255,208,64,.10)';
    context.fillRect(0,y(r.heading_y_min),canvas.width,y(r.heading_y_max)-y(r.heading_y_min));
    context.strokeStyle='#ffd040';context.lineWidth=2;context.setLineDash([8,6]);
    context.beginPath();context.moveTo(x(lineX(0)),y(0));context.lineTo(x(lineX(r.image_height-1)),y(r.image_height-1));context.stroke();
    context.lineWidth=1;context.setLineDash([4,6]);
    for(const row of [r.heading_y_min,r.reference_y,r.heading_y_max]) {
      context.beginPath();context.moveTo(0,y(row));context.lineTo(canvas.width,y(row));context.stroke();
    }
    context.setLineDash([]);context.lineWidth=2;
    const px=x(r.reference_x),py=y(r.reference_y);
    context.beginPath();context.moveTo(px-8,py);context.lineTo(px+8,py);context.moveTo(px,py-8);context.lineTo(px,py+8);context.stroke();
    context.beginPath();context.arc(px,py,5,0,Math.PI*2);context.stroke();
    // Arrow follows the same forward vector (-dx/dy, -1) as heading_error_rad.
    const ax=x(lineX(r.heading_y_min)),ay=y(r.heading_y_min);
    const direction=Math.atan2(-sy,-r.reference_dx_dy*sx);
    context.beginPath();context.moveTo(ax-10*Math.cos(direction-.5),ay-10*Math.sin(direction-.5));context.lineTo(ax,ay);
    context.lineTo(ax-10*Math.cos(direction+.5),ay-10*Math.sin(direction+.5));context.stroke();
    context.restore();
  }
  window.VisionControls=Object.freeze({init,receive,render,renderReference,renderCandidates,drawCandidates,reset:()=>{value=null;received=0;error='';
    candidateRequests.clear();pressedRequests.clear();selectionSession=null;
    const canvas=$('depth_camera-reference');canvas.getContext('2d').clearRect(0,0,canvas.width,canvas.height);},
    timeout:()=>{error='操作响应超时，结果未知；正在查询状态，请勿重复操作';},snapshot:()=>value});
})();
