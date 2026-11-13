//! Bonus: trajectory viewer. Writes a standalone HTML page (no external resource) with a
//! rotatable 3D view, a top view, the GPS fixes, the estimated trajectory, the true
//! trajectory when known, and the 1-sigma position uncertainty (variance).

use std::fmt::Write as _;
use std::fs;

pub struct Sample
{
    pub time: f64,
    pub position: [f64; 3],
    pub sigma: [f64; 3],
    pub truth: Option<[f64; 3]>,
}

pub struct Recorder
{
    enabled: bool,
    every: usize,
    seen: usize,
    samples: Vec<Sample>,
    gps: Vec<[f64; 3]>,
}

impl Recorder
{
    /// Keep one frame out of `every` (100 frames = 1 s).
    pub fn new(every: usize) -> Self
    {
        Recorder { enabled: true, every: every.max(1), seen: 0, samples: Vec::new(), gps: Vec::new() }
    }

    /// When disabled, nothing is stored (no memory growth without `--visual`).
    pub fn set_enabled(&mut self, enabled: bool)
    {
        self.enabled = enabled;
    }

    pub fn record(&mut self, time: f64, position: [f64; 3], sigma: [f64; 3], truth: Option<[f64; 3]>)
    {
        if !self.enabled
        {
            return;
        }
        if self.seen % self.every == 0
        {
            self.samples.push(Sample { time, position, sigma, truth });
        }
        self.seen += 1;
    }

    pub fn record_gps(&mut self, gps: [f64; 3])
    {
        if self.enabled
        {
            self.gps.push(gps);
        }
    }

    pub fn len(&self) -> usize
    {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool
    {
        self.samples.is_empty()
    }

    fn num(v: f64) -> String
    {
        if v.is_finite() { format!("{:.3}", v) } else { "0".to_string() }
    }

    pub fn to_html(&self) -> String
    {
        let mut est = String::new();
        let mut sig = String::new();
        let mut tru = String::new();
        let mut gps = String::new();
        let mut times = String::new();

        for s in &self.samples
        {
            let _ = write!(est, "{},{},{},", Self::num(s.position[0]), Self::num(s.position[1]), Self::num(s.position[2]));
            let _ = write!(sig, "{},", Self::num((s.sigma[0] * s.sigma[0] + s.sigma[1] * s.sigma[1] + s.sigma[2] * s.sigma[2]).sqrt()));
            let _ = write!(times, "{:.2},", s.time);
            if let Some(t) = s.truth
            {
                let _ = write!(tru, "{},{},{},", Self::num(t[0]), Self::num(t[1]), Self::num(t[2]));
            }
        }
        for g in &self.gps
        {
            let _ = write!(gps, "{},{},{},", Self::num(g[0]), Self::num(g[1]), Self::num(g[2]));
        }

        TEMPLATE
            .replace("__EST__", est.trim_end_matches(','))
            .replace("__SIG__", sig.trim_end_matches(','))
            .replace("__TRUE__", tru.trim_end_matches(','))
            .replace("__GPS__", gps.trim_end_matches(','))
            .replace("__TIME__", times.trim_end_matches(','))
    }

    pub fn write(&self, path: &str) -> Result<(), String>
    {
        fs::write(path, self.to_html()).map_err(|e| format!("Cannot write '{}': {}", path, e))
    }
}

const TEMPLATE: &str = r##"<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>ft_kalman trajectory</title>
<style>
body{margin:0;background:#111;color:#ddd;font-family:sans-serif}
#bar{padding:8px 12px;display:flex;gap:16px;align-items:center;flex-wrap:wrap}
canvas{display:block;width:100vw;height:calc(100vh - 48px)}
.k{display:inline-block;width:10px;height:10px;margin-right:4px}
</style></head><body>
<div id="bar">
<span><span class="k" style="background:#4af"></span>estimate</span>
<span><span class="k" style="background:#fa4"></span>GPS</span>
<span><span class="k" style="background:#5d5"></span>true (debug)</span>
<span><span class="k" style="background:#f55"></span>1&sigma; variance</span>
<label>view <select id="mode"><option value="3d">3D (drag to rotate, wheel to zoom)</option><option value="top">top (XY)</option></select></label>
<label>&sigma; x<input id="mag" type="range" min="1" max="500" value="50"></label>
<span id="info"></span></div>
<canvas id="c"></canvas>
<script>
const E=[__EST__],S=[__SIG__],T=[__TRUE__],G=[__GPS__],TM=[__TIME__];
const cv=document.getElementById('c'),cx=cv.getContext('2d');
let yaw=0.8,pitch=0.5,zoom=1,drag=null;
const n=E.length/3;
let mn=[1e18,1e18,1e18],mx=[-1e18,-1e18,-1e18];
for(let i=0;i<n;i++)for(let k=0;k<3;k++){mn[k]=Math.min(mn[k],E[3*i+k]);mx[k]=Math.max(mx[k],E[3*i+k]);}
const ctr=mn.map((v,k)=>(v+mx[k])/2),span=Math.max(1,...mx.map((v,k)=>v-mn[k]));
function proj(p){
 let x=p[0]-ctr[0],y=p[1]-ctr[1],z=p[2]-ctr[2];
 if(document.getElementById('mode').value=='top'){return [x,-y,0];}
 const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
 let x1=x*cy-y*sy,y1=x*sy+y*cy;
 return [x1,-(z*cp-y1*sp),y1*cp+z*sp];
}
function draw(){
 cv.width=cv.clientWidth;cv.height=cv.clientHeight;
 const sc=zoom*0.8*Math.min(cv.width,cv.height)/span,ox=cv.width/2,oy=cv.height/2;
 const P=a=>{const q=proj(a);return [ox+q[0]*sc,oy+q[1]*sc];};
 cx.clearRect(0,0,cv.width,cv.height);
 const mag=+document.getElementById('mag').value;
 cx.strokeStyle='rgba(255,80,80,.35)';
 for(let i=0;i<n;i+=Math.max(1,Math.floor(n/300))){
  const p=P([E[3*i],E[3*i+1],E[3*i+2]]);
  cx.beginPath();cx.arc(p[0],p[1],Math.max(1,S[i]*mag*sc),0,6.2832);cx.stroke();
 }
 function path(a,col){
  if(a.length<6)return;cx.strokeStyle=col;cx.beginPath();
  for(let i=0;i<a.length/3;i++){const p=P([a[3*i],a[3*i+1],a[3*i+2]]);i?cx.lineTo(p[0],p[1]):cx.moveTo(p[0],p[1]);}
  cx.stroke();
 }
 path(T,'#5d5');path(E,'#4af');
 cx.fillStyle='#fa4';
 for(let i=0;i<G.length/3;i++){const p=P([G[3*i],G[3*i+1],G[3*i+2]]);cx.fillRect(p[0]-2,p[1]-2,4,4);}
 const last=n-1;
 document.getElementById('info').textContent=n+' points, duration '+(TM[last]||0).toFixed(0)+' s, final sigma '+(S[last]||0).toFixed(3)+' m (drawn x'+mag+')';
}
cv.onmousedown=e=>{drag=[e.clientX,e.clientY];};
window.onmouseup=()=>{drag=null;};
window.onmousemove=e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*0.01;pitch+=(e.clientY-drag[1])*0.01;drag=[e.clientX,e.clientY];draw();};
cv.onwheel=e=>{e.preventDefault();zoom*=e.deltaY<0?1.1:0.9;draw();};
document.getElementById('mode').onchange=draw;document.getElementById('mag').oninput=draw;
window.onresize=draw;draw();
</script></body></html>
"##;
