#!/usr/bin/env python3
"""
3D view of a scan: inventory_scanned.json -> one self-contained HTML file.

    python3 report/view_inventory.py
    python3 report/view_inventory.py --no-truth     # estimates only
    xdg-open out/inventory_3d.html

The point is to see, in one glance, that the pipeline put every product at a
sensible shelf coordinate. An error hiding in row 300 of a 432 row table is a
single dot outside its rack here.

Drawn:
  * the world axes and a floor grid, so the coordinate frame is visible
  * every rack cell as a wireframe box (face x bay x level)
  * the eight floor markers that the drift correction uses
  * each decoded product as a dot, coloured by its distance from ground truth
  * each box that was never decoded as a hollow square, at its true position
  * an error vector for anything more than 15 cm out
  * identity, location, coordinate and error under the cursor

NO EXTERNAL DEPENDENCY. A small perspective projection and canvas drawing are
embedded. A three.js from a CDN would not open offline, and this file is meant
to survive being mailed to someone.

GROUND TRUTH is read for DISPLAY ONLY - where the missed boxes are, and how
far each estimate landed from the truth. It never enters an estimate.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from warehouse_model import (CONFIG, GROUND_TRUTH, INVENTORY, REPO_ROOT,
                             bounds, error_to_truth, load_config,
                             load_ground_truth, load_markers, load_run,
                             percentile, rack_cells, records)

HTML = r"""<!doctype html>
<meta charset="utf-8"><title>3D Inventory - Warehouse Scan</title>
<style>
  html,body{margin:0;height:100%;background:#11151c;color:#dfe6ef;
            font-family:system-ui,sans-serif;overflow:hidden}
  canvas{display:block;width:100vw;height:100vh;cursor:grab}
  canvas.drag{cursor:grabbing}
  #hud{position:fixed;top:12px;left:14px;font-size:13px;line-height:1.5;
       background:#0009;padding:10px 13px;border-radius:7px;max-width:330px}
  #hud b{font-size:15px}
  #tip{position:fixed;pointer-events:none;background:#000d;border:1px solid #4a5568;
       padding:7px 10px;border-radius:6px;font-size:12.5px;line-height:1.45;
       display:none;white-space:nowrap}
  .k{display:inline-block;width:11px;height:11px;border-radius:50%;
     margin-right:6px;vertical-align:-1px}
  /* a missed box is a hollow square in the drawing; the legend has to show the
     same shape or it reads as the "large error" red */
  .sq{border-radius:0;background:none!important;border:2px solid #ff4d6d}
  #help{position:fixed;bottom:10px;left:14px;font-size:12px;color:#8b98a9}
</style>
<canvas id="c"></canvas>
<div id="hud"></div><div id="tip"></div>
<div id="help">drag: rotate &nbsp;·&nbsp; wheel: zoom &nbsp;·&nbsp;
  right-drag: pan &nbsp;·&nbsp; hover a product for detail</div>
<script>
const D = __DATA__;

// ---------------------------------------------------------------- camera
let az = -0.9, el = 0.38, dist = 34, target = [D.center[0], D.center[1], 2.4];
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
let W, H, dpr;
function resize(){
  dpr = window.devicePixelRatio || 1;
  W = cv.clientWidth; H = cv.clientHeight;
  cv.width = W*dpr; cv.height = H*dpr; ctx.setTransform(dpr,0,0,dpr,0,0);
  draw();
}
window.addEventListener('resize', resize);

/* World (Gazebo ENU: x east, y north, z up) -> camera -> screen.
   The camera orbits the target; world +Z is up. */
function project(p){
  const ca=Math.cos(az), sa=Math.sin(az), ce=Math.cos(el), se=Math.sin(el);
  const eye=[target[0]+dist*ce*ca, target[1]+dist*ce*sa, target[2]+dist*se];
  const f=[target[0]-eye[0], target[1]-eye[1], target[2]-eye[2]];
  const fl=Math.hypot(...f); const fw=f.map(v=>v/fl);
  // right = forward x up(0,0,1); up = right x forward
  let r=[fw[1], -fw[0], 0]; const rl=Math.hypot(...r)||1; r=r.map(v=>v/rl);
  const u=[r[1]*fw[2]-r[2]*fw[1], r[2]*fw[0]-r[0]*fw[2], r[0]*fw[1]-r[1]*fw[0]];
  const d=[p[0]-eye[0], p[1]-eye[1], p[2]-eye[2]];
  const z=d[0]*fw[0]+d[1]*fw[1]+d[2]*fw[2];
  if(z<=0.05) return null;                       // behind the camera
  const x=d[0]*r[0]+d[1]*r[1]+d[2]*r[2];
  const y=d[0]*u[0]+d[1]*u[1]+d[2]*u[2];
  const fov=Math.min(W,H)*0.9;
  return [W/2 + fov*x/z, H/2 - fov*y/z, z];
}
function line(a,b,style,w){
  const p=project(a), q=project(b); if(!p||!q) return;
  ctx.strokeStyle=style; ctx.lineWidth=w||1;
  ctx.beginPath(); ctx.moveTo(p[0],p[1]); ctx.lineTo(q[0],q[1]); ctx.stroke();
}
function boxEdges(c,h){
  const [x,y,z]=c, [a,b,d]=h;
  const v=[[x-a,y-b,z-d],[x+a,y-b,z-d],[x+a,y+b,z-d],[x-a,y+b,z-d],
           [x-a,y-b,z+d],[x+a,y-b,z+d],[x+a,y+b,z+d],[x-a,y+b,z+d]];
  return [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],
          [0,4],[1,5],[2,6],[3,7]].map(e=>[v[e[0]],v[e[1]]]);
}

/* Position error -> colour. Green at or under GOOD, red at or over BAD.
   Without ground truth every dot is neutral blue: the view still shows where
   the pipeline put things, it just cannot say how wrong that is. */
const GOOD=0.10, BAD=0.25;
function errColor(e){
  if(e===undefined||e===null) return 'rgb(90,150,220)';
  const t=Math.max(0,Math.min(1,(e-GOOD)/(BAD-GOOD)));
  const r=Math.round(60*(1-t)+230*t), g=Math.round(200*(1-t)+70*t);
  return `rgb(${r},${g},90)`;
}

let screenPts=[];
function draw(){
  ctx.fillStyle='#11151c'; ctx.fillRect(0,0,W,H);

  // floor grid, 2 m
  for(let x=D.bounds[0]; x<=D.bounds[1]+0.01; x+=2)
    line([x,D.bounds[2],0],[x,D.bounds[3],0],'#1e2836');
  for(let y=D.bounds[2]; y<=D.bounds[3]+0.01; y+=2)
    line([D.bounds[0],y,0],[D.bounds[1],y,0],'#1e2836');

  // world axes
  line([0,0,0],[3,0,0],'#e05252',2.5);
  line([0,0,0],[0,3,0],'#52c65c',2.5);
  line([0,0,0],[0,0,3],'#5b8dfc',2.5);

  // rack cells
  for(const b of D.racks)
    for(const e of boxEdges(b.c,b.h)) line(e[0],e[1],'#33455e',1);

  // floor markers, the absolute fixes the drift correction reads
  for(const m of D.markers){
    const s=m.size/2;
    line([m.x-s,m.y-s,0.01],[m.x+s,m.y-s,0.01],'#4fd6d6',1.6);
    line([m.x+s,m.y-s,0.01],[m.x+s,m.y+s,0.01],'#4fd6d6',1.6);
    line([m.x+s,m.y+s,0.01],[m.x-s,m.y+s,0.01],'#4fd6d6',1.6);
    line([m.x-s,m.y+s,0.01],[m.x-s,m.y-s,0.01],'#4fd6d6',1.6);
  }

  // products, far to near (painter's algorithm)
  screenPts=[];
  const pts=[];
  for(const it of D.items){
    const p=project([it.x,it.y,it.z]); if(!p) continue;
    pts.push([p,it]);
  }
  pts.sort((a,b)=>b[0][2]-a[0][2]);
  for(const [p,it] of pts){
    const r=Math.max(2.0, 62/p[2]);
    if(it.tx!==undefined && it.err>0.15){
      const q=project([it.tx,it.ty,it.tz]);      // error vector
      if(q){ ctx.strokeStyle='#ffb020'; ctx.lineWidth=1.2;
             ctx.beginPath(); ctx.moveTo(p[0],p[1]); ctx.lineTo(q[0],q[1]); ctx.stroke(); }
    }
    if(it.missed){                                // never decoded: hollow square
      ctx.strokeStyle='#ff4d6d'; ctx.lineWidth=1.8;
      ctx.strokeRect(p[0]-r, p[1]-r, 2*r, 2*r);
    } else {
      ctx.fillStyle=errColor(it.err);
      ctx.beginPath(); ctx.arc(p[0],p[1],r,0,7); ctx.fill();
    }
    screenPts.push([p[0],p[1],r,it]);
  }
  hud();
}

function hud(){
  const n=D.items.filter(i=>!i.missed).length, m=D.items.length-n;
  let s=`<b>3D Inventory</b><br>${n} products decoded`;
  if(m) s+=` · <span style="color:#ff4d6d">${m} missed</span>`;
  s+=`<br><span style="font-size:12px;color:#8b98a9">${D.source}</span><hr
      style="border:0;border-top:1px solid #2a3648;margin:7px 0">`;
  if(D.truth){
    s+=`<span class="k" style="background:${errColor(0.05)}"></span>within 10 cm<br>`;
    s+=`<span class="k" style="background:${errColor(0.30)}"></span>over 25 cm out<br>`;
    if(m) s+=`<span class="k sq"></span>never decoded (ground truth)<br>`;
    s+=`<span class="k" style="background:#ffb020"></span>error vector (&gt;15 cm)<br>`;
  } else {
    s+=`<span class="k" style="background:${errColor(null)}"></span>estimated position<br>`;
  }
  s+=`<span class="k" style="background:#4fd6d6"></span>floor marker<br>`;
  s+=`<span style="color:#e05252">■</span> X &nbsp;<span style="color:#52c65c">■</span> Y
      &nbsp;<span style="color:#5b8dfc">■</span> Z`;
  if(D.stats) s+=`<br><span style="font-size:12px;color:#8b98a9">${D.stats}</span>`;
  document.getElementById('hud').innerHTML=s;
}

// --------------------------------------------------------------- interaction
let drag=null;
cv.addEventListener('mousedown',e=>{drag={x:e.clientX,y:e.clientY,b:e.button};
  cv.classList.add('drag');});
window.addEventListener('mouseup',()=>{drag=null;cv.classList.remove('drag');});
cv.addEventListener('contextmenu',e=>e.preventDefault());
window.addEventListener('mousemove',e=>{
  if(drag){
    const dx=e.clientX-drag.x, dy=e.clientY-drag.y;
    drag.x=e.clientX; drag.y=e.clientY;
    if(drag.b===2){                                  // pan
      const ca=Math.cos(az), sa=Math.sin(az), k=dist*0.0016;
      target[0]+= (sa*dx)*k; target[1]+= (-ca*dx)*k; target[2]+= dy*k;
    } else {
      az-=dx*0.006; el=Math.max(-1.45,Math.min(1.45, el+dy*0.006));
    }
    draw(); return;
  }
  const t=document.getElementById('tip');
  let best=null, bd=1e9;
  for(const [x,y,r,it] of screenPts){
    const d=Math.hypot(e.clientX-x, e.clientY-y);
    if(d<Math.max(7,r+4) && d<bd){ bd=d; best=it; }
  }
  if(!best){ t.style.display='none'; return; }
  let h=`<b>${best.id}</b><br>${best.qr}<br>`
      + `${best.shelf}-${String(best.bay).padStart(2,'0')}-L${best.level}<br>`
      + `x ${best.x.toFixed(2)}  y ${best.y.toFixed(2)}  z ${best.z.toFixed(2)}`;
  if(best.missed) h+=`<br><span style="color:#ff4d6d">NEVER DECODED</span>`;
  else if(best.err!==undefined) h+=`<br>position error ${(best.err*100).toFixed(1)} cm`;
  if(best.truth_at) h+=`<br><span style="color:#8b98a9">truth ${best.truth_at}</span>`;
  t.innerHTML=h; t.style.display='block';
  t.style.left=(e.clientX+14)+'px'; t.style.top=(e.clientY+12)+'px';
});
cv.addEventListener('wheel',e=>{
  e.preventDefault();
  dist=Math.max(4,Math.min(90, dist*(e.deltaY>0?1.1:0.9))); draw();
},{passive:false});
resize();
</script>
"""


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inventory", type=Path, default=INVENTORY)
    ap.add_argument("--ground-truth", type=Path, default=GROUND_TRUTH)
    ap.add_argument("--config", type=Path, default=CONFIG)
    ap.add_argument("--out", type=Path,
                    default=REPO_ROOT / "out" / "inventory_3d.html")
    ap.add_argument("--no-truth", dest="truth", action="store_false",
                    help="draw the estimates alone: no missed boxes, no error "
                         "vectors, no error colouring")
    args = ap.parse_args()

    cfg = load_config(args.config)
    run = load_run(args.inventory)
    truth = load_ground_truth(args.ground_truth) if args.truth else {}

    items, errs = [], []
    for rec in records(run, cfg):
        item = {"id": rec["product_id"], "qr": rec["qr"],
                "x": rec["x"], "y": rec["y"], "z": rec["z"],
                "shelf": rec["shelf"], "level": rec["level"], "bay": rec["bay"],
                "missed": False}
        t = truth.get(rec["qr"])
        if t:
            tx, ty, tz = t["label_pose_xyzrpy"][:3]
            err = error_to_truth(rec, truth)
            item.update(tx=tx, ty=ty, tz=tz, err=round(err, 3),
                        truth_at=f"{t['row']}-{t['bay']:02d}-L{t['level']}")
            errs.append(err)
        items.append(item)

    # Missed boxes: in ground truth, absent from the scan. Drawn at their true
    # position, which is the only place they can be drawn.
    seen = {it["qr"] for it in items}
    for payload, c in truth.items():
        if payload in seen:
            continue
        x, y, z = c["label_pose_xyzrpy"][:3]
        items.append({"id": c["caption"] or payload, "qr": payload,
                      "x": x, "y": y, "z": z, "shelf": c["row"],
                      "level": c["level"], "bay": c["bay"], "missed": True})

    markers = [{"x": m["label_pose_xyzrpy"][0], "y": m["label_pose_xyzrpy"][1],
                "size": m["label_size_m"][0]} for m in load_markers(args.ground_truth)]

    b = bounds(cfg)
    stats = ""
    if errs:
        e = sorted(errs)
        stats = (f"position error: median {e[len(e)//2]*100:.1f} cm · "
                 f"p95 {percentile(e, 0.95)*100:.1f} cm")

    data = {
        "items": items,
        "racks": rack_cells(cfg),
        "markers": markers,
        "bounds": b,
        "center": [round((b[0] + b[1]) / 2, 2), round((b[2] + b[3]) / 2, 2)],
        "source": f"{args.inventory.name} · {run.get('scan_date', 'unknown date')}",
        "truth": bool(truth),
        "stats": stats,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(HTML.replace("__DATA__", json.dumps(data, ensure_ascii=False)))

    n_missed = sum(1 for i in items if i["missed"])
    print(f"{len(items) - n_missed} products"
          + (f" + {n_missed} never decoded" if n_missed else "")
          + f", {len(data['racks'])} rack cells, {len(markers)} floor markers")
    if stats:
        print(stats)
    print(f"\n{args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
