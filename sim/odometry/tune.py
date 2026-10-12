"""Window increment errors of wheel_gyro, lidar and lidar_gyro over 1 s on tuning arenas 1000-1003 (sets sig_win_lidar)."""
import numpy as np, drift, sys
from multiprocessing import Pool
import arena2d as a2
def rel(A,B):
    return drift.from_se2(np.linalg.inv(drift.se2(*A))@drift.se2(*B))
def one(seed):
    log,marks=drift.drive(seed,'nominal')
    rng=np.random.default_rng([seed,99,1])
    arena,frames=drift.lidar_frames(seed,log,rng)
    est,info,start=drift.run_estimators(seed,'nominal',log,frames,arena,rng)
    idx=[f[0] for f in frames]
    out={}
    for e in ('wheel_gyro','lidar','lidar_gyro'):
        errs=[]
        for a,b in zip(idx[:-10:10],idx[10::10]):
            d=rel(est[e][a],est[e][b])-rel(log[a,1:4],log[b,1:4]); d[2]=drift.wrap(d[2]); 
            dist=np.hypot(*(log[b,1:3]-log[a,1:3])); errs.append([*d,dist])
        out[e]=np.array(errs)
    return out
with Pool(4) as p: res=p.map(one,[1000,1001,1002,1003])
for e in ('wheel_gyro','lidar','lidar_gyro'):
    E=np.concatenate([r[e] for r in res])
    mov=E[:,3]>0.01
    print(e,'xy sd mm moving %.1f still %.1f'%(1000*np.std(E[mov,:2]),1000*np.std(E[~mov,:2])),'yaw sd deg %.3f'%np.degrees(np.std(E[:,2])), 'p95 xy mm %.1f'%(1000*np.percentile(np.abs(E[:,:2]),95)), 'mean dist mm %.1f'%(1000*E[mov,3].mean()))
