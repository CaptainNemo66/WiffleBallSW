import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec


# =============== DESIGN / INPUTS ===============
l2      = 55.00
alpha1  = 45.00
l3      = 115.0
alpha2  = -55.00
l4      = 87.00
e4      = 150           # End Effector to J14 distance
e3      = 95          # End Effector to J34 distance

m_ball = 45             # Wiffle Ball Mass

Axy     = (60.0, -80.0)
OFFSET  = 89

TH2_START_DEG =  0
TH2_END_DEG   =  360
W2_DEG_S      =  1800


TRACE_LINK    = 4
TRACE_R       = 21
TRACE_PHI_DEG = 0.0

FPS           = 360
PADDING       = 20
# ==============================================


A = np.array(Axy, dtype=float)
m1 = math.tan(math.radians(alpha1 + 90.0))
m2 = math.tan(math.radians(alpha2 + 90.0))


if TRACE_R is None:
    TRACE_R = {1: l4/2, 2: l2/2, 3: l3/2, 4: l4/2}.get(TRACE_LINK, l4/2)

dtheta = TH2_END_DEG - TH2_START_DEG
duration = abs(dtheta) / max(1e-6, abs(W2_DEG_S))
n_frames = int(duration * FPS) + 1
t = np.linspace(0.0, duration, n_frames)
dt = t[1] - t[0] if n_frames > 1 else 1.0/FPS
theta2_deg = np.linspace(TH2_START_DEG, TH2_END_DEG, n_frames)
theta2_rad = np.deg2rad(theta2_deg)

def get_intersections(x0, y0, r0, x1, y1, r1):
    d=np.sqrt((x1-x0)**2 + (y1-y0)**2)
    
    # non intersecting
    if d > r0 + r1 :
        return None
    # One circle within other
    if d < abs(r0-r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=np.sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d
        
        return (x3, y3, x4, y4)

def solve_quadratic(a, b, c, prefer=None):
    disc = b*b - 4*a*c
    if disc < 0 and disc > -1e-9: disc = 0.0
    if disc < 0: return None
    rdisc = math.sqrt(disc)
    x1 = (-b + rdisc)/(2*a)
    x2 = (-b - rdisc)/(2*a)
    if prefer is None: return x1 if abs(x1) < abs(x2) else x2
    return x1 if abs(x1 - prefer) <= abs(x2 - prefer) else x2

def compute_positions(theta2_deg_seq):
    ptsA, ptsB, ptsC, ptsD, ptsE = [], [], [], [], []
    prev_t, prev_s = None, None
    for th_deg in theta2_deg_seq:
        th = math.radians(th_deg)
        Bx = A[0] + l2 * math.cos(th)
        By = A[1] + l2 * math.sin(th)
        a = 1.0 + m1*m1
        b = -2.0*(Bx + m1*(OFFSET + By))
        c = (Bx*Bx) + (OFFSET + By)**2 - l3*l3
        t_sol = solve_quadratic(a, b, c, prev_t)
        if t_sol is None:
            if ptsA:
                ptsA.append(ptsA[-1]); ptsB.append(ptsB[-1]); ptsC.append(ptsC[-1]); ptsD.append(ptsD[-1])
                continue
            else:
                t_sol = 0.0
        Cx, Cy = t_sol, m1*t_sol - OFFSET
        prev_t = t_sol
        aa = 1.0 + m2*m2
        bb = -2.0*(Cx + m2*Cy)
        cc = Cx*Cx + Cy*Cy - l4*l4
        s_sol = solve_quadratic(aa, bb, cc, prev_s)
        if s_sol is None:
            if ptsA:
                ptsA.append(ptsA[-1]); ptsB.append(ptsB[-1]); ptsC.append(ptsC[-1]); ptsD.append(ptsD[-1])
                continue
            else:
                s_sol = 0.0
        Dx, Dy = s_sol, m2*s_sol
        prev_s = s_sol
        Ex,Ey,_,_ = get_intersections(Dx,Dy,e4,Cx,Cy,e3)
        ptsA.append([A[0], A[1]])
        ptsB.append([Bx, By])
        ptsC.append([Cx, Cy])
        ptsD.append([Dx, Dy])
        ptsE.append([Ex,Ey])
    return np.array(ptsA), np.array(ptsB), np.array(ptsC), np.array(ptsD), np.array(ptsE)

A_traj, B_traj, C_traj, D_traj, E_traj = compute_positions(theta2_deg)

E_traj_arr = np.array(E_traj)

def link_angle(P0, P1):
    return np.arctan2(P1[:,1]-P0[:,1], P1[:,0]-P0[:,0])

th2_unw = np.unwrap(np.deg2rad(theta2_deg))
th3_unw = np.unwrap(link_angle(B_traj, C_traj))
th4_unw = np.unwrap(link_angle(C_traj, D_traj))

w4_deg_s = np.rad2deg(np.gradient(th4_unw, dt))

def trace_point(link_id, r, phi_deg):
    phi = math.radians(phi_deg)
    if link_id == 2: P0, P1 = A_traj, B_traj
    elif link_id == 3: P0, P1 = B_traj, C_traj
    elif link_id == 4: P0, P1 = C_traj, D_traj
    else: P0, P1 = D_traj, A_traj
    theta_link = np.arctan2(P1[:,1]-P0[:,1], P1[:,0]-P0[:,0])
    ang = theta_link + phi
    X = P0[:,0] + r*np.cos(ang)
    Y = P0[:,1] + r*np.sin(ang)
    return np.column_stack((X, Y))

trace_pts = trace_point(TRACE_LINK, TRACE_R, TRACE_PHI_DEG)
vx = np.gradient(trace_pts[:,0], dt)
vy = np.gradient(trace_pts[:,1], dt)
speed_trace = np.hypot(vx, vy)

vx_e = np.gradient(E_traj_arr[:,0],dt)
vy_e = np.gradient(E_traj_arr[:,1],dt)
ax_e = np.gradient(vx_e,dt)
ay_e = np.gradient(vy_e,dt)
e_speed_trace = np.hypot(vx_e,vy_e) / 1000 # Dividing by 1000 to convert from mm/s to m/s
e_accel = np.hypot(ax_e,ay_e) / 1000 # Dividing by 1000 to convert from mm/s^2 to m/s^2

fig = plt.figure(figsize=(12, 7))
gs = GridSpec(3, 3, figure=fig, width_ratios=[2.0, 1.0, 1.0], height_ratios=[1.0, 1.0, 1.0], hspace=0.4, wspace=0.35)

ax_anim = fig.add_subplot(gs[:, 0])
ax_anim.set_aspect('equal')
all_pts = np.vstack([A_traj, B_traj, C_traj, D_traj, E_traj, trace_pts])
ax_anim.set_xlim(all_pts[:,0].min()-PADDING, all_pts[:,0].max()+PADDING)
ax_anim.set_ylim(all_pts[:,1].min()-PADDING, all_pts[:,1].max()+PADDING)
ax_anim.set_title("Mechanism Animation (A–B–C–D)")
ax_anim.set_xlabel("x")
ax_anim.set_ylabel("y")

line_L2, = ax_anim.plot([], [], 'r-', lw=2)
line_L3, = ax_anim.plot([], [], 'g-', lw=2)
line_L4, = ax_anim.plot([], [], 'b-', lw=2)
line_L1, = ax_anim.plot([], [], 'k-', lw=2)
line_L5, = ax_anim.plot([], [], 'o-', lw=2)
ptA, = ax_anim.plot([], [], 'ko', ms=5)
ptB, = ax_anim.plot([], [], 'ro', ms=5)
ptC, = ax_anim.plot([], [], 'go', ms=5)
ptD, = ax_anim.plot([], [], 'bo', ms=5)
ptE, = ax_anim.plot([], [], 'yo', ms=5)
ptT, = ax_anim.plot([], [], 'mo', ms=5)
trailT, = ax_anim.plot([], [], 'm--', lw=1, alpha=0.6)
trailC, = ax_anim.plot([], [], 'g--', lw=1, alpha=0.4)
trailD, = ax_anim.plot([], [], 'b--', lw=1, alpha=0.4)

ax_spd = fig.add_subplot(gs[0,1]); ax_spd.plot(t, speed_trace); ax_spd.set_title("Trace speed vs time"); ax_spd.grid()
spd_marker, = ax_spd.plot([], [], 'ko', ms=5)
ax_w4t = fig.add_subplot(gs[0,2]); ax_w4t.plot(t, w4_deg_s); ax_w4t.set_title("ω4 vs time"); ax_w4t.grid()
w4t_marker, = ax_w4t.plot([], [], 'ko', ms=5)
ax_th = fig.add_subplot(gs[1,1]); ax_th.plot(np.rad2deg(th2_unw), np.rad2deg(th4_unw)); ax_th.set_title("θ4 vs θ2"); ax_th.grid()
th_marker, = ax_th.plot([], [], 'ko', ms=5)
ax_w4th = fig.add_subplot(gs[1,2]); ax_w4th.plot(theta2_deg, w4_deg_s); ax_w4th.set_title("ω4 vs θ2"); ax_w4th.grid(); ax_w4th.set_xlabel("θ2 (deg)"); ax_w4th.set_ylabel("ω4 (deg/s)")
w4th_marker, = ax_w4th.plot([], [], 'ko', ms=5)
ax_ev = fig.add_subplot(gs[2,1]); ax_ev.plot(t,e_speed_trace); ax_ev.set_title("Effector Velocity vs time"); ax_ev.grid(); ax_ev.set_xlabel("Time (s)"); ax_ev.set_ylabel("Velocity (m/s)")
ev_marker, = ax_ev.plot([],[], 'ko', ms=5)
ax_eacc = fig.add_subplot(gs[2,2]); ax_eacc.plot(t,e_accel); ax_eacc.set_title("Effector Accel vs time"); ax_eacc.grid(); ax_eacc.set_xlabel("Time (s)"); ax_eacc.set_ylabel("Acceleration (m/s^2)")
eacc_marker, = ax_eacc.plot([], [], 'ko', ms=5)

def init_anim():
    for ln in (line_L1,line_L2,line_L3,line_L4): ln.set_data([],[])
    for p in (ptA,ptB,ptC,ptD,ptT): p.set_data([],[])
    for tr in (trailT,trailC,trailD): tr.set_data([],[])
    for m in (spd_marker,w4t_marker,th_marker,w4th_marker,ev_marker,eacc_marker): m.set_data([],[])
    return (line_L1,line_L2,line_L3,line_L4,ptA,ptB,ptC,ptD,ptT,trailT,trailC,trailD,spd_marker,w4t_marker,th_marker,w4th_marker,ev_marker,eacc_marker)


def update_anim(i):
    Ax,Ay=A_traj[i];Bx,By=B_traj[i];Cx,Cy=C_traj[i];Dx,Dy=D_traj[i];Ex,Ey=E_traj[i];Tx,Ty=trace_pts[i]
    line_L2.set_data([Ax,Bx],[Ay,By])
    line_L3.set_data([Bx,Cx],[By,Cy])
    line_L4.set_data([Cx,Dx],[Cy,Dy])
    line_L1.set_data([Dx,Ax],[Dy,Ay])
    line_L5.set_data([Cx,Ex],[Cy,Ey])
    ptA.set_data([Ax],[Ay]); ptB.set_data([Bx],[By]); ptC.set_data([Cx],[Cy]); ptD.set_data([Dx],[Dy]); ptT.set_data([Tx],[Ty])
    trailT.set_data(trace_pts[:i+1,0],trace_pts[:i+1,1])
    trailC.set_data(C_traj[:i+1,0],C_traj[:i+1,1])
    trailD.set_data(D_traj[:i+1,0],D_traj[:i+1,1])
    spd_marker.set_data([t[i]],[speed_trace[i]])
    w4t_marker.set_data([t[i]],[w4_deg_s[i]])
    th_marker.set_data([np.rad2deg(th2_unw[i])],[np.rad2deg(th4_unw[i])])
    w4th_marker.set_data([theta2_deg[i]],[w4_deg_s[i]])
    ev_marker.set_data([t[i]],[e_speed_trace[i]])
    eacc_marker.set_data([t[i]],[e_accel[i]])
    return (line_L1,line_L2,line_L3,line_L4,line_L5,ptA,ptB,ptC,ptD,ptT,trailT,trailC,trailD,spd_marker,w4t_marker,th_marker,w4th_marker,ev_marker,eacc_marker)


ani = FuncAnimation(fig, update_anim, frames=n_frames, init_func=init_anim, interval=1000/FPS, blit=True)
plt.show()