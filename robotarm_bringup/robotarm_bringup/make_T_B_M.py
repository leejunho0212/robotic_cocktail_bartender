import numpy as np

# =========================
# ✅ 티칭펜던트에서 찍은 값 (mm)
# =========================
p0_mm = [270,   0, 135]    # 마커 중심
px_mm = [230,   0, 135]    # +X 방향 (빨강) 40mm
py_mm = [270, -40, 135]    # +Y 방향 (초록) 40mm

SAVE_PATH = "T_B_M.npz"

def normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("Zero length vector")
    return v / n

def make_T_B_M(p0, px, py):
    x = px - p0
    y = py - p0
    x_u = normalize(x)
    y_u = normalize(y)

    z_u = normalize(np.cross(x_u, y_u))      # 오른손 좌표계
    y_u = normalize(np.cross(z_u, x_u))      # 직교 보정

    R = np.column_stack([x_u, y_u, z_u])     # columns = marker axes in base

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = p0
    return T

def main():
    p0 = np.array(p0_mm, dtype=np.float64) / 1000.0
    px = np.array(px_mm, dtype=np.float64) / 1000.0
    py = np.array(py_mm, dtype=np.float64) / 1000.0

    T_B_M = make_T_B_M(p0, px, py)
    np.savez(SAVE_PATH, T_B_M=T_B_M)

    print(f"Saved -> {SAVE_PATH}")
    print("T_B_M =\n", T_B_M)
    print("det(R) =", np.linalg.det(T_B_M[:3,:3]))

if __name__ == "__main__":
    main()
