"""Simple dynamic obstacle tracking helpers.

Provides a Tracker (1D/2D constant velocity Kalman filter) and a Manager to
maintain identity across frames using greedy assignment. This is intentionally
lightweight for debugging and integration with the existing navigator.
"""

import math
import time


class SimpleKalman2D:
    """Small constant-velocity Kalman filter for 2D position.

    State: [x, y, vx, vy]
    """
    def __init__(self, x=0.0, y=0.0, vx=0.0, vy=0.0, dt=0.1, process_var=1e-2, meas_var=1e-1):
        self.dt = dt
        self.x = [x, y, vx, vy]
        # covariance P (4x4)
        self.P = [[1e-1 if i==j else 0.0 for j in range(4)] for i in range(4)]
        # process noise Q
        q = process_var
        self.Q = [[q if i==j else 0.0 for j in range(4)] for i in range(4)]
        # measurement noise R (2x2)
        r = meas_var
        self.R = [[r, 0.0], [0.0, r]]

    def _matmul(self, A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(len(B))) for j in range(len(B[0]))] for i in range(len(A))]

    def _transpose(self, A):
        return list(map(list, zip(*A)))

    def predict(self):
        dt = self.dt
        x, y, vx, vy = self.x
        # F matrix
        F = [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
        # state prediction
        nx = [F[0][0]*x + F[0][2]*vx, F[1][1]*y + F[1][3]*vy, vx, vy]
        self.x = nx
        # covariance P = F P F^T + Q
        FP = self._matmul(F, self.P)
        FPT = self._transpose(F)
        self.P = [[sum(FP[i][k] * FPT[k][j] for k in range(4)) + self.Q[i][j] for j in range(4)] for i in range(4)]
        return (self.x[0], self.x[1])

    def update(self, mx, my):
        # measurement z
        z = [mx, my]
        # measurement matrix H (2x4)
        H = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
        # y = z - Hx
        y = [z[0] - (H[0][0]*self.x[0] + H[0][1]*self.x[1]), z[1] - (H[1][0]*self.x[0] + H[1][1]*self.x[1])]
        # S = H P H^T + R (2x2)
        # compute HPHt
        HPHt = [[0.0, 0.0], [0.0, 0.0]]
        for i in range(2):
            for j in range(2):
                s = 0.0
                for k in range(4):
                    for l in range(4):
                        s += H[i][k] * self.P[k][l] * H[j][l]
                HPHt[i][j] = s
        S = [[HPHt[0][0] + self.R[0][0], HPHt[0][1] + self.R[0][1]], [HPHt[1][0] + self.R[1][0], HPHt[1][1] + self.R[1][1]]]

        # invert 2x2 S
        det = S[0][0]*S[1][1] - S[0][1]*S[1][0]
        if abs(det) < 1e-9:
            invS = [[0.0, 0.0],[0.0,0.0]]
        else:
            invS = [[S[1][1]/det, -S[0][1]/det], [-S[1][0]/det, S[0][0]/det]]

        # K = P H^T S^-1  (4x2)
        HT = self._transpose(H)
        PHt = self._matmul(self.P, HT)
        K = [[sum(PHt[i][k] * invS[k][j] for k in range(2)) for j in range(2)] for i in range(4)]

        # update state x = x + K y
        for i in range(4):
            self.x[i] = self.x[i] + K[i][0]*y[0] + K[i][1]*y[1]

        # update P = (I - K H) P
        I = [[1.0 if i==j else 0.0 for j in range(4)] for i in range(4)]
        KH = [[sum(K[i][k]*H[k][j] for k in range(2)) for j in range(4)] for i in range(4)]
        IMKH = [[I[i][j] - KH[i][j] for j in range(4)] for i in range(4)]
        self.P = self._matmul(IMKH, self.P)

        return (self.x[0], self.x[1])


class Track:
    def __init__(self, id, x, y, dt=0.1):
        self.id = id
        self.kf = SimpleKalman2D(x=x, y=y, dt=dt)
        self.last_seen = time.time()

    def predict(self):
        return self.kf.predict()

    def update(self, x, y):
        self.last_seen = time.time()
        return self.kf.update(x, y)


def _cost_matrix(tracks, dets):
    m = len(tracks)
    n = len(dets)
    C = [[0.0 for _ in range(n)] for _ in range(m)]
    for i, t in enumerate(tracks):
        tx, ty = t.kf.x[0], t.kf.x[1]
        for j, (dx, dy) in enumerate(dets):
            C[i][j] = math.hypot(tx - dx, ty - dy)
    return C


def _hungarian_assign(cost):
    """Simple Hungarian assignment for rectangular cost matrix using O(n^3) algorithm.
    Returns list of (row, col) assignments for minimal total cost. This is a
    compact implementation adapted for small matrices.
    """
    # We'll implement a simple version using the algorithm by Kuh-Munkres.
    # For brevity and robustness on small matrices, fallback to greedy if empty.
    if not cost:
        return []
    m = len(cost)
    n = len(cost[0])
    import copy
    C = copy.deepcopy(cost)
    # pad to square
    N = max(m, n)
    for row in C:
        row.extend([1e6] * (N - n))
    for _ in range(N - m):
        C.append([1e6] * N)

    u = [0]* (N+1)
    v = [0]* (N+1)
    p = [0]* (N+1)
    way = [0]* (N+1)
    for i in range(1, N+1):
        p[0] = i
        j0 = 0
        minv = [1e9]*(N+1)
        used = [False]*(N+1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = 1e9
            j1 = 0
            for j in range(1, N+1):
                if used[j]:
                    continue
                cur = C[i0-1][j-1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(0, N+1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    # p[j] - row assigned to column j
    assignment = []
    for j in range(1, N+1):
        i = p[j]
        if i<=m and j<=n:
            assignment.append((i-1, j-1))
    return assignment


class Manager:
    def __init__(self, max_age=1.0, match_dist=0.8, dt=0.1):
        self.tracks = []
        self.next_id = 1
        self.max_age = max_age
        self.match_dist = match_dist
        self.dt = dt

    def predict_all(self):
        preds = []
        for t in self.tracks:
            px, py = t.predict()
            preds.append((t.id, px, py))
        return preds

    def update(self, detections):
        """detections: list of (x,y) in world coords"""
        if not detections:
            # age and prune
            now = time.time()
            self.tracks = [t for t in self.tracks if now - t.last_seen <= self.max_age]
            return

        if not self.tracks:
            for dx, dy in detections:
                nt = Track(self.next_id, dx, dy, dt=self.dt)
                self.next_id += 1
                self.tracks.append(nt)
            return

        C = _cost_matrix(self.tracks, detections)
        assigns = _hungarian_assign(C)
        assigned_tracks = set()
        assigned_dets = set()
        for ti, di in assigns:
            if ti < len(self.tracks) and di < len(detections):
                if C[ti][di] <= self.match_dist:
                    self.tracks[ti].update(detections[di][0], detections[di][1])
                    assigned_tracks.add(ti); assigned_dets.add(di)

        # create tracks for unassigned detections
        for idx, (dx, dy) in enumerate(detections):
            if idx in assigned_dets:
                continue
            nt = Track(self.next_id, dx, dy, dt=self.dt)
            self.next_id += 1
            self.tracks.append(nt)

        # prune old tracks
        now = time.time()
        self.tracks = [t for t in self.tracks if now - t.last_seen <= self.max_age]

    def get_tracks(self):
        return [(t.id, t.kf.x[0], t.kf.x[1]) for t in self.tracks]
