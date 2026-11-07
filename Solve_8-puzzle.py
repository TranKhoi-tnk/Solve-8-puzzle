"""
Riêng Time (ms) phụ thuộc phần cứng, CPU nên KHÔNG thể giống tuyệt
đối giữa các máy.
"""
import heapq
import time

# ---------------------------------------------------------------------------
# KHỐI 1: HEURISTIC
# ---------------------------------------------------------------------------
GOAL_POS = {
    1: (0, 0), 2: (0, 1), 3: (0, 2),
    4: (1, 0), 5: (1, 1), 6: (1, 2),
    7: (2, 0), 8: (2, 1),
}


def h1_misplaced(state, goal):
    """h1: số ô (1..8) nằm sai vị trí so với đích."""
    count = 0
    for r in range(3):
        for c in range(3):
            val = state[r][c]
            if val != 0 and val != goal[r][c]:
                count += 1
    return count


def manhattan(state):
    """h2: tổng khoảng cách Manhattan của 8 ô số (bỏ qua ô trống)."""
    total_dist = 0
    for r in range(3):
        for c in range(3):
            val = state[r][c]
            if val != 0:
                goal_r, goal_c = GOAL_POS[val]
                total_dist += abs(r - goal_r) + abs(c - goal_c)
    return total_dist


# ---------------------------------------------------------------------------
# KHỐI 2: SINH TRẠNG THÁI KẾ TIẾP (thứ tự cố định: Lên, Xuống, Trái, Phải)
# ---------------------------------------------------------------------------
def get_neighbors(state):
    for r in range(3):
        for c in range(3):
            if state[r][c] == 0:
                zero_r, zero_c = r, c
    moves = [(-1, 0, 'Lên'), (1, 0, 'Xuống'), (0, -1, 'Trái'), (0, 1, 'Phải')]
    neighbors = []
    for dr, dc, action in moves:
        new_r, new_c = zero_r + dr, zero_c + dc
        if 0 <= new_r < 3 and 0 <= new_c < 3:
            new_state = [row[:] for row in state]
            new_state[zero_r][zero_c], new_state[new_r][new_c] = (
                new_state[new_r][new_c], new_state[zero_r][zero_c]
            )
            neighbors.append((new_state, action))
    return neighbors


# ---------------------------------------------------------------------------
# KHỐI 3: THUẬT TOÁN TÌM KIẾM TỔNG QUÁT (BFS / GBFS / A*)
# ---------------------------------------------------------------------------
def solve_8puzzle(start, goal, mode='ASTAR'):
    # pq lưu: (priority, g, state, path). Hòa priority -> so g, rồi state, rồi path
    # => thứ tự mở rộng hoàn toàn xác định.
    pq = [(0, 0, start, [])]
    visited = set()
    while pq:
        f, g, current, path = heapq.heappop(pq)
        if current == goal:
            return path, len(visited)

        state_tuple = tuple(map(tuple, current))
        if state_tuple in visited:
            continue
        visited.add(state_tuple)

        for next_state, action in get_neighbors(current):
            next_g = g + 1
            if mode == 'BFS':
                priority = next_g
            elif mode == 'GBFS':
                priority = h1_misplaced(next_state, goal)
            else:  # ASTAR
                priority = next_g + manhattan(next_state)
            heapq.heappush(pq, (priority, next_g, next_state, path + [action]))
    return None, len(visited)


# ---------------------------------------------------------------------------
# CHẠY THỰC NGHIỆM
# ---------------------------------------------------------------------------
GOAL_STATE = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]

S0_CASE_1 = [[4, 1, 2], [6, 0, 3], [7, 5, 8]]   # d* = 8
S0_CASE_2 = [[8, 1, 3], [2, 5, 6], [4, 7, 0]]   # d* = 12
S0_CASE_3 = [[5, 1, 3], [7, 0, 2], [8, 4, 6]]   # d* = 16

TEST_CASES = [(S0_CASE_1, 8), (S0_CASE_2, 12), (S0_CASE_3, 16)]
N_REPEAT = 20   # số lần chạy để lấy thời gian trung bình (chỉ ảnh hưởng cột Time)


def run_case(start, d_star):
    print(f"=== KẾT QUẢ CHẠY TRÊN S0 (d* = {d_star}) ===")
    best_path = None
    for label, mode in (("BFS", "BFS"), ("GBFS", "GBFS"), ("A*", "ASTAR")):
        t0 = time.perf_counter()
        for _ in range(N_REPEAT):
            path, expanded = solve_8puzzle(start, GOAL_STATE, mode)
        elapsed_ms = (time.perf_counter() - t0) / N_REPEAT * 1000
        print(f"[{label:<4}] Depth: {len(path):<2} | Expanded: {expanded} nodes "
              f"| Time: {elapsed_ms:.4f} ms")
        if mode == "ASTAR":
            best_path = path
    print("\n>>> Vết đường đi tối ưu (A* & BFS):")
    print(best_path)
    print()


if __name__ == "__main__":
    for start, d_star in TEST_CASES:
        run_case(start, d_star)