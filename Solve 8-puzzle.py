
import heapq
import time

# ==================================================
# 1. TRẠNG THÁI ĐÍCH
# ==================================================

GOAL = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 0]
]

GOAL_POS = {
    1: (0, 0), 2: (0, 1), 3: (0, 2),
    4: (1, 0), 5: (1, 1), 6: (1, 2),
    7: (2, 0), 8: (2, 1)
}


# ==================================================
# 2. CÁC HÀM HEURISTIC
# ==================================================

# h1: Số ô sai vị trí
def h1_misplaced(state, goal):
    count = 0

    for r in range(3):
        for c in range(3):
            val = state[r][c]

            if val != 0 and val != goal[r][c]:
                count += 1

    return count


# h2: Khoảng cách Manhattan
def manhattan(state):
    total_dist = 0

    for r in range(3):
        for c in range(3):
            val = state[r][c]

            if val != 0:
                goal_r, goal_c = GOAL_POS[val]

                total_dist += (
                    abs(r - goal_r) +
                    abs(c - goal_c)
                )

    return total_dist


# ==================================================
# 3. SINH TRẠNG THÁI KẾ TIẾP
# ==================================================

def get_neighbors(state):

    # Tìm vị trí ô trống (0)
    for r in range(3):
        for c in range(3):
            if state[r][c] == 0:
                zero_r, zero_c = r, c
                break
        else:
            continue
        break

    moves = [
        (-1, 0, 'Lên'),
        (1, 0, 'Xuống'),
        (0, -1, 'Trái'),
        (0, 1, 'Phải')
    ]

    neighbors = []

    for dr, dc, action in moves:

        new_r = zero_r + dr
        new_c = zero_c + dc

        if 0 <= new_r < 3 and 0 <= new_c < 3:

            new_state = [row[:] for row in state]

            # Hoán đổi ô trống với ô kế cận
            new_state[zero_r][zero_c], new_state[new_r][new_c] = (
                new_state[new_r][new_c],
                new_state[zero_r][zero_c]
            )

            neighbors.append((new_state, action))

    return neighbors


# ==================================================
# 4. THUẬT TOÁN TÌM KIẾM
# ==================================================

def solve_8puzzle(start, goal, mode='ASTAR'):

    start = [row[:] for row in start]
    goal = [row[:] for row in goal]

    # Hàng đợi ưu tiên: (f, g, state, path)
    pq = [(0, 0, start, [])]

    visited = set()
    expanded_nodes = 0

    while pq:

        f, g, current, path = heapq.heappop(pq)

        state_tuple = tuple(map(tuple, current))

        if state_tuple in visited:
            continue

        if current == goal:
            return path, len(path), expanded_nodes

        visited.add(state_tuple)
        expanded_nodes += 1

        # Mở rộng các trạng thái con
        for next_state, action in get_neighbors(current):

            next_tuple = tuple(map(tuple, next_state))

            if next_tuple in visited:
                continue

            next_g = g + 1

            # BFS: Ưu tiên số bước g(n)
            if mode == 'BFS':
                priority = next_g

            # GBFS: Ưu tiên heuristic h1(n)
            elif mode == 'GBFS':
                priority = h1_misplaced(next_state, goal)

            # A*: Ưu tiên f(n) = g(n) + h2(n)
            elif mode == 'ASTAR':
                priority = next_g + manhattan(next_state)

            else:
                raise ValueError("Mode không hợp lệ")

            heapq.heappush(
                pq,
                (priority, next_g, next_state, path + [action])
            )

    return None, None, expanded_nodes


# ==================================================
# 5. SO SÁNH VÀ ĐO THỜI GIAN
# ==================================================

def compare_algorithms(start, goal=GOAL):

    algorithms = [
        ('BFS', 'BFS'),
        ('GBFS', 'GBFS'),
        ('A*', 'ASTAR')
    ]

    print("\n========== 8-PUZZLE RESULTS ==========")

    print(
        f"{'Algorithm':<12}"
        f"{'Depth':<12}"
        f"{'Expanded Nodes':<18}"
        f"{'Time (ms)':<15}"
    )

    print("-" * 57)

    for name, mode in algorithms:

        # Bắt đầu đo thời gian
        start_time = time.perf_counter()

        path, depth, expanded = solve_8puzzle(
            start, goal, mode
        )

        # Kết thúc đo thời gian
        end_time = time.perf_counter()

        # Đổi giây sang milliseconds
        runtime = (end_time - start_time) * 1000

        print(
            f"{name:<12}"
            f"{str(depth):<12}"
            f"{expanded:<18}"
            f"{runtime:<15.4f}"
        )

        if path is not None:
            print("Path:", " -> ".join(path))
        else:
            print("No solution found")


# ==================================================
# 6. TEST
# ==================================================

if __name__ == "__main__":

    start =[
    [5, 1, 3],
    [7, 0, 2],
    [8, 4, 6]
]



    print("INITIAL STATE:")
    for row in start:
        print(row)

    print("\nGOAL STATE:")
    for row in GOAL:
        print(row)

    compare_algorithms(start)
