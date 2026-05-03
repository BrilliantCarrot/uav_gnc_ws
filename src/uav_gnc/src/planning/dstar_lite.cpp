#include "uav_gnc/dstar_lite.h"
#include <iostream>
#include <algorithm>
#include <stdexcept>

// ============================================================
// init(): 격자 및 D* Lite 내부 상태 초기화
// ============================================================
void DStarLite::init(int width, int height, double resolution,
                     double origin_x, double origin_y)
{
    width_  = width;
    height_ = height;
    res_    = resolution;
    ox_     = origin_x;
    oy_     = origin_y;

    int N = width * height;
    occ_.assign(N, false);
    g_.assign(N, INF);
    rhs_.assign(N, INF);

    open_set_.clear();
    open_map_.clear();
    km_          = 0.0;
    initialized_ = false;
}

// ============================================================
// setStart(): 드론 현재 위치를 시작점으로 설정
// plan() 호출 시 km가 누적되어 heuristic을 보정함
// ============================================================
void DStarLite::setStart(double wx, double wy)
{
    Cell c = worldToCell(wx, wy);
    // 격자 경계 클램핑
    c.x = std::max(0, std::min(width_  - 1, c.x));
    c.y = std::max(0, std::min(height_ - 1, c.y));
    start_ = c;
}

// ============================================================
// setGoal(): 목표점 설정 및 D* Lite 완전 재초기화
// 목표점이 바뀔 때마다 g, rhs, open list를 리셋해야 함
// ============================================================
void DStarLite::setGoal(double wx, double wy)
{
    Cell c = worldToCell(wx, wy);
    c.x = std::max(0, std::min(width_  - 1, c.x));
    c.y = std::max(0, std::min(height_ - 1, c.y));
    goal_ = c;

    // D* Lite 완전 초기화
    int N = width_ * height_;
    g_.assign(N, INF);
    rhs_.assign(N, INF);
    open_set_.clear();
    open_map_.clear();
    km_     = 0.0;
    s_last_ = start_;

    // D* Lite 핵심: 목표점의 rhs = 0, goal을 open list에 삽입
    // (A*와 달리 목표점에서 시작점 방향으로 역탐색)
    rhs_[cellIdx(goal_.x, goal_.y)] = 0.0;
    openInsert(goal_, calcKey(goal_));
    initialized_ = true;
}

// ============================================================
// markCircleObstacle(): 원형 장애물을 격자에 마킹
// 초기 정적 장애물 등록에 사용 (yaml에서 cx, cy, r 읽어서 호출)
// ============================================================
void DStarLite::markCircleObstacle(double cx, double cy, double radius)
{
    Cell center = worldToCell(cx, cy);
    // 반경을 셀 수로 변환 (+1 여유)
    int r_cells = static_cast<int>(std::ceil(radius / res_)) + 1;

    for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
            int xi = center.x + dx;
            int yi = center.y + dy;
            if (!inBounds(xi, yi)) continue;
            // 셀 중심의 실제 거리 확인
            auto [wx, wy] = cellToWorld({xi, yi});
            if (std::hypot(wx - cx, wy - cy) <= radius) {
                occ_[cellIdx(xi, yi)] = true;
            }
        }
    }
}

// ============================================================
// updateCell(): 셀 하나의 장애물 상태 변경 — LiDAR 연동 핵심
//
// [D* Lite의 증분 재계획이 여기서 시작됨]
//   장애물이 바뀐 셀과 그 이웃들의 rhs를 재계산 → open list에 삽입
//   이후 plan() 호출 시 computeShortestPath()가 변경된 부분만 전파
//   → 전체 재탐색이 아닌 "영향받는 노드만 업데이트"
// ============================================================
bool DStarLite::updateCell(int xi, int yi, bool occupied)
{
    if (!inBounds(xi, yi)) return false;
    int idx = cellIdx(xi, yi);
    if (occ_[idx] == occupied) return false;  // 변화 없음

    occ_[idx] = occupied;
    if (!initialized_) return true;  // 아직 계획 전이면 맵만 업데이트

    // 해당 셀로 진입하는 엣지가 바뀌었으므로
    // 해당 셀 자체와 이웃들의 rhs를 재계산
    Cell changed{xi, yi};
    updateVertex(changed);
    for (auto& nb : getNeighbors(changed)) {
        updateVertex(nb);
    }
    return true;
}

// ============================================================
// plan(): D* Lite 경로 계획 실행
//
// [초기 호출]: 목표에서 시작까지 전체 탐색
// [재계획 호출]: 변경된 부분만 증분 업데이트
//   시작점이 이동했으면 km로 heuristic 보정 후 재계획
// ============================================================
bool DStarLite::plan()
{
    if (!initialized_) {
        std::cerr << "[D* Lite] setGoal()을 먼저 호출하세요.\n";
        return false;
    }

    // 시작점이 이동했을 때 km 보정
    // km += h(s_last, s_start) → heuristic shift를 보상
    // (목표 기준 비용 맵이 유효하도록 유지)
    if (!(s_last_ == start_)) {
        km_ += h(s_last_, start_);
        s_last_ = start_;
    }

    computeShortestPath();

    // 시작점에서 목표까지의 비용이 INF면 경로 없음
    return g_[cellIdx(start_.x, start_.y)] < INF;
}

// ============================================================
// getPath(): 계획된 경로를 월드 좌표 리스트로 추출
//
// greedy descent: 시작점에서 g값이 낮아지는 방향으로 이동
// (D* Lite가 g값을 최적화해두었으므로 greedy가 최적 경로를 줌)
// ============================================================
std::vector<std::array<double, 2>> DStarLite::getPath() const
{
    std::vector<std::array<double, 2>> path;

    if (g_[cellIdx(start_.x, start_.y)] >= INF) return path;  // 경로 없음

    Cell cur      = start_;
    int  max_step = width_ * height_;  // 무한루프 방지

    for (int step = 0; step < max_step; ++step) {
        // 현재 셀의 월드 좌표 추가
        auto [cx, cy] = cellToWorld(cur);
        path.push_back({cx, cy});

        if (cur == goal_) break;

        // 이웃 중 edgeCost(cur→nb) + g(nb)가 최소인 셀로 이동
        double best_cost = INF;
        Cell   best_nb   = cur;

        for (auto& nb : getNeighbors(cur)) {
            if (isOccupied(nb.x, nb.y)) continue;
            double c = edgeCost(cur, nb) + g_[cellIdx(nb.x, nb.y)];
            if (c < best_cost) {
                best_cost = c;
                best_nb   = nb;
            }
        }

        if (best_nb == cur || best_cost >= INF) break;  // 막힌 경로
        cur = best_nb;
    }

    return path;
}

// ============================================================
// calcKey(): 셀 s의 open list 우선순위 키 계산
//
// k1 = min(g(s), rhs(s)) + h(s_start, s) + km
// k2 = min(g(s), rhs(s))
//
// k1이 작을수록 시작점에 가까운(유망한) 셀
// km은 시작점이 이동할 때마다 누적되어 기존 키들을 보정
// ============================================================
Key DStarLite::calcKey(const Cell& s) const
{
    double min_g_rhs = std::min(g_[cellIdx(s.x, s.y)], rhs_[cellIdx(s.x, s.y)]);
    return {min_g_rhs + h(start_, s) + km_, min_g_rhs};
}

// ============================================================
// updateVertex(): 셀 u의 rhs를 재계산하고 open list 갱신
//
// rhs(u) = min over neighbors n of (edgeCost(u,n) + g(n))
// → 이웃의 g값이 바뀌면 u의 rhs도 바뀜 → open list 재삽입
// ============================================================
void DStarLite::updateVertex(const Cell& u)
{
    int ui = cellIdx(u.x, u.y);

    if (!(u == goal_)) {
        // 이웃들 중 최소 (edgeCost + g) 를 rhs로 설정
        double min_rhs = INF;
        for (auto& nb : getNeighbors(u)) {
            if (isOccupied(nb.x, nb.y)) continue;
            double c = edgeCost(u, nb) + g_[cellIdx(nb.x, nb.y)];
            if (c < min_rhs) min_rhs = c;
        }
        rhs_[ui] = min_rhs;
    }

    // open list에서 제거 후, 비일관성이 있으면 재삽입
    openRemove(u);
    if (g_[ui] != rhs_[ui]) {
        openInsert(u, calcKey(u));
    }
}

// ============================================================
// computeShortestPath(): D* Lite 핵심 알고리즘
//
// open list에서 우선순위가 높은 셀부터 꺼내 처리:
//   - 과일관성(g > rhs): g를 rhs로 낮춤 → 이웃 업데이트
//   - 미일관성(g < rhs): g를 INF로 올림 → 이웃 업데이트
//   - key outdated: 새 key로 재삽입
//
// 종료 조건: open list 최소 key >= calcKey(start) AND rhs(start)==g(start)
//   → 시작점이 최적 상태 = 경로 계획 완료
// ============================================================
void DStarLite::computeShortestPath()
{
    int si = cellIdx(start_.x, start_.y);

    while (!openEmpty() &&
           (openTopKey() < calcKey(start_) || rhs_[si] != g_[si]))
    {
        Key  k_old = openTopKey();
        Cell u     = openPop();
        int  ui    = cellIdx(u.x, u.y);
        Key  k_new = calcKey(u);

        if (k_old < k_new) {
            // key가 outdated → 현재 상태에 맞는 새 key로 재삽입
            openInsert(u, k_new);
        }
        else if (g_[ui] > rhs_[ui]) {
            // 과일관성: g를 rhs로 낮춰서 "이 방향으로 가면 더 싸다" 반영
            g_[ui] = rhs_[ui];
            for (auto& nb : getNeighbors(u)) {
                updateVertex(nb);  // 이웃의 rhs도 재계산
            }
        }
        else {
            // 미일관성: g를 INF로 올려 "이 경로가 막혔다" 전파
            g_[ui] = INF;
            updateVertex(u);  // u 자신도 재계산
            for (auto& nb : getNeighbors(u)) {
                updateVertex(nb);
            }
        }
    }
}

// ============================================================
// getNeighbors(): 8방향 이웃 반환 (직선 4 + 대각선 4)
// ============================================================
std::vector<Cell> DStarLite::getNeighbors(const Cell& s) const
{
    // dx, dy: 8방향 오프셋
    static const int dx[] = {-1,  0,  1, -1,  1, -1,  0,  1};
    static const int dy[] = {-1, -1, -1,  0,  0,  1,  1,  1};

    std::vector<Cell> nbrs;
    nbrs.reserve(8);
    for (int i = 0; i < 8; ++i) {
        int nx = s.x + dx[i];
        int ny = s.y + dy[i];
        if (inBounds(nx, ny)) {
            nbrs.push_back({nx, ny});
        }
    }
    return nbrs;
}

// ============================================================
// edgeCost(): 셀 a → b 이동 비용
// 장애물 셀로의 이동은 INF, 대각선은 √2, 직선은 1.0
// ============================================================
double DStarLite::edgeCost(const Cell& a, const Cell& b) const
{
    if (isOccupied(b.x, b.y)) return INF;
    bool diagonal = (a.x != b.x) && (a.y != b.y);
    return diagonal ? 1.4142135623730951 : 1.0;
}

// ============================================================
// h(): 유클리드 heuristic (격자 단위, 과소추정 → A* admissible)
// ============================================================
double DStarLite::h(const Cell& a, const Cell& b) const
{
    double dx = static_cast<double>(a.x - b.x);
    double dy = static_cast<double>(a.y - b.y);
    return std::hypot(dx, dy);
}

// ── 좌표 변환 ────────────────────────────────────────────────

Cell DStarLite::worldToCell(double wx, double wy) const
{
    int xi = static_cast<int>(std::floor((wx - ox_) / res_));
    int yi = static_cast<int>(std::floor((wy - oy_) / res_));
    return {xi, yi};
}

std::array<double, 2> DStarLite::cellToWorld(const Cell& c) const
{
    // 셀 중심 좌표 반환
    double wx = ox_ + (c.x + 0.5) * res_;
    double wy = oy_ + (c.y + 0.5) * res_;
    return {wx, wy};
}

bool DStarLite::isOccupied(int xi, int yi) const
{
    if (!inBounds(xi, yi)) return true;  // 경계 밖 = 장애물 취급
    return occ_[cellIdx(xi, yi)];
}

bool DStarLite::inBounds(int x, int y) const
{
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

// ── Open List 조작 ───────────────────────────────────────────

void DStarLite::openInsert(const Cell& s, const Key& k)
{
    // 기존 항목 제거 후 새 key로 삽입 (key 업데이트)
    openRemove(s);
    open_set_.insert({k, s});
    open_map_[s] = k;
}

void DStarLite::openRemove(const Cell& s)
{
    auto it = open_map_.find(s);
    if (it == open_map_.end()) return;
    open_set_.erase({it->second, s});
    open_map_.erase(it);
}

bool DStarLite::openEmpty() const { return open_set_.empty(); }

Key DStarLite::openTopKey() const { return open_set_.begin()->k; }

Cell DStarLite::openPop()
{
    auto it   = open_set_.begin();
    Cell cell = it->c;
    open_map_.erase(cell);
    open_set_.erase(it);
    return cell;
}
