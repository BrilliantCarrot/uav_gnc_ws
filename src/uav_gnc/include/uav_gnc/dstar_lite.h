#pragma once
#include <vector>
#include <array>
#include <set>
#include <unordered_map>
#include <limits>
#include <cmath>

// ============================================================
// D* Lite — 2.5D 증분 경로 계획기
//
// [D* Lite vs A*]
//   A*     : 매번 처음부터 전체 재탐색 → 동적 환경에 부적합
//   D* Lite: 장애물이 바뀐 부분만 증분 업데이트 → 실시간 재계획
//
// [핵심 개념]
//   - 목표점 → 시작점 방향으로 역탐색 (시작점이 계속 변하므로)
//   - g(s)  : s에서 목표까지의 현재 비용 추정
//   - rhs(s): g의 one-step 선견(lookahead) 값
//   - 일관성(consistent): g(s) == rhs(s)  → 최적
//   - 과일관성(overconsistent): g(s) > rhs(s)  → 개선 가능
//   - 미일관성(underconsistent): g(s) < rhs(s)  → 증가 필요
//   - km    : 시작점 이동 시 누적 heuristic 보정값
//
// [Open List 구현]
//   std::set<OpenEntry>로 O(log n) 삽입/삭제/최소값 접근
//   (priority_queue 대신 set → key 업데이트가 깨끗함)
// ============================================================

struct Cell {
    int x{0}, y{0};
    bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
};

struct CellHash {
    size_t operator()(const Cell& c) const {
        // x와 y를 비트 혼합하여 해시 충돌 최소화
        return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 16);
    }
};

// D* Lite의 우선순위 키: (k1, k2) 쌍
// k1 = min(g,rhs) + h + km  → 주 정렬 기준
// k2 = min(g,rhs)           → 동점 시 보조 기준
using Key = std::pair<double, double>;

class DStarLite {
public:
    DStarLite() = default;

    // 격자 초기화
    // width/height : 셀 수
    // resolution   : 셀 하나의 실제 크기 (m)
    // origin_x/y   : 격자 (0,0) 셀의 월드 좌표 (m)
    void init(int width, int height, double resolution,
              double origin_x, double origin_y);

    // 시작점 설정 (월드 좌표) — 드론 현재 위치
    void setStart(double wx, double wy);

    // 목표점 설정 (월드 좌표) — 변경 시 D* Lite 내부 완전 재초기화
    void setGoal(double wx, double wy);

    // 원형 장애물 마킹 (초기 정적 장애물 등록용)
    void markCircleObstacle(double cx, double cy, double radius);

    // 셀 단위 장애물 업데이트 — LiDAR 실시간 연동 핵심 함수
    // 반환: 맵이 실제로 변경됐으면 true (재계획 필요 신호)
    bool updateCell(int xi, int yi, bool occupied);

    // 경로 계획 실행 (초기 계획 + 이후 증분 재계획 모두 이 함수 하나로)
    // 반환: 경로 발견 성공 여부
    bool plan();

    // 계획된 경로 반환 (월드 좌표 {x, y} 리스트)
    // 시작점에서 목표점으로의 greedy descent로 추출
    std::vector<std::array<double, 2>> getPath() const;

    // ── 유틸리티 ─────────────────────────────────────────────
    Cell worldToCell(double wx, double wy) const;
    std::array<double, 2> cellToWorld(const Cell& c) const;
    bool isOccupied(int xi, int yi) const;
    bool inBounds(int x, int y) const;

    int    getWidth()      const { return width_; }
    int    getHeight()     const { return height_; }
    double getResolution() const { return res_; }
    double getOriginX()    const { return ox_; }
    double getOriginY()    const { return oy_; }
    Cell   getStart()      const { return start_; }
    Cell   getGoal()       const { return goal_; }

private:
    // ── D* Lite 핵심 함수 ─────────────────────────────────────
    Key  calcKey(const Cell& s) const;
    void updateVertex(const Cell& u);
    void computeShortestPath();

    // ── 격자 유틸리티 ─────────────────────────────────────────
    std::vector<Cell> getNeighbors(const Cell& s) const;
    double edgeCost(const Cell& a, const Cell& b) const;
    double h(const Cell& a, const Cell& b) const;  // heuristic (유클리드)
    int    cellIdx(int x, int y) const { return y * width_ + x; }

    // ── Open List 조작 (std::set 기반) ────────────────────────
    void openInsert(const Cell& s, const Key& k);
    void openRemove(const Cell& s);
    bool openEmpty() const;
    Key  openTopKey() const;
    Cell openPop();

    // ── 맵 파라미터 ───────────────────────────────────────────
    int    width_{0}, height_{0};
    double res_{0.5};
    double ox_{0.0}, oy_{0.0};

    // 점유 맵 (true = 장애물)
    std::vector<bool> occ_;

    // D* Lite 상태값
    std::vector<double> g_;    // 현재 비용 추정
    std::vector<double> rhs_;  // one-step lookahead 비용

    Cell   start_{0, 0}, goal_{0, 0};
    Cell   s_last_{0, 0};  // 직전 시작점 (km 갱신용)
    double km_{0.0};       // heuristic 누적 보정값
    bool   initialized_{false};

    // ── Open List: set<OpenEntry> ─────────────────────────────
    // std::set은 항상 정렬 상태 유지 → begin()이 항상 최소 key
    struct OpenEntry {
        Key  k;
        Cell c;
        // Key 먼저 비교, 동점 시 셀 좌표로 결정 (전순서 보장)
        bool operator<(const OpenEntry& o) const {
            if (k.first  != o.k.first)  return k.first  < o.k.first;
            if (k.second != o.k.second) return k.second < o.k.second;
            if (c.x != o.c.x)          return c.x < o.c.x;
            return c.y < o.c.y;
        }
    };
    std::set<OpenEntry>                   open_set_;
    std::unordered_map<Cell, Key, CellHash> open_map_; // 셀 → 현재 key 추적

    static constexpr double INF = std::numeric_limits<double>::infinity();
};
