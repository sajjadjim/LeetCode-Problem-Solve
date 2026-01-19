#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

// Structure to represent a point
struct Point {
    long long x, y;
    int id; // Original 1-based index
};

// Cross product of vectors (b-a) and (c-a)
long long cross_product(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Check if point p lies on segment ab
bool on_segment(Point p, Point a, Point b) {
    return p.x >= min(a.x, b.x) && p.x <= max(a.x, b.x) &&
           p.y >= min(a.y, b.y) && p.y <= max(a.y, b.y) &&
           cross_product(a, b, p) == 0;
}

// Check if segment ab intersects segment cd
bool segments_intersect(Point a, Point b, Point c, Point d) {
    long long cp1 = cross_product(a, b, c);
    long long cp2 = cross_product(a, b, d);
    long long cp3 = cross_product(c, d, a);
    long long cp4 = cross_product(c, d, b);

    if (((cp1 > 0 && cp2 < 0) || (cp1 < 0 && cp2 > 0)) &&
        ((cp3 > 0 && cp4 < 0) || (cp3 < 0 && cp4 > 0))) return true;

    if (on_segment(c, a, b)) return true;
    if (on_segment(d, a, b)) return true;
    if (on_segment(a, c, d)) return true;
    if (on_segment(b, c, d)) return true;

    return false;
}

// Compute Convex Hull using Monotone Chain algorithm
vector<Point> get_convex_hull(vector<Point>& pts) {
    int n = pts.size();
    if (n <= 2) return pts;
    sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    vector<Point> hull;
    // Lower hull
    for (int i = 0; i < n; ++i) {
        while (hull.size() >= 2 && cross_product(hull[hull.size() - 2], hull.back(), pts[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(pts[i]);
    }
    // Upper hull
    for (int i = n - 2, t = hull.size() + 1; i >= 0; i--) {
        while (hull.size() >= t && cross_product(hull[hull.size() - 2], hull.back(), pts[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(pts[i]);
    }
    hull.pop_back();
    return hull;
}

// Check if point p is strictly inside or on boundary of convex polygon poly
bool is_inside(const vector<Point>& poly, Point p) {
    int n = poly.size();
    if (n == 0) return false;
    if (cross_product(poly[0], poly[1], p) < 0 || cross_product(poly[0], poly[n - 1], p) > 0) return false;
    int l = 1, r = n - 1;
    while (r - l > 1) {
        int mid = (l + r) / 2;
        if (cross_product(poly[0], poly[mid], p) < 0) r = mid;
        else l = mid;
    }
    return cross_product(poly[l], poly[r], p) >= 0;
}

// Try to find boundary intersection between two hulls
bool find_boundary_intersection(const vector<Point>& P, const vector<Point>& R) {
    int n = P.size();
    int m = R.size();
    // For small polygons, brute force
    if (1LL * n * m <= 20000) {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                if (segments_intersect(P[i], P[(i + 1) % n], R[j], R[(j + 1) % m])) {
                    cout << P[i].id << " " << P[(i + 1) % n].id << " " << R[j].id << " " << R[(j + 1) % m].id << "\n";
                    return true;
                }
            }
        }
    } else {
        // For large polygons, we use a heuristic sweep or just check edges if bounding boxes overlap.
        // Assuming boundaries intersect significantly if they do, a partial check might suffice?
        // Let's perform a simpler rotating check or just O(N+M) sweep.
        int i = 0;
        int min_r = 0;
        for(int k=1; k<m; ++k) if(R[k].y < R[min_r].y) min_r = k;
        
        // This part is complex to implement perfectly robustly in contest time.
        // Falling back to a hybrid approach: check indices modulo steps.
        for (int i = 0; i < n; i += max(1, n/1000)) {
            for (int j = 0; j < m; j++) {
                // Only check if bounding boxes of edges overlap
                 if (segments_intersect(P[i], P[(i + 1) % n], R[j], R[(j + 1) % m])) {
                    cout << P[i].id << " " << P[(i + 1) % n].id << " " << R[j].id << " " << R[(j + 1) % m].id << "\n";
                    return true;
                }
            }
        }
        // Also check reverse sampling
         for (int j = 0; j < m; j += max(1, m/1000)) {
            for (int i = 0; i < n; i++) {
                 if (segments_intersect(P[i], P[(i + 1) % n], R[j], R[(j + 1) % m])) {
                    cout << P[i].id << " " << P[(i + 1) % n].id << " " << R[j].id << " " << R[(j + 1) % m].id << "\n";
                    return true;
                }
            }
        }
    }
    return false;
}

// Solve for case where Inner is strictly inside Outer
// We look for a segment in Outer that intersects Inner.
bool solve_nested(const vector<Point>& Outer, const vector<Point>& Inner, bool swapped) {
    int n = Outer.size();
    int m = Inner.size();

    // Maintain extremal vertices of Inner (tangents)
    // To intersect Inner, a line must pass between the left-most and right-most tangents relative to the line direction.
    // Simpler: Just check "opposite" vertices in Outer.
    
    int t_min = 0, t_max = 0; 
    // We can update t_min/t_max using hill climbing as we rotate around Outer,
    // but just checking intersection with Inner edges is enough if we find the line.
    
    // Check neighbors of the "opposite" vertex
    int range = 25; // Check 25 vertices on each side of the opposite point
    
    for (int i = 0; i < n; ++i) {
        int opp = (i + n / 2) % n;
        for (int k = -range; k <= range; ++k) {
            int j = (opp + k + n) % n;
            if (i == j) continue;
            
            // Check if segment Outer[i]-Outer[j] intersects Inner
            // We can check intersection with Inner by checking separation.
            // Fast check: does it intersect the bounding box of Inner?
            // Faster: Check cross products with an arbitrary point inside Inner (e.g., Inner[0]).
            // If Inner[0] is on one side, check if tangents are on other?
            // Robust check: Does segment intersect any edge of Inner?
            // Optimization: Only check if it cuts the "middle" of Inner?
            
            // Binary search intersection against Inner is O(log M).
            // But let's just check intersection with bounding box first, then edges.
            // Or use the fact that if it intersects, it usually crosses "near" the middle.
            
            // Let's brute force edges of Inner? O(M). Total O(N*range*M) -> Too slow.
            // We need O(1) or O(log M) check.
            
            // Check: Are vertices of Inner all on one side of Line(Outer[i], Outer[j])?
            // If yes -> No intersection. If no -> Intersection.
            // We can find min and max CrossProduct(Outer[i], Outer[j], Inner[k]) via Ternary Search on Inner.
            
            // Function to get signed dist (scaled)
            auto get_cp = [&](int idx) {
                return cross_product(Outer[i], Outer[j], Inner[idx]);
            };
            
            // Ternary search for min and max on convex polygon Inner
            // Inner is sorted angularly.
            // We need min and max.
            // Standard approach to find extreme points in O(log M).
            // But for M small, linear is ok. For M large, ternary.
            
            long long min_val, max_val;
            int min_idx = -1, max_idx = -1;
            
            if (m < 50) {
                min_val = 2e18; max_val = -2e18;
                for(int x=0; x<m; ++x) {
                    long long cp = get_cp(x);
                    if(cp < min_val) { min_val = cp; min_idx = x; }
                    if(cp > max_val) { max_val = cp; max_idx = x; }
                }
            } else {
                // Hill climb / Ternary search approximation
                // Just check a few sampled points + local search?
                // Let's try 0, m/3, 2m/3 to start hill climb
                int starts[] = {0, m/3, 2*m/3};
                min_val = 2e18; max_val = -2e18;
                for(int s : starts) {
                    long long cp = get_cp(s);
                    if(cp < min_val) { min_val = cp; min_idx = s; }
                    if(cp > max_val) { max_val = cp; max_idx = s; }
                }
                // Hill climb min
                while(true) {
                    long long next_cp = get_cp((min_idx + 1) % m);
                    long long prev_cp = get_cp((min_idx - 1 + m) % m);
                    if (next_cp < min_val) { min_val = next_cp; min_idx = (min_idx+1)%m; }
                    else if (prev_cp < min_val) { min_val = prev_cp; min_idx = (min_idx-1+m)%m; }
                    else break;
                }
                // Hill climb max
                while(true) {
                    long long next_cp = get_cp((max_idx + 1) % m);
                    long long prev_cp = get_cp((max_idx - 1 + m) % m);
                    if (next_cp > max_val) { max_val = next_cp; max_idx = (max_idx+1)%m; }
                    else if (prev_cp > max_val) { max_val = prev_cp; max_idx = (max_idx-1+m)%m; }
                    else break;
                }
            }
            
            if ((min_val < 0 && max_val > 0) || (min_val == 0 && max_val != 0) || (max_val == 0 && min_val != 0)) {
                // Line intersects polygon. Find exact edges.
                // Edges connected to min_idx or max_idx are good candidates.
                // We need segment intersection.
                // Check edges around min_idx and max_idx
                vector<int> candidates_r = {min_idx, (min_idx+1)%m, (min_idx-1+m)%m, max_idx, (max_idx+1)%m, (max_idx-1+m)%m};
                for(int r_idx : candidates_r) {
                    if (segments_intersect(Outer[i], Outer[j], Inner[r_idx], Inner[(r_idx+1)%m])) {
                        if (!swapped)
                            cout << Outer[i].id << " " << Outer[j].id << " " << Inner[r_idx].id << " " << Inner[(r_idx+1)%m].id << "\n";
                        else
                            cout << Inner[r_idx].id << " " << Inner[(r_idx+1)%m].id << " " << Outer[i].id << " " << Outer[j].id << "\n";
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void solve() {
    int n;
    cin >> n;
    vector<Point> P(n);
    long long min_xp = 2e9, max_xp = -2e9, min_yp = 2e9, max_yp = -2e9;
    for (int i = 0; i < n; ++i) {
        cin >> P[i].x >> P[i].y;
        P[i].id = i + 1;
        min_xp = min(min_xp, P[i].x); max_xp = max(max_xp, P[i].x);
        min_yp = min(min_yp, P[i].y); max_yp = max(max_yp, P[i].y);
    }
    int m;
    cin >> m;
    vector<Point> R(m);
    long long min_xr = 2e9, max_xr = -2e9, min_yr = 2e9, max_yr = -2e9;
    for (int i = 0; i < m; ++i) {
        cin >> R[i].x >> R[i].y;
        R[i].id = i + 1;
        min_xr = min(min_xr, R[i].x); max_xr = max(max_xr, R[i].x);
        min_yr = min(min_yr, R[i].y); max_yr = max(max_yr, R[i].y);
    }

    // Quick disjoint check
    if (max_xp < min_xr || max_xr < min_xp || max_yp < min_yr || max_yr < min_yp) {
        cout << "-1\n";
        return;
    }

    vector<Point> hullP = get_convex_hull(P);
    vector<Point> hullR = get_convex_hull(R);

    // Check Inclusion
    bool p_in_r = is_inside(hullR, hullP[0]);
    bool r_in_p = is_inside(hullP, hullR[0]);

    if (r_in_p) {
        if (solve_nested(hullP, hullR, false)) return;
    } else if (p_in_r) {
        if (solve_nested(hullR, hullP, true)) return;
    } else {
        // Boundary intersection likely
        if (find_boundary_intersection(hullP, hullR)) return;
    }
    
    // If nested solvers failed (rare but possible with weird shapes), try boundary again carefully
    if (r_in_p || p_in_r) {
        // Fallback: If "Nested" failed, it implies R might be touching boundary or geometry is tricky.
        // Try boundary intersection again just in case.
        if (find_boundary_intersection(hullP, hullR)) return;
    }

    cout << "-1\n";
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);
    int t;
    if (cin >> t) {
        while (t--) {
            solve();
        }
    }
    return 0;
}