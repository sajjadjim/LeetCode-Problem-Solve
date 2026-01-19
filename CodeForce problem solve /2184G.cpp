#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

const int INF = 1e9 + 7;

// Segment Tree for Range Minimum Query (RMQ)
struct SegmentTree {
    int n;
    vector<int> tree;

    void init(int _n) {
        n = _n;
        tree.assign(4 * n, INF);
    }

    void build(const vector<int>& a, int node, int start, int end) {
        if (start == end) {
            tree[node] = a[start];
        } else {
            int mid = (start + end) / 2;
            build(a, 2 * node, start, mid);
            build(a, 2 * node + 1, mid + 1, end);
            tree[node] = min(tree[2 * node], tree[2 * node + 1]);
        }
    }

    void update(int node, int start, int end, int idx, int val) {
        if (start == end) {
            tree[node] = val;
        } else {
            int mid = (start + end) / 2;
            if (start <= idx && idx <= mid) {
                update(2 * node, start, mid, idx, val);
            } else {
                update(2 * node + 1, mid + 1, end, idx, val);
            }
            tree[node] = min(tree[2 * node], tree[2 * node + 1]);
        }
    }

    int query(int node, int start, int end, int l, int r) {
        if (r < start || end < l) {
            return INF;
        }
        if (l <= start && end <= r) {
            return tree[node];
        }
        int mid = (start + end) / 2;
        int p1 = query(2 * node, start, mid, l, r);
        int p2 = query(2 * node + 1, mid + 1, end, l, r);
        return min(p1, p2);
    }
};

void solve() {
    int n, q;
    cin >> n >> q;

    vector<int> a(n + 1);
    for (int i = 1; i <= n; ++i) {
        cin >> a[i];
    }

    SegmentTree st;
    st.init(n);
    st.build(a, 1, 1, n);

    while (q--) {
        int type;
        cin >> type;
        if (type == 1) {
            int i, x;
            cin >> i >> x;
            st.update(1, 1, n, i, x);
        } else {
            int l, r;
            cin >> l >> r;

            // Binary Search for d
            // We want to find d such that min(a[l...l+d]) == d
            // Function f(d) = min(a[l...l+d]) - d is strictly decreasing.
            // We search in range [0, r-l]
            
            int low = 0, high = r - l;
            bool found = false;

            while (low <= high) {
                int d = low + (high - low) / 2;
                int min_val = st.query(1, 1, n, l, l + d);

                if (min_val == d) {
                    found = true;
                    break;
                } else if (min_val < d) {
                    // min_val - d < 0. Since function is decreasing, 
                    // we are too far to the right (values became negative).
                    // We need a smaller d.
                    high = d - 1;
                } else {
                    // min_val - d > 0. We are positive.
                    // We need a larger d to subtract more.
                    low = d + 1;
                }
            }

            cout << (found ? 1 : 0) << "\n";
        }
    }
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);
    
    int t;
    cin >> t;
    while (t--) {
        solve();
    }
    return 0;
}