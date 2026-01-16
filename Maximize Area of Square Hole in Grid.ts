function maximizeSquareHoleArea(n: number, m: number, hBars: number[], vBars: number[]): number {
    const findLen = (bars: number[]): number => {
        bars.sort((a, b) => a - b);
        let longest = 1, curr = 1;
        for (let i = 1; i < bars.length; i++) {
            if (bars[i] === bars[i - 1] + 1) {
                curr++;
                longest = Math.max(longest, curr);
            } else {
                curr = 1;
            }
        }
        return longest;
    };

    const side = 1 + Math.min(findLen(hBars), findLen(vBars));
    return side * side;
};