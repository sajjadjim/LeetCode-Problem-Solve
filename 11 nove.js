function findMaxForm(strs, m, n) {
  const dp = new Map();

  function helper(i, zerosLeft, onesLeft) {
    if (i >= strs.length) return 0;

    const key = `${i},${zerosLeft},${onesLeft}`;
    if (dp.has(key)) return dp.get(key);

    const word = strs[i];
    const oneCount = countOnes(word);
    const zeroCount = countZeros(word);

    let picked = 0;
    if (oneCount <= onesLeft && zeroCount <= zerosLeft) {
      picked = 1 + helper(i + 1, zerosLeft - zeroCount, onesLeft - oneCount);
    }

    const notPicked = helper(i + 1, zerosLeft, onesLeft);
    const result = Math.max(picked, notPicked);

    dp.set(key, result);
    return result;
  }

  return helper(0, m, n);
}

function countOnes(s) {
  let count = 0;
  for (const ch of s) {
    if (ch === '1') count++;
  }
  return count;
}

function countZeros(s) {
  let count = 0;
  for (const ch of s) {
    if (ch === '0') count++;
  }
  return count;
}