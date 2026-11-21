// Sokoban solver: push-optimal, room by room.
//
// Idea: a generated level is one hallway plus 1-4 rooms. A box pushed out of
// a room into the hallway can never come back, so every box stays in its own
// room and the level optimum is the sum of the rooms' push-optima. Each room
// (3-4 boxes, ~6x6 floor) is solved on its own with a breadth-first search
// over pushes (first solved state found is push-optimal), pruned by dead
// squares and frozen 2x2 blocks. The per-room move strings are joined with
// the walks between doors. If a level is not shaped like that, fall back to
// a whole-level push search, then to a move A* that at least solves it.
//
// Deterministic by construction: fixed direction order, work budgets counted
// in states (never the wall clock), no randomness, no environment reads.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const DIRS = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];
// Work budgets, in states expanded. The generator keeps a room only when its
// own push search solves it inside 250k states, so 250k per room is enough
// for every generated room; the fallbacks are only a safety net.
const ROOM_STATE_CAP = 250000;
const WHOLE_STATE_CAP = 250000;
const MOVE_ASTAR_CAP = 200000;

function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++)
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// Squares a lone box can be pushed from onto some goal, found by pulling
// backwards from the goals. A box on any other square is dead: no push can
// ever save it, so states holding one are pruned.
function deadSquares(level) {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (y < 0 || y >= wall.length || y - d < 0 || y - d >= wall.length) continue;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

// Flood the player's reachable region (boxes are walls). Returns the lowest
// square of the region, which names the region in a state key.
function reachRegion(level, occupied, from, seen, stamp, stack) {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = k === 0 ? p - w : k === 1 ? p + w : k === 2 ? p - 1 : p + 1;
      if (q < 0 || q >= wall.length) continue;
      if (wall[q] === 0 && occupied[q] === 0 && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

// True when the box just pushed to `t` is stuck in a 2x2 block of walls and
// boxes with a box off its goal inside the block.
function frozen2x2(level, occupied, goalSet, t) {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (a < 0 || d >= wall.length) continue;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b])) continue;
    if (!(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goalSet[a]) || (occupied[b] && !goalSet[b])) return true;
    if ((occupied[c] && !goalSet[c]) || (occupied[d] && !goalSet[d])) return true;
  }
  return false;
}

// Shortest walk from `from` to `to` around walls and boxes (lowercase moves).
function walkTo(level, occupied, from, to) {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (q < 0 || q >= wall.length) continue;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: target unreachable');
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

// Rebuild the move string of a push-BFS/A* search from its stored states.
function rebuildMoves(level, statesBoxes, statesPlayer, parent, pushFrom, pushTo, found) {
  const path = [];
  for (let s = found; s > 0; s = parent[s]) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes;
  const occupied = new Uint8Array(level.wall.length);
  for (const s of path) {
    const nextBoxes = statesBoxes[s];
    const from = boxes.find((b) => !nextBoxes.includes(b));
    const to = nextBoxes.find((b) => !boxes.includes(b));
    const d = to - from;
    const dir = DIRS.find(([dx, dy]) => dy * level.w + dx === d);
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += dir[2];
    player = from;
    boxes = nextBoxes;
  }
  return { pushes: path.length, moves };
}

// Push-optimal BFS over one (sub)level. Returns {pushes, moves} or null.
function solvePushBfs(level, maxStates) {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goalSet = new Uint8Array(size);
  for (const g of goals) goalSet[g] = 1;
  const live = deadSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;
  const solved = (boxes) => boxes.every((b) => goalSet[b]);
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  const keyOf = (boxes, low) => {
    // Numeric keys are exact only while size ** (boxes + 1) stays under
    // 2 ** 53, which holds for a room (<= 4 boxes) but not for a whole
    // 16-box level, so large levels use a string key instead.
    if (boxes.length > 5) return `${low}|${boxes.join(',')}`;
    let k = low;
    for (const b of boxes) k = k * size + b;
    return k;
  };
  const statesBoxes = [level.boxes.slice()];
  const statesPlayer = [level.player];
  const parent = [-1];
  const index = new Map();
  for (const b of level.boxes) occupied[b] = 1;
  index.set(keyOf(level.boxes, reachRegion(level, occupied, level.player, seen, ++stamp, stack)), 0);
  for (const b of level.boxes) occupied[b] = 0;
  let found = solved(level.boxes) ? 0 : -1;
  for (let head = 0; found < 0 && head < statesBoxes.length; head++) {
    const boxes = statesBoxes[head];
    for (const b of boxes) occupied[b] = 1;
    reachRegion(level, occupied, statesPlayer[head], seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi];
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        if (t < 0 || t >= size || b - d < 0 || b - d >= size) continue;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozen2x2(level, occupied, goalSet, t);
        let key = -1;
        let next = null;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((a, b2) => a - b2);
          key = keyOf(next, reachRegion(level, occupied, b, mark, ++stamp, stack));
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || index.has(key)) continue;
        if (statesBoxes.length >= maxStates) {
          for (const x of boxes) occupied[x] = 0;
          return null;
        }
        index.set(key, statesBoxes.length);
        statesBoxes.push(next);
        statesPlayer.push(b);
        parent.push(head);
        if (solved(next)) {
          found = statesBoxes.length - 1;
          break;
        }
      }
    }
    for (const x of boxes) occupied[x] = 0;
  }
  if (found < 0) return null;
  return rebuildMoves(level, statesBoxes, statesPlayer, parent, null, null, found);
}

// Push A* fallback for a room the BFS gave up on: same states and pruning,
// ordered by pushes + a min-assignment Manhattan heuristic (admissible: one
// push moves one box one square). Still push-optimal when it finishes.
function solvePushAstar(level, maxStates) {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goalSet = new Uint8Array(size);
  for (const g of goals) goalSet[g] = 1;
  const live = deadSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;
  const solved = (boxes) => boxes.every((b) => goalSet[b]);
  const manhattan = (a, b) => Math.abs((a % w) - (b % w)) + Math.abs(Math.floor(a / w) - Math.floor(b / w));
  // Minimum over all box->goal assignments, by brute-force permutations.
  const heuristic = (boxes) => {
    const n = boxes.length;
    const perm = goals.slice(0, n);
    let best = Infinity;
    const rec = (i, used, sum) => {
      if (sum >= best) return;
      if (i === n) {
        best = sum;
        return;
      }
      for (let j = 0; j < n; j++) {
        if (used[j]) continue;
        used[j] = true;
        rec(i + 1, used, sum + manhattan(boxes[i], perm[j]));
        used[j] = false;
      }
    };
    rec(0, new Array(n).fill(false), 0);
    return best;
  };
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  const keyOf = (boxes, low) => {
    // Same numeric-key limit as in solvePushBfs: string keys past 5 boxes.
    if (boxes.length > 5) return `${low}|${boxes.join(',')}`;
    let k = low;
    for (const b of boxes) k = k * size + b;
    return k;
  };
  const statesBoxes = [level.boxes.slice()];
  const statesPlayer = [level.player];
  const parent = [-1];
  const cost = [0];
  const best = new Map();
  for (const b of level.boxes) occupied[b] = 1;
  best.set(keyOf(level.boxes, reachRegion(level, occupied, level.player, seen, ++stamp, stack)), 0);
  for (const b of level.boxes) occupied[b] = 0;
  // Binary heap on [f, g, state], ties: deeper g first, then earlier state.
  const heap = [[heuristic(level.boxes), 0, 0]];
  const less = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        [heap[i], heap[c]] = [heap[c], heap[i]];
        i = c;
      }
    }
    return top;
  };
  let expanded = 0;
  let found = solved(level.boxes) ? 0 : -1;
  while (heap.length > 0 && expanded < maxStates && found < 0) {
    const [, g, s] = heapPop();
    if (cost[s] !== g) continue;
    const boxes = statesBoxes[s];
    expanded++;
    for (const b of boxes) occupied[b] = 1;
    reachRegion(level, occupied, statesPlayer[s], seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        if (t < 0 || t >= size || b - d < 0 || b - d >= size) continue;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozen2x2(level, occupied, goalSet, t);
        let key = -1;
        let next = null;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((a, b2) => a - b2);
          key = keyOf(next, reachRegion(level, occupied, b, mark, ++stamp, stack));
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen) continue;
        const prev = best.get(key);
        if (prev !== undefined && prev <= g + 1) continue;
        best.set(key, g + 1);
        const ns = statesBoxes.length;
        statesBoxes.push(next);
        statesPlayer.push(b);
        parent.push(s);
        cost.push(g + 1);
        if (solved(next)) {
          found = ns;
          break;
        }
        heapPush([g + 1 + heuristic(next), g + 1, ns]);
      }
      if (found >= 0) break;
    }
    for (const x of boxes) occupied[x] = 0;
  }
  if (found < 0) return null;
  return rebuildMoves(level, statesBoxes, statesPlayer, parent, null, null, found);
}

// Move A* last resort: plain A* over steps (fewest moves), with dead-square
// pruning on pushes so it does not waste its budget on dead states. A
// solution found here may cost extra pushes, but extra pushes still score
// (500 x P*/pushes) where an empty answer scores 0.
function solveMoveAstar(level, maxStates) {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goalSet = new Uint8Array(size);
  for (const g of goals) goalSet[g] = 1;
  const live = deadSquares(level);
  const dist = (b) => Math.min(...goals.map((g) => Math.abs((g % w) - (b % w)) + Math.abs(Math.floor(g / w) - Math.floor(b / w))));
  const est = (bs) => bs.reduce((a, b) => a + dist(b), 0);
  const key = (p, bs) => `${p}|${bs.join(',')}`;
  const sp = [level.player];
  const sb = [level.boxes.slice()];
  const sg = [0];
  const parent = [-1];
  const step = [''];
  const best = new Map([[key(level.player, level.boxes), 0]]);
  const heap = [[est(level.boxes), 0, 0]];
  const less = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        [heap[i], heap[c]] = [heap[c], heap[i]];
        i = c;
      }
    }
    return top;
  };
  let expanded = 0;
  while (heap.length > 0 && expanded < maxStates) {
    const [, g, s] = heapPop();
    if (best.get(key(sp[s], sb[s])) !== g) continue;
    const bs = sb[s];
    if (bs.every((b) => goalSet[b])) {
      let moves = '';
      for (let t = s; t > 0; t = parent[t]) moves = step[t] + moves;
      return { pushes: -1, moves };
    }
    expanded++;
    const p = sp[s];
    for (const [dx, dy, letter] of DIRS) {
      const d = dy * w + dx;
      const q = p + d;
      if (q < 0 || q >= size || wall[q]) continue;
      let nb = bs;
      const at = bs.indexOf(q);
      if (at >= 0) {
        if (wall[q + d] || bs.includes(q + d) || !live[q + d]) continue;
        nb = bs.slice();
        nb[at] = q + d;
        nb.sort((a, b) => a - b);
      }
      const k = key(q, nb);
      const seenG = best.get(k);
      if (seenG !== undefined && seenG <= g + 1) continue;
      const t = sp.length;
      sp.push(q);
      sb.push(nb);
      sg.push(g + 1);
      parent.push(s);
      step.push(letter);
      best.set(k, g + 1);
      heapPush([g + 1 + est(nb), g + 1, t]);
    }
  }
  return null;
}

// Split a level into rooms: cut the hallway row's dead squares out of the
// floor; each remaining component holding boxes is a room. Null when the
// level is not a set of rooms (a box in the hallway, or boxes != goals).
function roomsOf(level) {
  const { w, wall, goals, boxes, player } = level;
  const live = deadSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i] + dy * w + dx;
        if (q < 0 || q >= wall.length) continue;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [], done: false });
  }
  for (const b of boxes) {
    if (part[b] < 0) return null;
    rooms[part[b]].boxes.push(b);
  }
  for (const g of goals) {
    if (part[g] < 0) return null;
    rooms[part[g]].goals.push(g);
  }
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

// Apply a move string to a full level state; returns the player's end square.
function endSquare(level, from, moves) {
  let p = from;
  for (const ch of moves) p += ch === 'u' ? -level.w : ch === 'd' ? level.w : ch === 'l' ? -1 : 1;
  return p;
}

function solveLevel(level) {
  const rooms = roomsOf(level);
  if (rooms !== null) {
    // Rooms are solved in scan order first. If any room fails from the
    // player position the previous rooms left, retry in reverse order: the
    // rooms are independent, so either order reaches the same push total,
    // and a different entry position can unstick a room.
    const tryRooms = (order) => {
      let player = level.player;
      let pushes = 0;
      let moves = '';
      for (const room of order) {
        const wall = level.wall.slice();
        for (const other of rooms)
          if (other !== room)
            for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
        const sub = {
          w: level.w,
          h: level.h,
          wall,
          goals: room.goals.slice().sort((a, b) => a - b),
          boxes: room.boxes.slice().sort((a, b) => a - b),
          player,
        };
        const solved = solvePushBfs(sub, ROOM_STATE_CAP) ?? solvePushAstar(sub, ROOM_STATE_CAP);
        if (solved === null) return null;
        pushes += solved.pushes;
        moves += solved.moves;
        player = endSquare(level, player, solved.moves);
        room.done = true;
      }
      return { pushes, moves };
    };
    const first = tryRooms(rooms);
    if (first !== null) return first;
    for (const r of rooms) r.done = false;
    const reversed = tryRooms(rooms.slice().reverse());
    if (reversed !== null) return reversed;
  }
  // Not a room set: whole-level push search, then move A*.
  const whole = solvePushBfs(level, WHOLE_STATE_CAP);
  if (whole !== null) return whole;
  return solveMoveAstar(level, MOVE_ASTAR_CAP);
}

function solve(text) {
  const level = parseLevel(text);
  if (level.boxes.every((b) => level.goals.includes(b))) return '';
  const solved = solveLevel(level);
  return solved === null ? '' : solved.moves;
}

for (const id of readdirSync(INPUT).sort()) {
  const moves = solve(readFileSync(join(INPUT, id, 'level.txt'), 'utf8'));
  mkdirSync(join(OUTPUT, id), { recursive: true });
  writeFileSync(join(OUTPUT, id, 'solution.txt'), `${moves}\n`);
}
