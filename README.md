# sokoban-solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Abdee36/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Abdee36/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for make.fun's "Build the best Sokoban solver, 21 November 2026" bounty. A generated level is one hallway plus 1–4 rooms, and a box pushed out of a room into the hallway can never come back — so every box stays in its own room and the level optimum is the sum of the rooms' push-optima. The solver splits each level into rooms, solves every room on its own with a breadth-first search over pushes (the first solved state found is push-optimal), pruned by dead squares and frozen 2×2 blocks, then joins the per-room move strings with the walks between doors. A whole-level push search and a move A* stand by as fallbacks for levels that are not shaped like rooms. No clock, no randomness: work budgets are counted in states, so two runs write identical bytes.

Practice set: **1,000,000 / 1,000,000** — every practice level solved in exactly P* pushes (13, 16, 29, 47, 60), in under a second for the whole set. The house baseline scores 200,000 on the same set.

## Run it

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` reads `/input/<id>/level.txt` for `i001`–`i005` and writes `/output/<id>/solution.txt` (moves `u d l r`) for each.

---

<sub>Entered on [make.fun](https://make.fun/?ref=ztkayn92&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
