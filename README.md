# Traffic Management & Simulation System

A road-network traffic simulation built as a B.Tech Data Structures project.

## Tech Stack
- **Core engine**: C++17 with g++ (MSYS2 MinGW)
- **Build**: Makefile
- **UI** (Phase 6): plain HTML + JS + Canvas
- **Data format**: JSON (custom parser, no external libs)

## Build & Run

```bash
# From the project root
make        # compile
make run    # compile + run
make clean  # remove binary
```

## Project Structure
```
Traffic Flow Analysis/
  src/        C++ source files
  data/       Road network JSON
  web/        HTML/JS front-end (Phase 6)
  report/     Notes and report
  Makefile
  README.md
```

## Phases
| Phase | Feature | Status |
|-------|---------|--------|
| 1 | Graph + data model | ✅ Done |
| 2 | Min-heap, Dijkstra, A* | ⏳ Next |
| 3 | Traffic simulation | 🔜 |
| 4 | Traffic assignment (MSA) | 🔜 |
| 5 | Dynamic updates + sensor feed | 🔜 |
| 6 | JSON export + web UI | 🔜 |
| 7 | Parallel demo + polish | 🔜 |
