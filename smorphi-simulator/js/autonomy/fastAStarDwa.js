/**
 * RoboRoarZ Smorphi Simulator - Competition-Grade Autonomous Navigation System
 * Architecture:
 *   Online 2D Occupancy Grid (LiDAR Bresenham Raycaster)
 *     -> Morphology-Aware Costmap Inflation
 *     -> 8-Connected A* Global Planner with Corner-Cutting Guard & Replan Trigger
 *     -> Line-of-Sight Path Simplification & Adaptive Lookahead
 *     -> 3-DOF Holonomic Dynamic Window Approach (DWA) with Acceleration Limits
 *     -> Hard Collision Rejection & Braking Feasibility Guard
 *     -> Morphology Manager (Hysteresis-based narrow corridor passage)
 *     -> Anti-Stall & Contact Recovery Watchdog
 *
 * File: smorphi-simulator/js/autonomy/fastAStarDwa.js
 */

(function (global) {
  "use strict";

  // ============================================================
  // CONFIGURATION CONSTANTS
  // ============================================================
  const DEFAULT_CONFIG = {
    // Arena Boundaries (RoboRoarZ metric coordinate system: [0.0, 5.0] x [0.0, 5.0])
    GRID_MIN_X: 0.0,
    GRID_MIN_Y: 0.0,
    GRID_MAX_X: 5.0,
    GRID_MAX_Y: 5.0,
    GRID_RESOLUTION: 0.10, // 10 cm per cell -> 50x50 cells

    // Cell States
    UNKNOWN: 0,
    FREE: 1,
    OCCUPIED: 2,

    // LiDAR Mapping
    LIDAR_MIN_RANGE: 0.05,
    LIDAR_MAX_RANGE: 4.85,
    MAP_UPDATE_INTERVAL: 0.10, // 10 Hz map update cadence

    // Robot Kinematics & Physical Limits (matching CONFIG.ROBOT)
    MAX_LINEAR_SPEED: 0.58, // Fast open-space cruise (close to physical 0.60 m/s limit)
    MAX_ANGULAR_SPEED: 3.0, // rad/s
    LINEAR_ACCEL: 2.5, // m/s^2
    ANGULAR_ACCEL: 8.0, // rad/s^2

    // DWA Parameters
    DWA_DT: 0.08, // Simulation prediction step (s)
    DWA_HORIZON: 0.80, // Prediction horizon (s) -> 10 steps
    DWA_UPDATE_INTERVAL: 0.05, // 20 Hz controller rate
    VX_SAMPLES: 7,
    VY_SAMPLES: 7,
    OMEGA_SAMPLES: 5,

    // Planning & Simplification
    ASTAR_REPLAN_INTERVAL: 0.35, // Replanning rate (~3 Hz)
    PATH_LOOKAHEAD: 0.45, // Lookahead anchor distance (m)
    PATH_SIMPLIFY_CLEARANCE: 0.18, // Line-of-sight safety clearance

    // Arrival & Dwell
    ARRIVAL_TOLERANCE: 0.06, // 6 cm radius to declare target arrival
    FINAL_DECEL_DIST: 0.45, // Distance threshold for adaptive deceleration
    DWELL_TIME: 5.0, // Configurable dwell time in seconds (can be 0)

    // Morphology Dimensions (Smorphi 16cm modular cubes)
    O_HALF_LENGTH: 0.16,
    O_HALF_WIDTH: 0.16,
    I_HALF_LENGTH: 0.32,
    I_HALF_WIDTH: 0.08,
    SAFETY_MARGIN: 0.04, // Buffer added to footprint

    // Corridor Detection (Smorphi "O" is 0.32m wide, narrow passage is ~0.27m)
    MORPH_TRIGGER_WIDTH: 0.40, // Trigger morph to "I" if passage width < 40 cm
    MORPH_EXIT_WIDTH: 0.65, // Return to "O" if passage width > 65 cm on both flanks

    // Recovery & Anti-Stall
    STALL_TIME: 0.45, // Seconds without progress before recovery triggers
    STALL_COMMAND_THRESH: 0.20, // Min commanded speed to trigger stall watchdog
    RECOVERY_DURATION: 0.40, // Duration of lateral recovery pulse

    // Multi-Objective DWA Scoring Weights
    SCORE_PROGRESS: 5.2,
    SCORE_PATH: 2.8,
    SCORE_CLEARANCE: 4.2,
    SCORE_SPEED: 2.4,
    SCORE_DIRECTION: 2.0,
    SCORE_ROTATION: 0.6,
    SCORE_CMD_SMOOTH: 0.4,
    UNKNOWN_PENALTY: 0.35,

    // Debugging
    DEBUG: false,
  };

  // ============================================================
  // MATH & UTILITY FUNCTIONS
  // ============================================================
  function clamp(val, min, max) {
    return Math.max(min, Math.min(max, val));
  }

  function lerp(a, b, t) {
    return a + (b - a) * t;
  }

  function normalizeAngle(rad) {
    while (rad > Math.PI) rad -= 2 * Math.PI;
    while (rad < -Math.PI) rad += 2 * Math.PI;
    return rad;
  }

  function safeNum(val, fallback = 0) {
    return Number.isFinite(val) ? val : fallback;
  }

  function approach(curr, target, maxDelta) {
    if (curr < target) return Math.min(curr + maxDelta, target);
    return Math.max(curr - maxDelta, target);
  }

  function getTimeSec() {
    if (typeof performance !== "undefined" && performance.now) {
      return performance.now() / 1000;
    }
    return Date.now() / 1000;
  }

  // ============================================================
  // OCCUPANCY GRID & COSTMAP
  // ============================================================
  class OccupancyGrid {
    constructor(cfg) {
      this.cfg = cfg;
      this.res = cfg.GRID_RESOLUTION;
      this.width = Math.round((cfg.GRID_MAX_X - cfg.GRID_MIN_X) / this.res);
      this.height = Math.round((cfg.GRID_MAX_Y - cfg.GRID_MIN_Y) / this.res);

      this.cells = new Uint8Array(this.width * this.height);
      this.cost = new Float32Array(this.width * this.height);

      this.initPerimeterWalls();
    }

    initPerimeterWalls() {
      this.cells.fill(this.cfg.UNKNOWN);
      this.cost.fill(0);

      // Pre-mark arena boundaries (0.1m perimeter wall) as OCCUPIED
      for (let x = 0; x < this.width; x++) {
        this.cells[this.index(x, 0)] = this.cfg.OCCUPIED;
        this.cells[this.index(x, this.height - 1)] = this.cfg.OCCUPIED;
      }
      for (let y = 0; y < this.height; y++) {
        this.cells[this.index(0, y)] = this.cfg.OCCUPIED;
        this.cells[this.index(this.width - 1, y)] = this.cfg.OCCUPIED;
      }
    }

    validCell(gx, gy) {
      return gx >= 0 && gx < this.width && gy >= 0 && gy < this.height;
    }

    index(gx, gy) {
      return gy * this.width + gx;
    }

    worldToGrid(x, y) {
      return {
        x: Math.floor((x - this.cfg.GRID_MIN_X) / this.res),
        y: Math.floor((y - this.cfg.GRID_MIN_Y) / this.res),
      };
    }

    gridToWorld(gx, gy) {
      return {
        x: this.cfg.GRID_MIN_X + (gx + 0.5) * this.res,
        y: this.cfg.GRID_MIN_Y + (gy + 0.5) * this.res,
      };
    }

    get(gx, gy) {
      if (!this.validCell(gx, gy)) return this.cfg.OCCUPIED;
      return this.cells[this.index(gx, gy)];
    }

    set(gx, gy, val) {
      if (!this.validCell(gx, gy)) return;
      this.cells[this.index(gx, gy)] = val;
    }

    getCost(gx, gy) {
      if (!this.validCell(gx, gy)) return Infinity;
      return this.cost[this.index(gx, gy)];
    }

    setCost(gx, gy, val) {
      if (!this.validCell(gx, gy)) return;
      this.cost[this.index(gx, gy)] = val;
    }

    markFree(gx, gy) {
      if (!this.validCell(gx, gy)) return;
      // Never overwrite perimeter boundaries
      if (gx === 0 || gx === this.width - 1 || gy === 0 || gy === this.height - 1) return;
      const idx = this.index(gx, gy);
      if (this.cells[idx] !== this.cfg.OCCUPIED) {
        this.cells[idx] = this.cfg.FREE;
      }
    }

    markOccupied(gx, gy) {
      if (!this.validCell(gx, gy)) return;
      this.cells[this.index(gx, gy)] = this.cfg.OCCUPIED;
    }
  }

  // Bresenham 2D Line Raycaster on Grid
  function traceRayCells(grid, x0, y0, x1, y1) {
    const a = grid.worldToGrid(x0, y0);
    const b = grid.worldToGrid(x1, y1);

    let x = a.x;
    let y = a.y;
    const dx = Math.abs(b.x - a.x);
    const dy = Math.abs(b.y - a.y);
    const sx = a.x < b.x ? 1 : -1;
    const sy = a.y < b.y ? 1 : -1;
    let err = dx - dy;

    const cells = [];
    let count = 0;
    while (count < 250) {
      count++;
      cells.push({ x, y });
      if (x === b.x && y === b.y) break;
      const e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
      }
    }
    return cells;
  }

  // ============================================================
  // MIN-HEAP PRIORITY QUEUE FOR A*
  // ============================================================
  class MinHeap {
    constructor() {
      this.items = [];
    }

    push(item) {
      this.items.push(item);
      let i = this.items.length - 1;
      while (i > 0) {
        const p = Math.floor((i - 1) / 2);
        if (this.items[p].f <= this.items[i].f) break;
        const tmp = this.items[p];
        this.items[p] = this.items[i];
        this.items[i] = tmp;
        i = p;
      }
    }

    pop() {
      if (this.items.length === 0) return null;
      const top = this.items[0];
      const last = this.items.pop();
      if (this.items.length > 0) {
        this.items[0] = last;
        let i = 0;
        const len = this.items.length;
        while (true) {
          const l = i * 2 + 1;
          const r = i * 2 + 2;
          let smallest = i;
          if (l < len && this.items[l].f < this.items[smallest].f) smallest = l;
          if (r < len && this.items[r].f < this.items[smallest].f) smallest = r;
          if (smallest === i) break;
          const tmp = this.items[i];
          this.items[i] = this.items[smallest];
          this.items[smallest] = tmp;
          i = smallest;
        }
      }
      return top;
    }

    get length() {
      return this.items.length;
    }
  }

  // ============================================================
  // FAST A* + HOLONOMIC DWA NAVIGATOR CLASS
  // ============================================================
  class FastAStarDwaNavigator {
    constructor(userConfig = {}) {
      this.cfg = Object.assign({}, DEFAULT_CONFIG, userConfig);

      // Online Grid & Map
      this.grid = new OccupancyGrid(this.cfg);

      // Navigation State
      this.path = [];
      this.pathIndex = 0;
      this.rawPath = [];
      this.simplifiedPath = [];
      this.currentTarget = { x: null, y: null };
      this.targetCompleted = false;
      this.lastLookaheadPoint = null;
      this.lastClearance = 0;
      this.lastCorridorWidth = null;
      this.corridorRequiresI = false;

      // Timers & Cadence Tracking
      this.lastMapTime = -Infinity;
      this.lastPlanTime = -Infinity;
      this.lastDwaTime = -Infinity;
      this.dwellStartTime = null;

      // Morphology
      this.desiredShape = "O";
      this.lastShapeCommand = null;
      this.lastMorphingState = false;

      // Velocity State & History
      this.lastCmd = { vx: 0, vy: 0, omega: 0 };
      this.prevPose = null;
      this.actualSpeed = 0;

      // Recovery State
      this.stallStartTime = null;
      this.recoveryStartTime = null;
      this.recoveryDirection = 1; // +1 = strafe left, -1 = strafe right

      // Debugging
      this.debugEnabled = !!this.cfg.DEBUG;
      this.lastLogTime = 0;
    }

    /**
     * Primary tick interface: called every simulation frame by CodeEngine
     * @param {Object} sensors - Public sensor snapshot
     * @param {Object} robot - Public robot API { setVelocity, setShape, stop, log }
     * @param {number} dt - Delta time (s)
     */
    update(sensors, robot, dt) {
      const t = getTimeSec();

      if (!sensors || !sensors.pose || !sensors.target) {
        return;
      }

      const pose = {
        x: safeNum(sensors.pose.x),
        y: safeNum(sensors.pose.y),
        theta: safeNum(sensors.pose.theta),
      };

      const target = {
        x: safeNum(sensors.target.x),
        y: safeNum(sensors.target.y),
        distance: safeNum(sensors.target.distance),
        angle: safeNum(sensors.target.angle),
        reached: !!sensors.target.reached,
      };

      // 1. Estimate Actual Speed from Pose / Wheel Velocity
      if (Number.isFinite(sensors.pose.vx) && Number.isFinite(sensors.pose.vy)) {
        this.actualSpeed = Math.hypot(sensors.pose.vx, sensors.pose.vy);
      } else if (this.prevPose) {
        const elapsed = t - this.prevPose.t;
        if (elapsed > 0.001) {
          const moved = Math.hypot(pose.x - this.prevPose.x, pose.y - this.prevPose.y);
          this.actualSpeed = moved / elapsed;
        }
      }
      this.prevPose = { x: pose.x, y: pose.y, t };

      // 2. Track Dynamic Target Lifecycle
      const targetChanged =
        this.currentTarget.x === null ||
        Math.hypot(target.x - this.currentTarget.x, target.y - this.currentTarget.y) > 0.05;

      if (targetChanged) {
        this.currentTarget.x = target.x;
        this.currentTarget.y = target.y;
        this.targetCompleted = false;
        this.dwellStartTime = null;
        this.path = [];
        this.corridorRequiresI = false;
        this.lastPlanTime = -Infinity;
        if (this.debugEnabled) {
          robot.log(`[NAVIGATOR] New target registered: (${target.x.toFixed(2)}, ${target.y.toFixed(2)})`);
        }
      }

      // 3. Online Occupancy Mapping (10 Hz Cadence)
      if (t - this.lastMapTime >= this.cfg.MAP_UPDATE_INTERVAL) {
        this.updateLiDARMap(sensors, pose);
        this.lastMapTime = t;
      }

      // 4. Morphology Management (Hysteresis & Narrow Passages)
      const currentShape = sensors.shape || "O";
      const isMorphing = !!sensors.isMorphing;

      this.updateMorphology(sensors, robot, currentShape, isMorphing);

      // If morphology just completed, force immediate A* replanning
      if (this.lastMorphingState && !isMorphing) {
        this.lastPlanTime = -Infinity;
      }
      this.lastMorphingState = isMorphing;

      // 5. Target Arrival & Configurable Dwell Logic
      const distToGoal = Math.hypot(target.x - pose.x, target.y - pose.y);
      const isArrived = distToGoal <= this.cfg.ARRIVAL_TOLERANCE || target.reached;

      if (isArrived) {
        robot.setVelocity(0, 0, 0);
        this.lastCmd = { vx: 0, vy: 0, omega: 0 };

        if (this.dwellStartTime === null) {
          this.dwellStartTime = t;
          robot.log(`[TARGET REACHED] (${target.x.toFixed(2)}, ${target.y.toFixed(2)}) - Dwell started.`);
        }

        if (t - this.dwellStartTime >= this.cfg.DWELL_TIME) {
          if (!this.targetCompleted) {
            this.targetCompleted = true;
            robot.log(`[MISSION COMPLETE] Target dwell finished. Standing by for new target.`);
          }
        }
        return;
      }
      this.dwellStartTime = null;

      // 6. Recovery Watchdog Check (Collision or Stall)
      const inCollision = !!sensors.collision;
      const isStalled =
        Math.hypot(this.lastCmd.vx, this.lastCmd.vy) >= this.cfg.STALL_COMMAND_THRESH &&
        this.actualSpeed < 0.04;

      if (inCollision || isStalled) {
        if (this.stallStartTime === null) this.stallStartTime = t;
      } else {
        this.stallStartTime = null;
      }

      if (this.stallStartTime !== null && t - this.stallStartTime >= this.cfg.STALL_TIME) {
        if (this.recoveryStartTime === null) {
          this.recoveryStartTime = t;
          // Determine clearer side to strafe away
          let leftClear = 1.0;
          let rightClear = 1.0;
          if (sensors.lidar && typeof sensors.lidar.getLeft === "function") {
            leftClear = sensors.lidar.getLeft(30);
            rightClear = sensors.lidar.getRight(30);
          }
          this.recoveryDirection = leftClear >= rightClear ? 1 : -1;
          robot.log(`[RECOVERY] Anti-stall triggered! Strafing ${this.recoveryDirection > 0 ? "LEFT" : "RIGHT"}`);
        }
      }

      // Execute Recovery Maneuver if Active
      if (this.recoveryStartTime !== null) {
        if (t - this.recoveryStartTime <= this.cfg.RECOVERY_DURATION) {
          const recVy = 0.28 * this.recoveryDirection;
          robot.setVelocity(-0.10, recVy, 0.0);
          this.lastCmd = { vx: -0.10, vy: recVy, omega: 0 };
          return;
        } else {
          // Recovery complete: reset watchdog and force global A* replan
          this.recoveryStartTime = null;
          this.stallStartTime = null;
          this.lastPlanTime = -Infinity;
          this.path = [];
        }
      }

      // 7. Morphology-Aware Costmap Inflation
      // Use conservative footprint while morphing to guarantee zero collisions
      const planningShape = isMorphing ? "O" : currentShape;
      this.buildCostmap(planningShape);

      // 8. Global A* Planner & Path Simplification (~3 Hz or on Event)
      const needsAstar =
        this.path.length === 0 ||
        targetChanged ||
        t - this.lastPlanTime >= this.cfg.ASTAR_REPLAN_INTERVAL ||
        this.distanceToPath(pose, this.path) > 0.40;

      if (needsAstar) {
        this.replanGlobalPath(pose, target, planningShape);
        this.lastPlanTime = t;
      }

      // 9. Holonomic Dynamic Window Approach (DWA) Execution (~20 Hz)
      const dwaCommand = this.computeDWA(pose, target, this.path, planningShape, isMorphing);

      // 10. Output Velocity Commands to Robot
      robot.setVelocity(dwaCommand.vx, dwaCommand.vy, dwaCommand.omega);
      this.lastCmd = dwaCommand;

      // Update global debug state for visualizer
      if (typeof window !== "undefined") {
        window.__plannerDebugState = {
          path: this.simplifiedPath,
          rawPath: this.rawPath,
          target: target,
          lookahead: this.lastLookaheadPoint,
          cmd: dwaCommand,
          shape: currentShape,
          speed: this.actualSpeed,
          state: isArrived
            ? "ARRIVED"
            : this.recoveryStartTime !== null
            ? "RECOVERY"
            : isMorphing
            ? "MORPHING"
            : "NAVIGATING",
          recovery: this.recoveryStartTime !== null,
          clearance: this.lastClearance || 0,
          corridorWidth: this.lastCorridorWidth,
        };
      }
    }

    // ============================================================
    // LIDAR RAYCASTING & ONLINE MAPPING
    // ============================================================
    updateLiDARMap(sensors, pose) {
      if (!sensors.lidar) return;
      const ranges = sensors.lidar.ranges || sensors.lidar;
      if (!ranges || typeof ranges.length !== "number") return;

      const numBeams = ranges.length;
      const theta = pose.theta;

      for (let i = 0; i < numBeams; i++) {
        let r = ranges[i];
        if (!Number.isFinite(r)) continue;

        r = clamp(r, this.cfg.LIDAR_MIN_RANGE, this.cfg.LIDAR_MAX_RANGE);
        const beamAngle = theta + (i * Math.PI) / 180.0;
        const isObstacleHit = r < this.cfg.LIDAR_MAX_RANGE - 0.05;
        const endDist = isObstacleHit ? r : this.cfg.LIDAR_MAX_RANGE;

        const endX = pose.x + Math.cos(beamAngle) * endDist;
        const endY = pose.y + Math.sin(beamAngle) * endDist;

        const ray = traceRayCells(this.grid, pose.x, pose.y, endX, endY);
        const freeCount = isObstacleHit ? ray.length - 1 : ray.length;

        for (let j = 0; j < freeCount; j++) {
          this.grid.markFree(ray[j].x, ray[j].y);
        }

        if (isObstacleHit && ray.length > 0) {
          const endpoint = ray[ray.length - 1];
          this.grid.markOccupied(endpoint.x, endpoint.y);
        }
      }
    }

    // ============================================================
    // MORPHOLOGY-AWARE COSTMAP INFLATION
    // ============================================================
    buildCostmap(shape) {
      this.grid.cost.fill(0);

      // Footprint radius calculation
      let radius;
      if (shape === "I") {
        // Streamlined corridor profile: width 16cm -> half-width 8cm
        radius = Math.hypot(this.cfg.I_HALF_WIDTH, this.cfg.I_HALF_WIDTH) + this.cfg.SAFETY_MARGIN;
      } else {
        // Standard compact 2x2 footprint: 32cm x 32cm
        radius = Math.hypot(this.cfg.O_HALF_LENGTH, this.cfg.O_HALF_WIDTH) + this.cfg.SAFETY_MARGIN;
      }

      const lethalRadius = radius * 0.85;
      const inflationRadius = radius + 0.08;
      const cellSpan = Math.ceil(inflationRadius / this.grid.res);

      const w = this.grid.width;
      const h = this.grid.height;

      for (let gy = 0; gy < h; gy++) {
        for (let gx = 0; gx < w; gx++) {
          // Perimeter boundaries are always lethal
          if (gx <= 1 || gx >= w - 2 || gy <= 1 || gy >= h - 2) {
            this.grid.setCost(gx, gy, Infinity);
            continue;
          }

          if (this.grid.get(gx, gy) === this.cfg.OCCUPIED) {
            this.grid.setCost(gx, gy, Infinity);

            // Inflate into neighborhood
            for (let dy = -cellSpan; dy <= cellSpan; dy++) {
              for (let dx = -cellSpan; dx <= cellSpan; dx++) {
                const nx = gx + dx;
                const ny = gy + dy;
                if (!this.grid.validCell(nx, ny)) continue;

                const dist = Math.hypot(dx, dy) * this.grid.res;
                if (dist > inflationRadius) continue;

                if (dist <= lethalRadius) {
                  this.grid.setCost(nx, ny, Infinity);
                } else {
                  const gradient = 1.0 - (dist - lethalRadius) / (inflationRadius - lethalRadius);
                  const currentCost = this.grid.getCost(nx, ny);
                  if (currentCost !== Infinity && gradient > currentCost) {
                    this.grid.setCost(nx, ny, gradient);
                  }
                }
              }
            }
          }
        }
      }
    }

    // ============================================================
    // A* GLOBAL PLANNER (8-CONNECTED WITH CORNER-CUTTING GUARD)
    // ============================================================
    searchAStar(startGrid, goalGrid) {
      const w = this.grid.width;
      const h = this.grid.height;
      const totalCells = w * h;

      const openSet = new MinHeap();
      const closedSet = new Uint8Array(totalCells);
      const gScores = new Float32Array(totalCells);
      const parents = new Int32Array(totalCells);

      gScores.fill(Infinity);
      parents.fill(-1);

      const startIndex = this.grid.index(startGrid.x, startGrid.y);
      const goalIndex = this.grid.index(goalGrid.x, goalGrid.y);

      gScores[startIndex] = 0;
      const startH = this.octileDistance(startGrid.x, startGrid.y, goalGrid.x, goalGrid.y);
      openSet.push({ x: startGrid.x, y: startGrid.y, idx: startIndex, f: startH });

      // 8-Connected Neighbors [dx, dy, cost]
      const neighbors = [
        [-1, 0, 1.0],
        [1, 0, 1.0],
        [0, -1, 1.0],
        [0, 1, 1.0],
        [-1, -1, 1.4142],
        [-1, 1, 1.4142],
        [1, -1, 1.4142],
        [1, 1, 1.4142],
      ];

      let iterations = 0;
      let pathFound = false;

      while (openSet.length > 0 && iterations < 4500) {
        iterations++;
        const curr = openSet.pop();

        if (curr.idx === goalIndex) {
          pathFound = true;
          break;
        }

        if (closedSet[curr.idx]) continue;
        closedSet[curr.idx] = 1;

        for (let i = 0; i < 8; i++) {
          const dx = neighbors[i][0];
          const dy = neighbors[i][1];
          const moveCost = neighbors[i][2];

          const nx = curr.x + dx;
          const ny = curr.y + dy;

          if (!this.grid.validCell(nx, ny)) continue;
          if (!this.isCellTraversable(nx, ny)) continue;

          // CORNER-CUTTING GUARD: Prevent diagonal traversal between two blocked cells
          if (dx !== 0 && dy !== 0) {
            if (!this.isCellTraversable(curr.x + dx, curr.y) || !this.isCellTraversable(curr.x, curr.y + dy)) {
              continue;
            }
          }

          const nIdx = this.grid.index(nx, ny);
          if (closedSet[nIdx]) continue;

          // Soft cost addition
          let cellPenalty = 0;
          const state = this.grid.get(nx, ny);
          if (state === this.cfg.UNKNOWN) {
            cellPenalty += this.cfg.UNKNOWN_PENALTY;
          }
          const inflCost = this.grid.getCost(nx, ny);
          if (Number.isFinite(inflCost)) {
            cellPenalty += inflCost * 2.2;
          }

          const tentativeG = gScores[curr.idx] + moveCost + cellPenalty;

          if (tentativeG < gScores[nIdx]) {
            gScores[nIdx] = tentativeG;
            parents[nIdx] = curr.idx;
            const h = this.octileDistance(nx, ny, goalGrid.x, goalGrid.y);
            openSet.push({ x: nx, y: ny, idx: nIdx, f: tentativeG + h });
          }
        }
      }

      if (!pathFound) return null;

      // Reconstruct path from parents
      const rawPath = [];
      let currIdx = goalIndex;
      while (currIdx !== -1) {
        const gx = currIdx % w;
        const gy = Math.floor(currIdx / w);
        rawPath.push(this.grid.gridToWorld(gx, gy));
        currIdx = parents[currIdx];
      }
      rawPath.reverse();
      return rawPath;
    }

    replanGlobalPath(pose, target, planningShape = "O") {
      const startGrid = this.grid.worldToGrid(pose.x, pose.y);
      let goalGrid = this.grid.worldToGrid(target.x, target.y);

      if (!this.grid.validCell(startGrid.x, startGrid.y) || !this.grid.validCell(goalGrid.x, goalGrid.y)) {
        return;
      }

      let actualStart = startGrid;
      if (!this.isCellTraversable(startGrid.x, startGrid.y)) {
        const altStart = this.findNearestTraversableCell(startGrid.x, startGrid.y, 6);
        if (altStart) actualStart = altStart;
      }

      let actualGoal = goalGrid;
      if (!this.isCellTraversable(goalGrid.x, goalGrid.y)) {
        const altGoal = this.findNearestTraversableCell(goalGrid.x, goalGrid.y, 8);
        if (altGoal) {
          actualGoal = altGoal;
        } else {
          return;
        }
      }

      let rawPath = this.searchAStar(actualStart, actualGoal);

      // If standard conservative planning fails, check if streamlined "I" unlocks a narrow passage
      if (!rawPath && planningShape !== "I") {
        this.buildCostmap("I");
        rawPath = this.searchAStar(actualStart, actualGoal);
        if (rawPath) {
          this.corridorRequiresI = true;
        } else {
          // Restore planning shape costmap
          this.buildCostmap(planningShape);
        }
      }

      if (rawPath && rawPath.length > 0) {
        this.rawPath = rawPath;
        this.simplifiedPath = this.simplifyPath(rawPath);
        this.path = this.simplifiedPath;
        this.pathIndex = 0;
      }
    }

    isCellTraversable(gx, gy) {
      if (!this.grid.validCell(gx, gy)) return false;
      if (this.grid.get(gx, gy) === this.cfg.OCCUPIED) return false;
      const c = this.grid.getCost(gx, gy);
      return Number.isFinite(c);
    }

    findNearestTraversableCell(gx, gy, searchRadius) {
      let best = null;
      let bestDist = Infinity;
      for (let dy = -searchRadius; dy <= searchRadius; dy++) {
        for (let dx = -searchRadius; dx <= searchRadius; dx++) {
          const nx = gx + dx;
          const ny = gy + dy;
          if (this.isCellTraversable(nx, ny)) {
            const d = dx * dx + dy * dy;
            if (d < bestDist) {
              bestDist = d;
              best = { x: nx, y: ny };
            }
          }
        }
      }
      return best;
    }

    octileDistance(x1, y1, x2, y2) {
      const dx = Math.abs(x1 - x2);
      const dy = Math.abs(y1 - y2);
      return Math.max(dx, dy) + 0.41421356 * Math.min(dx, dy);
    }

    // ============================================================
    // PATH SIMPLIFICATION (STRING PULLING)
    // ============================================================
    simplifyPath(path) {
      if (path.length <= 2) return path;

      const simplified = [path[0]];
      let anchor = 0;

      while (anchor < path.length - 1) {
        let furthest = anchor + 1;
        for (let i = anchor + 2; i < path.length; i++) {
          if (this.hasLineOfSight(path[anchor], path[i])) {
            furthest = i;
          } else {
            break;
          }
        }
        simplified.push(path[furthest]);
        anchor = furthest;
      }
      return simplified;
    }

    hasLineOfSight(p1, p2) {
      const ray = traceRayCells(this.grid, p1.x, p1.y, p2.x, p2.y);
      for (let i = 0; i < ray.length; i++) {
        if (!this.isCellTraversable(ray[i].x, ray[i].y)) {
          return false;
        }
      }
      return true;
    }

    // ============================================================
    // LOOKAHEAD & PATH ADHERENCE
    // ============================================================
    getLookaheadPoint(path, pose) {
      if (!path || path.length === 0) return null;

      let nearestIdx = 0;
      let minDistance = Infinity;

      for (let i = 0; i < path.length; i++) {
        const d = Math.hypot(path[i].x - pose.x, path[i].y - pose.y);
        if (d < minDistance) {
          minDistance = d;
          nearestIdx = i;
        }
      }

      // Walk forward along path until lookahead distance is satisfied
      for (let i = nearestIdx; i < path.length; i++) {
        const d = Math.hypot(path[i].x - pose.x, path[i].y - pose.y);
        if (d >= this.cfg.PATH_LOOKAHEAD) {
          return path[i];
        }
      }
      return path[path.length - 1];
    }

    distanceToPath(point, path) {
      if (!path || path.length === 0) return Infinity;
      let minD = Infinity;
      for (let i = 0; i < path.length - 1; i++) {
        const a = path[i];
        const b = path[i + 1];
        const abx = b.x - a.x;
        const aby = b.y - a.y;
        const apx = point.x - a.x;
        const apy = point.y - a.y;
        const abLenSq = abx * abx + aby * aby;
        let t = abLenSq > 0 ? (apx * abx + apy * aby) / abLenSq : 0;
        t = clamp(t, 0, 1);
        const projX = a.x + abx * t;
        const projY = a.y + aby * t;
        const d = Math.hypot(point.x - projX, point.y - projY);
        if (d < minD) minD = d;
      }
      return minD;
    }

    // ============================================================
    // HOLONOMIC DYNAMIC WINDOW APPROACH (DWA)
    // ============================================================
    computeDWA(pose, target, path, shape, isMorphing) {
      const lookahead = this.getLookaheadPoint(path, pose) || target;
      const distToGoal = Math.hypot(target.x - pose.x, target.y - pose.y);

      // 1. Current Robot Body Velocities (from last command or sensors)
      const currVx = this.lastCmd.vx;
      const currVy = this.lastCmd.vy;
      const currOmega = this.lastCmd.omega;

      // 2. Dynamic Window based on actual physical acceleration limits
      const maxLin = this.cfg.MAX_LINEAR_SPEED;
      const maxAng = this.cfg.MAX_ANGULAR_SPEED;
      const dtDwa = this.cfg.DWA_DT;

      const linDelta = this.cfg.LINEAR_ACCEL * dtDwa;
      const angDelta = this.cfg.ANGULAR_ACCEL * dtDwa;

      const minVx = clamp(currVx - linDelta, -maxLin, maxLin);
      const maxVx = clamp(currVx + linDelta, -maxLin, maxLin);
      const minVy = clamp(currVy - linDelta, -maxLin, maxLin);
      const maxVy = clamp(currVy + linDelta, -maxLin, maxLin);
      const minW = clamp(currOmega - angDelta, -maxAng, maxAng);
      const maxW = clamp(currOmega + angDelta, -maxAng, maxAng);

      // 3. Adaptive Speed Profile
      let desiredSpeed = maxLin;
      if (distToGoal < this.cfg.FINAL_DECEL_DIST) {
        desiredSpeed *= clamp(distToGoal / this.cfg.FINAL_DECEL_DIST, 0.20, 1.0);
      }
      if (isMorphing) {
        desiredSpeed = Math.min(desiredSpeed, 0.20);
      } else if (shape === "I") {
        desiredSpeed = Math.min(desiredSpeed, 0.36);
      }

      // Sample Candidates
      const vxList = this.sampleLinRange(minVx, maxVx, this.cfg.VX_SAMPLES);
      const vyList = this.sampleLinRange(minVy, maxVy, this.cfg.VY_SAMPLES);
      const wList = this.sampleLinRange(minW, maxW, this.cfg.OMEGA_SAMPLES);

      let bestScore = -Infinity;
      let bestCmd = null;

      for (let i = 0; i < vxList.length; i++) {
        const vx = vxList[i];
        for (let j = 0; j < vyList.length; j++) {
          const vy = vyList[j];
          const speed = Math.hypot(vx, vy);
          if (speed > maxLin) continue;

          for (let k = 0; k < wList.length; k++) {
            const omega = wList[k];

            // Trajectory Simulation & Kinematic Integration
            const trajResult = this.predictTrajectory(pose, vx, vy, omega, shape);
            if (!trajResult.valid) continue; // HARD CONSTRAINT REJECTION

            // Braking Feasibility Guard
            const brakeDist = (speed * speed) / (2 * this.cfg.LINEAR_ACCEL) + 0.03;
            if (trajResult.minClearance < brakeDist) continue;

            // Score Candidate
            const score = this.scoreTrajectory(
              trajResult,
              pose,
              target,
              lookahead,
              desiredSpeed,
              vx,
              vy,
              omega
            );

            if (score > bestScore) {
              bestScore = score;
              bestCmd = { vx, vy, omega };
              this.lastClearance = trajResult.minClearance;
            }
          }
        }
      }

      this.lastLookaheadPoint = lookahead;

      // Safe Fallback if all sampled trajectories hit obstacles
      if (!bestCmd) {
        this.lastClearance = 0.15;
        return this.computeFallbackCmd(pose, lookahead);
      }
      return bestCmd;
    }

    sampleLinRange(minVal, maxVal, count) {
      if (count <= 1) return [(minVal + maxVal) / 2];
      const res = [];
      const step = (maxVal - minVal) / (count - 1);
      for (let i = 0; i < count; i++) {
        res.push(minVal + step * i);
      }
      return res;
    }

    // ============================================================
    // TRAJECTORY PREDICTION (Ramping Dynamics matching smorphi.js)
    // ============================================================
    predictTrajectory(pose, cmdVx, cmdVy, cmdOmega, shape) {
      const steps = Math.round(this.cfg.DWA_HORIZON / this.cfg.DWA_DT);
      let x = pose.x;
      let y = pose.y;
      let theta = pose.theta;

      let vx = this.lastCmd.vx;
      let vy = this.lastCmd.vy;
      let omega = this.lastCmd.omega;

      const maxLinDelta = this.cfg.LINEAR_ACCEL * this.cfg.DWA_DT;
      const maxAngDelta = this.cfg.ANGULAR_ACCEL * this.cfg.DWA_DT;

      let minClearance = Infinity;
      const finalStates = [];

      for (let s = 0; s < steps; s++) {
        // Accelerate toward target commands (smorphi.js physics ramp)
        vx += clamp(cmdVx - vx, -maxLinDelta, maxLinDelta);
        vy += clamp(cmdVy - vy, -maxLinDelta, maxLinDelta);
        omega += clamp(cmdOmega - omega, -maxAngDelta, maxAngDelta);

        // Body velocities -> Global velocities
        const cosT = Math.cos(theta);
        const sinT = Math.sin(theta);
        const globalVx = vx * cosT - vy * sinT;
        const globalVy = vx * sinT + vy * cosT;

        // Position integration
        x += globalVx * this.cfg.DWA_DT;
        y += globalVy * this.cfg.DWA_DT;
        theta = normalizeAngle(theta + omega * this.cfg.DWA_DT);

        // Hard Boundary Collision Check
        if (x < 0.18 || x > 4.82 || y < 0.18 || y > 4.82) {
          return { valid: false, minClearance: 0 };
        }

        // Hard Footprint Collision Check
        if (this.checkFootprintCollision(x, y, theta, shape)) {
          return { valid: false, minClearance: 0 };
        }

        // Clearance Estimation
        const cell = this.grid.worldToGrid(x, y);
        const cost = this.grid.getCost(cell.x, cell.y);
        const clearance = Number.isFinite(cost) ? Math.max(0.01, 0.40 * (1.0 - cost)) : 0;
        if (clearance < minClearance) minClearance = clearance;

        finalStates.push({ x, y, theta, vx, vy, omega });
      }

      return {
        valid: true,
        minClearance: minClearance,
        finalState: finalStates[finalStates.length - 1],
      };
    }

    checkFootprintCollision(x, y, theta, shape) {
      const halfL = shape === "I" ? this.cfg.I_HALF_LENGTH : this.cfg.O_HALF_LENGTH;
      const halfW = shape === "I" ? this.cfg.I_HALF_WIDTH : this.cfg.O_HALF_WIDTH;

      const cosT = Math.cos(theta);
      const sinT = Math.sin(theta);

      // Sample key boundary and corner points of oriented bounding box
      const sampleOffsets = [
        { lx: -halfL, ly: -halfW },
        { lx: halfL, ly: -halfW },
        { lx: halfL, ly: halfW },
        { lx: -halfL, ly: halfW },
        { lx: 0, ly: 0 },
        { lx: halfL, ly: 0 },
        { lx: -halfL, ly: 0 },
        { lx: 0, ly: halfW },
        { lx: 0, ly: -halfW },
      ];

      for (let i = 0; i < sampleOffsets.length; i++) {
        const off = sampleOffsets[i];
        const px = x + off.lx * cosT - off.ly * sinT;
        const py = y + off.lx * sinT + off.ly * cosT;
        const cell = this.grid.worldToGrid(px, py);

        if (this.grid.get(cell.x, cell.y) === this.cfg.OCCUPIED) {
          return true; // HARD COLLISION
        }
      }
      return false;
    }

    // ============================================================
    // MULTI-OBJECTIVE DWA SCORING
    // ============================================================
    scoreTrajectory(traj, pose, target, lookahead, desiredSpeed, cmdVx, cmdVy, cmdOmega) {
      const endState = traj.finalState;

      // 1. Goal Progress
      const curDist = Math.hypot(target.x - pose.x, target.y - pose.y);
      const endDist = Math.hypot(target.x - endState.x, target.y - endState.y);
      const progress = curDist - endDist;

      // 2. Path Adherence (Distance to lookahead vector)
      const lookDist = Math.hypot(lookahead.x - endState.x, lookahead.y - endState.y);
      const pathScore = Math.exp(-lookDist / 0.35);

      // 3. Obstacle Clearance
      const clearanceScore = clamp(traj.minClearance / 0.35, 0, 1);

      // 4. Speed Tracking
      const candidateSpeed = Math.hypot(cmdVx, cmdVy);
      const speedScore = 1.0 - Math.abs(desiredSpeed - candidateSpeed) / Math.max(desiredSpeed, 0.1);

      // 5. Holonomic Direction Alignment
      // Evaluates translation vector alignment toward lookahead WITHOUT forcing rotation
      const toLookX = lookahead.x - pose.x;
      const toLookY = lookahead.y - pose.y;
      const lookMag = Math.hypot(toLookX, toLookY);

      let dirScore = 0;
      if (candidateSpeed > 0.05 && lookMag > 0.05) {
        const cosT = Math.cos(pose.theta);
        const sinT = Math.sin(pose.theta);
        const gVx = cmdVx * cosT - cmdVy * sinT;
        const gVy = cmdVx * sinT + cmdVy * cosT;
        dirScore = (gVx * toLookX + gVy * toLookY) / (candidateSpeed * lookMag);
      }

      // 6. Smoothness & Anti-Oscillation
      const cmdDiff = Math.hypot(cmdVx - this.lastCmd.vx, cmdVy - this.lastCmd.vy);
      const rotPenalty = Math.abs(cmdOmega);

      return (
        this.cfg.SCORE_PROGRESS * progress +
        this.cfg.SCORE_PATH * pathScore +
        this.cfg.SCORE_CLEARANCE * clearanceScore +
        this.cfg.SCORE_SPEED * speedScore +
        this.cfg.SCORE_DIRECTION * dirScore -
        this.cfg.SCORE_ROTATION * rotPenalty -
        this.cfg.SCORE_CMD_SMOOTH * cmdDiff
      );
    }

    computeFallbackCmd(pose, lookahead) {
      // Direct conservative proportional vector
      const dx = lookahead.x - pose.x;
      const dy = lookahead.y - pose.y;
      const cosT = Math.cos(pose.theta);
      const sinT = Math.sin(pose.theta);

      let localVx = dx * cosT + dy * sinT;
      let localVy = -dx * sinT + dy * cosT;
      const mag = Math.hypot(localVx, localVy);

      if (mag > 0.01) {
        const scale = Math.min(0.20 / mag, 1.0);
        localVx *= scale;
        localVy *= scale;
      }
      return { vx: localVx, vy: localVy, omega: 0.0 };
    }

    // ============================================================
    // MORPHOLOGY MANAGER (HYSTERESIS-PROTECTED)
    // ============================================================
    updateMorphology(sensors, robot, currentShape, isMorphing) {
      if (!sensors.lidar) return;

      let leftDist = Infinity;
      let rightDist = Infinity;

      if (typeof sensors.lidar.getLeft === "function") {
        leftDist = sensors.lidar.getLeft(25);
        rightDist = sensors.lidar.getRight(25);
      }

      if (!Number.isFinite(leftDist) || !Number.isFinite(rightDist)) return;

      // Estimated passage clearance between walls
      const corridorWidth = leftDist + rightDist + (currentShape === "I" ? 0.16 : 0.32);
      this.lastCorridorWidth = corridorWidth;

      let targetShape = currentShape;

      // Check if global route requires streamlined "I" or corridor is detected locally
      if (currentShape !== "I" && (this.corridorRequiresI || corridorWidth < this.cfg.MORPH_TRIGGER_WIDTH)) {
        targetShape = "I";
      }
      // Exit narrow corridor -> Restore stable "O"
      else if (currentShape === "I" && !this.corridorRequiresI && leftDist > this.cfg.MORPH_EXIT_WIDTH && rightDist > this.cfg.MORPH_EXIT_WIDTH) {
        targetShape = "O";
      }

      // Guarded Shape Request: never repeat command while active morph is interpolating
      if (!isMorphing && targetShape !== currentShape && targetShape !== this.lastShapeCommand) {
        if (typeof robot.setShape === "function") {
          robot.setShape(targetShape);
          this.lastShapeCommand = targetShape;
          robot.log(`[MORPHOLOGY] Corridor width: ${corridorWidth.toFixed(2)}m -> Morphing to '${targetShape}' shape.`);
        }
      }
    }
  }

  // Expose to browser window and CommonJS / Node.js
  if (typeof window !== "undefined") {
    window.FastAStarDwaNavigator = FastAStarDwaNavigator;
  }
  if (typeof module !== "undefined" && module.exports) {
    module.exports = FastAStarDwaNavigator;
  }
})(typeof window !== "undefined" ? window : global);
