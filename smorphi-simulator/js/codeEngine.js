/**
 * Interactive Code Injection & Script Execution Engine
 * Compiles and executes user JavaScript navigation scripts in real-time,
 * provides sandboxed error isolation, persistent memory across ticks, and preset algorithms.
 * 
 * Features SmorphiRoboRoarZ Tangential Artificial Potential Field (APF):
 * - TwistVelocity ROS 2 Standard: { vx (forward), vy (lateral), wz (angular) }
 * - Target Waypoints: (0, 2) -> (2, 3) -> (-2, 4)
 * - 5-Second Mandatory Arrival Dwell Timer (Non-blocking)
 * - Tangential LiDAR Repulsion (Rotates 90° to slide around obstacles, breaking local minima)
 * - Anti-Stall Watchdog (Injects orthogonal strafe pulse if trapped)
 * - IMU Heading-Locked Diagonal Holonomic Translation (wz = 0.0)
 */

/**
 * SmorphiRoboRoarZ
 * Direct JavaScript translation of the enhanced C++ SmorphiRoboRoarZ class.
 * Features normalized goal attraction, radial + tangential LiDAR obstacle repulsion,
 * and an anti-stall watchdog.
 */
class SmorphiRoboRoarZ {
  constructor(options = {}) {
    // Target Waypoints: (0,2), (2,3), (-2,4)
    this.waypoints = options.waypoints || [
      { x: 0.0, y: 2.0 },
      { x: 2.0, y: 3.0 },
      { x: -2.0, y: 4.0 },
    ];
    this.currentWpIndex = 0;

    // Interval Tracking
    this.isWaiting = false;
    this.arrivalTime = null;
    this.dwellTime = options.dwellTime !== undefined ? options.dwellTime : 5; // 5 seconds mandatory stop

    // Navigation Gains
    this.goalTolerance = options.goalTolerance !== undefined ? options.goalTolerance : 0.08; // 8 cm threshold
    this.maxSpeed = options.maxSpeed !== undefined ? options.maxSpeed : 1.2; // 1.2 m/s
    this.kpAttract = options.kpAttract !== undefined ? options.kpAttract : 0.9;
    this.kpRepulse = options.kpRepulse !== undefined ? options.kpRepulse : 1.8; // Radial pushback gain
    this.kpTangent = options.kpTangent !== undefined ? options.kpTangent : 1.4; // Lateral circulating gain around walls
    this.dangerZone = options.dangerZone !== undefined ? options.dangerZone : 0.55; // 0.55m LiDAR threshold

    // Anti-Stall State
    this.stallCounter = 0;

    // State flags & Callbacks
    this.isMissionComplete = false;
    this.lastLogTime = 0;
    this.publishCallback = options.publishCallback || null;
    this.logCallback = options.logCallback || null;
  }

  /**
   * Main Control Loop - Executes at 10-60 Hz
   * @param {number} robotX - Global X position from EKF/Odometry
   * @param {number} robotY - Global Y position from EKF/Odometry
   * @param {number} imuYaw - Heading in radians (-PI to PI) from IMU
   * @param {Array|Float32Array|Object} lidarRanges - 360-degree array of distances
   * @param {Object} [robotAPI] - Optional proxy { setVelocity, stop, log }
   * @returns {Object} TwistVelocity { vx, vy, wz }
   */
  computeAutonomousMovement(robotX, robotY, imuYaw, lidarRanges, robotAPI = null) {
    if (this.currentWpIndex >= this.waypoints.length) {
      if (robotAPI && typeof robotAPI.setVelocity === "function") {
        robotAPI.setVelocity(0.0, 0.0, 0.0);
      }
      if (!this.isMissionComplete) {
        this.isMissionComplete = true;
        this.log("All waypoints reached. Mission complete.", robotAPI);
      }
      return { vx: 0.0, vy: 0.0, wz: 0.0 };
    }

    const target = this.waypoints[this.currentWpIndex];
    const globalDx = target.x - robotX;
    const globalDy = target.y - robotY;
    const distanceToTarget = Math.hypot(globalDx, globalDy);

    // 1. Waypoint Dwell Interval Logic
    if (distanceToTarget <= this.goalTolerance) {
      const now = typeof performance !== "undefined" ? performance.now() : Date.now();
      if (!this.isWaiting) {
        this.isWaiting = true;
        this.arrivalTime = now;
        this.log(`Target ${this.currentWpIndex + 1} (${target.x}, ${target.y}) Reached. Waiting 5s...`, robotAPI);
      } else {
        const elapsed = (now - this.arrivalTime) / 1000;
        if (elapsed >= this.dwellTime) {
          this.isWaiting = false;
          this.currentWpIndex++;
          this.log("Timer complete. Proceeding to next target.", robotAPI);
        }
      }

      if (robotAPI && typeof robotAPI.setVelocity === "function") {
        robotAPI.setVelocity(0.0, 0.0, 0.0);
      }
      return { vx: 0.0, vy: 0.0, wz: 0.0 };
    }

    // 2. Goal Attraction Vector (Robot Local Frame)
    const localTargetX =  globalDx * Math.cos(imuYaw) + globalDy * Math.sin(imuYaw);
    const localTargetY = -globalDx * Math.sin(imuYaw) + globalDy * Math.cos(imuYaw);

    const targetNorm = Math.hypot(localTargetX, localTargetY);
    let attractVx = 0.0;
    let attractVy = 0.0;
    if (targetNorm > 1e-6) {
      attractVx = (localTargetX / targetNorm) * this.kpAttract;
      attractVy = (localTargetY / targetNorm) * this.kpAttract;
    }

    // 3. Tangential LiDAR Repulsion (Breaks Local Minima)
    let repulseVx = 0.0;
    let repulseVy = 0.0;
    let obstacleNear = false;

    const ranges = (lidarRanges && lidarRanges.ranges) ? lidarRanges.ranges : lidarRanges;
    const count = (ranges && ranges.length) || 360;

    for (let i = 0; i < count; i++) {
      const r = ranges[i];

      if (r > 0.02 && r < this.dangerZone) {
        obstacleNear = true;
        const angleRad = (i * Math.PI) / 180.0;

        // Radial force magnitude (pushes straight away from wall)
        const fRadial = Math.pow((this.dangerZone - r) / this.dangerZone, 2) * this.kpRepulse;
        const radX = -Math.cos(angleRad) * fRadial;
        const radY = -Math.sin(angleRad) * fRadial;

        // Tangential force magnitude (rotates 90 deg to slide along wall)
        const fTan = fRadial * this.kpTangent;
        let tanX = -Math.sin(angleRad) * fTan;
        let tanY =  Math.cos(angleRad) * fTan;

        // Align tangential direction toward the target
        const dot = (tanX * attractVx) + (tanY * attractVy);
        if (dot < 0) {
          tanX = -tanX;
          tanY = -tanY;
        }

        repulseVx += (radX + tanX);
        repulseVy += (radY + tanY);
      }
    }

    // 4. Force Synthesis
    let finalVx = attractVx + repulseVx;
    let finalVy = attractVy + repulseVy;

    // 5. Anti-Stall Watchdog
    // If speed drops near zero in front of an obstacle, inject an orthogonal strafe pulse
    let commandedSpeed = Math.hypot(finalVx, finalVy);
    if (obstacleNear && commandedSpeed < 0.15) {
      this.stallCounter++;
      if (this.stallCounter > 5) { // Stuck for > 5 ticks
        finalVx = 0.0;
        finalVy = (localTargetY >= 0 ? 0.6 : -0.6); // Force immediate lateral crab-walk
        this.log(`[ANTI-STALL] Stuck threshold reached. Pulsing lateral strafe (vy=${finalVy}).`, robotAPI, true);
      }
    } else {
      this.stallCounter = 0;
    }

    // 6. Velocity Clamping
    commandedSpeed = Math.hypot(finalVx, finalVy);
    if (commandedSpeed > this.maxSpeed) {
      finalVx = (finalVx / commandedSpeed) * this.maxSpeed;
      finalVy = (finalVy / commandedSpeed) * this.maxSpeed;
    }

    const cmd = { vx: finalVx, vy: finalVy, wz: 0.0 };

    if (robotAPI && typeof robotAPI.setVelocity === "function") {
      robotAPI.setVelocity(cmd.vx, cmd.vy, cmd.wz);
    } else if (this.publishCallback) {
      this.publishCallback(cmd.vx, cmd.vy, cmd.wz);
    }

    return cmd;
  }

  /**
   * Compatibility wrapper for earlier executeCycle calls
   */
  executeCycle(imuYaw, lidarFrontClearance, ekfPosX, ekfPosY, robotAPI = null) {
    const fakeLidar = new Float32Array(360);
    fakeLidar.fill(5.0);
    fakeLidar[0] = lidarFrontClearance;
    return this.computeAutonomousMovement(ekfPosX, ekfPosY, imuYaw, fakeLidar, robotAPI);
  }

  stopMotors(robotAPI = null) {
    if (robotAPI && typeof robotAPI.stop === "function") {
      robotAPI.stop();
    } else if (robotAPI && typeof robotAPI.setVelocity === "function") {
      robotAPI.setVelocity(0.0, 0.0, 0.0);
    }
  }

  reset() {
    this.currentWpIndex = 0;
    this.isWaiting = false;
    this.arrivalTime = null;
    this.stallCounter = 0;
    this.isMissionComplete = false;
  }

  log(message, robotAPI = null, throttled = false) {
    const now = typeof performance !== "undefined" ? performance.now() : Date.now();
    if (throttled && now - this.lastLogTime < 500) return;
    if (throttled) this.lastLogTime = now;

    if (robotAPI && typeof robotAPI.log === "function") {
      robotAPI.log(message);
    } else if (this.logCallback) {
      this.logCallback(message);
    } else {
      console.log(message);
    }
  }
}

// Backward-compatibility alias
const SmorphiAutonomousController = SmorphiRoboRoarZ;

class CodeEngine {
  constructor() {
    this.compiledFunction = null;
    this.memory = {}; // Preserved state between ticks
    this.isRunning = false;
    this.hasError = false;
    this.lastErrorMessage = null;
    this.logCallbacks = [];
    this.lastLogTime = 0;

    // Integrated autonomous controller instance
    this.autonomousController = new SmorphiRoboRoarZ();

    // Default Preset Scripts
    this.presets = {
      // 1. Translated Autonomous Navigation with Tangential APF & Anti-Stall (Default)
      default_avoidance: `/**
 * SMORPHI ROBOROARZ AUTONOMOUS TANGENTIAL APF & 3-WAYPOINT CONTROLLER
 * --------------------------------------------------------------------
 * Ported directly from C++ SmorphiRoboRoarZ
 *
 * Algorithm Features:
 *   - Target Waypoints: (0, 2) -> (2, 3) -> (-2, 4)
 *   - 5-Second Mandatory Arrival Dwell Timer
 *   - Normalized Goal Attraction Vector (Robot Local Frame)
 *   - Tangential LiDAR Repulsion (Rotates 90° to slide around obstacles, breaking local minima)
 *   - Anti-Stall Watchdog: Orthogonal strafe pulse if trapped in front of an obstacle
 *   - Holonomic Velocity Clamping: Max 1.2 m/s with locked heading (wz = 0.0)
 *
 * Inputs per tick:
 *   - sensors.pose:  { x, y, theta } (EKF state)
 *   - sensors.imu:   { headingRad }
 *   - sensors.lidar: 360-deg laser array (ranges)
 * Outputs:
 *   - robot.setVelocity(vx, vy, wz): Forward (vx), Lateral Crab (vy), Rotation (wz)
 */

// 1. Initialization Structure: Load waypoints and control parameters
if (!memory.initialized) {
  memory.waypoints = [
    { x: 0.0, y: 2.0 },
    { x: 2.0, y: 3.0 },
    { x: -2.0, y: 4.0 },
  ];
  memory.currentWpIndex = 0;
  memory.isWaiting = false;
  memory.arrivalTime = null;
  memory.DWELL_TIME = 5;          // 5 seconds mandatory stop
  memory.GOAL_TOLERANCE = 0.08;   // 8 cm arrival threshold
  memory.MAX_SPEED = 1.2;         // 1.2 m/s
  memory.KP_ATTRACT = 0.9;
  memory.KP_REPULSE = 1.8;        // Radial pushback gain
  memory.KP_TANGENT = 1.4;        // Lateral circulating gain around walls
  memory.DANGER_ZONE = 0.55;      // 0.55m LiDAR threshold
  memory.stallCounter = 0;
  memory.lastLogTime = 0;
  memory.missionComplete = false;
  memory.initialized = true;

  robot.setShape("O"); // Start with stable 2x2 shape
  robot.log("SmorphiRoboRoarZ Tangential APF Controller Initialized.");
  robot.log("Target Sequence: [(0, 2), (2, 3), (-2, 4)] with 5s mandatory dwell.");
}

// 1. Check Mission Status
if (memory.currentWpIndex >= memory.waypoints.length) {
  robot.setVelocity(0.0, 0.0, 0.0);
  if (!memory.missionComplete) {
    memory.missionComplete = true;
    robot.log("All waypoints reached. Mission complete.");
  }
  return;
}

const target = memory.waypoints[memory.currentWpIndex];
const robotX = sensors.pose.x;
const robotY = sensors.pose.y;
const imuYaw = (sensors.imu && sensors.imu.headingRad !== undefined)
  ? sensors.imu.headingRad
  : sensors.pose.theta;

const globalDx = target.x - robotX;
const globalDy = target.y - robotY;
const distanceToTarget = Math.hypot(globalDx, globalDy);

// 1. Waypoint Dwell Interval Logic
if (distanceToTarget <= memory.GOAL_TOLERANCE) {
  const now = performance.now();
  if (!memory.isWaiting) {
    memory.isWaiting = true;
    memory.arrivalTime = now;
    robot.setVelocity(0.0, 0.0, 0.0);
    robot.log(\`Target \${memory.currentWpIndex + 1} (\${target.x}, \${target.y}) Reached. Waiting 5s...\`);
  } else {
    const elapsed = (now - memory.arrivalTime) / 1000;
    if (elapsed >= memory.DWELL_TIME) {
      memory.isWaiting = false;
      memory.currentWpIndex++;
      robot.log("Timer complete. Proceeding to next target.");
    }
  }
  return; // Halt movement during 5-second interval
}

// 2. Goal Attraction Vector (Robot Local Frame)
const localTargetX =  globalDx * Math.cos(imuYaw) + globalDy * Math.sin(imuYaw);
const localTargetY = -globalDx * Math.sin(imuYaw) + globalDy * Math.cos(imuYaw);

const targetNorm = Math.hypot(localTargetX, localTargetY);
let attractVx = 0.0;
let attractVy = 0.0;
if (targetNorm > 1e-6) {
  attractVx = (localTargetX / targetNorm) * memory.KP_ATTRACT;
  attractVy = (localTargetY / targetNorm) * memory.KP_ATTRACT;
}

// 3. Tangential LiDAR Repulsion (Breaks Local Minima)
let repulseVx = 0.0;
let repulseVy = 0.0;
let obstacleNear = false;

const ranges = (sensors.lidar && sensors.lidar.ranges) ? sensors.lidar.ranges : sensors.lidar;
const numBeams = (ranges && ranges.length) || 360;

for (let i = 0; i < numBeams; i++) {
  const r = ranges[i];

  if (r > 0.02 && r < memory.DANGER_ZONE) {
    obstacleNear = true;
    const angleRad = (i * Math.PI) / 180.0;

    // Radial force magnitude (pushes straight away from wall)
    const fRadial = Math.pow((memory.DANGER_ZONE - r) / memory.DANGER_ZONE, 2) * memory.KP_REPULSE;
    const radX = -Math.cos(angleRad) * fRadial;
    const radY = -Math.sin(angleRad) * fRadial;

    // Tangential force magnitude (rotates 90 deg to slide along wall)
    const fTan = fRadial * memory.KP_TANGENT;
    let tanX = -Math.sin(angleRad) * fTan;
    let tanY =  Math.cos(angleRad) * fTan;

    // Align tangential direction toward the target
    const dot = (tanX * attractVx) + (tanY * attractVy);
    if (dot < 0) {
      tanX = -tanX;
      tanY = -tanY;
    }

    repulseVx += (radX + tanX);
    repulseVy += (radY + tanY);
  }
}

// 4. Force Synthesis
let finalVx = attractVx + repulseVx;
let finalVy = attractVy + repulseVy;

// 5. Anti-Stall Watchdog
// If speed drops near zero in front of an obstacle, inject an orthogonal strafe pulse
let commandedSpeed = Math.hypot(finalVx, finalVy);
if (obstacleNear && commandedSpeed < 0.15) {
  memory.stallCounter++;
  if (memory.stallCounter > 5) { // Stuck for > 5 ticks
    finalVx = 0.0;
    finalVy = (localTargetY >= 0 ? 0.6 : -0.6); // Force immediate lateral crab-walk
    const now = performance.now();
    if (now - memory.lastLogTime > 600) {
      memory.lastLogTime = now;
      robot.log(\`[ANTI-STALL] Stuck threshold reached. Pulsing lateral strafe (vy=\${finalVy}).\`);
    }
  }
} else {
  memory.stallCounter = 0;
}

// 6. Velocity Clamping
commandedSpeed = Math.hypot(finalVx, finalVy);
if (commandedSpeed > memory.MAX_SPEED) {
  finalVx = (finalVx / commandedSpeed) * memory.MAX_SPEED;
  finalVy = (finalVy / commandedSpeed) * memory.MAX_SPEED;
}

// Return velocity commands (wz = 0 to maintain straight heading)
robot.setVelocity(finalVx, finalVy, 0.0);
`,

      // Alias for explicit selection
      smorphi_autonomous: `/**
 * SMORPHI 3-WAYPOINT AUTONOMOUS MISSION (TANGENTIAL APF PORT)
 * Direct controller class invocation
 */
if (!memory.controller) {
  memory.controller = new SmorphiRoboRoarZ();
  robot.setShape("O");
  robot.log("SmorphiRoboRoarZ instance initialized.");
}

const imuYaw = (sensors.imu && sensors.imu.headingRad !== undefined)
  ? sensors.imu.headingRad
  : sensors.pose.theta;
const lidarRanges = (sensors.lidar && sensors.lidar.ranges) ? sensors.lidar.ranges : sensors.lidar;

memory.controller.computeAutonomousMovement(sensors.pose.x, sensors.pose.y, imuYaw, lidarRanges, robot);
`,

      // Competition-Grade Fast A* Global Planner + Holonomic DWA Navigator
      fast_astar_dwa: `/**
 * FAST A* GLOBAL PLANNER + HOLONOMIC DWA LOCAL NAVIGATOR
 * -----------------------------------------------------------
 * Architecture:
 *   - Online 2D Occupancy Grid Mapping (LiDAR Raycasting)
 *   - Morphology-Aware Costmap Inflation (O vs I Footprint)
 *   - 8-Connected A* Global Search with String-Pulling Simplification
 *   - 3-DOF Holonomic DWA with Dynamic Physical Acceleration Bounds
 *   - Hard Collision Rejection & Braking Feasibility Guard
 *   - Narrow Corridor Passage Morphology Hysteresis
 *   - Anti-Stall & Contact Recovery Watchdog
 */

if (!memory.navigator) {
  if (typeof FastAStarDwaNavigator !== "undefined") {
    memory.navigator = new FastAStarDwaNavigator();
  } else if (typeof window !== "undefined" && window.FastAStarDwaNavigator) {
    memory.navigator = new window.FastAStarDwaNavigator();
  } else {
    robot.log("[ERROR] FastAStarDwaNavigator class not found!");
  }
  robot.log("Fast A* + Holonomic DWA Navigation System Active.");
}

if (memory.navigator) {
  memory.navigator.update(sensors, robot, dt);
}
`,

      // 2. Goal Seeking with Artificial Potential Field & Dynamic Morphing
      goal_seeker: `/**
 * ROBO-ROARZ GOAL-SEEKING & RECONFIGURATION NAVIGATOR
 * Combines attractive goal vector with LiDAR repulsive obstacle forces.
 */

if (!memory.init) {
  memory.init = true;
  robot.setShape("O");
  robot.log("Target Seeking Navigator Started!");
}

const target = sensors.target;
const frontDist = sensors.lidar.getFront(35);
const leftDist = sensors.lidar.getLeft(45);
const rightDist = sensors.lidar.getRight(45);

// Check if Goal Reached!
if (target.reached) {
  robot.setVelocity(0, 0, 0);
  robot.log("MISSION ACCOMPLISHED: Target Objective Reached!");
  return;
}

// Check for tight choke points on the way to goal
if (leftDist < 0.35 && rightDist < 0.35) {
  robot.setShape("I"); // Morph to squeeze through
} else if (sensors.shape === "I" && leftDist > 0.6 && rightDist > 0.6) {
  robot.setShape("O");
}

// 1. Attractive force towards target
let targetAngle = target.angle; // radians relative to heading
let attractiveVx = Math.cos(targetAngle) * 0.32;
let attractiveVy = Math.sin(targetAngle) * 0.32;

// 2. Repulsive force from obstacles
let repulseVx = 0;
let repulseVy = 0;

if (frontDist < 0.65) {
  const urgency = (0.65 - frontDist) / 0.65;
  repulseVx -= urgency * 0.45;
  // Push toward clearer side
  if (leftDist > rightDist) {
    repulseVy += urgency * 0.35;
  } else {
    repulseVy -= urgency * 0.35;
  }
}

// Combine forces for Mecanum holonomic locomotion
let vx = attractiveVx + repulseVx;
let vy = attractiveVy + repulseVy;
let omega = targetAngle * 1.5; // Rotate to face target

// Keep rotation smooth
omega = Math.max(-2.0, Math.min(2.0, omega));

robot.setVelocity(vx, vy, omega);
`,

      // 3. Mecanum Holonomic Omnidirectional Strafe Demo
      holonomic_drift: `/**
 * MECANUM HOLONOMIC DRIFT & ORBIT DEMO
 * Demonstrates 3-DOF crabbing (lateral motion) without turning heading!
 */

if (!memory.t) {
  memory.t = 0;
  robot.setShape("O");
  robot.log("Holonomic Mecanum Strafe Demo Initialized.");
}

memory.t += dt;

// Circular drift trajectory:
// Moves sideways and forward while keeping heading fixed at 0 rad!
const speed = 0.28;
const vx = Math.cos(memory.t * 0.8) * speed;
const vy = Math.sin(memory.t * 0.8) * speed;

// Check front LiDAR
if (sensors.lidar.getFront(25) < 0.4) {
  robot.setVelocity(-0.15, vy, 0);
} else {
  robot.setVelocity(vx, vy, 0.0); // Zero rotation! Pure holonomic translation!
}
`,

      // 4. Wall Follower (PID)
      wall_follower: `/**
 * PID RIGHT-WALL FOLLOWER
 * Maintains constant 0.38m distance to right wall using LiDAR
 */

if (!memory.pid) {
  memory.targetDist = 0.38;
  memory.prevError = 0;
  memory.integral = 0;
  memory.pid = true;
  robot.setShape("O");
  robot.log("Right Wall Follower Initialized.");
}

const frontDist = sensors.lidar.getFront(35);
const rightDist = sensors.lidar.getRight(40);

if (frontDist < 0.5) {
  // Obstacle ahead: Turn left immediately
  robot.setVelocity(0.05, 0.0, 1.5);
  return;
}

// PD Controller on wall distance
const error = rightDist - memory.targetDist;
const derivative = (error - memory.prevError) / dt;
memory.prevError = error;

const Kp = 1.8;
const Kd = 0.4;
const steer = Kp * error + Kd * derivative;

robot.setVelocity(0.28, 0.0, -steer);
`,
    };
  }

  /**
   * Compile user script text into an executable function
   * @param {string} codeText
   */
  compileScript(codeText) {
    this.hasError = false;
    this.lastErrorMessage = null;

    try {
      // Sandboxed function wrapping
      // Arguments: sensors, robot, memory, dt
      this.compiledFunction = new Function("sensors", "robot", "memory", "dt", codeText);
      this.memory = {}; // Reset persistent memory on fresh compile
      if (this.autonomousController) {
        this.autonomousController.reset();
      }
      this.log("Code compiled successfully.");
      return { success: true };
    } catch (err) {
      this.hasError = true;
      this.lastErrorMessage = `Syntax Error: ${err.message}`;
      this.log(`[SYNTAX ERROR] ${err.message}`, "error");
      return { success: false, error: err.message };
    }
  }

  /**
   * Execute one simulation tick of user code
   * @param {Object} sensorsInput - Read-only sensor snapshot
   * @param {SmorphiRobot} robot - Robot model instance
   * @param {number} dt - Delta time
   */
  executeTick(sensorsInput, robot, dt) {
    if (!this.isRunning || !this.compiledFunction || this.hasError) {
      return;
    }

    // Safe robot controller proxy
    const robotAPI = {
      setVelocity: (vx, vy, omega) => {
        robot.setVelocity(vx, vy, omega);
      },
      setShape: (shape) => {
        return robot.setShape(shape);
      },
      stop: () => {
        robot.stop();
      },
      log: (msg) => {
        this.log(String(msg), "user");
      },
    };

    try {
      this.compiledFunction(sensorsInput, robotAPI, this.memory, dt);
    } catch (err) {
      this.hasError = true;
      this.lastErrorMessage = `Runtime Error: ${err.message}`;
      this.log(`[RUNTIME ERROR] ${err.message}`, "error");
      robot.stop(); // Fail-safe stop
    }
  }

  /**
   * Execute one cycle directly using the SmorphiRoboRoarZ instance
   * @param {Object} sensorsInput
   * @param {SmorphiRobot} robot
   */
  executeAutonomousCycle(sensorsInput, robot) {
    const imuYaw = (sensorsInput.imu && sensorsInput.imu.headingRad !== undefined)
      ? sensorsInput.imu.headingRad
      : sensorsInput.pose.theta;
    const lidarRanges = (sensorsInput.lidar && sensorsInput.lidar.ranges)
      ? sensorsInput.lidar.ranges
      : sensorsInput.lidar;
    const ekfX = sensorsInput.pose.x;
    const ekfY = sensorsInput.pose.y;

    const robotAPI = {
      setVelocity: (vx, vy, omega) => robot.setVelocity(vx, vy, omega),
      setShape: (shape) => robot.setShape(shape),
      stop: () => robot.stop(),
      log: (msg) => this.log(String(msg), "user"),
    };

    return this.autonomousController.computeAutonomousMovement(ekfX, ekfY, imuYaw, lidarRanges, robotAPI);
  }

  /**
   * Logging facility for simulator console
   */
  log(message, type = "info") {
    const now = performance.now();
    // Throttle duplicate rapid logs
    if (type === "user" && now - this.lastLogTime < 50) return;
    this.lastLogTime = now;

    const entry = {
      time: new Date().toLocaleTimeString(),
      type,
      text: message,
    };

    for (const cb of this.logCallbacks) {
      cb(entry);
    }
  }

  onLog(callback) {
    this.logCallbacks.push(callback);
  }

  start() {
    this.isRunning = true;
    this.hasError = false;
  }

  stop() {
    this.isRunning = false;
  }

  resetMemory() {
    this.memory = {};
    if (this.autonomousController) {
      this.autonomousController.reset();
    }
  }
}

// Attach controller classes to CodeEngine namespace
CodeEngine.SmorphiRoboRoarZ = SmorphiRoboRoarZ;
CodeEngine.SmorphiAutonomousController = SmorphiAutonomousController;

// Expose globally for in-browser execution
if (typeof window !== "undefined") {
  window.CodeEngine = CodeEngine;
  window.SmorphiRoboRoarZ = SmorphiRoboRoarZ;
  window.SmorphiAutonomousController = SmorphiAutonomousController;
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = CodeEngine;
}
