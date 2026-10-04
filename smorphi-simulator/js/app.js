/**
 * Smorphi Robotics Simulator - Main Application Controller
 * Coordinates physics, 3D viewport, sensor raycasting, Code Editor,
 * Telemetry HUD, audio synthesis, and event handlers.
 */

class SmorphiApp {
  constructor() {
    // Simulator State
    this.isSimRunning = true;
    this.simSpeed = 1.0;
    this.manualControlEnabled = false;
    this.keysDown = {};
    this.audioEnabled = true;

    // Timing
    this.lastFrameTime = performance.now();
    this.simTime = 0.0;
    this.frameCount = 0;
    this.fps = 60;
    this.lastFpsUpdate = performance.now();

    // Instantiate Core Subsystems
    this.map = new ArenaMap(CONFIG.ARENA.WIDTH, CONFIG.ARENA.HEIGHT);
    this.robot = new SmorphiRobot(this.map.spawn.x, this.map.spawn.y, 0);
    this.physics = new PhysicsEngine(this.map);
    this.sensors = new SensorSuite(this.map);
    this.codeEngine = new CodeEngine();

    // DOM & Viewports
    this.view3d = new Viewport3D("viewport-3d", this.map);
    this.telemetry = new TelemetryDashboard("lidar-radar-canvas");

    // Audio Synthesizer (Web Audio API)
    this.audioCtx = null;

    // Code Editor
    this.editor = null;

    this.init();
  }

  init() {
    this.setupCodeEditor();
    this.setupConsoleLogger();
    this.setupUIEventListeners();
    this.setupKeyboardListeners();

    // Initial sensor scan
    this.sensors.update(this.robot);
    this.telemetry.update(this.robot, this.sensors, this.map);

    // Initial code compilation of default template
    if (this.editor) {
      this.codeEngine.compileScript(this.editor.getValue());
    }

    // Start Simulation Loop
    requestAnimationFrame((t) => this.simulationLoop(t));
    console.log("[SmorphiApp] Simulator initialized successfully.");
  }

  /**
   * Setup Ace Editor for interactive JavaScript coding
   */
  setupCodeEditor() {
    if (typeof ace !== "undefined") {
      this.editor = ace.edit("code-editor");
      this.editor.setTheme("ace/theme/monokai");
      this.editor.session.setMode("ace/mode/javascript");
      this.editor.setOptions({
        fontSize: "13px",
        showPrintMargin: false,
        wrap: true,
        enableBasicAutocompletion: true,
        enableLiveAutocompletion: true,
        tabSize: 2,
      });

      // Set default script from codeEngine
      this.editor.setValue(this.codeEngine.presets.default_avoidance, -1);
    } else {
      console.warn("[SmorphiApp] Ace Editor not loaded, falling back to textarea");
    }
  }

  /**
   * Hook simulator console logging to UI console tab
   */
  setupConsoleLogger() {
    const consoleLogsContainer = document.getElementById("console-logs");
    if (!consoleLogsContainer) return;

    this.codeEngine.onLog((entry) => {
      const line = document.createElement("div");
      line.className = "text-xs font-mono py-0.5 leading-relaxed flex items-start gap-2";

      let colorClass = "text-slate-300";
      let badge = "INFO";
      let badgeClass = "bg-slate-700/60 text-slate-300";

      if (entry.type === "error") {
        colorClass = "text-red-400 font-semibold";
        badge = "ERR";
        badgeClass = "bg-red-500/20 text-red-400 border border-red-500/40";
        this.playBeep(240, 0.2, "sawtooth");
      } else if (entry.type === "user") {
        colorClass = "text-cyan-300";
        badge = "LOG";
        badgeClass = "bg-cyan-500/20 text-cyan-400 border border-cyan-500/40";
      }

      line.innerHTML = `
        <span class="text-slate-500 select-none">${entry.time}</span>
        <span class="px-1 rounded text-[10px] ${badgeClass}">${badge}</span>
        <span class="${colorClass}">${this.escapeHtml(entry.text)}</span>
      `;

      consoleLogsContainer.appendChild(line);

      // Keep last 150 entries
      while (consoleLogsContainer.childNodes.length > 150) {
        consoleLogsContainer.removeChild(consoleLogsContainer.firstChild);
      }

      consoleLogsContainer.scrollTop = consoleLogsContainer.scrollHeight;
    });
  }

  escapeHtml(str) {
    return str.replace(/[&<>'"]/g, 
      tag => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;' }[tag] || tag)
    );
  }

  /**
   * Wire buttons, sliders, camera switches, and presets
   */
  setupUIEventListeners() {
    // 1. Code Editor Actions
    const btnRun = document.getElementById("btn-run-code");
    const btnStop = document.getElementById("btn-stop-code");
    const btnResetCode = document.getElementById("btn-reset-code");
    const presetSelect = document.getElementById("code-preset-select");
    const btnClearConsole = document.getElementById("btn-clear-console");

    if (btnRun) {
      btnRun.addEventListener("click", () => {
        this.initAudio();
        const code = this.editor ? this.editor.getValue() : "";
        const res = this.codeEngine.compileScript(code);
        if (res.success) {
          this.codeEngine.start();
          this.manualControlEnabled = false;
          this.updateModeIndicator("AUTONOMOUS");
          this.playBeep(650, 0.1, "sine");
        }
      });
    }

    if (btnStop) {
      btnStop.addEventListener("click", () => {
        this.codeEngine.stop();
        this.robot.stop();
        this.updateModeIndicator("STOPPED / PAUSED");
      });
    }

    if (btnResetCode) {
      btnResetCode.addEventListener("click", () => {
        if (this.editor) {
          this.editor.setValue(this.codeEngine.presets.default_avoidance, -1);
          this.codeEngine.compileScript(this.editor.getValue());
          this.codeEngine.log("Editor reset to default Obstacle Avoidance template.");
        }
      });
    }

    if (presetSelect) {
      presetSelect.addEventListener("change", (e) => {
        const val = e.target.value;
        if (this.codeEngine.presets[val] && this.editor) {
          this.editor.setValue(this.codeEngine.presets[val], -1);
          this.codeEngine.compileScript(this.editor.getValue());
          this.codeEngine.log(`Loaded preset template: ${val}`);
        }
      });
    }

    if (btnClearConsole) {
      btnClearConsole.addEventListener("click", () => {
        const c = document.getElementById("console-logs");
        if (c) c.innerHTML = "";
      });
    }

    // 2. Simulation Environment Controls
    const btnNewMap = document.getElementById("btn-new-map");
    const densitySelect = document.getElementById("map-density-select");
    const btnResetRobot = document.getElementById("btn-reset-robot");
    const btnSimPlayPause = document.getElementById("btn-sim-play-pause");
    const btnSimStep = document.getElementById("btn-sim-step");
    const simSpeedSelect = document.getElementById("sim-speed-select");

    if (btnNewMap) {
      btnNewMap.addEventListener("click", () => {
        const density = densitySelect ? densitySelect.value : "MEDIUM";
        this.map.generateMap(density);
        this.view3d.rebuildObstacleMeshes();
        this.robot.resetPose(this.map.spawn.x, this.map.spawn.y, 0);
        this.codeEngine.resetMemory();
        this.codeEngine.log(`New arena generated [Density: ${density}]`);
        this.playBeep(440, 0.1, "triangle");
      });
    }

    if (densitySelect) {
      densitySelect.addEventListener("change", () => {
        if (btnNewMap) btnNewMap.click();
      });
    }

    if (btnResetRobot) {
      btnResetRobot.addEventListener("click", () => {
        this.robot.resetPose(this.map.spawn.x, this.map.spawn.y, 0);
        this.codeEngine.resetMemory();
        this.codeEngine.log("Robot pose reset to spawn.");
      });
    }

    if (btnSimPlayPause) {
      btnSimPlayPause.addEventListener("click", () => {
        this.isSimRunning = !this.isSimRunning;
        btnSimPlayPause.innerHTML = this.isSimRunning
          ? `<i data-lucide="pause" class="w-4 h-4"></i> Pause`
          : `<i data-lucide="play" class="w-4 h-4"></i> Play`;
        if (window.lucide) lucide.createIcons();
      });
    }

    if (btnSimStep) {
      btnSimStep.addEventListener("click", () => {
        this.stepSimulation(CONFIG.SIM.DT);
      });
    }

    if (simSpeedSelect) {
      simSpeedSelect.addEventListener("change", (e) => {
        this.simSpeed = parseFloat(e.target.value) || 1.0;
      });
    }

    // 3. Camera View Switches
    const camButtons = document.querySelectorAll("[data-cam-mode]");
    camButtons.forEach((btn) => {
      btn.addEventListener("click", () => {
        const mode = btn.getAttribute("data-cam-mode");
        this.view3d.setCameraMode(mode);
        camButtons.forEach((b) => b.classList.remove("bg-cyan-500", "text-white"));
        btn.classList.add("bg-cyan-500", "text-white");
      });
    });

    // 4. Manual Morphology Selector Buttons
    const shapeButtons = document.querySelectorAll("[data-shape-target]");
    shapeButtons.forEach((btn) => {
      btn.addEventListener("click", () => {
        const shape = btn.getAttribute("data-shape-target");
        this.initAudio();
        const ok = this.robot.setShape(shape);
        if (ok) {
          this.codeEngine.log(`Manual shape shift -> ${shape}`);
          this.playServoSound();
        }
      });
    });

    // 5. Visual Toggles (LiDAR Beams, Trail, Sound)
    const toggleLidar = document.getElementById("toggle-lidar-beams");
    if (toggleLidar) {
      toggleLidar.addEventListener("change", (e) => {
        this.view3d.showLidarBeams = e.target.checked;
      });
    }

    const toggleTrail = document.getElementById("toggle-trajectory-trail");
    if (toggleTrail) {
      toggleTrail.addEventListener("change", (e) => {
        this.view3d.showTrajectory = e.target.checked;
      });
    }

    const togglePlanner = document.getElementById("toggle-planner-path");
    if (togglePlanner) {
      togglePlanner.addEventListener("change", (e) => {
        this.view3d.showPlannerPath = e.target.checked;
      });
    }

    const toggleAudio = document.getElementById("toggle-audio-sfx");
    if (toggleAudio) {
      toggleAudio.addEventListener("change", (e) => {
        this.audioEnabled = e.target.checked;
      });
    }

    // 6. Manual Teleop Mode Toggle
    const btnTeleop = document.getElementById("btn-manual-teleop");
    if (btnTeleop) {
      btnTeleop.addEventListener("click", () => {
        this.manualControlEnabled = !this.manualControlEnabled;
        if (this.manualControlEnabled) {
          this.codeEngine.stop();
          this.updateModeIndicator("MANUAL TELEOP (WASD + Q/E)");
          btnTeleop.classList.add("bg-amber-600", "text-white");
          btnTeleop.classList.remove("bg-slate-800");
        } else {
          this.robot.stop();
          this.updateModeIndicator("IDLE");
          btnTeleop.classList.remove("bg-amber-600", "text-white");
          btnTeleop.classList.add("bg-slate-800");
        }
      });
    }
  }

  updateModeIndicator(text) {
    const el = document.getElementById("sim-status-mode");
    if (el) {
      el.textContent = text;
      if (text.includes("AUTONOMOUS")) {
        el.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-emerald-500/20 text-emerald-400 border border-emerald-500/40";
      } else if (text.includes("MANUAL")) {
        el.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-amber-500/20 text-amber-400 border border-amber-500/40";
      } else {
        el.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-slate-700/40 text-slate-400 border border-slate-600/40";
      }
    }
  }

  /**
   * Keyboard controls for manual teleoperation and quick shape testing
   */
  setupKeyboardListeners() {
    window.addEventListener("keydown", (e) => {
      // Don't intercept typing if user is focused inside code editor
      if (this.editor && this.editor.isFocused()) return;

      this.keysDown[e.key.toLowerCase()] = true;

      // Quick Shape Switching via numeric keys 1-7
      const shapeKeys = { "1": "I", "2": "O", "3": "L", "4": "T", "5": "Z", "6": "S", "7": "J" };
      if (shapeKeys[e.key]) {
        this.robot.setShape(shapeKeys[e.key]);
        this.playServoSound();
      }
    });

    window.addEventListener("keyup", (e) => {
      this.keysDown[e.key.toLowerCase()] = false;
    });
  }

  /**
   * Process manual Mecanum locomotion if teleoperation active
   */
  processManualInput() {
    if (!this.manualControlEnabled) return;

    let vx = 0;
    let vy = 0;
    let omega = 0;

    const maxLin = 0.45;
    const maxAng = 2.0;

    // W/S: Forward / Backward (vx)
    if (this.keysDown["w"] || this.keysDown["arrowup"]) vx += maxLin;
    if (this.keysDown["s"] || this.keysDown["arrowdown"]) vx -= maxLin;

    // A/D: Lateral Crabbing / Strafing (vy)
    if (this.keysDown["a"] || this.keysDown["arrowleft"]) vy += maxLin;
    if (this.keysDown["d"] || this.keysDown["arrowright"]) vy -= maxLin;

    // Q/E: In-place Rotation (omega)
    if (this.keysDown["q"]) omega += maxAng;
    if (this.keysDown["e"]) omega -= maxAng;

    this.robot.setVelocity(vx, vy, omega);
  }

  /**
   * Main Simulation & Animation Loop
   */
  simulationLoop(currentTime) {
    requestAnimationFrame((t) => this.simulationLoop(t));

    // Calculate real frame delta
    let dt = (currentTime - this.lastFrameTime) / 1000;
    this.lastFrameTime = currentTime;

    // Clamp dt to avoid huge physics leaps on tab backgrounding
    if (dt > 0.1) dt = 0.1;

    // FPS calculation
    this.frameCount++;
    if (currentTime - this.lastFpsUpdate > 500) {
      this.fps = Math.round((this.frameCount * 1000) / (currentTime - this.lastFpsUpdate));
      this.frameCount = 0;
      this.lastFpsUpdate = currentTime;
      const fpsEl = document.getElementById("sim-fps");
      if (fpsEl) fpsEl.textContent = `${this.fps} FPS`;
    }

    if (this.isSimRunning) {
      const scaledDt = dt * this.simSpeed;
      this.stepSimulation(scaledDt);
    }

    // Always render 3D View and Telemetry
    this.view3d.update(this.robot, this.sensors, dt);
    this.telemetry.update(this.robot, this.sensors, this.map);
  }

  /**
   * Step physics, execute code, and update sensors
   */
  stepSimulation(dt) {
    this.simTime += dt;

    // 1. Process Manual Keyboard input (if enabled)
    this.processManualInput();

    // 2. Read Sensors before script execution
    this.sensors.update(this.robot);

    // 3. Execute Autonomous Script (if running)
    if (this.codeEngine.isRunning) {
      const scriptSensors = this.sensors.getScriptInput(this.robot);
      this.codeEngine.executeTick(scriptSensors, this.robot, dt);

      // Check if code threw runtime error
      if (this.codeEngine.hasError) {
        this.updateModeIndicator("ERROR IN CODE");
      }
    }

    // 4. Run Physics & Multi-Body Collision Detection
    this.physics.step(this.robot, dt);

    // 5. Sound trigger on collision
    if (this.robot.inCollision && Math.random() < 0.08) {
      this.playCollisionSound();
    }
  }

  // --- Audio Synthesis Engine (Zero dependencies) ---
  initAudio() {
    if (!this.audioEnabled) return;
    if (!this.audioCtx) {
      const AudioContext = window.AudioContext || window.webkitAudioContext;
      if (AudioContext) {
        this.audioCtx = new AudioContext();
      }
    }
    if (this.audioCtx && this.audioCtx.state === "suspended") {
      this.audioCtx.resume();
    }
  }

  playBeep(freq = 440, duration = 0.1, type = "sine") {
    if (!this.audioEnabled || !this.audioCtx) return;
    try {
      const osc = this.audioCtx.createOscillator();
      const gain = this.audioCtx.createGain();
      osc.type = type;
      osc.frequency.setValueAtTime(freq, this.audioCtx.currentTime);
      gain.gain.setValueAtTime(0.08, this.audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this.audioCtx.currentTime + duration);
      osc.connect(gain);
      gain.connect(this.audioCtx.destination);
      osc.start();
      osc.stop(this.audioCtx.currentTime + duration);
    } catch (e) {}
  }

  playServoSound() {
    if (!this.audioEnabled || !this.audioCtx) return;
    try {
      const osc = this.audioCtx.createOscillator();
      const gain = this.audioCtx.createGain();
      osc.type = "sawtooth";
      osc.frequency.setValueAtTime(120, this.audioCtx.currentTime);
      osc.frequency.linearRampToValueAtTime(320, this.audioCtx.currentTime + 0.25);
      gain.gain.setValueAtTime(0.05, this.audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this.audioCtx.currentTime + 0.35);
      osc.connect(gain);
      gain.connect(this.audioCtx.destination);
      osc.start();
      osc.stop(this.audioCtx.currentTime + 0.35);
    } catch (e) {}
  }

  playCollisionSound() {
    if (!this.audioEnabled || !this.audioCtx) return;
    try {
      const osc = this.audioCtx.createOscillator();
      const gain = this.audioCtx.createGain();
      osc.type = "triangle";
      osc.frequency.setValueAtTime(180, this.audioCtx.currentTime);
      osc.frequency.exponentialRampToValueAtTime(40, this.audioCtx.currentTime + 0.08);
      gain.gain.setValueAtTime(0.12, this.audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this.audioCtx.currentTime + 0.09);
      osc.connect(gain);
      gain.connect(this.audioCtx.destination);
      osc.start();
      osc.stop(this.audioCtx.currentTime + 0.09);
    } catch (e) {}
  }
}

// Bootstrap once DOM is ready
window.addEventListener("DOMContentLoaded", () => {
  window.smorphiApp = new SmorphiApp();
});
