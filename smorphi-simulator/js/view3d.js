/**
 * Three.js 3D Viewport & Visualizer for Smorphi Modular Robot
 * Features 4 modular cubes with animated hinges, 16 spinning Mecanum wheels,
 * 360° LiDAR laser point cloud/beams, realistic shadows, and multi-camera modes.
 */

class Viewport3D {
  constructor(containerId, arenaMap) {
    this.container = document.getElementById(containerId);
    this.map = arenaMap;

    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;

    // Camera modes: "perspective" | "topdown" | "follow"
    this.cameraMode = "perspective";

    // Visual elements
    this.robotGroup = null;
    this.moduleMeshes = [];
    this.wheelMeshes = [];
    this.hingeMeshes = [];
    this.lidarScannerMesh = null;
    this.lidarLaserLines = null;
    this.lidarPointGeometry = null;
    this.lidarPointsMesh = null;
    this.obstacleMeshes = [];
    this.trajectoryLine = null;
    this.goalMesh = null;

    // Visualization toggles
    this.showLidarBeams = true;
    this.showTrajectory = true;
    this.showBoundingBoxes = false;
    this.plannerPathLine = null;
    this.lookaheadMarker = null;
    this.showPlannerPath = true;

    this.init();
  }

  init() {
    if (!this.container) return;

    const width = this.container.clientWidth || 800;
    const height = this.container.clientHeight || 600;

    // 1. Scene setup
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x090d16);
    this.scene.fog = new THREE.FogExp2(0x090d16, 0.04);

    // 2. Camera setup
    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.05, 50);
    // Initial view: isometric looking at center of 5x5 arena
    this.camera.position.set(2.5, 4.8, 6.2);
    this.camera.lookAt(2.5, 0, 2.5);

    // 3. WebGL Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.appendChild(this.renderer.domElement);

    // 4. OrbitControls
    if (typeof THREE.OrbitControls !== "undefined") {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.08;
      this.controls.target.set(2.5, 0, 2.5);
      this.controls.maxPolarAngle = Math.PI / 2 - 0.05; // Prevent camera going below ground
      this.controls.minDistance = 1.0;
      this.controls.maxDistance = 12.0;
    }

    // 5. Lighting
    this.setupLighting();

    // 6. Arena Environment (Floor, Walls, Grid)
    this.buildArenaEnvironment();

    // 7. Smorphi Robot 3D Model
    this.buildRobotModel();

    // 8. LiDAR Ray Visualization
    this.buildLidarVisualizer();

    // 9. Trajectory Trail
    this.buildTrajectoryVisualizer();

    // 9.5 Autonomous Global Planner Path & Lookahead Marker
    this.buildPlannerVisualizer();

    // 10. Goal Beacon
    this.buildGoalMarker();

    // 11. Obstacles
    this.rebuildObstacleMeshes();

    // 12. Resize listener
    window.addEventListener("resize", () => this.onWindowResize());
  }

  setupLighting() {
    // Ambient light
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambientLight);

    // Main directional sunlight with soft shadow
    const sunLight = new THREE.DirectionalLight(0xffffff, 0.9);
    sunLight.position.set(4, 8, 3);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 2048;
    sunLight.shadow.mapSize.height = 2048;
    sunLight.shadow.camera.near = 0.5;
    sunLight.shadow.camera.far = 18;
    const d = 4.0;
    sunLight.shadow.camera.left = -d;
    sunLight.shadow.camera.right = d;
    sunLight.shadow.camera.top = d;
    sunLight.shadow.camera.bottom = -d;
    sunLight.shadow.bias = -0.0005;
    this.scene.add(sunLight);

    // Secondary fill light (cool cyan tone)
    const fillLight = new THREE.DirectionalLight(0x06b6d4, 0.4);
    fillLight.position.set(-2, 4, -2);
    this.scene.add(fillLight);
  }

  buildArenaEnvironment() {
    const W = this.map.width;
    const H = this.map.height;

    // Floor Mesh (Y is UP in Three.js, so arena X is X, arena Y is Z!)
    const floorGeo = new THREE.PlaneGeometry(W, H);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.8,
      metalness: 0.2,
    });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.set(W / 2, 0, H / 2);
    floor.receiveShadow = true;
    this.scene.add(floor);

    // Coordinate Metric Grid (every 0.5m)
    const gridHelper = new THREE.GridHelper(W, 10, 0x38bdf8, 0x1e293b);
    gridHelper.position.set(W / 2, 0.002, H / 2);
    this.scene.add(gridHelper);

    // Perimeter Border Walls
    const wallMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.6,
      metalness: 0.4,
    });
    const wallThick = CONFIG.ARENA.WALL_THICKNESS;
    const wallH = CONFIG.ARENA.WALL_HEIGHT;

    const wallsData = [
      // Bottom (z = 0)
      { w: W + wallThick * 2, h: wallH, d: wallThick, x: W / 2, y: wallH / 2, z: -wallThick / 2 },
      // Top (z = H)
      { w: W + wallThick * 2, h: wallH, d: wallThick, x: W / 2, y: wallH / 2, z: H + wallThick / 2 },
      // Left (x = 0)
      { w: wallThick, h: wallH, d: H, x: -wallThick / 2, y: wallH / 2, z: H / 2 },
      // Right (x = W)
      { w: wallThick, h: wallH, d: H, x: W + wallThick / 2, y: wallH / 2, z: H / 2 },
    ];

    for (const w of wallsData) {
      const geo = new THREE.BoxGeometry(w.w, w.h, w.d);
      const mesh = new THREE.Mesh(geo, wallMat);
      mesh.position.set(w.x, w.y, w.z);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.scene.add(mesh);

      // Warning neon stripe on top of perimeter
      const stripeGeo = new THREE.BoxGeometry(w.w, 0.02, w.d);
      const stripeMat = new THREE.MeshBasicMaterial({ color: 0x0284c7 });
      const stripe = new THREE.Mesh(stripeGeo, stripeMat);
      stripe.position.set(w.x, wallH + 0.01, w.z);
      this.scene.add(stripe);
    }
  }

  buildRobotModel() {
    this.robotGroup = new THREE.Group();
    this.scene.add(this.robotGroup);

    const s = CONFIG.ROBOT.MODULE_SIZE; // 0.16m
    const modColors = CONFIG.COLORS.MODULES;

    // 4 Modular Cubes
    for (let i = 0; i < CONFIG.ROBOT.NUM_MODULES; i++) {
      const modGroup = new THREE.Group();

      // Main Cube Body with bevel look
      const bodyGeo = new THREE.BoxGeometry(s * 0.94, s * 0.75, s * 0.94);
      const bodyMat = new THREE.MeshStandardMaterial({
        color: modColors[i],
        roughness: 0.35,
        metalness: 0.6,
      });
      const body = new THREE.Mesh(bodyGeo, bodyMat);
      body.position.y = s * 0.75 / 2 + 0.02; // Elevated above ground for wheels
      body.castShadow = true;
      body.receiveShadow = true;
      modGroup.add(body);

      // Top Status LED Ring
      const ringGeo = new THREE.CylinderGeometry(s * 0.25, s * 0.25, 0.01, 16);
      const ringMat = new THREE.MeshStandardMaterial({
        color: i === 0 ? 0x38bdf8 : 0x10b981,
        emissive: i === 0 ? 0x0284c7 : 0x059669,
        emissiveIntensity: 0.6,
      });
      const ring = new THREE.Mesh(ringGeo, ringMat);
      ring.position.y = s * 0.75 + 0.025;
      modGroup.add(ring);

      // Module ID Decal / Center Accent
      const decalGeo = new THREE.BoxGeometry(s * 0.2, 0.015, s * 0.2);
      const decalMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
      const decal = new THREE.Mesh(decalGeo, decalMat);
      decal.position.y = s * 0.75 + 0.03;
      modGroup.add(decal);

      // 4 Mecanum Wheels for this module
      const wheelOffsets = [
        { x:  s * 0.42, z:  s * 0.42 }, // Front-Left
        { x:  s * 0.42, z: -s * 0.42 }, // Front-Right
        { x: -s * 0.42, z:  s * 0.42 }, // Rear-Left
        { x: -s * 0.42, z: -s * 0.42 }, // Rear-Right
      ];

      const modWheels = [];
      const wheelR = CONFIG.ROBOT.WHEEL_RADIUS;
      const wheelW = CONFIG.ROBOT.WHEEL_WIDTH;

      for (let w = 0; w < 4; w++) {
        const wheelGroup = new THREE.Group();
        const wheelGeo = new THREE.CylinderGeometry(wheelR, wheelR, wheelW, 12);
        const wheelMat = new THREE.MeshStandardMaterial({
          color: 0x334155,
          roughness: 0.5,
          metalness: 0.7,
        });
        const wheelMesh = new THREE.Mesh(wheelGeo, wheelMat);
        wheelMesh.rotation.z = Math.PI / 2; // Orient horizontally
        wheelMesh.castShadow = true;
        wheelGroup.add(wheelMesh);

        // Angled Mecanum Rollers texture/groove accent
        const rollerRingGeo = new THREE.TorusGeometry(wheelR * 0.85, 0.004, 6, 12);
        const rollerMat = new THREE.MeshBasicMaterial({ color: 0x94a3b8 });
        const rollerRing = new THREE.Mesh(rollerRingGeo, rollerMat);
        rollerRing.rotation.y = Math.PI / 2;
        wheelGroup.add(rollerRing);

        wheelGroup.position.set(wheelOffsets[w].x, wheelR, wheelOffsets[w].z);
        modGroup.add(wheelGroup);
        modWheels.push(wheelGroup);
        this.wheelMeshes.push(wheelGroup);
      }

      this.robotGroup.add(modGroup);
      this.moduleMeshes.push({
        group: modGroup,
        bodyMesh: body,
        wheels: modWheels,
      });
    }

    // 3 Motorized Hinges between modules
    for (let h = 0; h < 3; h++) {
      const hingeGeo = new THREE.CylinderGeometry(0.018, 0.018, s * 0.5, 12);
      const hingeMat = new THREE.MeshStandardMaterial({
        color: 0x64748b,
        metalness: 0.9,
        roughness: 0.2,
      });
      const hingeMesh = new THREE.Mesh(hingeGeo, hingeMat);
      hingeMesh.position.y = s * 0.4;
      hingeMesh.castShadow = true;
      this.robotGroup.add(hingeMesh);
      this.hingeMeshes.push(hingeMesh);
    }

    // Center 2D LiDAR Turret on Module 1 (Master)
    const lidarBaseGeo = new THREE.CylinderGeometry(0.035, 0.04, 0.03, 16);
    const lidarBaseMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, metalness: 0.8, roughness: 0.3 });
    const lidarBase = new THREE.Mesh(lidarBaseGeo, lidarBaseMat);
    lidarBase.position.y = s * 0.85;

    const lidarHeadGeo = new THREE.CylinderGeometry(0.032, 0.032, 0.025, 16);
    const lidarHeadMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.9, roughness: 0.2 });
    this.lidarScannerMesh = new THREE.Mesh(lidarHeadGeo, lidarHeadMat);
    this.lidarScannerMesh.position.y = s * 0.88;

    // Glowing laser aperture diode
    const diodeGeo = new THREE.BoxGeometry(0.015, 0.008, 0.008);
    const diodeMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const diode = new THREE.Mesh(diodeGeo, diodeMat);
    diode.position.set(0.03, 0, 0);
    this.lidarScannerMesh.add(diode);

    this.robotGroup.add(lidarBase);
    this.robotGroup.add(this.lidarScannerMesh);
  }

  buildLidarVisualizer() {
    const numBeams = CONFIG.LIDAR.NUM_BEAMS;

    // Point cloud for 360 hits
    this.lidarPointGeometry = new THREE.BufferGeometry();
    const positions = new Float32Array(numBeams * 3);
    const colors = new Float32Array(numBeams * 3);

    this.lidarPointGeometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    this.lidarPointGeometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));

    const pMat = new THREE.PointsMaterial({
      size: 0.06,
      vertexColors: true,
      transparent: true,
      opacity: 0.9,
    });
    this.lidarPointsMesh = new THREE.Points(this.lidarPointGeometry, pMat);
    this.scene.add(this.lidarPointsMesh);

    // Laser Lines Fan
    const lineGeo = new THREE.BufferGeometry();
    const linePositions = new Float32Array(numBeams * 2 * 3);
    lineGeo.setAttribute("position", new THREE.BufferAttribute(linePositions, 3));

    const lineMat = new THREE.LineBasicMaterial({
      color: 0x06b6d4,
      transparent: true,
      opacity: 0.18,
    });
    this.lidarLaserLines = new THREE.LineSegments(lineGeo, lineMat);
    this.scene.add(this.lidarLaserLines);
  }

  buildTrajectoryVisualizer() {
    const maxPoints = 300;
    const geom = new THREE.BufferGeometry();
    const positions = new Float32Array(maxPoints * 3);
    geom.setAttribute("position", new THREE.BufferAttribute(positions, 3));

    const mat = new THREE.LineBasicMaterial({
      color: 0x38bdf8,
      linewidth: 2,
      transparent: true,
      opacity: 0.7,
    });

    this.trajectoryLine = new THREE.Line(geom, mat);
    this.scene.add(this.trajectoryLine);
  }

  buildPlannerVisualizer() {
    const maxPoints = 120;
    const geom = new THREE.BufferGeometry();
    const positions = new Float32Array(maxPoints * 3);
    geom.setAttribute("position", new THREE.BufferAttribute(positions, 3));

    const mat = new THREE.LineBasicMaterial({
      color: 0x10b981, // Vibrant emerald green
      linewidth: 3,
      transparent: true,
      opacity: 0.9,
    });

    this.plannerPathLine = new THREE.Line(geom, mat);
    this.scene.add(this.plannerPathLine);

    // Lookahead target point marker (cyan ring)
    const markerGeo = new THREE.RingGeometry(0.04, 0.08, 16);
    const markerMat = new THREE.MeshBasicMaterial({
      color: 0x06b6d4, // Cyan
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.85,
    });
    this.lookaheadMarker = new THREE.Mesh(markerGeo, markerMat);
    this.lookaheadMarker.rotation.x = -Math.PI / 2;
    this.lookaheadMarker.position.y = 0.03;
    this.lookaheadMarker.visible = false;
    this.scene.add(this.lookaheadMarker);
  }

  buildGoalMarker() {
    this.goalGroup = new THREE.Group();

    // Floating diamond / RoboRoarZ trophy beacon
    const beaconGeo = new THREE.OctahedronGeometry(0.18, 0);
    const beaconMat = new THREE.MeshStandardMaterial({
      color: 0xfacc15,
      emissive: 0xeab308,
      emissiveIntensity: 0.8,
      metalness: 0.8,
      roughness: 0.2,
    });
    this.goalBeacon = new THREE.Mesh(beaconGeo, beaconMat);
    this.goalBeacon.position.y = 0.45;
    this.goalGroup.add(this.goalBeacon);

    // Vertical hologram light cylinder
    const cylGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.9, 16, 1, true);
    const cylMat = new THREE.MeshBasicMaterial({
      color: 0xfacc15,
      transparent: true,
      opacity: 0.25,
      side: THREE.DoubleSide,
    });
    const cyl = new THREE.Mesh(cylGeo, cylMat);
    cyl.position.y = 0.45;
    this.goalGroup.add(cyl);

    // Base ring on floor
    const ringGeo = new THREE.RingGeometry(0.2, 0.28, 32);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0xfacc15,
      side: THREE.DoubleSide,
    });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.005;
    this.goalGroup.add(ring);

    this.scene.add(this.goalGroup);
  }

  rebuildObstacleMeshes() {
    // Remove existing meshes
    for (const mesh of this.obstacleMeshes) {
      this.scene.remove(mesh);
    }
    this.obstacleMeshes = [];

    const crateMat = new THREE.MeshStandardMaterial({
      color: 0x475569,
      roughness: 0.5,
      metalness: 0.3,
    });

    const corridorMat = new THREE.MeshStandardMaterial({
      color: 0x334155,
      roughness: 0.4,
      metalness: 0.5,
    });

    const obsH = 0.35; // 35 cm obstacle height

    for (const obs of this.map.obstacles) {
      let mesh;
      const isCorridor = obs.type === "corridor";

      if (obs.type === "pillar") {
        const radius = Math.min(obs.w, obs.h) / 2;
        const geo = new THREE.CylinderGeometry(radius, radius, obsH, 16);
        mesh = new THREE.Mesh(geo, crateMat);
        mesh.position.set(obs.x + obs.w / 2, obsH / 2, obs.y + obs.h / 2);
      } else {
        const geo = new THREE.BoxGeometry(obs.w, obsH, obs.h);
        mesh = new THREE.Mesh(geo, isCorridor ? corridorMat : crateMat);
        mesh.position.set(obs.x + obs.w / 2, obsH / 2, obs.y + obs.h / 2);

        // Corridor warning edge highlight
        if (isCorridor) {
          const edgeGeo = new THREE.BoxGeometry(obs.w, 0.02, obs.h);
          const edgeMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b });
          const edge = new THREE.Mesh(edgeGeo, edgeMat);
          edge.position.y = obsH / 2 + 0.01;
          mesh.add(edge);
        }
      }

      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.scene.add(mesh);
      this.obstacleMeshes.push(mesh);
    }
  }

  /**
   * Update 3D elements on each frame
   */
  update(robot, sensorSuite, dt) {
    // In Three.js:
    // Robot arena X is 3D X
    // Robot arena Y is 3D Z!
    // Robot heading theta is rotation around 3D Y axis!

    // 1. Update Robot Position & Heading
    this.robotGroup.position.set(robot.x, 0, robot.y);
    // Heading 0 points along +X, so rotation.y is -theta
    this.robotGroup.rotation.y = -robot.theta;

    // 2. Update 4 Modules Relative Offsets (Morphology Animation)
    for (let i = 0; i < CONFIG.ROBOT.NUM_MODULES; i++) {
      const off = robot.currentOffsets[i];
      // off.x is along robot heading (3D X), off.y is lateral left (3D -Z)
      this.moduleMeshes[i].group.position.set(off.x, 0, -off.y);
    }

    // 3. Update Motorized Hinges between modules
    for (let h = 0; h < 3; h++) {
      const mA = robot.currentOffsets[h];
      const mB = robot.currentOffsets[h + 1];
      const midX = (mA.x + mB.x) / 2;
      const midZ = (-mA.y + -mB.y) / 2;
      this.hingeMeshes[h].position.set(midX, CONFIG.ROBOT.MODULE_SIZE * 0.4, midZ);
    }

    // 4. Spin Mecanum Wheels according to kinematics speeds
    for (let w = 0; w < CONFIG.ROBOT.TOTAL_WHEELS; w++) {
      if (this.wheelMeshes[w]) {
        this.wheelMeshes[w].rotation.x = robot.wheelAngles[w];
      }
    }

    // 5. Spin LiDAR Turret (10 Hz rotation)
    if (this.lidarScannerMesh) {
      this.lidarScannerMesh.rotation.y += Math.PI * 2 * 10 * dt;
    }

    // 6. Update LiDAR Rays & Points
    this.updateLidarVisualization(sensorSuite.lidarPoints, robot);

    // 7. Update Trajectory Breadcrumbs
    this.updateTrajectoryVisualization(robot.trajectory);

    // 7.5. Update Autonomous Planner Path & Lookahead
    this.updatePlannerVisualization();

    // 8. Update Goal Marker Animation
    if (this.goalGroup) {
      this.goalGroup.position.set(this.map.goal.x, 0, this.map.goal.y);
      this.goalBeacon.rotation.y += 1.8 * dt;
      this.goalBeacon.rotation.x += 0.8 * dt;
    }

    // 9. Camera Controls & Camera Modes
    this.updateCamera(robot);

    // 10. Render WebGL Scene
    if (this.renderer && this.scene && this.camera) {
      this.renderer.render(this.scene, this.camera);
    }
  }

  updateLidarVisualization(points, robot) {
    if (!this.lidarPointsMesh || !points || points.length === 0) return;

    this.lidarPointsMesh.visible = this.showLidarBeams;
    this.lidarLaserLines.visible = this.showLidarBeams;

    if (!this.showLidarBeams) return;

    const posAttr = this.lidarPointGeometry.attributes.position;
    const colAttr = this.lidarPointGeometry.attributes.color;
    const linePosAttr = this.lidarLaserLines.geometry.attributes.position;

    const sensorY = CONFIG.ROBOT.MODULE_SIZE * 0.88;

    for (let i = 0; i < points.length; i++) {
      const pt = points[i];
      // 3D coordinates (arena Y is 3D Z)
      const hitX = pt.x;
      const hitY = sensorY;
      const hitZ = pt.y;

      posAttr.setXYZ(i, hitX, hitY, hitZ);

      // Color coding: Red < 0.6m, Amber < 1.2m, Cyan >= 1.2m
      if (pt.distance < 0.6) {
        colAttr.setXYZ(i, 0.94, 0.27, 0.27); // Red
      } else if (pt.distance < 1.2) {
        colAttr.setXYZ(i, 0.96, 0.62, 0.07); // Amber
      } else {
        colAttr.setXYZ(i, 0.02, 0.71, 0.83); // Cyan
      }

      // Line from robot to hit
      const lIdx = i * 2;
      linePosAttr.setXYZ(lIdx, robot.x, sensorY, robot.y);
      linePosAttr.setXYZ(lIdx + 1, hitX, hitY, hitZ);
    }

    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    linePosAttr.needsUpdate = true;
  }

  updateTrajectoryVisualization(trajectory) {
    if (!this.trajectoryLine) return;
    this.trajectoryLine.visible = this.showTrajectory;

    if (!this.showTrajectory || trajectory.length < 2) return;

    const posAttr = this.trajectoryLine.geometry.attributes.position;
    const count = Math.min(trajectory.length, 300);

    for (let i = 0; i < count; i++) {
      posAttr.setXYZ(i, trajectory[i].x, 0.015, trajectory[i].y);
    }

    this.trajectoryLine.geometry.setDrawRange(0, count);
    posAttr.needsUpdate = true;
  }

  updatePlannerVisualization() {
    if (!this.plannerPathLine) return;
    const dbg = (typeof window !== "undefined") ? window.__plannerDebugState : null;
    const hasPath = dbg && dbg.path && dbg.path.length > 0;

    this.plannerPathLine.visible = this.showPlannerPath && hasPath;
    if (this.lookaheadMarker) {
      this.lookaheadMarker.visible = this.showPlannerPath && !!(dbg && dbg.lookahead);
    }

    if (!this.showPlannerPath || !hasPath) {
      this.plannerPathLine.geometry.setDrawRange(0, 0);
      return;
    }

    const path = dbg.path;
    const posAttr = this.plannerPathLine.geometry.attributes.position;
    const count = Math.min(path.length, 120);

    for (let i = 0; i < count; i++) {
      // In Three.js: arena x -> 3D X, arena y -> 3D Z
      posAttr.setXYZ(i, path[i].x, 0.025, path[i].y);
    }

    this.plannerPathLine.geometry.setDrawRange(0, count);
    posAttr.needsUpdate = true;

    if (this.lookaheadMarker && dbg.lookahead) {
      this.lookaheadMarker.position.set(dbg.lookahead.x, 0.03, dbg.lookahead.y);
    }
  }

  updateCamera(robot) {
    if (this.cameraMode === "topdown") {
      // Direct Top-Down 2D Tactical View
      this.camera.position.set(this.map.width / 2, 7.5, this.map.height / 2);
      this.camera.lookAt(this.map.width / 2, 0, this.map.height / 2);
      if (this.controls) this.controls.enabled = false;
    } else if (this.cameraMode === "follow") {
      // Smooth Follow / Chase Camera behind Smorphi
      if (this.controls) this.controls.enabled = false;
      const dist = 1.6;
      const camHeight = 1.0;
      const cosT = Math.cos(robot.theta);
      const sinT = Math.sin(robot.theta);

      const targetCamX = robot.x - cosT * dist;
      const targetCamZ = robot.y - sinT * dist;

      this.camera.position.lerp(new THREE.Vector3(targetCamX, camHeight, targetCamZ), 0.08);
      this.camera.lookAt(robot.x + cosT * 0.5, 0.2, robot.y + sinT * 0.5);
    } else {
      // Perspective with OrbitControls
      if (this.controls) {
        this.controls.enabled = true;
        this.controls.update();
      }
    }
  }

  setCameraMode(mode) {
    this.cameraMode = mode;
    if (mode === "perspective" && this.controls) {
      this.controls.target.set(this.map.width / 2, 0, this.map.height / 2);
    }
  }

  onWindowResize() {
    if (!this.container || !this.renderer || !this.camera) return;
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = Viewport3D;
}
