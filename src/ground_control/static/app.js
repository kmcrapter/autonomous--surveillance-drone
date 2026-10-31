// AeroGuard 3D Digital Twin Ground Control Station Frontend Engine

document.addEventListener('DOMContentLoaded', () => {
  // 1. Initialize Tactical Leaflet Map
  const DATUM_LAT = 44.500000;
  const DATUM_LON = -110.500000;
  const METERS_PER_DEG_LAT = 111139.0;
  const METERS_PER_DEG_LON = METERS_PER_DEG_LAT * Math.cos(DATUM_LAT * Math.PI / 180.0);

  function cartesianToGps(x, y) {
    const lat = DATUM_LAT + (y / METERS_PER_DEG_LAT);
    const lon = DATUM_LON + (x / METERS_PER_DEG_LON);
    return [lat, lon];
  }

  const map = L.map('tacticalMap', {
    center: [DATUM_LAT, DATUM_LON],
    zoom: 17,
    zoomControl: false,
    attributionControl: false
  });

  // Tactical High-Contrast Map Tiles (100% Free, Zero API Key Required)
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19
  }).addTo(map);

  // Sector Geofences Polygons
  const sectors = [
    {
      name: "Zone Alpha (Perimeter Fence)",
      color: "#00f0ff",
      bounds: [[-45, -55], [45, -55], [45, 28], [-45, 28]]
    },
    {
      name: "Zone Bravo (Northern Forest)",
      color: "#00e599",
      bounds: [[-85, 25], [-25, 25], [-25, 85], [-85, 85]]
    },
    {
      name: "Zone Charlie (River Corridor & Bridge)",
      color: "#ffb800",
      bounds: [[-90, 50], [90, 50], [90, 75], [-90, 75]]
    },
    {
      name: "Zone Delta (Outpost Ridge)",
      color: "#e024c3",
      bounds: [[45, -35], [95, -35], [95, 30], [45, 30]]
    }
  ];

  sectors.forEach(sec => {
    const latlngs = sec.bounds.map(pt => cartesianToGps(pt[0], pt[1]));
    L.polygon(latlngs, {
      color: sec.color,
      weight: 1.5,
      fillColor: sec.color,
      fillOpacity: 0.08,
      dashArray: '3, 4'
    }).addTo(map).bindTooltip(sec.name, { sticky: true, className: 'map-tooltip' });
  });

  // International Border Line Demarcation (Y = 32.0m, Sovereign Boundary)
  const borderP1 = cartesianToGps(-105, 32.0);
  const borderP2 = cartesianToGps(105, 32.0);
  const borderLine = L.polyline([borderP1, borderP2], {
    color: '#ffd000',
    weight: 3.5,
    dashArray: '8, 6',
    opacity: 0.95
  }).addTo(map);
  borderLine.bindTooltip("INTERNATIONAL BORDER LINE (COUNTRY A / COUNTRY B)", { permanent: true, direction: "top", className: "border-tooltip" });

  // Foreign Territory Shading (North of Border Line Y > 32m)
  const foreignBounds = [[-105, 32], [105, 32], [105, 95], [-105, 95]].map(p => cartesianToGps(p[0], p[1]));
  L.polygon(foreignBounds, {
    color: '#ff3b5c',
    weight: 1,
    fillColor: '#ff3b5c',
    fillOpacity: 0.05,
    dashArray: '4, 8'
  }).addTo(map).bindTooltip("FOREIGN TERRITORY (COUNTRY B - SECTOR BRAVO / CHARLIE)", { sticky: true, className: "map-tooltip" });

  // Tactical Concentric Distance Range Rings (Offline Vector Grid Overlay)
  [25, 50, 75, 100].forEach(r_m => {
    L.circle(cartesianToGps(0, 0), {
      radius: r_m,
      color: 'rgba(0, 240, 255, 0.22)',
      weight: 1,
      dashArray: '3, 6',
      fill: false,
      interactive: false
    }).addTo(map);
  });

  // Tactical Map Actions Toolbar
  let isFollowDrone = true;
  const btnMapFollow = document.getElementById('btnMapFollow');
  const btnMapHome = document.getElementById('btnMapHome');
  const btnMapFit = document.getElementById('btnMapFit');
  const btnMapClearTrail = document.getElementById('btnMapClearTrail');

  if (btnMapFollow) {
    btnMapFollow.addEventListener('click', () => {
      isFollowDrone = !isFollowDrone;
      btnMapFollow.classList.toggle('active', isFollowDrone);
      if (isFollowDrone && droneMarker) {
        map.panTo(droneMarker.getLatLng(), { animate: true });
      }
    });
  }

  if (btnMapHome) {
    btnMapHome.addEventListener('click', () => {
      isFollowDrone = false;
      btnMapFollow?.classList.remove('active');
      map.flyTo(cartesianToGps(0, 0), 17);
    });
  }

  if (btnMapFit) {
    btnMapFit.addEventListener('click', () => {
      isFollowDrone = false;
      btnMapFollow?.classList.remove('active');
      const b1 = cartesianToGps(-90, -60);
      const b2 = cartesianToGps(90, 85);
      map.fitBounds([b1, b2], { padding: [20, 20] });
    });
  }

  if (btnMapClearTrail) {
    btnMapClearTrail.addEventListener('click', () => {
      flownTrail.setLatLngs([]);
      trailPoints = [];
    });
  }

  // Ensure Leaflet recalculates dimensions immediately to eliminate blank/grey map glitches
  setTimeout(() => map.invalidateSize(), 150);
  setTimeout(() => map.invalidateSize(), 600);
  setTimeout(() => map.invalidateSize(), 1800);
  window.addEventListener('resize', () => map.invalidateSize());

  // Real-Time Area Coverage Layer & Grid Cache
  const coverageLayerGroup = L.layerGroup().addTo(map);
  const renderedGridCells = new Set();

  // Home Landing Pad Marker
  const homeGps = cartesianToGps(0, 0);
  const homeIcon = L.divIcon({
    className: 'home-pad-marker',
    html: '<div style="background:#ffd000;width:14px;height:14px;border:2px solid #000;border-radius:3px;display:flex;align-items:center;justify-content:center;font-size:9px;font-weight:bold;color:#000;">H</div>',
    iconSize: [14, 14],
    iconAnchor: [7, 7]
  });
  L.marker(homeGps, { icon: homeIcon }).addTo(map).bindTooltip("Home Landing Pad & Inductive Dock", { permanent: false });

  // Flown Trail Polyline
  const flownTrail = L.polyline([], { color: '#00f0ff', weight: 2, opacity: 0.7 }).addTo(map);

  // Country A Sovereign Airspace Standoff Boundary (Drone strictly stays South of this line)
  const sovereignGps1 = cartesianToGps(-105, 28.5);
  const sovereignGps2 = cartesianToGps(105, 28.5);
  L.polyline([sovereignGps1, sovereignGps2], {
    color: '#00f0ff',
    weight: 2,
    dashArray: '6, 6',
    opacity: 0.9
  }).addTo(map).bindTooltip("SOVEREIGN AIRSPACE BOUNDARY (DRONE STRICTLY IN COUNTRY A - Y <= 28.5m)", { permanent: false, sticky: true });

  // Planned Surveillance Pattern Polyline (Boustrophedon Lawnmower Trajectory)
  const plannedPatternPolyline = L.polyline([], {
    color: '#ffd000',
    weight: 1.5,
    dashArray: '4, 6',
    opacity: 0.75
  }).addTo(map);

  // Tactical Event Pins Layer Group
  const eventPinsGroup = L.layerGroup().addTo(map);
  const plottedEventIds = new Set();

  // Drone Position & Heading Marker with Pulsating Radar Beam
  const droneIcon = L.divIcon({
    className: 'drone-svg-marker',
    html: `
      <div id="droneMapIcon" style="transform: rotate(0deg); transition: transform 0.1s linear;">
        <svg width="28" height="28" viewBox="0 0 28 28">
          <!-- Animated Radar Sweep Cone -->
          <polygon points="14,14 0,0 28,0" fill="rgba(0, 240, 255, 0.35)" />
          <line x1="14" y1="14" x2="14" y2="0" stroke="#00f0ff" stroke-width="1.5" stroke-dasharray="2, 2" />
          <!-- Aircraft Hexacopter Triangle -->
          <polygon points="14,4 5,24 14,19 23,24" fill="#ffd000" stroke="#000" stroke-width="1.5" />
          <circle cx="14" cy="14" r="3" fill="#00f0ff" />
        </svg>
      </div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14]
  });
  const droneMarker = L.marker(homeGps, { icon: droneIcon }).addTo(map);

  // Tactical Web Audio Alert Tone Synthesizer
  let isAudioEnabled = true;
  let audioCtx = null;
  function playTacticalTone(freq = 880, duration = 0.08, type = 'sine') {
    if (!isAudioEnabled) return;
    try {
      if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
      if (audioCtx.state === 'suspended') audioCtx.resume();
      const osc = audioCtx.createOscillator();
      const gain = audioCtx.createGain();
      osc.type = type;
      osc.frequency.setValueAtTime(freq, audioCtx.currentTime);
      gain.gain.setValueAtTime(0.05, audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + duration);
      osc.connect(gain);
      gain.connect(audioCtx.destination);
      osc.start();
      osc.stop(audioCtx.currentTime + duration);
    } catch (e) {}
  }

  const btnAudioToggle = document.getElementById('btnAudioToggle');
  const audioText = document.getElementById('audioText');
  const audioIcon = document.getElementById('audioIcon');
  if (btnAudioToggle) {
    btnAudioToggle.addEventListener('click', () => {
      isAudioEnabled = !isAudioEnabled;
      btnAudioToggle.classList.toggle('muted', !isAudioEnabled);
      if (audioText) audioText.textContent = isAudioEnabled ? 'AUDIO ON' : 'AUDIO MUTED';
      if (audioIcon) audioIcon.textContent = isAudioEnabled ? '🔊' : '🔇';
      if (isAudioEnabled) playTacticalTone(880, 0.08);
    });
  }

  // 2. WebSocket Telemetry Streaming
  let ws = null;
  const flightStateText = document.getElementById('flightStateText');
  const flightStatePill = document.getElementById('flightStatePill');
  const activeZoneText = document.getElementById('activeZoneText');
  const missionTimeText = document.getElementById('missionTimeText');
  const batteryPercentText = document.getElementById('batteryPercentText');
  const batteryFillBar = document.getElementById('batteryFillBar');
  const gpsFixText = document.getElementById('gpsFixText');

  // Instruments elements
  const rollText = document.getElementById('rollText');
  const pitchText = document.getElementById('pitchText');
  const adiHorizonGroup = document.getElementById('adiHorizonGroup');
  const altAglM = document.getElementById('altAglM');
  const altAglFt = document.getElementById('altAglFt');
  const altMslM = document.getElementById('altMslM');
  const altMslFt = document.getElementById('altMslFt');

  const compassRoseGroup = document.getElementById('compassRoseGroup');
  const needleHeading = document.getElementById('needleHeading');
  const needleCog = document.getElementById('needleCog');
  const needleWp = document.getElementById('needleWp');
  const needleHome = document.getElementById('needleHome');
  const compassHeadingVal = document.getElementById('compassHeadingVal');
  const compassCogVal = document.getElementById('compassCogVal');
  const compassWpVal = document.getElementById('compassWpVal');
  const compassHomeVal = document.getElementById('compassHomeVal');

  const speedGround = document.getElementById('speedGround');
  const speedVertical = document.getElementById('speedVertical');
  const coordsXy = document.getElementById('coordsXy');
  const coordsZ = document.getElementById('coordsZ');

  const battVoltage = document.getElementById('battVoltage');
  const battPower = document.getElementById('battPower');
  const battRemaining = document.getElementById('battRemaining');
  const battRthThreshold = document.getElementById('battRthThreshold');
  const battEstTime = document.getElementById('battEstTime');
  const dockingStatus = document.getElementById('dockingStatus');

  const cameraFps = document.getElementById('cameraFps');
  const cameraLatency = document.getElementById('cameraLatency');

  const btnStart = document.getElementById('btnStartMission');
  const btnPause = document.getElementById('btnPauseMission');
  const btnResume = document.getElementById('btnResumeMission');
  const btnRth = document.getElementById('btnRthMission');
  const btnAbort = document.getElementById('btnAbortMission');
  const btnReset = document.getElementById('btnResetMission');

  let trailPoints = [];

  function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;
    ws = new WebSocket(wsUrl);

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        updateTelemetryUI(data);
      } catch (err) {
        console.error("Telemetry parse error", err);
      }
    };

    ws.onclose = () => {
      setTimeout(connectWebSocket, 1500);
    };

    ws.onerror = () => {
      ws.close();
    };
  }

  function formatTime(seconds) {
    if (!seconds || seconds < 0) return "00:00:00";
    const hrs = Math.floor(seconds / 3600);
    const mins = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);
    return `${hrs.toString().padStart(2, '0')}:${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  }

  function updateTelemetryUI(telem) {
    // Top HUD
    const state = telem.flight_state || telem.state || "IDLE";
    flightStateText.textContent = state;
    flightStatePill.className = `hud-pill ${state === 'IDLE' ? 'pill-state-idle' : (state === 'SURVEYING' || state === 'TAKEOFF') ? 'pill-state-active' : 'pill-state-warning'}`;
    
    // Sync flight control mode
    if (telem.control_mode && typeof syncFlightModeUI === 'function') {
      syncFlightModeUI(telem.control_mode);
    }

    // Sync surveillance pattern
    if (telem.surveillance_pattern && typeof syncPatternUI === 'function') {
      syncPatternUI(telem.surveillance_pattern);
    }

    activeZoneText.textContent = (telem.current_zone || "ZONE ALPHA").toUpperCase();
    missionTimeText.textContent = formatTime(telem.mission_duration_sec || 0);

    const batt = telem.battery || {};
    const battPct = (batt.percentage !== undefined) ? batt.percentage : 100.0;
    batteryPercentText.textContent = `${battPct.toFixed(1)}%`;
    batteryFillBar.style.width = `${Math.max(0, Math.min(100, battPct))}%`;
    batteryFillBar.style.background = battPct > 35 ? '#00e599' : battPct > 20 ? '#ffb800' : '#ff3b5c';

    // GPS Status in Top HUD
    const gps = telem.gps || {};
    if (gpsFixText) {
      const fix = gps.fix_status || "3D RTK";
      const sats = gps.satellites_visible || 21;
      gpsFixText.textContent = `${fix} (${sats} SV)`;
    }

    // Camera Performance Metrics
    const camMeta = telem.camera_status || {};
    if (cameraFps) cameraFps.textContent = (camMeta.fps || 20.0).toFixed(1);
    if (cameraLatency) cameraLatency.textContent = (camMeta.latency_ms || 12.4).toFixed(1);

    // ADI Artificial Horizon
    const roll = telem.orientation?.roll_deg || 0.0;
    const pitch = telem.orientation?.pitch_deg || 0.0;
    rollText.textContent = `${roll.toFixed(1)}°`;
    pitchText.textContent = `${pitch.toFixed(1)}°`;
    const pitchPx = pitch * 1.8;
    adiHorizonGroup.setAttribute('transform', `rotate(${-roll}, 100, 80) translate(0, ${pitchPx})`);

    // Dual Altitudes
    const alts = telem.altitudes || {};
    altAglM.textContent = (alts.agl_m || 0.0).toFixed(1);
    altAglFt.textContent = (alts.agl_ft || 0.0).toFixed(1);
    altMslM.textContent = (alts.datum_msl_m || 1800.5).toFixed(1);
    altMslFt.textContent = (alts.datum_msl_ft || 5907.1).toFixed(1);

    // Compass
    const comp = telem.compass || {};
    const heading = comp.heading_deg || 0.0;
    const cog = comp.direction_of_travel_deg || heading;
    const wpBearing = comp.bearing_to_next_wp_deg || 0.0;
    const homeBearing = comp.bearing_to_home_deg || 0.0;

    needleHeading.setAttribute('transform', `rotate(${heading}, 100, 100)`);
    needleCog.setAttribute('transform', `rotate(${cog}, 100, 100)`);
    needleWp.setAttribute('transform', `rotate(${wpBearing}, 100, 100)`);
    needleHome.setAttribute('transform', `rotate(${homeBearing}, 100, 100)`);

    compassHeadingVal.textContent = `${Math.round(heading).toString().padStart(3, '0')}°`;
    compassCogVal.textContent = `${Math.round(cog).toString().padStart(3, '0')}°`;
    compassWpVal.textContent = `${Math.round(wpBearing).toString().padStart(3, '0')}°`;
    compassHomeVal.textContent = `${Math.round(homeBearing).toString().padStart(3, '0')}°`;

    // Speeds & 3D Coordinates
    const vels = telem.velocities || {};
    speedGround.innerHTML = `${(vels.ground_speed_ms || 0.0).toFixed(1)} <small>m/s</small>`;
    speedVertical.innerHTML = `${(vels.v_speed_ms || 0.0).toFixed(1)} <small>m/s</small>`;

    const pos = telem.position || { x: 0, y: 0, z: 0.49 };
    coordsXy.textContent = `${pos.x.toFixed(1)}, ${pos.y.toFixed(1)}`;
    coordsZ.textContent = `${pos.z.toFixed(2)} m`;

    // Power
    battVoltage.textContent = `${(batt.voltage || 25.2).toFixed(1)} V`;
    battPower.textContent = `${(batt.power_w || 18.0).toFixed(0)} W`;
    battRemaining.textContent = `${(batt.remaining_wh || 555.0).toFixed(1)} Wh`;
    battRthThreshold.textContent = `${(telem.rth?.rth_trigger_pct || 20.0).toFixed(1)}%`;
    battEstTime.textContent = `${(batt.remaining_time_min || 52.0).toFixed(1)} min`;
    dockingStatus.textContent = batt.is_charging ? "CHARGING (+450W)" : (state === "LANDING" ? "DOCKING..." : "DISCONNECTED");
    dockingStatus.className = batt.is_charging ? "font-mono text-emerald" : "font-mono";

    // Adaptive Button Enabled / Disabled States
    if (state === 'IDLE' || state === 'COMPLETE' || state === 'CHARGING') {
      if (btnStart) btnStart.disabled = false;
      if (btnPause) btnPause.disabled = true;
      if (btnResume) btnResume.disabled = true;
      if (btnRth) btnRth.disabled = true;
      if (btnAbort) btnAbort.disabled = true;
    } else if (state === 'SURVEYING' || state === 'TAKEOFF' || state === 'PRECHECK') {
      if (btnStart) btnStart.disabled = true;
      if (btnPause) btnPause.disabled = false;
      if (btnResume) btnResume.disabled = true;
      if (btnRth) btnRth.disabled = false;
      if (btnAbort) btnAbort.disabled = false;
    } else if (state === 'PAUSED') {
      if (btnStart) btnStart.disabled = true;
      if (btnPause) btnPause.disabled = true;
      if (btnResume) btnResume.disabled = false;
      if (btnRth) btnRth.disabled = false;
      if (btnAbort) btnAbort.disabled = false;
    } else if (state === 'RETURNING' || state === 'LANDING') {
      if (btnStart) btnStart.disabled = true;
      if (btnPause) btnPause.disabled = true;
      if (btnResume) btnResume.disabled = true;
      if (btnRth) btnRth.disabled = true;
      if (btnAbort) btnAbort.disabled = false;
    }

    // Update Tactical Map
    const currentGps = cartesianToGps(pos.x, pos.y);
    droneMarker.setLatLng(currentGps);

    if (isFollowDrone && map) {
      map.panTo(currentGps, { animate: false });
    }

    const mapIconEl = document.getElementById('droneMapIcon');
    if (mapIconEl) {
      mapIconEl.style.transform = `rotate(${heading}deg)`;
    }

    // Real-Time Thermal Radiometry HUD Updates
    const rad = telem.thermal_radiometry || {};
    const centerT = rad.t_center || 36.8;
    const maxT = rad.t_max || 42.0;
    latestThermalCenterTemp = centerT;
    const centerEl = document.getElementById('thermalCenterTemp');
    const maxEl = document.getElementById('thermalMaxTemp');
    if (centerEl) centerEl.textContent = `CTR: ${centerT.toFixed(1)}°C`;
    if (maxEl) maxEl.textContent = `MAX: ${maxT.toFixed(1)}°C`;

    // Real-Time Subsystem Avionics Diagnostics
    if (telem.avionics) {
      const av = telem.avionics;
      const rpmBox = document.getElementById('motorRpmChips');
      if (rpmBox && Array.isArray(av.motor_rpms)) {
        rpmBox.innerHTML = av.motor_rpms.map((rpm, i) => {
          const isSpinning = rpm > 0;
          const statusClass = isSpinning ? 'rpm-chip active font-mono' : 'rpm-chip stopped font-mono';
          return `<span class="${statusClass}" title="${isSpinning ? 'Propeller Spinning' : 'Propeller Stopped'}">M${i+1}: ${rpm}</span>`;
        }).join('');
      }
      const lipoBox = document.getElementById('lipoCellChips');
      if (lipoBox && Array.isArray(av.lipo_cells_v)) {
        lipoBox.innerHTML = av.lipo_cells_v.map((v, i) => `<span class="cell-chip font-mono">C${i+1}: ${v.toFixed(2)}V</span>`).join('');
      }
      const rfEl = document.getElementById('rfLinkStatus');
      if (rfEl && av.rf_link) {
        rfEl.textContent = `RSSI ${av.rf_link.rssi_dbm} dBm | SNR ${av.rf_link.snr_db} dB | ${av.rf_link.quality_pct}%`;
      }
      const escEl = document.getElementById('escTempStatus');
      if (escEl && Array.isArray(av.esc_temps_c)) {
        const avgEsc = (av.esc_temps_c.reduce((a, b) => a + b, 0) / av.esc_temps_c.length).toFixed(1);
        escEl.textContent = `${avgEsc}°C (NOMINAL)`;
      }
    }

    // Real-Time Target & Contact Matrix Table
    const matrixContainer = document.getElementById('targetMatrixContainer');
    const matrixCountBadge = document.getElementById('targetMatrixCountBadge');
    const matrixHeaderBadge = document.getElementById('targetMatrixHeaderBadge');
    if (telem.target_matrix && Array.isArray(telem.target_matrix)) {
      if (matrixCountBadge) matrixCountBadge.textContent = telem.target_matrix.length;
      if (matrixHeaderBadge) matrixHeaderBadge.textContent = `${telem.target_matrix.length} CONTACTS`;
      if (matrixContainer) {
        if (telem.target_matrix.length === 0) {
          matrixContainer.innerHTML = '<div class="empty-state">No contacts in optical or thermal range. Move drone along surveillance pattern to detect targets.</div>';
        } else {
          matrixContainer.innerHTML = telem.target_matrix.map(c => {
            const isForeign = c.territory.includes('FOREIGN');
            const catColor = c.category === 'person' ? '#00f0ff' : c.category === 'vehicle' ? '#ffd000' : c.category === 'wildlife' ? '#00e599' : '#ff3b5c';
            return `
              <div class="contact-card ${isForeign ? 'is-foreign' : ''}">
                <div class="contact-header-row">
                  <div>
                    <span class="contact-tid">${c.track_id}</span>
                    <span class="contact-label-tag" style="margin-left: 6px;">${c.label}</span>
                  </div>
                  <span class="badge" style="background:${catColor}20;color:${catColor};border:1px solid ${catColor}50;font-size:8px;">${c.category.toUpperCase()}</span>
                </div>
                <div class="contact-grid-info">
                  <div><span class="cg-label">RANGE</span><span class="cg-val">${c.range_m.toFixed(1)}m</span></div>
                  <div><span class="cg-label">BEARING</span><span class="cg-val">${Math.round(c.bearing_deg)}°</span></div>
                  <div><span class="cg-label">TEMP</span><span class="cg-val text-amber">${c.temp_c.toFixed(1)}°C</span></div>
                  <div><span class="cg-label">TERRITORY</span><span class="cg-val" style="color:${isForeign ? '#ff3b5c' : '#00e599'}">${isForeign ? 'FOREIGN' : 'HOME'}</span></div>
                </div>
              </div>
            `;
          }).join('');
        }
      }
    }

    // Real-Time Area Coverage Grid & Progress HUD
    if (telem.coverage) {
      const cov = telem.coverage;
      const covPctEl = document.getElementById('mapCoveragePct');
      const covAreaEl = document.getElementById('mapCoverageArea');
      const covBarEl = document.getElementById('mapCoverageBar');
      if (covPctEl) covPctEl.textContent = `${cov.percentage.toFixed(1)}%`;
      if (covAreaEl) covAreaEl.textContent = `(${(cov.area_covered_m2 || 0).toLocaleString()} m²)`;
      if (covBarEl) covBarEl.style.width = `${Math.min(100, cov.percentage)}%`;

      if (cov.cells_sample && Array.isArray(cov.cells_sample)) {
        cov.cells_sample.forEach(([gx, gy]) => {
          const key = `${gx}_${gy}`;
          if (!renderedGridCells.has(key)) {
            renderedGridCells.add(key);
            const p1 = cartesianToGps(gx - 2.5, gy - 2.5);
            const p2 = cartesianToGps(gx + 2.5, gy + 2.5);
            L.rectangle([p1, p2], {
              color: '#00e599',
              weight: 0.5,
              fillColor: '#00e599',
              fillOpacity: 0.20
            }).addTo(coverageLayerGroup);
          }
        });
      }
    }

    // Append to trail if moved
    if (trailPoints.length === 0 || Math.hypot(pos.x - trailPoints[trailPoints.length - 1][0], pos.y - trailPoints[trailPoints.length - 1][1]) > 0.8) {
      trailPoints.push([pos.x, pos.y]);
      flownTrail.addLatLng(currentGps);
      if (trailPoints.length > 600) {
        trailPoints.shift();
      }
    }

    // Planned Surveillance Pattern Trajectory (Boustrophedon Lawnmower Path)
    if (telem.planned_pattern && Array.isArray(telem.planned_pattern) && plannedPatternPolyline.getLatLngs().length === 0) {
      const patternGps = telem.planned_pattern.map(wp => cartesianToGps(wp.x, wp.y));
      plannedPatternPolyline.setLatLngs(patternGps);
    }

    // Dynamic Target Discovery: ONLY show targets on map AFTER the drone actually finds them!
    if (telem.detections && Array.isArray(telem.detections)) {
      telem.detections.forEach(det => {
        const trkId = det.track_id || det.label;
        if (!plottedEventIds.has(trkId) && det.world_pos) {
          plottedEventIds.add(trkId);
          const [wx, wy] = det.world_pos;
          const pinGps = cartesianToGps(wx, wy);
          const isForeign = (wy > 32.0 || (det.territory || '').includes('FOREIGN'));
          const isWoman = (det.gender === 'woman' || (det.label || '').toLowerCase().includes('woman'));
          const isMan = (!isWoman && (det.gender === 'man' || (det.label || '').toLowerCase().includes('man')));
          const isAnimal = (det.category === 'wildlife' || (det.label || '').toLowerCase().includes('animal') || (det.label || '').toLowerCase().includes('deer') || (det.label || '').toLowerCase().includes('elk'));
          const pinColor = isWoman ? '#ff66cc' : isMan ? (isForeign ? '#ff8c00' : '#00f0ff') : isAnimal ? '#00e599' : det.category === 'vehicle' ? '#ffd000' : '#ff3b5c';

          const pinIcon = L.divIcon({
            className: 'discovered-target-pin',
            html: `
              <div class="target-radar-ping" style="border-color:${pinColor};"></div>
              <div style="background:${pinColor};width:12px;height:12px;border-radius:50%;border:2px solid #fff;box-shadow:0 0 8px ${pinColor};position:relative;z-index:2;"></div>
            `,
            iconSize: [24, 24],
            iconAnchor: [12, 12]
          });

          L.marker(pinGps, { icon: pinIcon }).addTo(eventPinsGroup)
            .bindPopup(`
              <div style="font-family:sans-serif;font-size:12px;min-width:180px;">
                <div style="color:${pinColor};font-weight:bold;margin-bottom:4px;">🎯 DISCOVERED BY DRONE</div>
                <b>${det.label}</b><br>
                <span>Territory: ${isForeign ? 'Country B (Foreign)' : 'Country A (Home)'}</span><br>
                <span>Coordinates: [${wx.toFixed(1)}, ${wy.toFixed(1)}]</span><br>
                <span>Confidence: ${Math.round((det.confidence || 0.9) * 100)}%</span>
              </div>
            `).openPopup();
        }
      });
    }
  }

  // 3. Camera Layout Switcher & 5-Camera Array Controls
  const layoutButtons = document.querySelectorAll('.btn-toggle');
  const cameraViewports = document.getElementById('cameraViewports');
  const boxOverview = document.getElementById('camBoxOverview');
  const boxForward = document.getElementById('camBoxForward');
  const boxThermal = document.getElementById('camBoxThermal');
  const boxNvg = document.getElementById('camBoxNvg');
  const boxDownward = document.getElementById('camBoxDownward');
  const boxRear = document.getElementById('camBoxRear');
  const boxLeft = document.getElementById('camBoxLeft');
  const boxRight = document.getElementById('camBoxRight');

  const allCamBoxes = [boxOverview, boxForward, boxThermal, boxNvg, boxDownward, boxRear, boxLeft, boxRight];

  function showCamBoxes(activeBoxes, viewClass = 'single-view') {
    if (!cameraViewports) return;
    cameraViewports.className = `camera-viewports ${viewClass}`;
    allCamBoxes.forEach(b => {
      if (!b) return;
      if (activeBoxes.includes(b)) {
        b.classList.remove('hidden');
      } else {
        b.classList.add('hidden');
      }
    });
    setTimeout(() => map && map.invalidateSize(), 150);
  }

  layoutButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      layoutButtons.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const layout = btn.dataset.layout;

      if (layout === 'dual') {
        showCamBoxes([boxOverview, boxForward], 'dual-view');
      } else if (layout === 'overview') {
        showCamBoxes([boxOverview], 'single-view');
      } else if (layout === 'drone') {
        showCamBoxes([boxForward], 'single-view');
      } else if (layout === 'downward') {
        showCamBoxes([boxDownward], 'single-view');
      } else if (layout === 'rear') {
        showCamBoxes([boxRear], 'single-view');
      } else if (layout === 'left') {
        showCamBoxes([boxLeft], 'single-view');
      } else if (layout === 'right') {
        showCamBoxes([boxRight], 'single-view');
      } else if (layout === 'thermal') {
        showCamBoxes([boxThermal], 'single-view');
      } else if (layout === 'nvg') {
        showCamBoxes([boxNvg], 'single-view');
      } else if (layout === 'quad') {
        showCamBoxes([boxOverview, boxForward, boxThermal, boxNvg], 'quad-view');
      } else if (layout === 'pentagrid') {
        showCamBoxes([boxForward, boxRear, boxLeft, boxRight, boxDownward], 'pentagrid-view');
      }
    });
  });

  // 3a. FLIR LWIR Radiometric Thermal Controls & Palettes
  let latestThermalCenterTemp = 36.8;
  const paletteButtons = document.querySelectorAll('.btn-palette');
  paletteButtons.forEach(btn => {
    btn.addEventListener('click', async () => {
      paletteButtons.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const pal = btn.dataset.palette;
      try {
        await fetch('/api/thermal/palette', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ palette: pal })
        });
        playTacticalTone(880, 0.05);
      } catch (e) {
        console.error("Thermal palette error", e);
      }
    });
  });

  let isIsothermActive = true;
  const btnToggleIsotherm = document.getElementById('btnToggleIsotherm');
  if (btnToggleIsotherm) {
    btnToggleIsotherm.addEventListener('click', async () => {
      isIsothermActive = !isIsothermActive;
      btnToggleIsotherm.classList.toggle('active', isIsothermActive);
      try {
        await fetch('/api/thermal/isotherm', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ enabled: isIsothermActive, threshold_c: 35.0 })
        });
        playTacticalTone(660, 0.05);
      } catch (e) {
        console.error("Isotherm error", e);
      }
    });
  }

  // Interactive Spot Radiometer Hover Probe on Thermal Video
  const thermalImgWrap = document.getElementById('thermalImgWrap');
  const streamThermalImg = document.getElementById('streamThermalImg');
  const thermalProbeBadge = document.getElementById('thermalProbeBadge');

  if (streamThermalImg && thermalProbeBadge) {
    streamThermalImg.addEventListener('mousemove', (e) => {
      const rect = streamThermalImg.getBoundingClientRect();
      const px = (e.clientX - rect.left) / rect.width;
      const py = (e.clientY - rect.top) / rect.height;
      if (px >= 0 && px <= 1 && py >= 0 && py <= 1) {
        thermalProbeBadge.classList.remove('hidden');
        thermalProbeBadge.style.left = `${e.clientX - rect.left + 14}px`;
        thermalProbeBadge.style.top = `${e.clientY - rect.top - 12}px`;
        const baseT = latestThermalCenterTemp || 36.8;
        const distCenter = Math.hypot(px - 0.5, py - 0.5);
        const spotT = Math.max(12.0, baseT - (distCenter * 26.0) + (Math.sin(px * 12 + py * 12) * 1.5));
        thermalProbeBadge.textContent = `${spotT.toFixed(1)}°C / ${(spotT * 9/5 + 32).toFixed(1)}°F`;
      }
    });
    streamThermalImg.addEventListener('mouseleave', () => {
      thermalProbeBadge.classList.add('hidden');
    });
  }

  // 3b. Flight Mode, Pattern Selection & Remote Teleop Controls
  const btnModeAuto = document.getElementById('btnModeAuto');
  const btnModeManual = document.getElementById('btnModeManual');
  const teleopDock = document.getElementById('teleopDock');
  const patternSelect = document.getElementById('patternSelect');
  const teleopSpeedSlider = document.getElementById('teleopSpeedSlider');
  const teleopSpeedVal = document.getElementById('teleopSpeedVal');

  let currentFlightMode = 'AUTONOMOUS';

  function syncFlightModeUI(mode) {
    if (currentFlightMode === mode) return;
    currentFlightMode = mode;
    if (mode === 'MANUAL_REMOTE') {
      btnModeManual?.classList.add('active');
      btnModeAuto?.classList.remove('active');
      teleopDock?.classList.remove('hidden');
    } else {
      btnModeAuto?.classList.add('active');
      btnModeManual?.classList.remove('active');
      teleopDock?.classList.add('hidden');
    }
  }

  function syncPatternUI(pattern) {
    if (patternSelect && document.activeElement !== patternSelect && patternSelect.value !== pattern) {
      patternSelect.value = pattern;
    }
  }

  if (btnModeAuto) {
    btnModeAuto.addEventListener('click', () => {
      syncFlightModeUI('AUTONOMOUS');
      dispatchCommand('SET_MODE', { mode: 'AUTONOMOUS' });
    });
  }

  if (btnModeManual) {
    btnModeManual.addEventListener('click', () => {
      syncFlightModeUI('MANUAL_REMOTE');
      dispatchCommand('SET_MODE', { mode: 'MANUAL_REMOTE' });
    });
  }

  if (patternSelect) {
    patternSelect.addEventListener('change', () => {
      const chosen = patternSelect.value;
      dispatchCommand('SET_PATTERN', { pattern: chosen });
      plannedPatternPolyline.setLatLngs([]);
    });
  }

  if (teleopSpeedSlider && teleopSpeedVal) {
    teleopSpeedSlider.addEventListener('input', () => {
      teleopSpeedVal.textContent = parseFloat(teleopSpeedSlider.value).toFixed(1);
    });
  }

  async function sendTeleop(vx, vy, vz, vyaw) {
    try {
      await fetch('/api/teleop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          linear_x: vx,
          linear_y: vy,
          linear_z: vz,
          angular_z: vyaw
        })
      });
    } catch (err) {
      console.error("Teleop dispatch error:", err);
    }
  }

  function getTeleopSpeed() {
    return parseFloat(teleopSpeedSlider?.value || 3.5);
  }

  const teleopBtnCommands = {
    'tpForward': () => sendTeleop(getTeleopSpeed(), 0.0, 0.0, 0.0),
    'tpBack': () => sendTeleop(-getTeleopSpeed(), 0.0, 0.0, 0.0),
    'tpLeft': () => sendTeleop(0.0, getTeleopSpeed(), 0.0, 0.0),
    'tpRight': () => sendTeleop(0.0, -getTeleopSpeed(), 0.0, 0.0),
    'tpUp': () => sendTeleop(0.0, 0.0, 2.0, 0.0),
    'tpDown': () => sendTeleop(0.0, 0.0, -2.0, 0.0),
    'tpYawLeft': () => sendTeleop(0.0, 0.0, 0.0, 0.8),
    'tpYawRight': () => sendTeleop(0.0, 0.0, 0.0, -0.8),
    'tpBrake': () => sendTeleop(0.0, 0.0, 0.0, 0.0)
  };

  Object.entries(teleopBtnCommands).forEach(([id, fn]) => {
    const el = document.getElementById(id);
    if (!el) return;
    let repeatTimer = null;

    const onStart = (e) => {
      e.preventDefault();
      el.classList.add('pressing');
      fn();
      if (id !== 'tpBrake') {
        repeatTimer = setInterval(fn, 150);
      }
    };

    const onEnd = () => {
      el.classList.remove('pressing');
      if (repeatTimer) {
        clearInterval(repeatTimer);
        repeatTimer = null;
      }
      sendTeleop(0.0, 0.0, 0.0, 0.0);
    };

    el.addEventListener('mousedown', onStart);
    el.addEventListener('mouseup', onEnd);
    el.addEventListener('mouseleave', onEnd);
    el.addEventListener('touchstart', onStart, { passive: false });
    el.addEventListener('touchend', onEnd);
  });

  const pressedKeys = new Set();
  let keyRepeatTimer = null;

  function handleKeyVelocity() {
    if (pressedKeys.size === 0) {
      sendTeleop(0.0, 0.0, 0.0, 0.0);
      return;
    }
    const spd = getTeleopSpeed();
    let vx = 0.0, vy = 0.0, vz = 0.0, vyaw = 0.0;
    if (pressedKeys.has('KeyW')) vx += spd;
    if (pressedKeys.has('KeyS')) vx -= spd;
    if (pressedKeys.has('KeyA')) vy += spd;
    if (pressedKeys.has('KeyD')) vy -= spd;
    if (pressedKeys.has('KeyI')) vz += 2.0;
    if (pressedKeys.has('KeyK')) vz -= 2.0;
    if (pressedKeys.has('KeyJ')) vyaw += 0.8;
    if (pressedKeys.has('KeyL')) vyaw -= 0.8;
    if (pressedKeys.has('Space')) {
      vx = 0.0; vy = 0.0; vz = 0.0; vyaw = 0.0;
    }
    sendTeleop(vx, vy, vz, vyaw);
  }

  window.addEventListener('keydown', (e) => {
    if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return;
    const hotkeys = ['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyI', 'KeyK', 'KeyJ', 'KeyL', 'Space'];
    if (hotkeys.includes(e.code)) {
      e.preventDefault();
      if (!pressedKeys.has(e.code)) {
        pressedKeys.add(e.code);
        if (!keyRepeatTimer) {
          handleKeyVelocity();
          keyRepeatTimer = setInterval(handleKeyVelocity, 150);
        }
      }
    }
  });

  window.addEventListener('keyup', (e) => {
    if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return;
    const hotkeys = ['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyI', 'KeyK', 'KeyJ', 'KeyL', 'Space'];
    if (hotkeys.includes(e.code)) {
      e.preventDefault();
      pressedKeys.delete(e.code);
      if (pressedKeys.size === 0 && keyRepeatTimer) {
        clearInterval(keyRepeatTimer);
        keyRepeatTimer = null;
        sendTeleop(0.0, 0.0, 0.0, 0.0);
      } else {
        handleKeyVelocity();
      }
    }
  });

  // 4. Mission Action Controls
  async function dispatchCommand(action, params = {}) {
    try {
      const res = await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action, params })
      });
      const data = await res.json();
      console.log("Command dispatched:", data);
    } catch (err) {
      console.error("Command error:", err);
    }
  }

  if (btnStart) {
    btnStart.addEventListener('click', () => {
      const sc = document.getElementById('scenarioSelect').value;
      dispatchCommand('START', { scenario: sc });
      // Reset tactical map pins for fresh live discovery
      eventPinsGroup.clearLayers();
      plottedEventIds.clear();
      if (sc === 'night_nvg') {
        document.querySelector('.btn-toggle[data-layout="nvg"]')?.click();
      } else if (sc === 'wildfire_hazard') {
        document.querySelector('.btn-toggle[data-layout="thermal"]')?.click();
      }
    });
  }

  if (btnPause) btnPause.addEventListener('click', () => dispatchCommand('PAUSE'));
  if (btnResume) btnResume.addEventListener('click', () => dispatchCommand('RESUME'));
  if (btnRth) btnRth.addEventListener('click', () => dispatchCommand('RTH'));
  if (btnAbort) btnAbort.addEventListener('click', () => dispatchCommand('ABORT'));
  if (btnReset) {
    btnReset.addEventListener('click', () => {
      dispatchCommand('RESET');
      flownTrail.setLatLngs([]);
      trailPoints = [];
      coverageLayerGroup.clearLayers();
      renderedGridCells.clear();
      eventPinsGroup.clearLayers();
      plottedEventIds.clear();
      const covPctEl = document.getElementById('mapCoveragePct');
      const covAreaEl = document.getElementById('mapCoverageArea');
      const covBarEl = document.getElementById('mapCoverageBar');
      if (covPctEl) covPctEl.textContent = '0.0%';
      if (covAreaEl) covAreaEl.textContent = '(0 m²)';
      if (covBarEl) covBarEl.style.width = '0%';
    });
  }

  // 5. Event Center Polling, Triage & Map Pinning
  const eventLogContainer = document.getElementById('eventLogContainer');
  const eventCountBadge = document.getElementById('eventCountBadge');
  let currentFilter = 'all';

  document.querySelectorAll('.filter-pill').forEach(pill => {
    pill.addEventListener('click', () => {
      document.querySelectorAll('.filter-pill').forEach(p => p.classList.remove('active'));
      pill.classList.add('active');
      currentFilter = pill.dataset.filter;
      loadEvents();
    });
  });

  async function loadEvents() {
    try {
      const res = await fetch('/api/events?limit=40');
      const data = await res.json();
      const events = data.events || [];
      if (eventCountBadge) eventCountBadge.textContent = `${events.length} EVENTS`;

      const filtered = (currentFilter === 'all')
        ? events
        : events.filter(e => {
            const lbl = (e.label || '').toLowerCase();
            const cat = (e.category || '').toLowerCase();
            const gen = (e.gender || '').toLowerCase();
            if (currentFilter === 'woman') return gen === 'woman' || lbl.includes('woman');
            if (currentFilter === 'man') return (gen === 'man' || lbl.includes('man')) && !lbl.includes('woman');
            if (currentFilter === 'wildlife') return cat === 'wildlife' || lbl.includes('animal') || lbl.includes('deer') || lbl.includes('elk');
            if (currentFilter === 'vehicle') return cat === 'vehicle';
            if (currentFilter === 'fire') return cat === 'fire' || cat === 'hazard' || cat === 'smoke';
            return cat === currentFilter;
          });

      if (!eventLogContainer) return;

      if (filtered.length === 0) {
        eventLogContainer.innerHTML = '<div class="empty-state">No matching events logged.</div>';
        return;
      }

      eventLogContainer.innerHTML = filtered.map(ev => {
        const timeStr = new Date(ev.timestamp * 1000).toLocaleTimeString();
        const catClass = `cat-${ev.category}`;
        const isWoman = (ev.gender === 'woman' || (ev.label || '').toLowerCase().includes('woman'));
        const isMan = (!isWoman && (ev.gender === 'man' || (ev.label || '').toLowerCase().includes('man')));
        const isAnimal = (ev.category === 'wildlife' || (ev.label || '').toLowerCase().includes('animal') || (ev.label || '').toLowerCase().includes('deer') || (ev.label || '').toLowerCase().includes('elk'));
        const isForeign = (ev.world_y > 32.0 || (ev.territory || '').includes('FOREIGN'));
        const borderAlert = (ev.border_alert === 'BORDER_PROXIMITY_ALERT' || (isForeign && Math.abs(ev.world_y - 32.0) <= 12.0));

        const subBadge = isWoman ? "WOMAN" : isMan ? "MAN" : isAnimal ? "ANIMAL" : (ev.category || 'OBJECT').toUpperCase();
        const badgeColor = isWoman ? '#ff66cc' : isMan ? '#00f0ff' : isAnimal ? '#00e599' : ev.category === 'vehicle' ? '#ffd000' : '#ff3b5c';
        const territoryBadge = isForeign ? "COUNTRY B (FOREIGN)" : "COUNTRY A (HOME)";
        const territoryColor = isForeign ? "#ff3b5c" : "#00f0ff";
        const statusColor = ev.review_status === 'CONFIRMED' ? '#00e599' : ev.review_status === 'DISMISSED' ? '#8b9bb4' : '#ffb800';

        return `
          <div class="event-card ${catClass}" data-id="${ev.id}">
            <div class="event-card-top">
              <span class="event-badge" style="color:${badgeColor};border:1px solid ${badgeColor}40;">${subBadge}</span>
              <span class="event-badge" style="color:${territoryColor};border:1px solid ${territoryColor}40;margin-left:4px;">${territoryBadge}</span>
              <span class="event-time">${timeStr}</span>
            </div>
            <div class="event-label">${ev.label || 'Observable Entity'}</div>
            ${borderAlert ? `<div class="border-alert-pill">⚠️ BORDER PROXIMITY ALERT (COUNTRY B)</div>` : ''}
            <div class="event-meta">
              <span>CONF: ${((ev.confidence || 0.88) * 100).toFixed(0)}%</span>
              <span>POS: (${(ev.world_x || 0).toFixed(1)}, ${(ev.world_y || 0).toFixed(1)})</span>
              <span style="color:${statusColor};font-weight:600;">${ev.review_status || 'PENDING'}</span>
            </div>
            <div class="event-actions">
              <button class="btn-mini ${ev.review_status === 'CONFIRMED' ? 'btn-mini-active' : ''}" onclick="reviewEvent('${ev.id}', 'CONFIRMED')">CONFIRM</button>
              <button class="btn-mini ${ev.review_status === 'DISMISSED' ? 'btn-mini-active' : ''}" onclick="reviewEvent('${ev.id}', 'DISMISSED')">DISMISS</button>
            </div>
          </div>
        `;
      }).join('');
    } catch (err) {
      console.error("Events fetch error:", err);
    }
  }

  window.reviewEvent = async function(eventId, status) {
    try {
      await fetch(`/api/events/${eventId}/review`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ review_status: status })
      });
      loadEvents();
    } catch (err) {
      console.error("Review event error:", err);
    }
  };

  // =========================================================================
  // 6. Personnel Identification & Verification Panel Logic
  // =========================================================================
  const tabSurveillanceBtn = document.getElementById('tabSurveillanceBtn');
  const tabTargetMatrixBtn = document.getElementById('tabTargetMatrixBtn');
  const tabPersonnelBtn = document.getElementById('tabPersonnelBtn');
  const surveillanceTabPane = document.getElementById('surveillanceTabPane');
  const targetMatrixTabPane = document.getElementById('targetMatrixTabPane');
  const personnelTabPane = document.getElementById('personnelTabPane');

  const allTabs = [
    { btn: tabSurveillanceBtn, pane: surveillanceTabPane, onShow: null },
    { btn: tabTargetMatrixBtn, pane: targetMatrixTabPane, onShow: null },
    { btn: tabPersonnelBtn, pane: personnelTabPane, onShow: () => loadPersonnelData() }
  ];

  allTabs.forEach(({ btn, pane, onShow }) => {
    if (!btn) return;
    btn.addEventListener('click', () => {
      allTabs.forEach(t => {
        t.btn?.classList.remove('active');
        t.pane?.classList.remove('active');
      });
      btn.classList.add('active');
      pane?.classList.add('active');
      if (onShow) onShow();
    });
  });

  // Personnel Scenario Controller
  const personnelScenarioSelect = document.getElementById('personnelScenarioSelect');
  const activeScenarioLabel = document.getElementById('activeScenarioLabel');

  if (personnelScenarioSelect) {
    personnelScenarioSelect.addEventListener('change', async (e) => {
      const chosen = e.target.value;
      if (activeScenarioLabel) activeScenarioLabel.textContent = chosen;
      try {
        await fetch('/api/personnel/scenario', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ scenario: chosen })
        });
        loadPersonnelData();
      } catch (err) {
        console.error("Scenario switch error:", err);
      }
    });
  }

  const personnelListContainer = document.getElementById('personnelListContainer');
  const personnelCountBadge = document.getElementById('personnelCountBadge');
  const personnelAlertCountBadge = document.getElementById('personnelAlertCountBadge');
  const noIdCardAlertContainer = document.getElementById('noIdCardAlertContainer');
  const personnelTimelineList = document.getElementById('personnelTimelineList');

  window.dispatchVoiceAnnouncement = async function(targetId) {
    try {
      const res = await fetch('/api/personnel/voice_announce', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          target_id: targetId,
          notes: "Operator dispatched simulated voice advisory: Identity verification required."
        })
      });
      const data = await res.json();
      alert(`Simulated Voice Advisory Dispatched:\n"Identity verification required. Please remain in the designated safe area and contact authorized personnel."`);
      loadPersonnelData();
    } catch (err) {
      console.error("Voice announce error:", err);
    }
  };

  window.initiateVisualVerification = async function(targetId) {
    try {
      await fetch('/api/personnel/verify', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          target_id: targetId,
          notes: "Operator initiated facial verification check"
        })
      });
      loadPersonnelData();
    } catch (err) {
      console.error("Visual verify error:", err);
    }
  };

  window.reviewPersonnel = async function(entityId, status) {
    try {
      await fetch('/api/personnel/review', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          entity_id: entityId,
          review_status: status,
          operator_notes: `Cleared by operator at ${new Date().toLocaleTimeString()}`
        })
      });
      loadPersonnelData();
    } catch (err) {
      console.error("Review personnel error:", err);
    }
  };

  async function loadPersonnelData() {
    try {
      // 1. Fetch live identities state
      const idRes = await fetch('/api/personnel/identities');
      const idData = await idRes.json();
      const entities = idData.identities || [];

      if (personnelCountBadge) {
        personnelCountBadge.textContent = `${entities.length} DETECTED`;
      }

      // Check for individuals with NO ID CARD
      let noIdEntity = entities.find(e => e.has_id_card === false || e.tag_type === 'NO_ID_CARD_DETECTED' || e.tag_id === 'NONE');

      if (noIdEntity) {
        if (personnelAlertCountBadge) {
          personnelAlertCountBadge.style.display = 'inline-block';
          personnelAlertCountBadge.textContent = 'NO ID ALERT';
        }
        if (noIdCardAlertContainer) {
          const wpos = noIdEntity.world_pos || { x: 0, y: 0 };
          noIdCardAlertContainer.innerHTML = `
            <div class="alert-banner-noid">
              <div class="alert-banner-title">
                <span>⚠️ SECURITY ALERT: NO ELECTRONIC ID CARD DETECTED</span>
                <span class="badge" style="background:#00e59920;color:#00e599;border:1px solid #00e59950;font-size:8px;">NON-HOSTILE POLICY</span>
              </div>
              <div class="alert-banner-body">
                Individual <b>${noIdEntity.full_name || 'Subject'}</b> detected at (${wpos.x.toFixed(1)}, ${wpos.y.toFixed(1)}) with <b>NO valid electronic credential</b>. State: <b>UNVERIFIED</b>. Hostile classification withheld.
              </div>
              <div class="alert-banner-actions">
                <button class="btn-voice-advisory" onclick="dispatchVoiceAnnouncement('${noIdEntity.entity_id}')">
                  📢 DISPATCH VOICE SAFETY ANNOUNCEMENT
                </button>
                <button class="btn-p-action" onclick="initiateVisualVerification('${noIdEntity.entity_id}')">
                  🔍 REQUEST VISUAL VERIFICATION
                </button>
              </div>
            </div>
          `;
        }
      } else {
        if (personnelAlertCountBadge) {
          personnelAlertCountBadge.style.display = 'none';
        }
        if (noIdCardAlertContainer) {
          noIdCardAlertContainer.innerHTML = '';
        }
      }

      // Render personnel cards
      if (personnelListContainer) {
        if (entities.length === 0) {
          personnelListContainer.innerHTML = '<div class="empty-state">No personnel detected in camera or RF field. Select a scenario above or start mission.</div>';
        } else {
          personnelListContainer.innerHTML = entities.map(e => {
            const state = e.identity_state || 'UNVERIFIED';
            const stateClass = `state-${state.toLowerCase()}`;
            const badgeClass =
              state === 'AUTHENTICATED' ? 'badge-auth' :
              state === 'VISUAL_VERIFIED' ? 'badge-visual' :
              state === 'UNVERIFIED' ? 'badge-unverified' :
              state === 'CREDENTIAL_CONFLICT' ? 'badge-conflict' :
              state === 'IDENTITY_REVIEW_REQUIRED' ? 'badge-review' : 'badge-policy';

            const wpos = e.world_pos || { x: 0, y: 0 };
            const confPct = ((e.confidence || 0.5) * 100).toFixed(0);
            const tagDisplay = (e.has_id_card === false || e.tag_type === 'NO_ID_CARD_DETECTED')
              ? '<span style="color:#ffd000;font-weight:700;">⚠️ NO ID CARD</span>'
              : `${e.tag_type || 'N/A'} (${e.tag_id || 'NONE'})`;

            return `
              <div class="personnel-card ${stateClass}">
                <div class="personnel-card-header">
                  <div class="personnel-card-name">
                    ${e.full_name || 'Subject'} <small style="color:var(--text-muted);font-weight:400;">[${e.personnel_id || 'N/A'}]</small>
                  </div>
                  <span class="personnel-state-badge ${badgeClass}">${state.replace('_', ' ')}</span>
                </div>
                <div class="personnel-details-grid">
                  <div class="p-item">
                    <span class="p-label">ELECTRONIC TAG</span>
                    <span class="p-val">${tagDisplay}</span>
                  </div>
                  <div class="p-item">
                    <span class="p-label">MILITARY UNIT</span>
                    <span class="p-val">${e.assigned_unit || 'UNASSIGNED'}</span>
                  </div>
                  <div class="p-item">
                    <span class="p-label">FACE STATUS</span>
                    <span class="p-val">${e.face_verification_status || 'UNVERIFIED'}</span>
                  </div>
                  <div class="p-item">
                    <span class="p-label">CONFIDENCE</span>
                    <span class="p-val">${confPct}%</span>
                  </div>
                  <div class="p-item">
                    <span class="p-label">COORDINATES</span>
                    <span class="p-val">(${wpos.x.toFixed(1)}, ${wpos.y.toFixed(1)})</span>
                  </div>
                  <div class="p-item">
                    <span class="p-label">OPERATOR REVIEW</span>
                    <span class="p-val" style="color:${e.operator_review_status?.includes('CLEARED') ? '#00e599' : '#ffd000'}">${e.operator_review_status || 'PENDING'}</span>
                  </div>
                </div>
                <div class="personnel-card-actions">
                  <button class="btn-p-action" onclick="dispatchVoiceAnnouncement('${e.entity_id}')">📢 VOICE ADVISORY</button>
                  <button class="btn-p-action" onclick="initiateVisualVerification('${e.entity_id}')">🔍 VERIFY FACE</button>
                  <button class="btn-p-action" onclick="reviewPersonnel('${e.entity_id}', 'ACKNOWLEDGED_CLEARED')">✔ ACKNOWLEDGE</button>
                </div>
              </div>
            `;
          }).join('');
        }
      }

      // 2. Fetch timeline events
      const evRes = await fetch('/api/personnel/events?limit=8');
      const evData = await evRes.json();
      const logs = evData.events || [];

      if (personnelTimelineList && logs.length > 0) {
        personnelTimelineList.innerHTML = logs.map(l => {
          const timeStr = new Date((l.timestamp || Date.now() / 1000) * 1000).toLocaleTimeString();
          return `
            <div class="timeline-entry">
              <span class="timeline-time">${timeStr}</span>
              <span class="timeline-text"><b>${l.personnel_id || l.entity_id}</b>: [${l.identity_state}] ${l.full_name} (${l.tag_type})</span>
            </div>
          `;
        }).join('');
      }

    } catch (err) {
      console.error("Personnel fetch error:", err);
    }
  }

  // Start polling
  connectWebSocket();
  loadEvents();
  loadPersonnelData();
  setInterval(loadEvents, 2000);
  setInterval(loadPersonnelData, 2000);
});
