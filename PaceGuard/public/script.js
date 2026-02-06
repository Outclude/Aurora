document.addEventListener('DOMContentLoaded', () => {
    // DOM Elements
    const currentPaceEl = document.getElementById('currentPace');
    const targetPaceDisplay = document.getElementById('targetPaceDisplay');
    const totalDistanceEl = document.getElementById('totalDistance');
    const totalTimeEl = document.getElementById('totalTime');
    const rabbitIndicator = document.getElementById('rabbitIndicator');
    const toggleBtn = document.getElementById('toggleBtn');
    const iconPlay = document.getElementById('icon-play');
    const iconStop = document.getElementById('icon-stop');
    const btnText = document.getElementById('btnText');
    const modeOptions = document.querySelectorAll('.mode-option');

    // State
    let isRunning = false;
    let targetPaceSeconds = 300; // 5'00" per km
    let currentPaceSeconds = 0;
    
    // Simulator Instance
    let simulator = null;
    if (typeof PaceSimulator !== 'undefined') {
        simulator = new PaceSimulator();
        simulator.setUpdateCallback(onSimulationUpdate);
    }

    // Helper: Seconds to MM'SS" format
    function formatPace(seconds) {
        if (seconds === 0 || !isFinite(seconds)) return "--'--\"";
        const m = Math.floor(seconds / 60);
        const s = Math.floor(seconds % 60);
        return `${m}'${s.toString().padStart(2, '0')}"`;
    }

    // Helper: Format Duration HH:MM:SS
    function formatDuration(ms) {
        const totalSeconds = Math.floor(ms / 1000);
        const h = Math.floor(totalSeconds / 3600);
        const m = Math.floor((totalSeconds % 3600) / 60);
        const s = totalSeconds % 60;
        if (h > 0) {
            return `${h}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
        }
        return `${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    }

    // Callback from Simulator
    function onSimulationUpdate(data) {
        // Update local state for UI logic if needed
        currentPaceSeconds = data.currentPace;
        targetPaceSeconds = data.targetPace; // Sync back if simulator changes it

        // Update UI Elements
        currentPaceEl.textContent = formatPace(data.currentPace);
        if (totalDistanceEl) totalDistanceEl.textContent = data.totalDistance.toFixed(2);
        if (totalTimeEl) totalTimeEl.textContent = formatDuration(data.elapsedTime);

        updateVisualizer(data.currentPace, data.targetPace);
    }

    function updateVisualizer(currentPace, targetPace) {
        // Visualizer Logic
        // Slower (High Pace Value) -> Rabbit Moves Right (Away)
        // Faster (Low Pace Value) -> Rabbit Moves Left (Towards Center)
        
        const delta = currentPace - targetPace;
        const pxPerSecond = 1.5; // Sensitivity
        let offset = delta * pxPerSecond; 
        
        // Clamp offset
        offset = Math.max(-45, Math.min(45, offset));
        
        rabbitIndicator.style.left = `calc(50% + ${offset}%)`;
    }

    function startSimulation() {
        if (simulator) {
            simulator.setTargetPace(targetPaceSeconds);
            simulator.start();
        }
        isRunning = true;
        updateButtonStyle();
    }

    function stopSimulation() {
        if (simulator) {
            simulator.stop();
        }
        isRunning = false;
        updateButtonStyle();
        
        // Reset Visuals
        currentPaceSeconds = 0;
        rabbitIndicator.style.left = '50%';
        currentPaceEl.textContent = "--'--\"";
    }

    function updateButtonStyle() {
        if (isRunning) {
            toggleBtn.classList.add('stopping');
            btnText.textContent = "停止跟随";
            iconPlay.style.display = 'none';
            iconStop.style.display = 'block';
        } else {
            toggleBtn.classList.remove('stopping');
            btnText.textContent = "开始跟随";
            iconPlay.style.display = 'block';
            iconStop.style.display = 'none';
        }
    }

    // Event Listeners
    /* 
    toggleBtn.addEventListener('click', () => {
        if (isRunning) {
            stopSimulation();
        } else {
            startSimulation();
        }
    });

    modeOptions.forEach(opt => {
        opt.addEventListener('click', () => {
            modeOptions.forEach(o => o.classList.remove('active'));
            opt.classList.add('active');
        });
    });

    // Editable Target Pace
    targetPaceDisplay.parentElement.addEventListener('click', () => {
        const presets = [300, 270, 330, 360]; // 5'00", 4'30", 5'30", 6'00"
        const currentIndex = presets.indexOf(targetPaceSeconds);
        const nextIndex = (currentIndex + 1) % presets.length;
        
        targetPaceSeconds = presets[nextIndex];
        targetPaceDisplay.textContent = formatPace(targetPaceSeconds);
        
        // Update simulator if running
        if (simulator && isRunning) {
            simulator.setTargetPace(targetPaceSeconds);
        }
    });
    */

    // --- Device Scanning Logic ---
    const scanBtn = document.getElementById('scanBtn');
    const scanModal = document.getElementById('scanModal');
    const closeBtn = scanModal ? scanModal.querySelector('.close-btn') : null;
    const refreshScanBtn = document.getElementById('refreshScanBtn');
    const deviceList = document.getElementById('deviceList');
    const scanLoading = document.getElementById('scanLoading');

    function openModal() {
        if (scanModal) {
            scanModal.style.display = 'flex';
            startScan();
        }
    }

    function closeModal() {
        if (scanModal) {
            scanModal.style.display = 'none';
        }
    }

    async function startScan() {
        if (!deviceList || !scanLoading) return;
        
        // Clear list and show loading
        deviceList.innerHTML = '';
        scanLoading.style.display = 'flex';
        if (refreshScanBtn) refreshScanBtn.disabled = true;

        try {
            const response = await fetch('/api/scan-ble');
            if (!response.ok) throw new Error('Network response was not ok');
            const devices = await response.json();
            
            if (devices.error) {
                throw new Error(devices.error);
            }
            
            renderDevices(devices);
        } catch (error) {
            console.error('Scan failed:', error);
            deviceList.innerHTML = `<li class="device-item" style="color: var(--accent-orange); justify-content: center;">扫描失败: ${error.message}</li>`;
        } finally {
            scanLoading.style.display = 'none';
            if (refreshScanBtn) refreshScanBtn.disabled = false;
        }
    }

    function renderDevices(devices) {
        if (!devices || devices.length === 0) {
            deviceList.innerHTML = '<li class="device-item" style="justify-content: center; color: var(--text-muted);">未发现设备</li>';
            return;
        }

        devices.forEach(device => {
            const li = document.createElement('li');
            li.className = 'device-item';
            li.innerHTML = `
                <div class="device-info-text">
                    <span class="device-name">${device.name}</span>
                    <span class="device-address">${device.address}</span>
                </div>
                <span class="device-rssi">${device.rssi} dBm</span>
            `;
            // Click to select/connect (Placeholder)
            li.addEventListener('click', () => {
                alert(`选择了设备: ${device.name}\n地址: ${device.address}`);
                closeModal();
            });
            deviceList.appendChild(li);
        });
    }

    if (scanBtn) {
        scanBtn.addEventListener('click', openModal);
    }
    if (closeBtn) {
        closeBtn.addEventListener('click', closeModal);
    }
    if (refreshScanBtn) {
        refreshScanBtn.addEventListener('click', startScan);
    }

    // Close modal when clicking outside
    window.addEventListener('click', (event) => {
        if (event.target === scanModal) {
            closeModal();
        }
    });
});
