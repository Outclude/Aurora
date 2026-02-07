document.addEventListener('DOMContentLoaded', () => {
    // --- UI Elements ---
    const scanBtn = document.getElementById('scanBtn');
    const scanModal = document.getElementById('scanModal');
    const closeBtn = scanModal ? scanModal.querySelector('.close-btn') : null;
    const refreshScanBtn = document.getElementById('refreshScanBtn');
    const deviceList = document.getElementById('deviceList');
    const scanLoading = document.getElementById('scanLoading');

    // Chat Elements
    const chatContainer = document.getElementById('chatContainer');
    // const connectedDeviceName = document.getElementById('connectedDeviceName'); // Removed
    const disconnectBtn = document.getElementById('disconnectBtn');
    const messageLog = document.getElementById('messageLog');
    const messageInput = document.getElementById('messageInput');
    const sendBtn = document.getElementById('sendBtn');
    const connectionStatusEl = document.querySelector('.connection-status');
    const statusText = document.getElementById('statusText'); // Added

    // Running Controls
    const runningControls = document.getElementById('runningControls');
    const cadenceInput = document.getElementById('cadenceInput');
    const paceMinInput = document.getElementById('paceMinInput');
    const paceSecInput = document.getElementById('paceSecInput');
    const syncSettingsBtn = document.getElementById('syncSettingsBtn');
    const resultTime = document.getElementById('resultTime');
    const resultDistance = document.getElementById('resultDistance');

    let pollInterval = null;
    let isConnected = false;

    // Check initial connection status
    async function checkInitialStatus() {
        try {
            const response = await fetch('/api/status');
            const status = await response.json();
            
            if (status.connected) {
                console.log('Restoring connection session:', status);
                isConnected = true;
                if (statusText) statusText.textContent = status.name || '已连接设备';
                updateConnectionUI(true);
                appendSystemMessage('已恢复连接。');
                startPolling();
            }
        } catch (error) {
            console.error('Failed to check initial status:', error);
        }
    }
    
    // Call it on load
    checkInitialStatus();

    // --- Modal Logic ---
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

    // --- BLE Scanning ---
    async function startScan() {
        if (!deviceList || !scanLoading) return;
        
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
            li.addEventListener('click', () => connectToDevice(device));
            deviceList.appendChild(li);
        });
    }

    // --- Connection Management ---
    async function connectToDevice(device) {
        closeModal();
        
        // Show connecting state
        if (chatContainer) chatContainer.style.display = 'flex';
        if (statusText) statusText.textContent = `连接中...`;
        appendSystemMessage(`正在连接到 ${device.name} (${device.address})...`);

        try {
            const response = await fetch('/api/connect', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ address: device.address, name: device.name })
            });
            const result = await response.json();

            if (result.status === 'connected') {
                isConnected = true;
                if (statusText) statusText.textContent = device.name;
                updateConnectionUI(true);
                appendSystemMessage('已连接。');
                startPolling();
            } else {
                throw new Error(result.error || '连接失败');
            }
        } catch (error) {
            console.error('Connection error:', error);
            appendSystemMessage(`连接错误: ${error.message}`);
            if (statusText) statusText.textContent = '连接失败';
            updateConnectionUI(false);
        }
    }

    async function disconnectDevice() {
        try {
            await fetch('/api/disconnect', { method: 'POST' });
        } catch (e) { console.error(e); }
        
        isConnected = false;
        updateConnectionUI(false);
        appendSystemMessage('已断开连接。');
        stopPolling();
        if (statusText) statusText.textContent = '未连接';
        // Keep chat window open but indicate disconnection, or maybe just update UI
    }

    function updateConnectionUI(connected) {
        if (connectionStatusEl) {
            if (connected) {
                connectionStatusEl.classList.add('connected');
                if (scanBtn) scanBtn.style.display = 'none';
                if (disconnectBtn) disconnectBtn.style.display = 'flex';
                if (runningControls) runningControls.style.display = 'flex';
                if (chatContainer) chatContainer.style.display = 'flex';
            } else {
                connectionStatusEl.classList.remove('connected');
                if (scanBtn) scanBtn.style.display = 'flex';
                if (disconnectBtn) disconnectBtn.style.display = 'none';
                if (runningControls) runningControls.style.display = 'none';
                // chatContainer logic: maybe keep it but disable? Or hide? 
                // User said "连上ble设备后...显示". Implies hidden otherwise.
                // But previous code kept it open. Let's hide it on disconnect to be clean, 
                // or just leave it as is if I want to show history.
                // For now, I will hide it to match "show after connected".
                if (chatContainer) chatContainer.style.display = 'none';
            }
        }
    }

    // --- Messaging ---
    async function sendMessage() {
        if (!messageInput || !messageInput.value.trim()) return;
        
        const message = messageInput.value.trim();
        
        try {
            const response = await fetch('/api/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ message: message })
            });
            const result = await response.json();
            
            if (result.status === 'sent') {
                appendMessage(message, 'sent');
                messageInput.value = '';
            } else {
                appendSystemMessage(`发送失败: ${result.error}`);
            }
        } catch (error) {
            appendSystemMessage(`发送错误: ${error.message}`);
        }
    }

    function appendMessage(text, type) {
        if (!messageLog) return;
        const div = document.createElement('div');
        div.className = `message message-${type}`;
        div.textContent = text;
        messageLog.appendChild(div);
        messageLog.scrollTop = messageLog.scrollHeight;
    }

    function appendSystemMessage(text) {
        if (!messageLog) return;
        const div = document.createElement('div');
        div.className = 'message system-message';
        div.textContent = text;
        messageLog.appendChild(div);
        messageLog.scrollTop = messageLog.scrollHeight;
    }

    // --- Polling ---
    function startPolling() {
        if (pollInterval) clearInterval(pollInterval);
        pollInterval = setInterval(async () => {
            if (!isConnected) return;
            
            // Poll messages
            try {
                const msgRes = await fetch('/api/messages');
                const messages = await msgRes.json();
                if (messages && messages.length > 0) {
                    messages.forEach(msg => {
                        // Handle both string messages (legacy/simple) and object messages
                        const content = typeof msg === 'object' && msg.content ? msg.content : msg;

                        // Parse JSON for result display
                        try {
                            const data = JSON.parse(content);
                            if (data && data.type === 2) {
                                if (data.time !== undefined && resultTime) {
                                    const totalSeconds = parseInt(data.time, 10);
                                    if (!isNaN(totalSeconds)) {
                                        const minutes = Math.floor(totalSeconds / 60);
                                        const seconds = totalSeconds % 60;
                                        resultTime.value = `${minutes}'${seconds}''`;
                                    } else {
                                        resultTime.value = data.time;
                                    }
                                }
                                if (data.distance !== undefined && resultDistance) {
                                    const meters = parseFloat(data.distance);
                                    if (!isNaN(meters)) {
                                        const km = (meters / 1000).toFixed(2);
                                        resultDistance.value = km;
                                    } else {
                                        resultDistance.value = data.distance;
                                    }
                                }
                            }
                        } catch (e) {
                            // Ignore non-JSON messages
                        }

                        appendMessage(content, 'received');
                    });
                }
            } catch (e) { console.error('Poll messages error:', e); }

            // Poll status
            try {
                const statusRes = await fetch('/api/status');
                const status = await statusRes.json();
                if (!status.connected && isConnected) {
                    // Detect disconnection
                    isConnected = false;
                    updateConnectionUI(false);
                    appendSystemMessage('连接意外断开');
                    stopPolling();
                }
            } catch (e) { console.error('Poll status error:', e); }

        }, 1000);
    }

    function stopPolling() {
        if (pollInterval) clearInterval(pollInterval);
        pollInterval = null;
    }

    // --- Settings ---
    async function sendSettings() {
        if (!cadenceInput || !paceMinInput || !paceSecInput) return;

        const cadence = parseInt(cadenceInput.value, 10) || 0;
        const paceMin = parseInt(paceMinInput.value, 10) || 0;
        const paceSec = parseInt(paceSecInput.value, 10) || 0;

        const payload = {
            type: 1,
            cadence: cadence,
            pace_min: paceMin,
            pace_sec: paceSec
        };
        
        const message = JSON.stringify(payload);
        
        try {
            const response = await fetch('/api/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ message: message })
            });
            const result = await response.json();
            
            if (result.status === 'sent') {
                appendMessage(message, 'sent');
                appendSystemMessage('设置已同步');
            } else {
                appendSystemMessage(`同步失败: ${result.error}`);
            }
        } catch (error) {
            appendSystemMessage(`同步错误: ${error.message}`);
        }
    }

    // --- Event Listeners ---
    if (scanBtn) scanBtn.addEventListener('click', openModal);
    if (closeBtn) closeBtn.addEventListener('click', closeModal);
    if (refreshScanBtn) refreshScanBtn.addEventListener('click', startScan);
    
    if (disconnectBtn) disconnectBtn.addEventListener('click', disconnectDevice);
    if (sendBtn) sendBtn.addEventListener('click', sendMessage);
    if (syncSettingsBtn) syncSettingsBtn.addEventListener('click', sendSettings);
    if (messageInput) {
        messageInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') sendMessage();
        });
    }

    // Input Validation
    function validateInput(input, min, max) {
        input.addEventListener('change', () => {
            let val = parseInt(input.value, 10);
            if (isNaN(val)) val = min;
            if (val < min) val = min;
            if (max !== undefined && val > max) val = max;
            input.value = val;
        });
    }

    if (cadenceInput) validateInput(cadenceInput, 1);
    
    // Pace validation: allow 0, but check combined > 0
    function validatePace() {
        let min = parseInt(paceMinInput.value, 10) || 0;
        let sec = parseInt(paceSecInput.value, 10) || 0;
        
        // Basic range check first
        if (min < 0) { min = 0; paceMinInput.value = 0; }
        if (sec < 0) { sec = 0; paceSecInput.value = 0; }
        if (sec > 59) { sec = 59; paceSecInput.value = 59; }

        // Combined check
        if (min === 0 && sec === 0) {
            // If both are 0, force seconds to 1 (or revert to previous valid state if we tracked it, 
            // but for simplicity, force seconds to 1)
            paceSecInput.value = 1;
        }
    }

    if (paceMinInput) {
        paceMinInput.addEventListener('change', validatePace);
    }
    if (paceSecInput) {
        paceSecInput.addEventListener('change', validatePace);
    }

    window.addEventListener('click', (event) => {
        if (event.target === scanModal) closeModal();
    });
});
