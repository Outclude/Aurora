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
    const connectedDeviceName = document.getElementById('connectedDeviceName');
    const disconnectBtn = document.getElementById('disconnectBtn');
    const messageLog = document.getElementById('messageLog');
    const messageInput = document.getElementById('messageInput');
    const sendBtn = document.getElementById('sendBtn');
    const connectionStatusEl = document.querySelector('.connection-status');
    const connectionStatusText = connectionStatusEl ? connectionStatusEl.querySelector('span') : null;

    let pollInterval = null;
    let isConnected = false;

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
        if (connectedDeviceName) connectedDeviceName.textContent = `连接中: ${device.name}...`;
        appendSystemMessage(`正在连接到 ${device.name} (${device.address})...`);

        try {
            const response = await fetch('/api/connect', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ address: device.address })
            });
            const result = await response.json();

            if (result.status === 'connected') {
                isConnected = true;
                if (connectedDeviceName) connectedDeviceName.textContent = device.name;
                updateConnectionUI(true);
                appendSystemMessage('已连接。');
                startPolling();
            } else {
                throw new Error(result.error || '连接失败');
            }
        } catch (error) {
            console.error('Connection error:', error);
            appendSystemMessage(`连接错误: ${error.message}`);
            if (connectedDeviceName) connectedDeviceName.textContent = '连接失败';
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
        if (connectedDeviceName) connectedDeviceName.textContent = '未连接';
        // Keep chat window open but indicate disconnection, or maybe just update UI
    }

    function updateConnectionUI(connected) {
        if (connectionStatusEl) {
            if (connected) {
                connectionStatusEl.classList.add('connected');
                if (connectionStatusText) connectionStatusText.textContent = '已连接';
            } else {
                connectionStatusEl.classList.remove('connected');
                if (connectionStatusText) connectionStatusText.textContent = '未连接';
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

    // --- Event Listeners ---
    if (scanBtn) scanBtn.addEventListener('click', openModal);
    if (closeBtn) closeBtn.addEventListener('click', closeModal);
    if (refreshScanBtn) refreshScanBtn.addEventListener('click', startScan);
    
    if (disconnectBtn) disconnectBtn.addEventListener('click', disconnectDevice);
    if (sendBtn) sendBtn.addEventListener('click', sendMessage);
    if (messageInput) {
        messageInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') sendMessage();
        });
    }

    window.addEventListener('click', (event) => {
        if (event.target === scanModal) closeModal();
    });
});
