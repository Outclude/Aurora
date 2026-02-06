document.addEventListener('DOMContentLoaded', () => {
    // DOM Elements
    const currentPaceEl = document.getElementById('currentPace');
    const targetPaceDisplay = document.getElementById('targetPaceDisplay');
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
    let intervalId = null;

    // Helper: Seconds to MM'SS" format
    function formatPace(seconds) {
        const m = Math.floor(seconds / 60);
        const s = Math.floor(seconds % 60);
        return `${m}'${s.toString().padStart(2, '0')}"`;
    }

    // Helper: Parse MM'SS" to seconds (if needed, simplified here)
    
    // Simulation Logic
    function startSimulation() {
        isRunning = true;
        updateButtonStyle();
        
        // Initial current pace (start slow)
        currentPaceSeconds = targetPaceSeconds + 30; // 5'30"

        intervalId = setInterval(() => {
            // Simulate pace fluctuation
            // If we are "running", we try to stay near target, but fluctuate
            const fluctuation = (Math.random() - 0.5) * 4; // +/- 2 seconds change
            
            // Tendency to move towards target pace over time
            const diff = targetPaceSeconds - currentPaceSeconds;
            currentPaceSeconds += diff * 0.05 + fluctuation;

            updateUI();
        }, 1000);
    }

    function stopSimulation() {
        isRunning = false;
        clearInterval(intervalId);
        updateButtonStyle();
        // Reset Visuals
        currentPaceSeconds = 0;
        rabbitIndicator.style.left = '50%';
        currentPaceEl.textContent = "--'--\"";
    }

    function updateUI() {
        // Update Pace Display
        currentPaceEl.textContent = formatPace(currentPaceSeconds);

        // Update Visualizer
        // Logic: Runner Speed > Target Speed => Rabbit Closer to Center.
        // Pace (Time/Distance): Lower is Faster.
        // Delta = CurrentPace - TargetPace
        // If Current(4'50") < Target(5'00") -> Faster -> Delta is Negative (-10s).
        // If Current(5'10") > Target(5'00") -> Slower -> Delta is Positive (+10s).
        
        // Prompt: "Runner speed > Target speed -> Orange indicator (Rabbit) closer to center"
        // Interpretation: 
        // If I am Slower (Delta > 0), Rabbit moves AWAY (Forward/Right).
        // If I am Faster (Delta < 0), Rabbit moves CLOSER (Center/Left).
        // Let's map Delta to Position.
        // Center is 50%.
        // Max range: +/- 30 seconds difference?
        
        const delta = currentPaceSeconds - targetPaceSeconds;
        
        // Sensitivity factor
        const pxPerSecond = 1.5; // % per second difference
        
        // If Delta is Positive (Slower), Offset should be Positive (Away/Right).
        // If Delta is Negative (Faster), Offset should be Negative (Towards Center? No wait).
        
        // The Rabbit is the Target.
        // If I am Slower, Rabbit is Ahead (Right). Offset > 0.
        // If I am Faster, Rabbit is Behind (Left). Offset < 0.
        // Wait, "Closer to center".
        // The Rabbit IS the indicator.
        // If I am Slower, Rabbit is Ahead (Far from center).
        // If I am Faster, Rabbit is Behind (Far from center on the other side? or just closer?)
        
        // Prompt specific logic: "Runner speed > Target speed -> Orange indicator (Rabbit) closer to center (represents Runner); else away."
        // This implies the distance from center represents the "Lag" or "Lead".
        // BUT, it explicitly says "closer to center" when faster.
        // This might imply the Rabbit is *always* ahead, and running faster makes it appear closer?
        // Let's implement: Slower -> Rabbit Moves Right (Away). Faster -> Rabbit Moves Left (Towards Center).
        
        let offset = delta * pxPerSecond; 
        
        // Clamp offset to keep inside track (approx +/- 45%)
        offset = Math.max(-45, Math.min(45, offset));
        
        // Visual Position: Center (50%) + Offset
        // If Slower (+10s) -> 50 + 15 = 65% (Right/Ahead).
        // If Faster (-10s) -> 50 - 15 = 35% (Left/Behind).
        // If Matched (0s) -> 50% (Center/Overlapped).
        
        // However, prompt says "Runner speed > Target speed ... closer to center".
        // If I am super fast, I overtake. Rabbit is behind.
        // If I am matched, Rabbit is at center.
        // This works.
        
        rabbitIndicator.style.left = `calc(50% + ${offset}%)`;
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

    // Editable Target Pace (Simple toggle for prototype)
    targetPaceDisplay.parentElement.addEventListener('click', () => {
        // Cycle through a few presets for demo
        const presets = [300, 270, 330, 360]; // 5'00", 4'30", 5'30", 6'00"
        const currentIndex = presets.indexOf(targetPaceSeconds);
        const nextIndex = (currentIndex + 1) % presets.length;
        targetPaceSeconds = presets[nextIndex];
        targetPaceDisplay.textContent = formatPace(targetPaceSeconds);
    });
});
