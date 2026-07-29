/**
 * KeyboardControlUI - WebSocket keyboard control for one selected arm.
 *
 * The backend owns the control loop. These buttons only select the
 * left_keyboard/right_keyboard control mode and forward key events.
 */

const KEYBOARD_BUTTONS = {
    left: 'toggle-left-keyboard',
    right: 'toggle-right-keyboard'
};

const KEYBOARD_MODES = {
    left: 'left_keyboard',
    right: 'right_keyboard'
};

const KEYBOARD_LABELS = {
    left: 'Left Keyboard',
    right: 'Right Keyboard'
};

const MOTION_KEYS = new Set([
    'w', 's', 'a', 'd', 'q', 'e',
    'i', 'k', 'j', 'l', 'u', 'o',
    'z', 'x'
]);

const KEYBOARD_PANEL_ID = 'floating-keyboard-panel';

export class KeyboardControlUI {
    constructor(robotConnection, panelManager = null) {
        this.robotConnection = robotConnection;
        this.panelManager = panelManager;
        this.enabled = false;
        this.currentMode = 'position';
        this.buttons = {};
        this.panel = null;
        this.pressedKeys = new Set();
        this.repeatTimers = new Map();
        this.documentHandlersBound = false;
    }

    init() {
        this.panel = document.getElementById(KEYBOARD_PANEL_ID);
        this.bindButtons();
        this.bindPanelClose();
        this.bindKeyboardEvents();
        this.updateStatusDisplay();
        this.updateButtonState();
    }

    setEnabled(enabled) {
        this.enabled = Boolean(enabled);
        if (!this.enabled) this.releasePressedKeys();
        this.updateStatusDisplay();
        this.updateButtonState();
    }

    setMode(mode) {
        this.currentMode = mode || 'position';
        if (!this.isKeyboardMode()) this.releasePressedKeys();
        this.updateStatusDisplay();
        this.updateButtonState();
    }

    bindButtons() {
        Object.entries(KEYBOARD_BUTTONS).forEach(([side, buttonId]) => {
            const button = document.getElementById(buttonId);
            if (!button) return;

            this.buttons[side] = button;
            button.dataset.arm = side;
            button.addEventListener('click', () => {
                if (!this.enabled || !this.robotConnection.isConnected()) return;
                this.showPanel(button);
                this.robotConnection.setMode(KEYBOARD_MODES[side]);
                this.setMode(KEYBOARD_MODES[side]);
            });
        });
    }

    bindPanelClose() {
        const closeButton = this.panel?.querySelector('.panel-close-btn');
        closeButton?.addEventListener('click', () => this.hidePanel());
    }

    bindKeyboardEvents() {
        if (this.documentHandlersBound) return;
        this.documentHandlersBound = true;

        document.addEventListener('keydown', (event) => {
            if (!this.enabled || !this.isKeyboardMode()) return;
            if (event.target instanceof HTMLInputElement ||
                event.target instanceof HTMLTextAreaElement ||
                event.target instanceof HTMLSelectElement ||
                event.target.isContentEditable) {
                return;
            }

            const key = event.key.toLowerCase();
            if (key === 'r') {
                if (!this.pressedKeys.has(key)) {
                    event.preventDefault();
                    this.pressedKeys.add(key);
                    this.robotConnection.sendCommand('home');
                }
                return;
            }

            if (!MOTION_KEYS.has(key)) return;
            event.preventDefault();
            if (this.pressedKeys.has(key)) return;
            this.pressedKeys.add(key);
            this.robotConnection.sendKeyDown(key);
            this.startKeyRepeat(key);
        });

        document.addEventListener('keyup', (event) => {
            const key = event.key.toLowerCase();
            if (!this.pressedKeys.has(key)) return;
            event.preventDefault();
            this.pressedKeys.delete(key);
            this.stopKeyRepeat(key);
            if (key !== 'r') this.robotConnection.sendKeyUp(key);
        });

        window.addEventListener('blur', () => this.releasePressedKeys());
    }

    releasePressedKeys() {
        this.pressedKeys.forEach((key) => {
            this.stopKeyRepeat(key);
            if (key !== 'r') this.robotConnection.sendKeyUp(key);
        });
        this.pressedKeys.clear();
    }

    startKeyRepeat(key) {
        this.stopKeyRepeat(key);
        const timer = window.setInterval(() => {
            if (!this.enabled || !this.isKeyboardMode() ||
                !this.pressedKeys.has(key)) {
                this.stopKeyRepeat(key);
                return;
            }
            this.robotConnection.sendKeyDown(key);
        }, 80);
        this.repeatTimers.set(key, timer);
    }

    stopKeyRepeat(key) {
        const timer = this.repeatTimers.get(key);
        if (timer !== undefined) {
            window.clearInterval(timer);
            this.repeatTimers.delete(key);
        }
    }

    isKeyboardMode() {
        return this.currentMode === KEYBOARD_MODES.left ||
            this.currentMode === KEYBOARD_MODES.right;
    }

    getActiveSide() {
        return Object.entries(KEYBOARD_MODES)
            .find(([, mode]) => mode === this.currentMode)?.[0] || null;
    }

    showPanel(anchorButton = null) {
        if (!this.panel) return;

        if (this.panelManager) {
            this.panelManager.showPanel(KEYBOARD_PANEL_ID, 'flex', {
                anchorEl: anchorButton,
                align: 'right',
                offsetX: 40
            });
        } else {
            this.panel.style.display = 'flex';
        }
    }

    hidePanel() {
        if (!this.panel) return;

        if (this.panelManager) {
            this.panelManager.hidePanel(KEYBOARD_PANEL_ID);
        } else {
            this.panel.style.display = 'none';
        }
    }

    updateButtonState() {
        const activeSide = this.getActiveSide();
        Object.entries(this.buttons).forEach(([side, button]) => {
            const isActive = side === activeSide;
            button.disabled = !this.enabled;
            button.classList.toggle('active', isActive);
            button.setAttribute('aria-pressed', isActive ? 'true' : 'false');
            button.title = this.enabled
                ? `Select ${KEYBOARD_LABELS[side]}`
                : 'Connect to the robot first';
        });
    }

    updateStatusDisplay() {
        const statusDot = this.panel?.querySelector('.kb-status-dot');
        const statusText = this.panel?.querySelector('.kb-status-text');
        const activeSide = this.getActiveSide();

        if (statusDot) {
            statusDot.className = `kb-status-dot ${
                !this.enabled ? 'inactive' :
                    activeSide ? 'active' : 'ready'
            }`;
        }

        if (!statusText) return;
        if (!this.enabled) {
            statusText.textContent = 'Not connected';
        } else if (activeSide) {
            statusText.textContent = `${KEYBOARD_LABELS[activeSide]} active`;
        } else {
            statusText.textContent = 'Select an arm keyboard mode';
        }
    }
}
