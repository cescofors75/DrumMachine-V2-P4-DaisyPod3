var bpmEl = document.getElementById('bpm');
var statusEl = document.getElementById('status');

function setStatus(text) {
    statusEl.textContent = text;
}

function onHrmChange(hrmInfo) {
    if (hrmInfo.heartRate > 0) {
        bpmEl.textContent = hrmInfo.heartRate;
        setStatus('HRM activo');
    } else {
        setStatus('sensor sin lectura (ajusta el reloj en la muneca)');
    }
}

function onHrmError(error) {
    setStatus('error HRM: ' + error.message);
}

function startHrm() {
    if (!tizen.humanactivitymonitor.isSupported('HRM')) {
        setStatus('HRM no soportado en este dispositivo');
        return;
    }
    try {
        tizen.humanactivitymonitor.start('HRM', onHrmChange);
        setStatus('esperando lectura...');
    } catch (e) {
        onHrmError(e);
    }
}

document.addEventListener('tizenhwkey', function (e) {
    if (e.keyName === 'back') {
        try {
            tizen.application.getCurrentApplication().exit();
        } catch (err) {
            // ignore
        }
    }
});

window.onload = function () {
    if (window.tizen === undefined) {
        setStatus('tizen API no disponible (no es un dispositivo/emulador Tizen)');
        return;
    }
    startHrm();
};
