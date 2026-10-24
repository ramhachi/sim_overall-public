/* ===== FSAE Dashboard — JavaScript ===== */

// Plotly dark theme defaults
const PLOT_LAYOUT = {
    paper_bgcolor: '#1a1d27',
    plot_bgcolor: '#1a1d27',
    font: { color: '#e0e0e0', size: 12 },
    margin: { t: 40, b: 50, l: 60, r: 30 },
    xaxis: { gridcolor: '#2a2d3a', zerolinecolor: '#3a3d4a' },
    yaxis: { gridcolor: '#2a2d3a', zerolinecolor: '#3a3d4a' },
};
const PLOT_CONFIG = { responsive: true, displayModeBar: true, displaylogo: false };

// ===== Navigation =====
function goPage(name) {
    document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('#sidebar a').forEach(a => a.classList.remove('active'));
    const page = document.getElementById('page-' + name);
    if (page) page.classList.add('active');
    const link = document.querySelector(`#sidebar a[data-page="${name}"]`);
    if (link) link.classList.add('active');
    // lazy-load data for the page
    if (name === 'engine') loadEngineData();
    if (name === 'vehicle') loadVehicleParams();
    if (name === 'tire') loadTireData();
}

document.querySelectorAll('#sidebar a[data-page]').forEach(a => {
    a.addEventListener('click', e => {
        e.preventDefault();
        goPage(a.dataset.page);
    });
});

// ===== Language Switch =====
function switchLang(lang) {
    window.location.href = '/?lang=' + lang;
}

// ===== Engine Page =====
async function loadEngineData() {
    try {
        const resp = await fetch('/api/engine');
        const data = await resp.json();
        const eng = data.engine;
        const curves = data.curves;

        // Info cards
        const info = document.getElementById('engineInfo');
        info.innerHTML = `
            <div class="info-card">
                <div class="label">${T.engine.displacement}</div>
                <div class="value">${eng.displacement_cc} cc</div>
            </div>
            <div class="info-card">
                <div class="label">${T.engine.configuration}</div>
                <div class="value">${eng.configuration}</div>
            </div>
            <div class="info-card">
                <div class="label">${T.engine.max_power}</div>
                <div class="value">${eng.max_power_kW} kW</div>
            </div>
            <div class="info-card">
                <div class="label">${T.engine.max_torque}</div>
                <div class="value">${eng.max_torque_Nm} Nm</div>
            </div>
            <div class="info-card">
                <div class="label">${T.engine.idle_rpm}</div>
                <div class="value">${eng.idle_rpm}</div>
            </div>
            <div class="info-card">
                <div class="label">${T.engine.redline_rpm}</div>
                <div class="value">${eng.redline_rpm}</div>
            </div>
        `;

        // Torque + Power dual-axis chart
        const traceTorque = {
            x: curves.rpm_points,
            y: curves.torque_Nm,
            name: T.engine.torque_label,
            type: 'scatter',
            mode: 'lines+markers',
            line: { color: '#4f8cff', width: 3 },
            marker: { size: 5 },
        };
        const tracePower = {
            x: curves.rpm_points,
            y: curves.power_kW,
            name: T.engine.power_label,
            type: 'scatter',
            mode: 'lines+markers',
            line: { color: '#ff5c5c', width: 3, dash: 'dot' },
            marker: { size: 5 },
            yaxis: 'y2',
        };

        const layout = {
            ...PLOT_LAYOUT,
            title: T.engine.title,
            xaxis: { ...PLOT_LAYOUT.xaxis, title: T.engine.rpm_label },
            yaxis: { ...PLOT_LAYOUT.yaxis, title: T.engine.torque_label, side: 'left' },
            yaxis2: {
                title: T.engine.power_label,
                overlaying: 'y',
                side: 'right',
                gridcolor: '#2a2d3a',
                font: { color: '#ff5c5c' },
            },
            legend: { x: 0.02, y: 0.98, bgcolor: 'rgba(26,29,39,0.8)' },
            shapes: [
                { // Redline
                    type: 'line', x0: eng.redline_rpm, x1: eng.redline_rpm,
                    y0: 0, y1: 1, yref: 'paper',
                    line: { color: '#ff5c5c', width: 2, dash: 'dash' },
                },
                { // Peak torque RPM
                    type: 'line', x0: eng.max_torque_rpm, x1: eng.max_torque_rpm,
                    y0: 0, y1: 1, yref: 'paper',
                    line: { color: '#4f8cff', width: 1, dash: 'dot' },
                },
            ],
            annotations: [
                {
                    x: eng.max_torque_rpm, y: eng.max_torque_Nm,
                    text: `${T.engine.peak_torque}: ${eng.max_torque_Nm} Nm`,
                    showarrow: true, arrowhead: 2, ax: 40, ay: -30,
                    font: { color: '#4f8cff', size: 11 },
                },
            ],
        };

        Plotly.newPlot('engineChart', [traceTorque, tracePower], layout, PLOT_CONFIG);
    } catch (err) {
        console.error('Engine load error:', err);
    }
}

// ===== Vehicle Parameters Page =====
let vehicleParams = {};
const SECTION_LABELS = T.vehicle ? T.vehicle.sections : {};

async function loadVehicleParams() {
    try {
        const resp = await fetch('/api/params');
        vehicleParams = await resp.json();
        renderParamForm(vehicleParams);
    } catch (err) {
        console.error('Params load error:', err);
    }
}

function renderParamForm(params) {
    const container = document.getElementById('vehicleForm');
    container.innerHTML = '';
    for (const [section, values] of Object.entries(params)) {
        const label = SECTION_LABELS[section] || section;
        const div = document.createElement('div');
        div.className = 'param-section';
        let html = `<h3>📐 ${label}</h3><div class="param-grid">`;
        if (typeof values === 'object' && !Array.isArray(values)) {
            for (const [key, val] of Object.entries(values)) {
                if (Array.isArray(val)) {
                    html += `<div class="param-item">
                        <label>${key}</label>
                        <input type="text" data-section="${section}" data-key="${key}"
                               value="${val.join(', ')}" data-array="true">
                    </div>`;
                } else {
                    html += `<div class="param-item">
                        <label>${key}</label>
                        <input type="number" step="any" data-section="${section}" data-key="${key}"
                               value="${val}">
                    </div>`;
                }
            }
        }
        html += '</div>';
        div.innerHTML = html;
        container.appendChild(div);
    }
}

async function saveParams() {
    // collect from form
    document.querySelectorAll('#vehicleForm input').forEach(inp => {
        const sec = inp.dataset.section;
        const key = inp.dataset.key;
        if (sec && key && vehicleParams[sec] !== undefined) {
            if (inp.dataset.array === 'true') {
                vehicleParams[sec][key] = inp.value.split(',').map(v => parseFloat(v.trim()));
            } else {
                vehicleParams[sec][key] = parseFloat(inp.value);
            }
        }
    });
    try {
        await fetch('/api/params', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(vehicleParams),
        });
        showMsg('vehicleMsg', T.vehicle.saved_msg, 'success');
    } catch (err) {
        showMsg('vehicleMsg', T.common.error, 'error');
    }
}

async function resetParams() {
    try {
        const resp = await fetch('/api/params/reset', { method: 'POST' });
        const data = await resp.json();
        vehicleParams = data.params;
        renderParamForm(vehicleParams);
        showMsg('vehicleMsg', T.vehicle.reset_msg, 'success');
    } catch (err) {
        showMsg('vehicleMsg', T.common.error, 'error');
    }
}

function showMsg(id, text, type) {
    const el = document.getElementById(id);
    el.className = 'msg ' + type;
    el.textContent = text;
    setTimeout(() => { el.textContent = ''; el.className = 'msg'; }, 3000);
}

// ===== Simulation Page =====
async function runSim() {
    const btn = document.getElementById('runSimBtn');
    const box = document.getElementById('simResults');
    btn.disabled = true;
    btn.textContent = T.simulation.running;
    box.innerHTML = '<p>' + T.common.loading + '</p>';

    try {
        const scenario = document.getElementById('simScenario').value;
        const resp = await fetch('/api/simulate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ scenario }),
        });
        const data = await resp.json();
        if (data.success) {
            box.innerHTML = '<pre>' + escapeHtml(data.output || 'Simulation completed.') + '</pre>';
        } else {
            box.innerHTML = '<pre class="error">' + escapeHtml(data.error || data.stderr || 'Unknown error') + '</pre>';
        }
    } catch (err) {
        box.innerHTML = '<pre class="error">' + escapeHtml(err.message) + '</pre>';
    } finally {
        btn.disabled = false;
        btn.textContent = T.simulation.run;
    }
}

// ===== Regulation Check =====
async function runRegCheck() {
    try {
        const resp = await fetch('/api/regulation');
        const data = await resp.json();

        const overall = document.getElementById('regOverall');
        if (data.overall) {
            overall.className = 'reg-overall pass';
            overall.textContent = '✅ ' + (T.regulation.status_pass || 'ALL PASS');
        } else {
            overall.className = 'reg-overall fail';
            overall.textContent = '❌ ' + (T.regulation.status_fail || 'ISSUES FOUND');
        }

        const tbody = document.querySelector('#regTable tbody');
        tbody.innerHTML = '';
        for (const c of data.checks) {
            const cls = c.passed ? 'pass' : 'fail';
            const label = c.passed ? (T.regulation.status_pass || 'PASS') : (T.regulation.status_fail || 'FAIL');
            tbody.innerHTML += `<tr>
                <td>${c.id}</td>
                <td>${c.category}</td>
                <td>${c.rule}</td>
                <td>${c.value}</td>
                <td>${c.required}</td>
                <td><span class="badge ${cls}">${label}</span></td>
            </tr>`;
        }
    } catch (err) {
        console.error('Regulation check error:', err);
    }
}

// ===== Tire Page =====
async function loadTireData() {
    const Fz = document.getElementById('tireFz').value || 1000;
    try {
        const resp = await fetch(`/api/tire?Fz=${Fz}`);
        const data = await resp.json();

        // Fx vs slip ratio
        Plotly.newPlot('fxChart', [{
            x: data.slip_ratios,
            y: data.fx_values,
            type: 'scatter',
            mode: 'lines',
            line: { color: '#4f8cff', width: 2 },
            name: 'Fx',
        }], {
            ...PLOT_LAYOUT,
            title: T.tire.fx_vs_slip,
            xaxis: { ...PLOT_LAYOUT.xaxis, title: 'Slip Ratio [-]' },
            yaxis: { ...PLOT_LAYOUT.yaxis, title: 'Fx [N]' },
        }, PLOT_CONFIG);

        // Fy vs slip angle
        Plotly.newPlot('fyChart', [{
            x: data.slip_angles_deg,
            y: data.fy_values,
            type: 'scatter',
            mode: 'lines',
            line: { color: '#ff5c5c', width: 2 },
            name: 'Fy',
        }], {
            ...PLOT_LAYOUT,
            title: T.tire.fy_vs_alpha,
            xaxis: { ...PLOT_LAYOUT.xaxis, title: 'Slip Angle [deg]' },
            yaxis: { ...PLOT_LAYOUT.yaxis, title: 'Fy [N]' },
        }, PLOT_CONFIG);
    } catch (err) {
        console.error('Tire load error:', err);
    }
}

// ===== Export Page =====
async function previewExport() {
    const fmt = document.getElementById('exportFmt').value;
    const eng = document.getElementById('exportEngine').checked;
    const tire = document.getElementById('exportTire').checked;
    try {
        const resp = await fetch(`/api/export?fmt=${fmt}&include_engine=${eng}&include_tire=${tire}`);
        const data = await resp.json();
        document.getElementById('exportPreview').textContent = data.content;
    } catch (err) {
        document.getElementById('exportPreview').textContent = 'Error: ' + err.message;
    }
}

function downloadExport() {
    const content = document.getElementById('exportPreview').textContent;
    if (!content) { previewExport(); return; }
    const fmt = document.getElementById('exportFmt').value;
    const ext = fmt === 'yaml' ? 'yaml' : fmt === 'cpp_header' ? 'hpp' : 'json';
    const blob = new Blob([content], { type: 'text/plain' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `fsae_params.${ext}`;
    a.click();
}

// ===== Utility =====
function escapeHtml(str) {
    const div = document.createElement('div');
    div.textContent = str;
    return div.innerHTML;
}
