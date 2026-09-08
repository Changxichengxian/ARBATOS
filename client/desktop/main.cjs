const { app, BrowserWindow, dialog, session } = require('electron');
const { spawn } = require('node:child_process');
const { existsSync, mkdirSync, writeFileSync } = require('node:fs');
const path = require('node:path');
const readline = require('node:readline');

const root = path.resolve(__dirname, '../..');
const smokeDirectory = process.env.ARBATOS_SMOKE_OUTPUT;
let backend;
let window;
let stopping = false;
let backendError = '';
const smokeRequests = [];

app.setName('ARBATOS');
app.setPath('userData', path.join(root, 'local/cache/client/desktop'));
app.enableSandbox();
const primaryInstance = app.requestSingleInstanceLock();
if (!primaryInstance) app.quit();
app.on('second-instance', () => {
    if (window && !smokeDirectory) { window.show(); window.focus(); }
});

function startBackend() {
    return new Promise((resolve, reject) => {
        const localPython = path.join(root, 'local/cache/zephyrproject/.venv/Scripts/python.exe');
        const python = process.env.ARBATOS_PYTHON || (existsSync(localPython) ? localPython : 'python');
        backend = spawn(python, ['-X', 'utf8', path.join(root, 'client/backend/server.py'), '--workspace', root, '--parent-pipe'], {
            cwd: root, windowsHide: true, stdio: ['pipe', 'pipe', 'pipe'],
            env: { ...process.env, PYTHONUNBUFFERED: '1' },
        });
        const deadline = setTimeout(() => reject(new Error('本机服务启动超时。')), 30000);
        backend.stderr.setEncoding('utf8');
        backend.stderr.on('data', chunk => { backendError = (backendError + chunk).slice(-12000); });
        backend.once('error', error => { clearTimeout(deadline); reject(error); });
        backend.once('exit', code => {
            clearTimeout(deadline);
            if (!stopping) {
                reject(new Error(backendError || `本机服务退出：${code}`));
                if (window && !window.isDestroyed()) {
                    dialog.showErrorBox('本机服务已停止', backendError || '关闭窗口后重新启动客户端。');
                    window.destroy();
                    app.quit();
                }
            }
        });
        const lines = readline.createInterface({ input: backend.stdout });
        lines.once('line', line => {
            try {
                const connection = JSON.parse(line);
                const url = new URL(connection.url);
                if (url.protocol !== 'http:' || url.hostname !== '127.0.0.1' || !url.port || typeof connection.token !== 'string') {
                    throw new Error('本机服务返回了无效地址。');
                }
                clearTimeout(deadline);
                resolve(connection);
            } catch (error) { clearTimeout(deadline); reject(error); }
        });
    });
}

async function captureSmoke() {
    const until = Date.now() + 20000;
    let body = '';
    do {
        body = await window.webContents.executeJavaScript('document.body.innerText');
        if (body.includes('HERO-M') && body.includes('配置校验\n通过')) break;
        await new Promise(resolve => setTimeout(resolve, 250));
    } while (Date.now() < until);
    mkdirSync(smokeDirectory, { recursive: true });
    // 显示但不抢焦点，让合成器绘制最新数据后再保存实际窗口截图。
    window.showInactive();
    await window.webContents.executeJavaScript('new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)))');
    const screenshot = await window.webContents.capturePage();
    writeFileSync(path.join(smokeDirectory, 'desktop.png'), screenshot.toPNG());
    writeFileSync(path.join(smokeDirectory, 'desktop.json'), JSON.stringify({ title: window.getTitle(), loaded: body.includes('HERO-M') && body.includes('配置校验\n通过'), text: body, requests: smokeRequests, backendLog: backendError }, null, 2));
    app.quit();
}

app.whenReady().then(async () => {
    if (!primaryInstance) return;
    if (!existsSync(path.join(root, 'client/dist/index.html'))) throw new Error('尚未构建客户端，请运行根目录“打开客户端.cmd”。');
    const connection = await startBackend();
    // 本机工具只访问自己的回环服务，不继承系统的外网代理。
    await session.defaultSession.setProxy({ mode: 'direct' });
    if (smokeDirectory) {
        session.defaultSession.webRequest.onBeforeRequest((details, callback) => {
            smokeRequests.push({ starting: new URL(details.url).pathname });
            callback({});
        });
        session.defaultSession.webRequest.onCompleted(details => {
            smokeRequests.push({ path: new URL(details.url).pathname, status: details.statusCode });
        });
        session.defaultSession.webRequest.onErrorOccurred(details => {
            smokeRequests.push({ path: new URL(details.url).pathname, error: details.error });
        });
    }
    session.defaultSession.setPermissionRequestHandler((_contents, permission, callback) => {
        if (smokeDirectory) smokeRequests.push({ permission });
        callback(false);
    });
    session.defaultSession.setPermissionCheckHandler(() => false);
    window = new BrowserWindow({
        title: 'ARBATOS · 机器人配置工具', width: 1420, height: 940, minWidth: 1000, minHeight: 680,
        backgroundColor: '#202329', show: false, autoHideMenuBar: true,
        webPreferences: { nodeIntegration: false, contextIsolation: true, sandbox: true, webSecurity: true, backgroundThrottling: false },
    });
    window.setMenu(null);
    if (smokeDirectory) window.webContents.on('console-message', (_event, details) => {
        smokeRequests.push({ console: details.message });
    });
    window.webContents.setWindowOpenHandler(() => ({ action: 'deny' }));
    window.webContents.on('will-navigate', (event, url) => {
        if (new URL(url).origin !== connection.url) event.preventDefault();
    });
    window.webContents.on('will-prevent-unload', event => {
        const choice = dialog.showMessageBoxSync(window, {
            type: 'question', title: '关闭 ARBATOS',
            message: '当前有未保存修改或仍在运行的任务。',
            detail: '关闭会放弃未保存修改并取消任务。如果正在烧录，中断后需要重新烧录。',
            buttons: ['返回客户端', '放弃并关闭'], defaultId: 0, cancelId: 0,
        });
        if (choice === 1) event.preventDefault();
    });
    window.on('closed', () => { window = null; app.quit(); });
    await window.loadURL(`${connection.url}/#token=${encodeURIComponent(connection.token)}`);
    if (smokeDirectory) await captureSmoke();
    else window.showInactive();
}).catch(error => {
    if (smokeDirectory) {
        mkdirSync(smokeDirectory, { recursive: true });
        writeFileSync(path.join(smokeDirectory, 'error.txt'), String(error) + '\n' + backendError);
    } else dialog.showErrorBox('ARBATOS 启动失败', String(error) + '\n\n' + backendError);
    app.quit();
});

app.on('window-all-closed', () => app.quit());
app.on('before-quit', event => {
    if (window && !window.isDestroyed()) {
        event.preventDefault();
        // 先让页面处理未保存修改；用户返回编辑时后端必须继续可用。
        window.close();
        return;
    }
    if (!backend || backend.exitCode !== null || stopping) return;
    event.preventDefault();
    stopping = true;
    // 关闭父进程管道使后端先释放串口并结束其构建进程。
    backend.stdin.end();
    const deadline = setTimeout(() => {
        // 清理超时也要结束整个构建/下载子树，避免关窗后仍占用下载器。
        if (process.platform === 'win32' && backend.pid) {
            const killer = spawn('taskkill', ['/PID', String(backend.pid), '/T', '/F'], {
                windowsHide: true, stdio: 'ignore',
            });
            killer.once('exit', () => app.quit());
            killer.once('error', () => { backend.kill(); app.quit(); });
        } else { backend.kill(); app.quit(); }
    }, 12000);
    backend.once('exit', () => { clearTimeout(deadline); app.quit(); });
});
