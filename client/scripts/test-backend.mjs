import { spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const local = path.join(root, 'local/cache/zephyrproject/.venv/Scripts/python.exe');
const python = process.env.ARBATOS_PYTHON || (existsSync(local) ? local : 'python');
const result = spawnSync(python, ['-X', 'utf8', '-m', 'unittest', 'discover', '-s', 'client/backend/tests', '-v'], {
    cwd: root, stdio: 'inherit', windowsHide: true,
});
if (result.error) console.error(result.error.message);
process.exit(result.status ?? 1);
