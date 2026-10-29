"""用实际固件压缩函数验证日志无损往返，可传入一份 SD 日志作数据压力检查。"""
import argparse
import ctypes as C
import importlib.util
import json
from pathlib import Path
import random
import struct
import subprocess
import sys

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--log', type=Path)
parser.add_argument('--chunk-size', type=int, default=0, help='将原始记录流重新分批后验证无损压缩')
args = parser.parse_args()
if args.chunk_size < 0 or args.chunk_size > 65535:
    parser.error('--chunk-size 应为 0 或 1～65535')
spec = importlib.util.spec_from_file_location('viewer', root/'tools/sdlog/SdLogViewer.py')
viewer = importlib.util.module_from_spec(spec); sys.modules[spec.name] = viewer; spec.loader.exec_module(viewer)
source = (root/'shared/application/services/storage/SdLog.c').read_text(encoding='utf-8')
read32 = source[source.index('static uint32_t sdlog_read_u32_le_unaligned'):].split('#endif', 1)[0]
compress = source[source.index('static uint32_t sdlog_lz4_hash32'):].split('#endif', 1)[0]
folder = root/'local/build/host-tests/sdlog-compression'
folder.mkdir(parents=True, exist_ok=True)
unit = folder/'compression.c'; dll = folder/'compression.dll'
unit.write_text('#include <stdint.h>\n#include <string.h>\n#define SDLOG_LZ4_HASH_BITS 11u\n'
                'static uint16_t sdlog_lz4_hash[1u << SDLOG_LZ4_HASH_BITS];\n'+read32+compress+
                '\nint Compress(const uint8_t *s, uint32_t n, uint8_t *d, uint32_t cap, uint32_t *out) '
                '{ return sdlog_lz4_compress_block(s,n,d,cap,out); }\n', encoding='utf-8')
subprocess.run([str(Path('C:/Users/28111/AppData/Local/Microsoft/WinGet/Links/zig.exe')),
                'cc', '-shared', '-O2', '-Wall', '-Wextra', '-Werror', str(unit), '-o', str(dll)], check=True)
lib = C.CDLL(str(dll)); lib.Compress.argtypes = [C.c_void_p,C.c_uint32,C.c_void_p,C.c_uint32,C.POINTER(C.c_uint32)]
try:
    import lz4.block
    decode = lambda data, size: lz4.block.decompress(data, uncompressed_size=size)
except ImportError:
    decode = viewer.lz4_decompress_block
count = raw_bytes = encoded_bytes = 0

def check(raw):
    global count, raw_bytes, encoded_bytes
    buf = C.create_string_buffer(len(raw)+len(raw)//255+16); size = C.c_uint32()
    assert lib.Compress(raw,len(raw),buf,len(buf),C.byref(size)) == 0
    compressed = buf.raw[:size.value]
    assert viewer.lz4_decompress_block(compressed,len(raw)) == raw
    assert decode(compressed,len(raw)) == raw
    count += 1; raw_bytes += len(raw); encoded_bytes += min(len(raw),len(compressed))

rng = random.Random(42)
for size in (1,3,4,5,11,12,16,255,512,1024,4096,16384):
    for raw in (bytes(size), rng.randbytes(size), (b'abcdef012345'*((size+11)//12))[:size]):
        check(raw)
if args.log:
    pending = bytearray()
    with args.log.open('rb') as stream:
        header = stream.read(16); hsize = struct.unpack_from('<H',header,4)[0]; stream.read(hsize-16)
        while header := stream.read(20):
            magic,flags,hsize,rawlen,length,crc = struct.unpack('<IHHIII',header)
            assert magic == viewer.SDLOG_BLOCK_MAGIC
            stream.read(hsize-20); data=stream.read(length)
            raw=viewer.lz4_decompress_block(data,rawlen) if flags & 1 else data
            if args.chunk_size:
                pending.extend(raw)
                while len(pending) >= args.chunk_size:
                    check(bytes(pending[:args.chunk_size]))
                    del pending[:args.chunk_size]
            else:
                check(raw)
    if pending:
        check(bytes(pending))
print(json.dumps({'blocks_checked':count,'raw_bytes':raw_bytes,'stored_payload_bytes':encoded_bytes,
                  'ratio':encoded_bytes/raw_bytes,'lossless_roundtrip':True}))
