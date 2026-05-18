FFmpeg mini build for ARBATOS MP3 to U8 conversion

Upstream:
- FFmpeg n8.0.1
- Commit: 894da5ca7d742e4429ffb2af534fcda0103ef593
- Source: https://git.ffmpeg.org/ffmpeg.git

Build target:
- Windows x86_64 executable
- Cross-compiled in WSL Ubuntu with x86_64-w64-mingw32-gcc
- Static executable
- Size after strip: 1,098,752 bytes
- License reported by this build: LGPL version 2.1 or later

Purpose:
- Decode MP3 input files.
- Convert to mono 12 kHz raw unsigned 8-bit PCM.
- Keep the ARBATOS buzzer audio filter chain:
  acompressor=threshold=-18dB:ratio=2:attack=5:release=120:makeup=6,alimiter=limit=0.95

Configure command:

./configure \
  --target-os=mingw32 \
  --arch=x86_64 \
  --cross-prefix=x86_64-w64-mingw32- \
  --pkg-config=/usr/bin/false \
  --enable-cross-compile \
  --disable-everything \
  --disable-autodetect \
  --disable-doc \
  --disable-debug \
  --disable-network \
  --disable-x86asm \
  --disable-avdevice \
  --disable-swscale \
  --enable-small \
  --enable-static \
  --disable-shared \
  --enable-ffmpeg \
  --disable-ffplay \
  --disable-ffprobe \
  --enable-avcodec \
  --enable-avformat \
  --enable-avfilter \
  --enable-swresample \
  --enable-protocol=file \
  --enable-demuxer=mp3 \
  --enable-parser=mpegaudio \
  --enable-decoder=mp3 \
  --enable-decoder=mp3float \
  --enable-encoder=pcm_u8 \
  --enable-muxer=pcm_u8 \
  --enable-filter=acompressor \
  --enable-filter=alimiter \
  --enable-filter=aresample \
  --enable-filter=aformat \
  --enable-filter=anull \
  --extra-cflags='-Os' \
  --extra-ldflags='-static'

Build command:

make -j8 ffmpeg.exe
x86_64-w64-mingw32-strip ffmpeg.exe
