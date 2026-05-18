# MP3 to U8 Converter

Put `.mp3` files in this folder, then double-click `convert_mp3_to_u8.cmd`.

The output is a raw unsigned 8-bit mono PCM file with the same name and a `.U8` suffix.

This keeps the command found in the old Codex ARBATOS conversation:

```powershell
.\ffmpeg.exe -y -i "input.mp3" -vn -ac 1 -ar 12000 -af "acompressor=threshold=-18dB:ratio=2:attack=5:release=120:makeup=6,alimiter=limit=0.95" -c:a pcm_u8 -f u8 "input.U8"
```

The key parts are:

- `-ac 1`: mono.
- `-ar 12000`: 12 kHz sample rate, matching the current buzzer config.
- `-c:a pcm_u8 -f u8`: raw unsigned 8-bit PCM with no file header.
- `acompressor...alimiter...`: the light compression chain previously used for the buzzer audio.

You can also drag one or more MP3 files onto `convert_mp3_to_u8.cmd`; in that case, each `.U8` is written next to its source MP3.
