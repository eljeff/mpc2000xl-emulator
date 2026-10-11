# SPDX-License-Identifier: BSD-3-Clause
"""Validate an external XL ROM and copy optional media into a private session."""
import hashlib
import json
from pathlib import Path
import shutil
import struct

ROM_BYTES = 0x80000
ROM_SHA256 = '100e3375b45072c209ffa2b1dbaecbf740c8c4ea29162827ac11cc6bf9cc7bab'
CHECKSUM_OFFSET = 0x77ffe
RESET_OFFSET = 0x7fff0


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as incoming:
        for block in iter(lambda: incoming.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def validate(rom, disk=None):
    rom = Path(rom)
    if rom.stat().st_size != ROM_BYTES:
        raise ValueError('Expected a complete 512 KiB MPC2KXL.BIN, including the boot ROM.')
    data = rom.read_bytes()
    rom_digest = hashlib.sha256(data).hexdigest() ## modified sha
    checksum = sum(word[0] for word in struct.iter_unpack('<H', data[:CHECKSUM_OFFSET])) & 0xffff
    stored = struct.unpack_from('<H', data, CHECKSUM_OFFSET)[0]
    if checksum != stored:
        raise ValueError('MPC2000XL OS checksum mismatch.')
    entry_ip, entry_cs = struct.unpack_from('<HH', data, RESET_OFFSET + 1)
    if data[RESET_OFFSET] != 0xea or (entry_cs, entry_ip) != (0xfc00, 0x0400):
        raise ValueError('Unexpected MPC2000XL reset vector.')
    result = {'model': 'MPC2000XL', 'os_version': '1.20', 'rom_sha256': rom_digest, # return modified sha
              'rom_bytes': len(data), 'os_checksum': f'{checksum:04x}',
              'boot_mode': 'native_reset_vector', 'disk_sha256': None,
              'disk_bytes': None, 'disk_extension': None, 'disk_contents_verified': False}
    if disk is None:
        return result
    disk = Path(disk)
    size, suffix = disk.stat().st_size, disk.suffix.lower()
    with disk.open('rb') as incoming:
        header = incoming.read(16)
    if suffix == '.hfe':
        if not header.startswith(b'HXCPICFE') or not 512 <= size <= 32 * 1024 * 1024:
            raise ValueError('Expected an HFE v1 floppy image.')
    elif suffix == '.img':
        if size != 1474560:
            raise ValueError('Raw IMG support expects a 1.44 MB floppy.')
    else:
        raise ValueError('Choose a data floppy in .hfe or .img format.')
    result.update(disk_sha256=sha256(disk), disk_bytes=size, disk_extension=suffix)
    
    return result


def stage(rom, disk, session, expected):
    media = Path(session) / 'media'
    media.mkdir(mode=0o700, exist_ok=False)
    staged_rom = media / 'MPC2KXL.BIN'
    staged_disk = media / ('floppy' + expected['disk_extension']) if disk is not None else None
    inputs = [(rom, staged_rom, expected['rom_sha256'])]
    if disk is not None:
        inputs.append((disk, staged_disk, expected['disk_sha256']))
    for source, target, digest in inputs:
        with Path(source).open('rb') as incoming, target.open('xb') as outgoing:
            shutil.copyfileobj(incoming, outgoing)
        if sha256(target) != digest:
            raise ValueError('Input changed while copying; emulator was not launched.')
        target.chmod(0o600)
    staged_rom.chmod(0o400)
    report = dict(expected, source_rom=str(Path(rom).resolve()),
                  source_disk=str(Path(disk).resolve()) if disk is not None else None,
                  state='staged', original_files_modified=False)
    (media / 'manifest.json').write_text(json.dumps(report, indent=2) + '\n')
    return staged_rom, staged_disk
