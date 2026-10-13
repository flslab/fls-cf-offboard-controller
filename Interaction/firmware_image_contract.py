"""Check a mission against the actual parameter tables in a Bolt ELF image.

Offline only: no device access, parameter writes or flight commands. This
reads linked tables rather than source text, so compiled-out APIs are absent.
"""
import argparse
import gzip
import json
from pathlib import Path
import struct

from Interaction.firmware_analytic_profile import profile_parameters
from Interaction.mission_profiles import resolve_mission_profiles


def image_parameters(path):
    path = Path(path)
    data = gzip.decompress(path.read_bytes()) if path.suffix == '.gz' else path.read_bytes()
    if data[:7] != b'\x7fELF\x01\x01\x01':
        raise ValueError('expected a little-endian ELF32 Bolt image')
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    if header[2] != 40:
        raise ValueError('expected ARM ELF')
    section_offset, section_size, section_count = header[6], header[11], header[12]
    if section_size != 40:
        raise ValueError('unexpected ELF section layout')
    sections = [struct.unpack_from('<IIIIIIIIII', data, section_offset+i*section_size)
                for i in range(section_count)]

    def string_at(blob, offset):
        return blob[offset:blob.index(b'\0', offset)].decode('ascii')

    def bytes_at(address, length):
        for section in sections:
            _, kind, _, start, offset, size, *_ = section
            if kind != 8 and start <= address and address+length <= start+size:
                return data[offset+address-start:offset+address-start+length]
        raise ValueError(f'ELF address unavailable: {address:#x}')

    def cstring(address):
        result = bytearray()
        for i in range(128):
            char = bytes_at(address+i, 1)
            if char == b'\0':
                return result.decode('ascii')
            result.extend(char)
        raise ValueError('parameter name is not terminated within 128 bytes')

    names = set()
    groups = 0
    for section in sections:
        if section[1] != 2:  # SHT_SYMTAB
            continue
        strings = sections[section[6]]
        table = data[strings[4]:strings[4]+strings[5]]
        if section[9] != 16:
            raise ValueError('unexpected ELF32 symbol layout')
        for offset in range(section[4], section[4]+section[5], 16):
            name, address, size, _, _, _ = struct.unpack_from('<IIIBBH', data, offset)
            if not string_at(table, name).startswith('__params_'):
                continue
            if size < 40 or size % 20:
                raise ValueError('unexpected Bolt param_s layout')
            entries = bytes_at(address, size)
            group = cstring(struct.unpack_from('<I', entries, 4)[0])
            groups += 1
            for record in range(20, size-20, 20):
                field = cstring(struct.unpack_from('<I', entries, record+4)[0])
                names.add(group+'.'+field)
    if not groups:
        raise ValueError('ELF contains no parameter tables; use an unstripped image')
    return names


def mission_parameters(mission):
    mission = resolve_mission_profiles(mission)
    config = mission['Interaction']['config']
    options = config.get('level_coast', {})
    brake = config['wrench_interaction']['firmware_auto_brake']
    if not brake.get('enabled') or brake.get('mode') != 'scurve':
        raise ValueError('this packaging check requires an enabled S-curve mission')
    analytic = profile_parameters(brake['analytic_profile'])
    required = set(analytic) | {
        'kalmanPRel.enable', 'kalmanPRel.scEnable', 'hlCommander.pRelAuto',
        'hlCommander.pRelMode', 'hlCommander.pRelTau', 'hlCommander.pRelJoint',
        'hlCommander.pRelHost', 'hlCommander.pRelScD', 'hlCommander.pRelScT',
        'hlCommander.pRelScB', 'stabilizer.estimator'}
    if analytic.get('hlCommander.pRelHold') == 3:
        required.add('hlCommander.pRelEnd')
    if config.get('behavior') == 'level_coast':
        required.add('hlCommander.pRelHoldG')
    if analytic.get('hlCommander.pRelFric'):
        required.add('hlCommander.pRelMuVer')
    if options.get('estimator_hover_xy'):
        from Interaction.estimator_xy_loading import REQUIRED
        required.update('eskfXY.'+name for name in REQUIRED)
    if brake.get('response_model', {}).get('enabled'):
        required.update(('pRelResp.runtime', 'pRelResp.commit', 'pRelResp.ready',
                         'pRelResp.id', 'pRelResp.activeId'))
    return required


def check(elf, mission):
    available = image_parameters(elf)
    required = mission_parameters(mission)
    missing = sorted(required-available)
    return dict(passed=not missing, missing=missing,
                required=sorted(required), image_parameter_count=len(available))


if __name__ == '__main__':
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path)
    parser.add_argument('mission', type=Path)
    args = parser.parse_args()
    result = check(args.elf, yaml.safe_load(args.mission.read_text()))
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
