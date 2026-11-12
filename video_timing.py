"""Simple recording lifecycle timestamps on the shared epoch clock."""
import json
from pathlib import Path
import time


def epoch_clock_status():
    return {'assumed_synchronized': True}


def clock_sample():
    return dict(epoch_ns=time.time_ns(), monotonic_ns=time.monotonic_ns())


class RecordingTiming:
    def __init__(self, video, *, camera_id, run_id, request_id, command):
        self.video = Path(video)
        self.metadata_path = self.video.with_suffix('.timing.json')
        self.metadata = dict(schema='lightbender-video-timing', schema_version=2,
            camera_id=camera_id, run_id=run_id, request_id=request_id,
            video_file=self.video.name, command=command, valid=False,
            clock_assumption='camera and drone epoch clocks are assumed synchronized',
            time_origin='recording_process_started',
            recording_start_requested_epoch_s=time.time(), events=[])
        # These belong to the retired encoder-parsing implementation.
        for suffix in ('.frames.jsonl', '.encoder.log'):
            self.video.with_suffix(suffix).unlink(missing_ok=True)
        self.save()

    def save(self):
        temporary = self.metadata_path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.metadata, indent=2, allow_nan=False)+'\n')
        temporary.replace(self.metadata_path)

    def started(self):
        epoch = time.time()
        self.metadata.update(recording_process_started_epoch_s=epoch, video_start_epoch_s=epoch)
        self.save()

    def confirmed(self):
        self.metadata['recording_confirmed_epoch_s'] = time.time()
        self.save()

    def acknowledgement(self):
        return {'video_start_epoch_s': self.metadata['video_start_epoch_s'],
                'recording_confirmed_epoch_s': self.metadata.get('recording_confirmed_epoch_s')}

    def event(self, name, **values):
        self.metadata['events'].append(dict(name=name, epoch_s=time.time(), **values))
        self.save()

    def stop_requested(self):
        self.metadata['recording_stop_requested_epoch_s'] = time.time()
        self.save()

    def finish(self, returncode):
        self.metadata.update(recording_stopped_epoch_s=time.time(), encoder_returncode=returncode,
                             valid=returncode in (0, -2) and 'video_start_epoch_s' in self.metadata)
        if not self.metadata['valid']:
            self.metadata['error'] = f'recording did not finish successfully (status {returncode})'
        self.save()
