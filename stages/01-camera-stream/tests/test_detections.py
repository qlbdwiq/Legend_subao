import json

from drone_camera.detections import (
    STATE_NEW, STATE_REACQUIRED, STATE_TRACKED, RawTrack, TrackBook, build_observations)


def observe(book, tracks, camera="camera-01", sequence=1, factor=1.2):
    return build_observations(camera, sequence, 1000.5, 55.25, tracks, factor, book)


def test_observation_contains_all_data_fields():
    (obs,) = observe(TrackBook(), [RawTrack(3, 10, 20, 40, 60, 0.8)], sequence=7)
    assert (obs.camera_id, obs.frame_sequence, obs.track_id) == ("camera-01", 7, 3)
    assert (obs.received_at, obs.received_monotonic) == (1000.5, 55.25)
    assert obs.bbox == (10, 20, 40, 60) and obs.center == (25, 40)
    assert obs.base_radius == 25 and obs.safe_radius == 30 and obs.safety_factor == 1.2
    assert obs.confidence == 0.8 and obs.state == STATE_NEW
    json.dumps(obs.to_dict())  # Plain data only: ready for logging / later stages.


def test_track_lifecycle_new_tracked_reacquired():
    book = TrackBook()
    track = RawTrack(1, 0, 0, 10, 10, 0.9)
    assert observe(book, [track])[0].state == STATE_NEW
    assert observe(book, [track])[0].state == STATE_TRACKED
    observe(book, [])                       # a processed frame without this drone
    observe(book, [])
    assert observe(book, [track])[0].state == STATE_REACQUIRED
    assert observe(book, [track])[0].state == STATE_TRACKED


def test_separate_books_do_not_share_ids():
    first, second = TrackBook(), TrackBook()
    track = RawTrack(1, 0, 0, 10, 10, 0.9)
    observe(first, [track])
    assert observe(first, [track])[0].state == STATE_TRACKED
    assert observe(second, [track])[0].state == STATE_NEW   # same ID, other camera: unrelated


def test_degenerate_boxes_are_skipped_not_invented():
    result = observe(TrackBook(), [RawTrack(1, 5, 5, 5, 20, 0.9), RawTrack(2, 0, 0, 10, 10, 0.5)])
    assert [o.track_id for o in result] == [2]


def test_missing_track_id_is_preserved():
    (obs,) = observe(TrackBook(), [RawTrack(None, 0, 0, 10, 10, 0.5)])
    assert obs.track_id is None and obs.state == STATE_NEW
