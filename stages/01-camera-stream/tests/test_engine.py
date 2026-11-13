"""Engine tests. Three groups, reported separately:
* plain unit tests (no AI libraries),
* ``ultralytics_runtime``: the real torch/ultralytics libraries with a random-init model,
* ``real_model``: the trained drone model on real/synthetic images (skipped when not configured).
"""
from pathlib import Path
from types import SimpleNamespace

import cv2
import pytest

from drone_camera.engine import ModelError, YoloTrackerEngine, check_model_file, resolve_device


# ------------------------------------------------------------- plain unit tests
def test_model_file_checks_give_actionable_errors(tmp_path):
    with pytest.raises(ModelError, match="선택되지"):
        check_model_file("")
    with pytest.raises(ModelError, match="찾을 수 없습니다"):
        check_model_file(str(tmp_path / "moved.pt"))
    with pytest.raises(ModelError, match="파일이 아닙니다"):
        check_model_file(str(tmp_path))
    text = tmp_path / "model.txt"
    text.write_text("x")
    with pytest.raises(ModelError, match=r"\.pt"):
        check_model_file(str(text))
    empty = tmp_path / "empty.pt"
    empty.write_bytes(b"")
    with pytest.raises(ModelError, match="비어"):
        check_model_file(str(empty))
    ok = tmp_path / "한글 모델 폴더" / "best.pt"
    ok.parent.mkdir()
    ok.write_bytes(b"x")
    assert check_model_file(str(ok)) == ok


def fake_torch(cuda=False, mps=False):
    return SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: cuda),
                           backends=SimpleNamespace(mps=SimpleNamespace(is_available=lambda: mps)))


@pytest.mark.parametrize("preference,cuda,mps,expected,note", [
    ("auto", True, True, "cuda:0", False), ("auto", False, True, "cpu", False),
    ("auto", False, False, "cpu", False), ("cpu", True, True, "cpu", False),
    ("cuda", False, False, "cpu", True), ("cuda", True, False, "cuda:0", False),
    ("mps", False, True, "mps", False), ("mps", False, False, "cpu", True)])
def test_device_selection_falls_back_to_cpu(preference, cuda, mps, expected, note):
    device, message = resolve_device(preference, fake_torch(cuda, mps))
    assert device == expected and bool(message) is note


# ----------------------------------------------------- real library, no trained weights
@pytest.mark.ultralytics_runtime
def test_corrupt_model_file_is_reported_not_raised(tmp_path):
    pytest.importorskip("ultralytics")
    broken = tmp_path / "broken.pt"
    broken.write_bytes(b"this is not a pytorch checkpoint" * 100)
    with pytest.raises(ModelError, match="손상"):
        YoloTrackerEngine(str(broken), "cpu", tmp_path / "cfg").load()


@pytest.mark.ultralytics_runtime
def test_runtime_report_runs_bytetrack_and_random_init_model(tmp_path):
    pytest.importorskip("ultralytics")
    from drone_camera.engine import runtime_report
    report = runtime_report(tmp_path / "cfg")
    assert report["ok"], report
    assert len(set(report["bytetrack_ids"])) == 2


# ------------------------------------------------------------------- trained model
def nearest(point, paths, limit=45):
    best = min(range(len(paths)), key=lambda i: (paths[i][0] - point[0]) ** 2 + (paths[i][1] - point[1]) ** 2)
    distance = ((paths[best][0] - point[0]) ** 2 + (paths[best][1] - point[1]) ** 2) ** 0.5
    return best if distance <= limit else None


def run_scene(engine, frames, conf=0.25, iou=0.5, imgsz=960):
    return [engine.track(image, conf, iou, imgsz) for image, _ in frames]


@pytest.fixture(scope="module")
def scenes(real_model_path, samples_dir, tmp_path_factory):
    import synthetic_scene as ss
    crops = ss.load_crops(samples_dir)
    assert len(crops) >= 2, "need at least two labelled sample images"
    config = tmp_path_factory.mktemp("cfg")
    apart = ss.render_frames(crops, 60, hidden={1: range(25, 31)}, overlap=False)
    cross = ss.render_frames(crops, 60, overlap=True)
    engine = YoloTrackerEngine(real_model_path, "cpu", config)
    engine.load()
    return SimpleNamespace(apart=apart, cross=cross, apart_tracks=run_scene(engine, apart),
                           cross_tracks=run_scene(engine, cross), config=config)


@pytest.mark.real_model
def test_best_pt_loads_with_single_drone_class(real_model_path, tmp_path):
    info = YoloTrackerEngine(real_model_path, "cpu", tmp_path).load()
    assert info.names == {0: "drone"} and info.class_warning == ""
    assert info.device == "cpu"


@pytest.mark.real_model
def test_single_drone_detection_matches_labels_in_pixel_coordinates(real_model_path, samples_dir, tmp_path):
    engine = YoloTrackerEngine(real_model_path, "cpu", tmp_path)
    engine.load()
    checked = 0
    for image_path in sorted(samples_dir.glob("*.jpg")):
        label = image_path.with_suffix(".txt")
        if not label.is_file():
            continue
        image = cv2.imread(str(image_path))
        height, width = image.shape[:2]
        cx, cy, w, h = (float(v) for v in label.read_text().split()[1:5])
        truth = ((cx - w / 2) * width, (cy - h / 2) * height, (cx + w / 2) * width, (cy + h / 2) * height)
        engine.reset()
        tracks = engine.track(image, 0.25, 0.5, 960)
        assert tracks, image_path.name
        best = max(tracks, key=lambda t: t.confidence)
        ix = max(0, min(best.x2, truth[2]) - max(best.x1, truth[0]))
        iy = max(0, min(best.y2, truth[3]) - max(best.y1, truth[1]))
        union = (best.x2 - best.x1) * (best.y2 - best.y1) + (truth[2] - truth[0]) * (truth[3] - truth[1]) - ix * iy
        assert ix * iy / union > 0.5, image_path.name
        assert 0 <= best.x1 < best.x2 <= width and 0 <= best.y1 < best.y2 <= height
        checked += 1
    assert checked >= 3


@pytest.mark.real_model
def test_two_drones_are_detected_simultaneously(scenes):
    rates = [0, 0]
    for (_, paths), tracks in zip(scenes.apart, scenes.apart_tracks):
        seen = {nearest(((t.x1 + t.x2) / 2, (t.y1 + t.y2) / 2), paths) for t in tracks}
        for index in (0, 1):
            rates[index] += index in seen
    assert max(len([1 for t in f if True]) for f in scenes.apart_tracks) >= 2
    assert rates[0] / 60 >= 0.6 and rates[1] / 60 >= 0.45      # drone 1 is hidden for 6 frames


@pytest.mark.real_model
def test_bytetrack_keeps_ids_through_short_misses_and_never_shows_ghosts(scenes):
    ids = {0: set(), 1: set()}
    hidden = range(25, 31)
    for f, ((_, paths), tracks) in enumerate(zip(scenes.apart, scenes.apart_tracks)):
        for t in tracks:
            owner = nearest(((t.x1 + t.x2) / 2, (t.y1 + t.y2) / 2), paths)
            assert owner is not None, "detection far from every drone (false positive)"
            if f in hidden:
                assert owner != 1, "hidden drone must not be reported at a remembered position"
            assert t.track_id is not None
            ids[owner].add(t.track_id)
    assert len(ids[0]) == 1 and len(ids[1]) == 1 and ids[0] != ids[1]   # stable ID per drone, incl. re-appearance


@pytest.mark.real_model
def test_crossing_drones_are_still_tracked_but_ids_may_switch(scenes):
    observed = [len(t) for t in scenes.cross_tracks]
    assert sum(1 for n in observed if n >= 1) / 60 >= 0.7
    ids = {t.track_id for frame in scenes.cross_tracks for t in frame}
    switches = len(ids) - 2
    print(f"crossing scenario: {len(ids)} distinct IDs for 2 drones ({max(0, switches)} extra)")
    assert len(ids) >= 2


@pytest.mark.real_model
def test_camera_engines_are_independent_even_when_interleaved(scenes, real_model_path):
    solo = [[t.track_id for t in f] for f in scenes.apart_tracks]
    a = YoloTrackerEngine(real_model_path, "cpu", scenes.config)
    b = YoloTrackerEngine(real_model_path, "cpu", scenes.config)
    a.load(), b.load()
    interleaved_a, first_b = [], []
    for index, (image, _) in enumerate(scenes.apart):
        interleaved_a.append([t.track_id for t in a.track(image, 0.25, 0.5, 960)])
        other = scenes.cross[index][0]                       # a different scene in the other camera
        tracks_b = b.track(other, 0.25, 0.5, 960)
        if tracks_b and not first_b:
            first_b = [t.track_id for t in tracks_b]
    assert interleaved_a == solo                             # camera B never changes camera A's IDs
    assert first_b and set(first_b) <= {1, 2}                # B numbers its own drones from 1


@pytest.mark.real_model
def test_reset_restarts_ids_for_a_new_session(scenes, real_model_path):
    engine = YoloTrackerEngine(real_model_path, "cpu", scenes.config)
    engine.load()
    image = scenes.apart[0][0]
    first = [t.track_id for t in engine.track(image, 0.25, 0.5, 960)]
    engine.reset()
    again = [t.track_id for t in engine.track(image, 0.25, 0.5, 960)]
    assert first and again == first and min(again) == 1


@pytest.mark.real_model
def test_confidence_and_iou_thresholds_change_the_result(real_model_path, samples_dir, tmp_path):
    engine = YoloTrackerEngine(real_model_path, "cpu", tmp_path)
    engine.load()
    image = cv2.imread(str(sorted(samples_dir.glob("*.jpg"))[0]))

    def count(conf, iou):
        engine.reset()
        return len(engine.track(image, conf, iou, 960))
    assert count(0.9, 0.5) == 0 <= count(0.25, 0.5) <= count(0.05, 0.5)
    assert count(0.25, 0.95) >= count(0.25, 0.5)
    for size in (320, 640):                                  # other inference sizes also run
        engine.reset()
        engine.track(image, 0.25, 0.5, size)
