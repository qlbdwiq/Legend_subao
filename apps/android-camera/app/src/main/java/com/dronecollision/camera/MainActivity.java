package com.dronecollision.camera;

import android.Manifest;
import android.app.Activity;
import android.content.*;
import android.content.pm.PackageManager;
import android.graphics.*;
import android.hardware.camera2.*;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.*;
import android.net.*;
import android.os.*;
import android.util.*;
import android.view.*;
import android.widget.*;
import java.io.ByteArrayOutputStream;
import java.net.*;
import java.nio.ByteBuffer;
import java.util.*;

public final class MainActivity extends Activity {
    private TextView status, address;
    private ImageView preview;
    private Button start, copy, facingButton;
    private Spinner resolutionSpinner, fpsSpinner;
    private HandlerThread cameraThread;
    private Handler cameraHandler;
    private final Handler ui = new Handler(Looper.getMainLooper());
    private CameraDevice camera;
    private CameraCaptureSession session;
    private ImageReader reader;
    private FrameHub frames;
    private MjpegServer server;
    private volatile boolean streaming;
    private long nextEncodeAt, previewSequence;
    private int rotation;
    private long generation;
    private final String sessionToken = MjpegServer.newToken();
    private int sessionPort;
    private volatile int targetFps = 30;
    private int activeFacing = CameraCharacteristics.LENS_FACING_BACK;
    private String activeCameraId;
    private List<StreamModes.Mode> streamModes = List.of();
    private List<Integer> fpsChoices = List.of(30);
    private int activeWidth, activeHeight;
    private int encodedInWindow;
    private long fpsWindowStartNanos;
    private volatile double measuredEncodeFps;

    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        LinearLayout column = new LinearLayout(this);
        column.setOrientation(LinearLayout.VERTICAL);
        column.setPadding(24, 16, 24, 12);
        column.setBackgroundColor(Color.rgb(17, 24, 39));
        column.setOnApplyWindowInsetsListener((view, insets) -> {
            view.setPadding(24+insets.getSystemWindowInsetLeft(),16+insets.getSystemWindowInsetTop(),
                    24+insets.getSystemWindowInsetRight(),12+insets.getSystemWindowInsetBottom());
            return insets;
        });
        TextView title = new TextView(this);
        title.setText("DRONE CAMERA  ·  Android / Galaxy"); title.setTextSize(21); title.setTextColor(Color.WHITE);
        column.addView(title);
        preview = new ImageView(this); preview.setScaleType(ImageView.ScaleType.FIT_CENTER);
        preview.setBackgroundColor(Color.BLACK);
        column.addView(preview, new LinearLayout.LayoutParams(-1, 0, 1));
        status = new TextView(this); status.setTextColor(Color.WHITE); status.setText("같은 Wi-Fi 또는 USB로 연결 · 후면 카메라"); column.addView(status);
        address = new TextView(this); address.setTextColor(Color.rgb(120, 210, 255)); address.setTextIsSelectable(true);
        address.setTextSize(13); column.addView(address);
        LinearLayout formatRow = new LinearLayout(this);
        resolutionSpinner = new Spinner(this);
        fpsSpinner = new Spinner(this);
        formatRow.addView(resolutionSpinner, new LinearLayout.LayoutParams(0, -2, 2));
        formatRow.addView(fpsSpinner, new LinearLayout.LayoutParams(0, -2, 1));
        column.addView(formatRow);
        resolutionSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override public void onItemSelected(AdapterView<?> parent, View view, int position, long id) { updateFpsChoices(); }
            @Override public void onNothingSelected(AdapterView<?> parent) { }
        });
        LinearLayout actions = new LinearLayout(this);
        start = new Button(this); start.setText("송출 시작 / Start"); start.setOnClickListener(v -> { if (streaming) stop("송출 중지"); else requestCamera(); });
        copy = new Button(this); copy.setText("연결 주소 복사 / Copy URL"); copy.setEnabled(false);
        copy.setOnClickListener(v -> ((ClipboardManager)getSystemService(CLIPBOARD_SERVICE)).setPrimaryClip(ClipData.newPlainText("Drone Camera URL", address.getText())));
        facingButton = new Button(this); facingButton.setText("후면 / Back");
        facingButton.setOnClickListener(v -> {
            activeFacing = activeFacing == CameraCharacteristics.LENS_FACING_BACK
                    ? CameraCharacteristics.LENS_FACING_FRONT : CameraCharacteristics.LENS_FACING_BACK;
            facingButton.setText(activeFacing == CameraCharacteristics.LENS_FACING_BACK ? "후면 / Back" : "전면 / Front");
            refreshModes();
        });
        actions.addView(start); actions.addView(copy); actions.addView(facingButton); column.addView(actions);
        setContentView(column);
        refreshModes();
    }

    /** Enumerate live-streaming modes for the selected lens from Camera2. */
    private void refreshModes() {
        try {
            CameraManager manager = (CameraManager)getSystemService(CAMERA_SERVICE);
            activeCameraId = null;
            for (String candidate : manager.getCameraIdList()) {
                Integer lens = manager.getCameraCharacteristics(candidate).get(CameraCharacteristics.LENS_FACING);
                if (lens != null && lens == activeFacing) { activeCameraId = candidate; break; }
            }
            if (activeCameraId == null) {
                streamModes = List.of();
                status.setText("선택한 방향의 카메라를 찾을 수 없습니다.");
                return;
            }
            CameraCharacteristics characteristics = manager.getCameraCharacteristics(activeCameraId);
            Range<Integer>[] ranges = characteristics.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
            int aeMax = 0;
            if (ranges != null) for (Range<Integer> range : ranges) aeMax = Math.max(aeMax, range.getUpper());
            StreamConfigurationMap map = Objects.requireNonNull(
                    characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP));
            Size[] sizes = map.getOutputSizes(ImageFormat.YUV_420_888);
            List<StreamModes.Mode> modes = new ArrayList<>();
            if (sizes != null) for (Size size : sizes) {
                long duration = 0;
                try { duration = map.getOutputMinFrameDuration(ImageFormat.YUV_420_888, size); }
                catch (RuntimeException ignored) { }
                modes.add(new StreamModes.Mode(size.getWidth(), size.getHeight(),
                        StreamModes.maxFpsFromMinDuration(duration, aeMax)));
            }
            streamModes = StreamModes.normalize(modes);
            List<String> labels = new ArrayList<>();
            for (StreamModes.Mode mode : streamModes) labels.add(mode.label() + " · 최대 " + mode.maxFps() + " FPS");
            ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, labels);
            adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            resolutionSpinner.setAdapter(adapter);
            int selected = StreamModes.defaultIndex(streamModes);
            if (selected >= 0) resolutionSpinner.setSelection(selected);
            updateFpsChoices();
            StreamModes.Mode best = StreamModes.bestResolution(streamModes);
            StreamModes.Mode fastest = StreamModes.bestFps(streamModes);
            if (best != null && !streaming) status.setText(
                    "같은 Wi-Fi 또는 USB로 연결 · " + (activeFacing == CameraCharacteristics.LENS_FACING_BACK ? "후면" : "전면")
                    + " · 최고 " + best.label() + " @ " + best.maxFps() + " FPS · 최고 속도 "
                    + fastest.label() + " @ " + fastest.maxFps() + " FPS");
        } catch (Exception e) {
            streamModes = List.of();
            status.setText("카메라 지원 범위를 확인할 수 없습니다. 권한을 확인하세요.");
        }
    }

    private void updateFpsChoices() {
        int position = resolutionSpinner == null ? -1 : resolutionSpinner.getSelectedItemPosition();
        if (position < 0 || position >= streamModes.size()) { fpsChoices = List.of(30); return; }
        fpsChoices = StreamModes.fpsOptions(streamModes.get(position).maxFps());
        List<String> labels = new ArrayList<>();
        for (int fps : fpsChoices) labels.add(fps + " FPS");
        ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, labels);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        fpsSpinner.setAdapter(adapter);
        int preferred = fpsChoices.indexOf(30);
        fpsSpinner.setSelection(preferred >= 0 ? preferred : fpsChoices.size() - 1);
    }

    private StreamModes.Mode selectedMode() {
        int position = resolutionSpinner.getSelectedItemPosition();
        if (position >= 0 && position < streamModes.size()) return streamModes.get(position);
        int fallback = StreamModes.defaultIndex(streamModes);
        return fallback >= 0 ? streamModes.get(fallback) : new StreamModes.Mode(1280, 720, 30);
    }

    private int selectedFps(StreamModes.Mode mode) {
        int position = fpsSpinner.getSelectedItemPosition();
        if (position >= 0 && position < fpsChoices.size()) return fpsChoices.get(position);
        return Math.min(30, mode.maxFps());
    }
    private void requestCamera() {
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED)
            requestPermissions(new String[]{Manifest.permission.CAMERA}, 1);
        else begin();
    }
    @Override public void onRequestPermissionsResult(int code, String[] permissions, int[] results) {
        super.onRequestPermissionsResult(code, permissions, results);
        if (code == 1 && results.length > 0 && results[0] == PackageManager.PERMISSION_GRANTED) begin();
        else status.setText("카메라 권한이 필요합니다. 앱 설정에서 카메라 접근을 허용하세요.");
    }
    private void begin() {
        if (streaming) return;
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) return;
        if (streamModes.isEmpty()) refreshModes();
        final StreamModes.Mode mode = selectedMode();
        targetFps = selectedFps(mode);
        activeWidth = mode.width(); activeHeight = mode.height();
        final long current = ++generation;
        try {
            String facingName = activeFacing == CameraCharacteristics.LENS_FACING_BACK ? "back" : "front";
            frames = new FrameHub();
            server = new MjpegServer(frames, sessionPort, sessionToken,
                    StreamModes.capabilitiesJson(facingName, streamModes, mode));
            sessionPort = server.port();
            streaming = true; nextEncodeAt = 0; previewSequence = 0;
            encodedInWindow = 0; fpsWindowStartNanos = 0; measuredEncodeFps = 0;
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            start.setText("송출 중지 / Stop"); copy.setEnabled(true);
            resolutionSpinner.setEnabled(false); fpsSpinner.setEnabled(false); facingButton.setEnabled(false);
            address.setText(server.url(wifiAddress()));
            status.setText("연결 중 · URL을 데스크톱에 입력 · USB는 연결 도우미 사용");
            cameraThread = new HandlerThread("camera-encoder"); cameraThread.start();
            cameraHandler = new Handler(cameraThread.getLooper());
            CameraManager manager = (CameraManager)getSystemService(CAMERA_SERVICE);
            String id = activeCameraId;
            if (id == null) throw new IllegalStateException("No selected camera");
            CameraCharacteristics characteristics = manager.getCameraCharacteristics(id);
            Range<Integer>[] ranges = characteristics.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
            final int requestedFps = targetFps;
            Range<Integer> frameRate = ranges == null ? null : Arrays.stream(ranges)
                    .filter(r -> r.getLower() <= requestedFps && r.getUpper() >= requestedFps)
                    .min(Comparator.comparingInt(r -> Math.abs(r.getUpper()-requestedFps)+Math.abs(r.getLower()-requestedFps))).orElse(null);
            Size size = new Size(mode.width(), mode.height());
            Integer sensor = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION);
            int display = getWindowManager().getDefaultDisplay().getRotation() * 90;
            rotation = ((sensor == null ? 0 : sensor) - display + 360) % 360;
            reader = ImageReader.newInstance(size.getWidth(), size.getHeight(), ImageFormat.YUV_420_888, 3);
            reader.setOnImageAvailableListener(this::encode, cameraHandler);
            manager.openCamera(id, new CameraDevice.StateCallback() {
                @Override public void onOpened(CameraDevice device) {
                    if (!streaming || generation != current) { device.close(); return; }
                    camera = device;
                    try {
                        device.createCaptureSession(Collections.singletonList(reader.getSurface()), new CameraCaptureSession.StateCallback() {
                            @Override public void onConfigured(CameraCaptureSession configured) {
                                if (!streaming || generation != current) { configured.close(); return; }
                                session = configured;
                                try {
                                    CaptureRequest.Builder request = device.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                                    request.addTarget(reader.getSurface());
                                    request.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO);
                                    if (frameRate != null) request.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, frameRate);
                                    configured.setRepeatingRequest(request.build(), null, cameraHandler);
                                    ui.post(() -> status.setText(size.getWidth()+" × "+size.getHeight()
                                            +" · 설정 "+requestedFps+" FPS · 앱을 열린 상태로 유지"));
                                } catch (CameraAccessException | IllegalStateException e) { if (generation == current) failed(); }
                            }
                            @Override public void onConfigureFailed(CameraCaptureSession ignored) { if (generation == current) failed(); }
                        }, ui);
                    } catch (CameraAccessException | IllegalStateException e) { if (generation == current) failed(); }
                }
                @Override public void onDisconnected(CameraDevice device) { device.close(); if (generation == current) failed(); }
                @Override public void onError(CameraDevice device, int error) { device.close(); if (generation == current) failed(); }
            }, ui);
            ui.post(previewTick);
        } catch (Exception e) { stop("시작 실패: 카메라 권한, 선택한 해상도/FPS 지원 여부, 네트워크를 확인하세요."); }
    }
    private void encode(ImageReader source) {
        if (!streaming || source != reader) return;
        try (Image image = source.acquireLatestImage()) {
            if (image == null || !streaming) return;
            long now = System.nanoTime();
            if (now < nextEncodeAt) return;
            // Keep cadence across camera jitter rather than anchoring every
            // deadline to the last arrival (which can halve a nominal 30fps feed).
            nextEncodeAt = Math.max(nextEncodeAt + 1_000_000_000L / targetFps, now);
            Image.Plane[] p = image.getPlanes();
            byte[] nv21 = Yuv420.nv21(image.getWidth(), image.getHeight(),
                    new ByteBuffer[]{p[0].getBuffer(),p[1].getBuffer(),p[2].getBuffer()},
                    new int[]{p[0].getRowStride(),p[1].getRowStride(),p[2].getRowStride()},
                    new int[]{p[0].getPixelStride(),p[1].getPixelStride(),p[2].getPixelStride()});
            ByteArrayOutputStream bytes = new ByteArrayOutputStream();
            new YuvImage(nv21, ImageFormat.NV21, image.getWidth(), image.getHeight(), null)
                    .compressToJpeg(new Rect(0,0,image.getWidth(),image.getHeight()), 70, bytes);
            byte[] jpeg = bytes.toByteArray();
            if (rotation != 0) {
                Bitmap bitmap = BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
                Matrix matrix = new Matrix(); matrix.postRotate(rotation);
                Bitmap rotated = Bitmap.createBitmap(bitmap,0,0,bitmap.getWidth(),bitmap.getHeight(),matrix,true);
                bytes.reset(); rotated.compress(Bitmap.CompressFormat.JPEG,70,bytes); jpeg = bytes.toByteArray();
                if (bitmap != rotated) bitmap.recycle(); rotated.recycle();
            }
            FrameHub hub = frames;
            if (streaming && source == reader && hub != null) {
                hub.publish(jpeg);
                encodedInWindow++;
                if (fpsWindowStartNanos == 0) fpsWindowStartNanos = now;
                double elapsed = (now - fpsWindowStartNanos) / 1_000_000_000.0;
                if (elapsed >= 1.0) {
                    measuredEncodeFps = encodedInWindow / elapsed;
                    encodedInWindow = 0; fpsWindowStartNanos = now;
                    final double measured = measuredEncodeFps;
                    final int requested = targetFps;
                    ui.post(() -> { if (streaming) status.setText(activeWidth+" × "+activeHeight
                            +" · 설정 "+requested+" FPS · 실제 인코딩 "+String.format(Locale.ROOT, "%.1f", measured)
                            +" FPS · 앱을 열린 상태로 유지"); });
                }
            }
        } catch (IllegalStateException e) { if (streaming) failed(); }
    }
    private final Runnable previewTick = new Runnable() {
        @Override public void run() {
            if (!streaming || frames == null) return;
            FrameHub.Frame frame = frames.latest();
            if (frame != null && frame.sequence() != previewSequence) {
                preview.setImageBitmap(BitmapFactory.decodeByteArray(frame.jpeg(),0,frame.jpeg().length));
                previewSequence = frame.sequence();
            }
            ui.postDelayed(this, 100);
        }
    };
    private void failed() { ui.post(() -> stop("카메라 연결 오류 · 다른 앱의 사용 여부와 권한을 확인한 뒤 다시 시작하세요.")); }
    private String wifiAddress() {
        ConnectivityManager manager = (ConnectivityManager)getSystemService(CONNECTIVITY_SERVICE);
        for (Network network : manager.getAllNetworks()) {
            NetworkCapabilities capabilities = manager.getNetworkCapabilities(network);
            LinkProperties properties = manager.getLinkProperties(network);
            if (capabilities != null && capabilities.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) && properties != null)
                for (LinkAddress link : properties.getLinkAddresses())
                    if (link.getAddress() instanceof Inet4Address) return link.getAddress().getHostAddress();
        }
        return "127.0.0.1"; // Still usable by the desktop USB tunnel helper.
    }
    private void stop(String message) {
        streaming = false; ++generation; ui.removeCallbacks(previewTick);
        if (session != null) { session.close(); session = null; }
        if (camera != null) { camera.close(); camera = null; }
        ImageReader oldReader = reader; reader = null;
        HandlerThread oldThread = cameraThread;
        if (cameraHandler != null) cameraHandler.post(() -> {
            if (oldReader != null) oldReader.close();
            if (oldThread != null) oldThread.quitSafely();
        });
        if (server != null) { server.close(); server = null; }
        if (frames != null) { frames.close(); frames = null; }
        cameraThread = null; cameraHandler = null;
        getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        start.setText("송출 시작 / Start"); copy.setEnabled(false);
        resolutionSpinner.setEnabled(true); fpsSpinner.setEnabled(true); facingButton.setEnabled(true);
        address.setText("");
        preview.setImageDrawable(null); status.setText(message);
    }
    @Override protected void onPause() { stop("앱이 백그라운드로 이동하여 송출을 중지했습니다."); super.onPause(); }
}
