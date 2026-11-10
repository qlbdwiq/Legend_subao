package com.dronecollision.camera;

import android.Manifest;
import android.app.Activity;
import android.content.*;
import android.content.pm.PackageManager;
import android.graphics.*;
import android.hardware.camera2.*;
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
    private Button start, copy, rate;
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
        LinearLayout actions = new LinearLayout(this);
        start = new Button(this); start.setText("송출 시작 / Start"); start.setOnClickListener(v -> { if (streaming) stop("송출 중지"); else requestCamera(); });
        copy = new Button(this); copy.setText("연결 주소 복사 / Copy URL"); copy.setEnabled(false);
        copy.setOnClickListener(v -> ((ClipboardManager)getSystemService(CLIPBOARD_SERVICE)).setPrimaryClip(ClipData.newPlainText("Drone Camera URL", address.getText())));
        rate = new Button(this); rate.setText("30 FPS");
        rate.setOnClickListener(v -> { targetFps = targetFps == 30 ? 15 : 30; rate.setText(targetFps+" FPS"); });
        actions.addView(start); actions.addView(copy); actions.addView(rate); column.addView(actions);
        setContentView(column);
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
        final long current = ++generation;
        try {
            frames = new FrameHub(); server = new MjpegServer(frames, sessionPort, sessionToken); sessionPort = server.port();
            streaming = true; nextEncodeAt = 0; previewSequence = 0;
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            start.setText("송출 중지 / Stop"); copy.setEnabled(true);
            rate.setEnabled(false);
            address.setText(server.url(wifiAddress()));
            status.setText("연결 중 · URL을 데스크톱에 입력 · USB는 연결 도우미 사용");
            cameraThread = new HandlerThread("camera-encoder"); cameraThread.start();
            cameraHandler = new Handler(cameraThread.getLooper());
            CameraManager manager = (CameraManager)getSystemService(CAMERA_SERVICE);
            String id = null;
            for (String candidate : manager.getCameraIdList()) {
                Integer lens = manager.getCameraCharacteristics(candidate).get(CameraCharacteristics.LENS_FACING);
                if (lens != null && lens == CameraCharacteristics.LENS_FACING_BACK) { id = candidate; break; }
            }
            if (id == null) throw new IllegalStateException("No rear camera");
            CameraCharacteristics characteristics = manager.getCameraCharacteristics(id);
            Range<Integer>[] ranges = characteristics.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
            Range<Integer> frameRate = ranges == null ? null : Arrays.stream(ranges)
                    .filter(r -> r.getLower() <= targetFps && r.getUpper() >= targetFps)
                    .min(Comparator.comparingInt(r -> Math.abs(r.getUpper()-targetFps)+Math.abs(r.getLower()-targetFps))).orElse(null);
            Size[] sizes = Objects.requireNonNull(characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)).getOutputSizes(ImageFormat.YUV_420_888);
            Size size = Arrays.stream(sizes).filter(s -> s.getWidth() <= 1280 && s.getHeight() <= 720 && s.getWidth()%2==0 && s.getHeight()%2==0)
                    .max(Comparator.comparingInt(s -> s.getWidth()*s.getHeight())).orElseThrow(() -> new IllegalStateException("No supported size"));
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
                                    ui.post(() -> status.setText(size.getWidth()+" × "+size.getHeight()+" · MJPEG 최대 "+targetFps+" FPS · 앱을 열린 상태로 유지"));
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
        } catch (Exception e) { stop("시작 실패: 카메라 권한, 후면 카메라, 네트워크를 확인하세요."); }
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
            if (streaming && source == reader && hub != null) hub.publish(jpeg);
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
        start.setText("송출 시작 / Start"); copy.setEnabled(false); rate.setEnabled(true); address.setText("");
        preview.setImageDrawable(null); status.setText(message);
    }
    @Override protected void onPause() { stop("앱이 백그라운드로 이동하여 송출을 중지했습니다."); super.onPause(); }
}
