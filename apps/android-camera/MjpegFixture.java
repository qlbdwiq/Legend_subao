import com.dronecollision.camera.FrameHub;
import com.dronecollision.camera.MjpegServer;
import java.nio.file.*;

/** Desktop integration fixture, not included in the Android application. */
public class MjpegFixture {
    public static void main(String[] args) throws Exception {
        byte[] jpeg = Files.readAllBytes(Path.of(args[0]));
        FrameHub frames = new FrameHub();
        try (MjpegServer server = new MjpegServer(frames)) {
            System.out.println(server.url("127.0.0.1")); System.out.flush();
            while (!Thread.currentThread().isInterrupted()) {
                frames.publish(jpeg); Thread.sleep(67);
            }
        } finally { frames.close(); }
    }
}
