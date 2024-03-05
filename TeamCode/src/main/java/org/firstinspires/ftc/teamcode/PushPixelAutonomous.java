import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.EasyOpenCVWebcam;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "PushPixelAutonomous", group = "Autonomous")
public class PushPixelAutonomous extends LinearOpMode {
    private static final double MAX_VELOCITY = 40;
    private static final double MAX_ACCELERATION = 30;

    // EasyOpenCV webcam
    private EasyOpenCVWebcam easyOpenCVWebcam;

    @Override
    public void runOpMode() {
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "yourWebcamName"), hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        easyOpenCVWebcam = new EasyOpenCVWebcam(webcam);
        webcam.setPipeline(new EmptyPipeline());

        // Set up RoadRunner constraints
        DriveConstraints constraints = new DriveConstraints(MAX_VELOCITY, MAX_ACCELERATION);

        // Set up RoadRunner trajectory
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(), constraints)
                .splineTo(new Pose2d(24, 24, 0))
                .build();

        waitForStart();

        easyOpenCVWebcam.startStreaming();

        while (opModeIsActive()) {
            Mat frame = easyOpenCVWebcam.getFrame();
            telemetry.addLine("לא זוכר אם זה הפקודה בכלל");

            PixelCoordinates redPixelCoordinates = pixelIsRed(frame);

            if (redPixelCoordinates != null) {
                int x = redPixelCoordinates.x;
                int y = redPixelCoordinates.y;
                telemetry.addData("Red pixel found at coordinates: ", "(x: " + x + ", y: " + y + ")");
            }

            easyOpenCVWebcam.sendFrame(frame);
        }

        easyOpenCVWebcam.stopStreaming();
    }

    private PixelCoordinates pixelIsRed(Mat frame) {
        int width = frame.width();
        int height = frame.height();

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                Scalar pixelColor = frame.get(y, x);
    
                if (pixelColor.val[0] > 200 && pixelColor.val[1] < 100 && pixelColor.val[2] < 100) {
                    return new PixelCoordinates(x, y);
                }
            }
        }
        return null; 
    }

    static class PixelCoordinates {
        int x;
        int y;

        PixelCoordinates(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    static class EmptyPipeline extends OpenCvPipeline {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            return input;
        }
    }
}
