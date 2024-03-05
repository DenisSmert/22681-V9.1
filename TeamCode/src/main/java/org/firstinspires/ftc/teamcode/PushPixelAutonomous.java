
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Webcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.EasyOpenCVWebcam;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "PushPixelAutonomous", group = "Autonomous")
public class PushPixelAutonomous extends LinearOpMode {
    private static final double MAX_VELOCITY = 40; 
    private static final double MAX_ACCELERATION = 30; 

    // EasyOpenCV webcam
    private OpenCvCamera webcam;
    private EasyOpenCVWebcam easyOpenCVWebcam;

    @Override
    public void runOpMode() {
        HardwareMap hardwareMap = hardwareMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        easyOpenCVWebcam = new EasyOpenCVWebcam(webcam);

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

            if (pixelIsRed(frame)) {
                // להוסיף לדחוף רובוטמ מיזדיין
            }

            easyOpenCVWebcam.sendFrame(frame);
        }

        easyOpenCVWebcam.stopStreaming();
    }

    private boolean pixelIsRed(Mat frame) {
        Scalar pixelColor = frame.get(y, x);
        return pixelColor.val[0] > 200 && pixelColor.val[1] < 100 && pixelColor.val[2] < 100;
    }
}
