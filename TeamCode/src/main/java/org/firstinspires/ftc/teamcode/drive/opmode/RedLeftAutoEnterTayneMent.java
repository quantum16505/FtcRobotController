package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpModes.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class RedLeftAutoEnterTayneMent extends LinearOpMode {
    public static double DISTANCE = 65; // in
    // Added this for the claw servos
    HardwareProfile robot = new HardwareProfile();
    // the claw servos
    private Servo IntakeLeft;
    private Servo IntakeRight;
    // Begin the section from the AprilTag example

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 18 from the 36h11 family
    int left = 1;
    int middle = 2;
    int right = 3;
    AprilTagDetection tagOfInterest = null;

    // End the section from the AprilTag Example

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        // Added this for servos
        robot.init(hardwareMap);
        double openPos = 0.86;
        double startPos = 1;
        double closePos = 1;
        double liftPower = .5;
        // Begin section from AprilTag example
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == left || tag.id == middle || tag.id == right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        // End section from AprilTag example

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
        //        .forward(DISTANCE)
        //        .turn(Math.toRadians(90))
        //        .build();
        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);
        waitForStart();

        if (isStopRequested()) return;

        // drive.followTrajectory(trajectory);
        while (!isStopRequested()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .strafeRight(35)
                    // Raise lift to high here
                    .addDisplacementMarker(() -> {
                        // lift da cone
                        robot.Carousel.setTargetPosition(-2925);
                        robot.Carousel.setPower(liftPower);
                    })
                    .forward(65)
                    .turn(Math.toRadians(90))
                    // .strafeRight(14)


                    // Drive forward 6 inches
                    .forward(5)
                    // Release claw
                    .addDisplacementMarker(() -> {
                        // drop cone and lower to top of stack
                        robot.Carousel.setTargetPosition(-450);
                        robot.Carousel.setPower(liftPower);
                        robot.IntakeLeft.setPosition(openPos);
                        robot.IntakeRight.setPosition(openPos);
                    })
                    // Drive back 6 inches
                    .back(5)

                    // Strafe left
                    .strafeLeft(15)
                    .forward(51)
                    // Close claw
                    .addDisplacementMarker(() -> {
                        // grab cone
                        robot.IntakeLeft.setPosition(closePos);
                        robot.IntakeRight.setPosition(closePos);
                        robot.Carousel.setTargetPosition(-2150);
                        robot.Carousel.setPower(liftPower);
                    })
                    // lift to mid junction
                    .back(23)
                    .turn(Math.toRadians(90))
                    .strafeLeft(20)
                    .forward(5.5)
                    // open claw
                    .addDisplacementMarker(() -> {
                        // drop cone
                        robot.IntakeLeft.setPosition(openPos);
                        robot.IntakeRight.setPosition(openPos);
                        robot.Carousel.setTargetPosition(0);
                        robot.Carousel.setPower(liftPower);
                    })
                    .back(5)
                    .build();

            // park

            drive.followTrajectorySequence(trajSeq);
            if(tagOfInterest == null || tagOfInterest.id == left) {
                // left telemetry goes here
                TrajectorySequence trajLeft = drive.trajectorySequenceBuilder(trajSeq.end())
                        .turn(Math.toRadians(-90))
                        .forward(32)
                        .build();
                drive.followTrajectorySequence(trajLeft);
                robot.Carousel.setPower(0);

            }else if(tagOfInterest.id == middle){
                // middle telemetry goes here
                TrajectorySequence trajMiddle = drive.trajectorySequenceBuilder(trajSeq.end())
                        .turn(Math.toRadians(-90))
                        .forward(10)
                        .build();
                drive.followTrajectorySequence(trajMiddle);
                robot.Carousel.setPower(0);

            }else if(tagOfInterest.id == right){
                // right  telemetry goes here
                TrajectorySequence trajRight = drive.trajectorySequenceBuilder(trajSeq.end())
                        .turn(Math.toRadians(-90))
                        .back(16)
                        .build();
                drive.followTrajectorySequence(trajRight);
                robot.Carousel.setPower(0);

            }
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();

            while (!isStopRequested() && opModeIsActive()) ;
        }
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
