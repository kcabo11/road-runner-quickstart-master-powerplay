package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RedTerminalShort extends LinearOpMode {
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

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;


    @Override
    public double getRuntime() {
        return super.getRuntime();
    }

    @Override
    public void resetRuntime() {
        super.resetRuntime();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
    }

    @Override
    public void internalPreInit() {
        super.internalPreInit();
    }

    @Override
    public void internalPostInitLoop() {
        super.internalPostInitLoop();
    }

    @Override
    public void internalPostLoop() {
        super.internalPostLoop();
    }

    @Override
    public int hashCode() {
        return super.hashCode();
    }

    @Override
    public boolean equals(@Nullable Object obj) {
        return super.equals(obj);
    }

    @NonNull
    @Override
    protected Object clone() throws CloneNotSupportedException {
        return super.clone();
    }

    @NonNull
    @Override
    public String toString() {
        return super.toString();
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36.625, -62.375, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

//        waitForStart();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
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



        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag   snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */

        if (isStopRequested()) return;

        drive.servo.setPosition(1);
        sleep(1000);



        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(-36.625, -62.375, Math.toRadians(90))) //34.25
                    //.forward(22)
                    .splineTo(new Vector2d(-31,-4.5), Math.toRadians(40), //-24-24
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .waitSeconds(4)
                    .back(5)

                    .lineToSplineHeading(new Pose2d(-36, -11, Math.toRadians(0)))
                    .waitSeconds(1)
                    .back(25)

                    .addTemporalMarker(0, () -> {
                        //lift to high pos
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(-1);
                        drive.servo.setPosition(1);
                    })
                    .addTemporalMarker(4,() -> {
                        //drop lift
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(.1);
                    })
                    .addTemporalMarker(4.5, () -> {
                        drive.servo.setPosition(0);
                    })
                    //START -- TO HOLD POS BEFORE BACK
                    .addTemporalMarker(4.6, () -> {
                        //lift to stack pos
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(-1);
                    })
                    .addTemporalMarker(6,() -> {
                        //drop lift
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(.1);
                    })
                    .build();

            drive.followTrajectorySequence(trajSeq);
        }
        else if(tagOfInterest.id == MIDDLE) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(-36.625, -62.375, Math.toRadians(90))) //34.25
                    //.forward(22)
                    .splineTo(new Vector2d(-31,-4.5), Math.toRadians(40), //-24-24
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .waitSeconds(4)
                    .back(5)

                    .lineToSplineHeading(new Pose2d(-36, -11, Math.toRadians(0)))

                    .addTemporalMarker(0, () -> {
                        //lift to high pos
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(-1);
                        drive.servo.setPosition(1);
                    })
                    .addTemporalMarker(4,() -> {
                        //drop lift
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(.1);
                    })
                    .addTemporalMarker(4.5, () -> {
                        drive.servo.setPosition(0);
                    })
                    //START -- TO HOLD POS BEFORE BACK
                    .addTemporalMarker(4.6, () -> {
                        //lift to stack pos
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(-1);
                    })
                    .addTemporalMarker(6,() -> {
                        //drop lift
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(.1);
                    })
                    .build();
            drive.followTrajectorySequence(trajSeq);
        }
        else {//right {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(-36.625, -62.375, Math.toRadians(90))) //34.25
                    //.forward(22)
                    .splineTo(new Vector2d(-31,-4.5), Math.toRadians(40), //-24-24
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .waitSeconds(4)
                    .back(5)

                    .lineToSplineHeading(new Pose2d(-36, -11, Math.toRadians(0)))
                    .waitSeconds(1)
                    .forward(25)

                    .addTemporalMarker(0, () -> {
                        //lift to high pos
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(-1);
                        drive.servo.setPosition(1);
                    })
                    .addTemporalMarker(4,() -> {
                        //drop lift
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(.1);
                    })
                    .addTemporalMarker(4.5, () -> {
                        drive.servo.setPosition(0);
                    })
                    //START -- TO HOLD POS BEFORE BACK
                    .addTemporalMarker(4.6, () -> {
                        //lift to stack pos
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(-1);
                    })
                    .addTemporalMarker(6,() -> {
                        //drop lift
                        drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        drive.liftMotor.setPower(.1);
                    })
                    .build();
            drive.followTrajectorySequence(trajSeq);
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
