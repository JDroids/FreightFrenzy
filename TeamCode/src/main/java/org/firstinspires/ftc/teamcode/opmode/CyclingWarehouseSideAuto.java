package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.DepositCommand;
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarcodeTeamShippingElementPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class CyclingWarehouseSideAuto extends OpModeTemplate {
    private final Alliance alliance;
    private BarcodeTeamShippingElementPipeline.Randomization randomization;

    private Point region1Pos;
    private Point region2Pos;
    private Point region3Pos;

    public CyclingWarehouseSideAuto(Alliance alliance,
                                    Point region1Pos,
                                    Point region2Pos,
                                    Point region3Pos) {
        this.alliance = alliance;
        this.region1Pos = region1Pos;
        this.region2Pos = region2Pos;
        this.region3Pos = region3Pos;
    }

    @Override
    public void initialize() {
        initHardware(true);

        mecanumDrive.setPoseEstimate(new Pose2d(7,
                alliance.adjust(-62),
                Math.toRadians(alliance.adjust(270))));

        final TrajectorySequence toShippingHubLevel1 =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(7,
                                alliance.adjust(-62),
                                Math.toRadians(alliance.adjust(270))))
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, alliance.adjust(-41)),
                                Math.toRadians(alliance.adjust(90)))
                        .setReversed(false)
                        .build();

        final TrajectorySequence toShippingHubLevel2 =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(7,
                                alliance.adjust(-62),
                                Math.toRadians(alliance.adjust(270))))
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, alliance.adjust(-41)),
                                Math.toRadians(alliance.adjust(90)))
                        .setReversed(false)
                        .build();

        final TrajectorySequence toShippingHubLevel3 =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(7,
                                alliance.adjust(-62),
                                Math.toRadians(alliance.adjust(270))))
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, alliance.adjust(-44)),
                                Math.toRadians(alliance.adjust(90)))
                        .setReversed(false)
                        .build();

        final TrajectorySequence intakeCycle =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(-12,
                                alliance.adjust(-44),
                                Math.toRadians(alliance.adjust(270))))
                        .forward(20.0)
                        .turn(Math.toRadians(
                                alliance == Alliance.RED
                                        ? alliance.adjust(90.0)
                                        : alliance.adjust(80.0)))
                        .forward(60.0)
                        .build();

        final TrajectorySequence driveForwardsToIntake =
                mecanumDrive.trajectorySequenceBuilder(intakeCycle.end())
                    .forward(5.0)
                    .build();

        final TrajectorySequence toShippingHubForCycling =
                mecanumDrive.trajectorySequenceBuilder(driveForwardsToIntake.end())
                        .setReversed(true)
                        .back(20.0)
                        .splineTo(new Vector2d(-12, alliance.adjust(-44)),
                                Math.toRadians(alliance.adjust(90)))
                        .setReversed(false)
                        .build();

        OpenCvWebcam webcam;
        BarcodeTeamShippingElementPipeline pipeline = new BarcodeTeamShippingElementPipeline(
                region1Pos,
                region2Pos,
                region3Pos);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // intentional noop
            }
        });

        while (!isStarted() && !isStopRequested()) {
            randomization = pipeline.getRandomization();
            telemetry.addData("Randomization", randomization);
            telemetry.update();
        }

        webcam.closeCameraDevice();

        // change which ones based on vision
        TrajectorySequence toShippingHubPreload;
        Runnable depositExtension;

        switch (randomization) {
            case LEVEL_1:
                toShippingHubPreload = toShippingHubLevel1;
                depositExtension = deposit::goToLevel1;
                break;
            case LEVEL_2:
                toShippingHubPreload = toShippingHubLevel2;
                depositExtension = () -> deposit.goToHeight(14.0);
                break;
            default: // go to level 3, even if it's null because more points if just guessing
                toShippingHubPreload = toShippingHubLevel3;
                depositExtension = () -> deposit.goToHeight(24.5);
                break;
        }

        schedule(new SequentialCommandGroup(
                new InstantCommand(depositExtension),
                new FollowTrajectorySequence(mecanumDrive, toShippingHubPreload),
                new InstantCommand(deposit::deploy).andThen(new WaitCommand(1000)),
                cycleCommand(intakeCycle, driveForwardsToIntake, toShippingHubForCycling),
                new FollowTrajectorySequence(mecanumDrive, intakeCycle)
        ));
    }

    public Command cycleCommand(TrajectorySequence intakeCycle,
                                TrajectorySequence driveForwardsToIntake,
                                TrajectorySequence toShippingHub) {
        return new SequentialCommandGroup(
                new FollowTrajectorySequence(mecanumDrive, intakeCycle),
                new InstantCommand(deposit::retracted),
                new WaitCommand(1000),
                new InstantCommand(intake::intake),
                new FollowTrajectorySequence(mecanumDrive, driveForwardsToIntake),
                new InstantCommand(() -> deposit.goToHeight(24.5)),
                new WaitCommand(1000),
                new InstantCommand(intake::outtake),
                new FollowTrajectorySequence(mecanumDrive, toShippingHub),
                new InstantCommand(intake::stop),
                new InstantCommand(deposit::deploy).andThen(new WaitCommand(1000))
        );
    }
}
