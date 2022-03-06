package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
                        .splineTo(new Vector2d(-12, alliance.adjust(-40)),
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

        final TrajectorySequence intakeCycle1 =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(-12,
                                alliance.adjust(-44),
                                Math.toRadians(alliance.adjust(270))))
                        .splineTo(new Vector2d(12.0, alliance.adjust(-65)), 0.0)
                        .forward(36.0)
                        .build();

        final TrajectorySequence toShippingHubForCycling1 =
                mecanumDrive.trajectorySequenceBuilder(intakeCycle1.end())
                        .setReversed(true)
                        .back(30.0)
                        .splineTo(new Vector2d(-4, alliance.adjust(-46)),
                                Math.toRadians(alliance.adjust(110)))
                        .setReversed(false)
                        .build();

        final TrajectorySequence intakeCycle2 =
                mecanumDrive.trajectorySequenceBuilder(toShippingHubForCycling1.end())
                        .splineTo(new Vector2d(12.0, alliance.adjust(-65)), 0.0)
                        .forward(40.0)
                        .build();

        final TrajectorySequence toShippingHubForCycling2 =
                mecanumDrive.trajectorySequenceBuilder(intakeCycle2.end())
                        .setReversed(true)
                        .back(30.0)
                        .splineTo(new Vector2d(-20, alliance.adjust(-46)),
                                Math.toRadians(alliance.adjust(75)))
                        .setReversed(false)
                        .build();

        final TrajectorySequence park =
                mecanumDrive.trajectorySequenceBuilder(toShippingHubForCycling2.end())
                        .splineTo(new Vector2d(12.0, alliance.adjust(-68)), 0.0)
                        .forward(36.0)
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
                new FollowTrajectorySequence(mecanumDrive, toShippingHubPreload).alongWith(
                        new InstantCommand(depositExtension)
                        ),
                new InstantCommand(deposit::deploy).alongWith(new WaitCommand(1000)),
                cycleCommand(intakeCycle1, toShippingHubForCycling1),
                cycleCommand(intakeCycle2, toShippingHubForCycling2),
                new FollowTrajectorySequence(mecanumDrive, park).alongWith(
                    new InstantCommand(deposit::retract)
                )
        ));
    }

    public Command cycleCommand(TrajectorySequence intakeCycle,
                                TrajectorySequence toShippingHub) {
        return new SequentialCommandGroup(
                new FollowTrajectorySequence(mecanumDrive, intakeCycle).alongWith(
                    new InstantCommand(deposit::retract).andThen(
                            new WaitCommand(1000),
                            new InstantCommand(intake::intake))
                        ),
                new WaitCommand(1000),
                new InstantCommand(() -> intake.setPower(0.2)),
                new FollowTrajectorySequence(mecanumDrive, toShippingHub).alongWith(
                        new InstantCommand(() -> deposit.goToHeight(24.5)).andThen(
                                new WaitCommand(1000),
                                new InstantCommand(intake::outtake)
                            )
                    ),
                new InstantCommand(intake::stop),
                new InstantCommand(deposit::deploy).andThen(new WaitCommand(1000))
        );
    }
}
