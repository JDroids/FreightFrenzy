package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.CarouselForTime;
import org.firstinspires.ftc.teamcode.command.DepositCommand;
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarcodeTeamShippingElementPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class CarouselSideAuto extends OpModeTemplate {
    private final Alliance alliance;
    private BarcodeTeamShippingElementPipeline.Randomization randomization;

    private Point region1Pos;
    private Point region2Pos;
    private Point region3Pos;

    public CarouselSideAuto(Alliance alliance,
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

        mecanumDrive.setPoseEstimate(new Pose2d(-30.5,
                alliance.adjust(-62),
                Math.toRadians(alliance.adjust(270))));

        final TrajectorySequence toShippingHubLevel1 =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(-30.5,
                                alliance.adjust(-62),
                                Math.toRadians(alliance.adjust(270))))
                        .setReversed(true)
                        .splineTo(new Vector2d(-30, alliance.adjust(-20)),
                                Math.toRadians(alliance.adjust(0)))
                        .setReversed(false)
                        .build();
        final TrajectorySequence toCarouselLevel1 =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(-30, alliance.adjust(-20),
                                Math.toRadians(alliance.adjust(180+0))))
                        .splineTo(new Vector2d(-60, alliance.adjust(-40)),
                                Math.toRadians(alliance.adjust(180)))
                        .turn(Math.toRadians(alliance.adjust(-90)))
                        .back(18.0)
                        .build();

        final TrajectorySequence toShippingHubLevel2 =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(-30.5,
                                alliance.adjust(-62),
                                Math.toRadians(alliance.adjust(270))))
                        .setReversed(true)
                        .splineTo(new Vector2d(-30, alliance.adjust(-20)),
                                Math.toRadians(alliance.adjust(0)))
                        .setReversed(false)
                        .build();
        final TrajectorySequence toCarouselLevel2 =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(-30, alliance.adjust(-20),
                                Math.toRadians(alliance.adjust(180+0))))
                        .splineTo(new Vector2d(-60, alliance.adjust(-40)),
                                Math.toRadians(alliance.adjust(180)))
                        .turn(Math.toRadians(alliance.adjust(-90)))
                        .back(18.0)
                        .build();

        final TrajectorySequence toShippingHubLevel3 =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(-30.5,
                                alliance.adjust(-62),
                                Math.toRadians(alliance.adjust(270))))
                        .setReversed(true)
                        .splineTo(new Vector2d(-32, alliance.adjust(-28)),
                                Math.toRadians(alliance.adjust(15)))
                        .setReversed(false)
                        .build();
        final TrajectorySequence toCarouselLevel3 =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(-32, alliance.adjust(-28),
                                Math.toRadians(alliance.adjust(180+15))))
                        .splineTo(new Vector2d(-60, alliance.adjust(-40)),
                                Math.toRadians(alliance.adjust(180)))
                        .turn(Math.toRadians(alliance.adjust(-90)))
                        .back(18.0)
                        .build();

        final TrajectorySequence turnToCarouselForBlue = mecanumDrive.trajectorySequenceBuilder(
                new Pose2d(-60,
                alliance.adjust(-58),
                Math.toRadians(alliance.adjust(90))))
                    .forward(4.0)
                    .turn(alliance.equals(Alliance.RED) ? 0 : Math.toRadians(45))
                    .back(3.0)
                    .build();

        final TrajectorySequence park =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(-60,
                                alliance.adjust(-58),
                                Math.toRadians(alliance.adjust(90))))
                        .forward(alliance.equals(Alliance.RED) ? 23.0 : 25.0)
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
        TrajectorySequence toShippingHub;
        TrajectorySequence toCarousel;
        Runnable depositExtension;

        switch (randomization) {
            case LEVEL_1:
                toShippingHub = toShippingHubLevel1;
                toCarousel = toCarouselLevel1;
                depositExtension = () -> deposit.goToHeight(8.0);
                break;
            case LEVEL_2:
                toShippingHub = toShippingHubLevel2;
                toCarousel = toCarouselLevel2;
                depositExtension = () -> deposit.goToHeight(14.0);
                break;
            default: // go to level 3, even if it's null because more points if just guessing
                toShippingHub = toShippingHubLevel3;
                toCarousel = toCarouselLevel3;
                depositExtension = () -> deposit.goToHeight(26.0);
                break;
        }


        if (alliance.equals(Alliance.RED)) {
            schedule(new SequentialCommandGroup(
                    new DepositCommand(deposit, depositExtension).alongWith(
                            new FollowTrajectorySequence(mecanumDrive, toShippingHub)),
                    new DepositCommand(deposit, deposit::deploy),
                    new DepositCommand(deposit, deposit::retract).alongWith(
                            new FollowTrajectorySequence(mecanumDrive, toCarousel)),
                    new CarouselForTime(carousel, 3.0, alliance.adjust(-0.6)),
                    new FollowTrajectorySequence(mecanumDrive, park)
            ));
        }
        else {
            schedule(new SequentialCommandGroup(
                    new DepositCommand(deposit, depositExtension).alongWith(
                            new FollowTrajectorySequence(mecanumDrive, toShippingHub)),
                    new DepositCommand(deposit, deposit::deploy),
                    new DepositCommand(deposit, deposit::retract).alongWith(
                            new FollowTrajectorySequence(mecanumDrive, toCarousel)),
                    new FollowTrajectorySequence(mecanumDrive, turnToCarouselForBlue),
                    new CarouselForTime(carousel, 3.0, alliance.adjust(-0.6)),
                    new FollowTrajectorySequence(mecanumDrive, park)
            ));
        }
    }
}
