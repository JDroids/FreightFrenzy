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

public abstract class WarehouseSideAuto extends OpModeTemplate {
    private final Alliance alliance;
    private BarcodeTeamShippingElementPipeline.Randomization randomization;

    private Point region1Pos;
    private Point region2Pos;
    private Point region3Pos;

    public WarehouseSideAuto(Alliance alliance,
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

        final TrajectorySequence parkAll =
                mecanumDrive.trajectorySequenceBuilder(
                        new Pose2d(-12,
                                alliance.adjust(-44),
                                Math.toRadians(alliance.adjust(270))))
                        .forward(20.0)
                        .turn(Math.toRadians(alliance.adjust(90.0)))
                        .forward(60.0)
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
        TrajectorySequence park;
        Runnable depositExtension;

        switch (randomization) {
            case LEVEL_1:
                toShippingHub = toShippingHubLevel1;
                park = parkAll;
                depositExtension = deposit::goToLevel1;
                break;
            case LEVEL_2:
                toShippingHub = toShippingHubLevel2;
                park = parkAll;
                depositExtension = () -> deposit.goToHeight(14.0);
                break;
            default: // go to level 3, even if it's null because more points if just guessing
                toShippingHub = toShippingHubLevel3;
                park = parkAll;
                depositExtension = () -> deposit.goToHeight(24.5);
                break;
        }


        schedule(new SequentialCommandGroup(
                new DepositCommand(deposit, depositExtension).alongWith(
                        new FollowTrajectorySequence(mecanumDrive, toShippingHub)),
                new DepositCommand(deposit, deposit::deploy),
                new DepositCommand(deposit, deposit::retract).alongWith(
                        new FollowTrajectorySequence(mecanumDrive, park))
        ));
    }
}
