package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous
public class RedCarouselSideAuto extends CarouselSideAuto {
    public RedCarouselSideAuto() {
        super(Alliance.RED,
                new Point(0,140),
                new Point(130,140),
                new Point(300,140));
    }
}
