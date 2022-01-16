package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous
public class RedWarehouseSideAuto extends WarehouseSideAuto {
    public RedWarehouseSideAuto() {
        super(Alliance.RED,
                new Point(30,140),
                new Point(170,140),
                new Point(300,140));
    }
}
