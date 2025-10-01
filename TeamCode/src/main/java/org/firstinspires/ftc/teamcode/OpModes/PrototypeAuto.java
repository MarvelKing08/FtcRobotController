package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.OpModes.pedropathing.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

// at the top of the file:
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@Autonomous(name = "PrototypeAuto")
public class PrototypeAuto extends NextFTCOpMode {
    public PrototypeAuto() {
//        addComponents(
//                new SubsystemComponent(Lift.INSTANCE, Claw.INSTANCE),
//                BulkReadComponent.INSTANCE
//        );

        // pedropathing component
        follower().breakFollowing();

        addComponents(
                /* existing components */
                new PedroComponent(Constants::createFollower)
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
//                Lift.INSTANCE.toHigh,
//                new ParallelGroup(
//                        Lift.INSTANCE.toMiddle,
//                        Claw.INSTANCE.close
//                ),
//                new Delay(0.5),
//                new ParallelGroup(
//                        Claw.INSTANCE.open,
//                        Lift.INSTANCE.toLow
//                )
        );


        // pedropathing code
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }
}