package frc.robot;

import edu.wpi.first.wpilibj.command.Command;

class ChangeDrive extends Command {

public void init(){
    Robot.DriveByStick = !Robot.DriveByStick;
}

public boolean isFinished(){
 return true;
}
}