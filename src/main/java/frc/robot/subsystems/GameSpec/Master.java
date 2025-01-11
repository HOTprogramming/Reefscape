package frc.robot.subsystems.GameSpec;

public class Master {

        public GameSpecCommand masterCommand;
    
        public enum GameSpecCommand{
        PACKAGE(0,1,2,3),
        TROUGH(0,1,2,3),
        L1(0,1,2,3),
        L2(0,1,2,3),
        L3(0,1,2,3),
        L4(0,1,2,3),
        HP(0,1,2,3),
        FLOOR(0,1,2,3),
        BARGE(0,1,2,3);


        public final double elevatorPos;
        public final double armPos;
        public final double coralPos;
        public final double algaePos;
  
        private GameSpecCommand (double elevatorPos, double armPos, double coralPos, double algaePos) {
            this.elevatorPos = elevatorPos; 
            this.armPos = armPos;
            this.coralPos = coralPos;
            this.algaePos = algaePos;
        }
    }
    
}
