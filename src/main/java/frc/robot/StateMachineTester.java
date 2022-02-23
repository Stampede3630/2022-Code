// package frc.robot;

// import io.github.oblarg.oblog.annotations.Log;

// public class StateMachineTester {
//     private static StateMachineTester SINGLE_INSTANCE = new StateMachineTester();
//     @Log
//     boolean StateHasFinished = false;
//     @Log
//     Boolean StateHasInitialized = false;
//     @Log
//     String CurrentState = "";
//     boolean StartingStateOverride;
    
//     public enum BigStupid {
//         DUMBTHING(SINGLE_INSTANCE::stupidCringe, "DUMBTHING");

//         private Runnable action;
//         private String nextState;

//         BigStupid(Runnable _action, String _nextState) {
//             action = _action;
//             nextState = _nextState;

//         }

//         public Runnable getAction() {
//             return action;
//         }

//         public String getNextState() {
//             return nextState;
//         }
//     }

//     public void autoRunner(String _startingState){
//         if (_startingState != "" && StartingStateOverride){
//             CurrentState = _startingState;
//             StartingStateOverride = false;
//         } 
       
//         if (CurrentState == "") {
//             CurrentState = BigStupid.values()[0].toString();
//         }

//         //if we made one round with the state, we have successfully initialized
//         BigStupid.valueOf(CurrentState).getAction().run();
//         if (!StateHasInitialized) {StateHasInitialized = true;}
//         if (StateHasFinished){
//             CurrentState = BigStupid.valueOf(CurrentState).getNextState();
//             StateHasFinished = false; 
//             StateHasInitialized = false;
//         }
//     }

//     private void stupidCringe(int thing) {
//         System.out.println("This doesn't work");
//         System.out.println(thing);
//     }
// }
