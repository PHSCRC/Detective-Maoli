/* Tyler Chen 12/15/17 Move Interface
 * 
 */

public interface Move {
    public void execute(); // Performs the movement
    public boolean stop(); // Returns true if the movement is finished, false if it is in progress
}
