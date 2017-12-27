import javax.swing.JApplet;

public class MazeApplet extends JApplet {
    public void init() {  
        int width  = 693;
        int height = 545;
        MazePanel content = new MazePanel(width,height);
        setContentPane(content);
    }
}
    
