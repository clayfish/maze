package maze41;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Random;
import java.util.Stack;
import javax.swing.*;
import javax.swing.event.*;

/**
 *
 * @author Νίκος Κανάργιας, φοιτητής ΕΑΠ, ΠΛΗ31 2012-13
 * E-mail: nkana@tee.gr
 * @version 4.1
 * 
 * Το πρόγραμμα λύνει και οπτικοποιεί το πρόβλημα του σχεδιασμού κίνησης ρομπότ
 * (robot motion planning) υλοποιώντας μια παραλλαγή των αλγόριθμων
 * DFS, BFS και A*, όπως αυτοί περιγράφονται στο βιβλίο
 * "Τεχνητή Νοημοσύνη και Έμπειρα Συστήματα" της Ε. Κεραυνού, ΠΑΤΡΑ 2000
 * καθώς και τον αλγόριθμο της άπληστης αναζήτησης, σαν ειδική περίπτωση του Α*.
 * 
 * Το πρόγραμμα ακόμη υλοποιεί τον αλγόριθμο του Dijkstra όπως ακριβώς αυτός 
 * περιγράφεται το σχετικό άρθρο της Wikipedia.
 * http://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
 * 
 * Γίνεται ολοφάνερη η υπεροχή των αλγόριθμων Α* και Dijkstra απέναντι στους άλλους τρεις.
 * 
 * Ο χρήστης μπορεί μεταβάλει το πλήθος των κελιών του πλέγματος, δηλώνοντας
 * τον επιθυμητό αριθμό γραμμών και στηλών.
 * 
 * Ο χρήστης μπορεί να προσθέσει όσα εμπόδια θέλει, όπως θα σχεδίαζε ελεύθερες
 * καμπύλες με ένα σχεδιαστικό πρόγραμμα.
 * 
 * Αφαίρεση μεμονωμένων εμποδίων γίνεται κάνοντας κλικ επάνω τους.
 * 
 * Η θέση του ρομπότ ή/και του στόχου μπορεί να αλλάξει με σύρσιμο με το ποντίκι.
 * 
 * Μεταπήδηση από την αναζήτηση "Βήμα-Βήμα" στην αναζήτηση με "Κίνηση" και αντίστροφα
 * γίνεται πιέζοντας το αντίστοιχο κουμπί, ακόμη και όταν η αναζήτηση είναι σε εξέλιξη.
 * 
 * Η ταχύτητα μιας αναζήτησης με κίνηση μπορεί να μεταβληθεί, ακόμη και αν η
 * αναζήτηση είναι σε εξέλιξη, αρκεί να τοποθετηθεί ο  slider "Ταχύτητα" στην νέα
 * επιθυμητή θέση και στη συνέχεια πιεστεί το κουμπί "Κίνηση".
 * 
 * Η εφαρμογή θεωρεί ότι το ίδιο το ρομπότ έχει κάποιον όγκο. Συνεπώς δεν μπορεί
 * να κινηθεί διαγώνια προς ελεύθερο κελί ανάμεσα από δύο εμπόδια που εφάπτονται
 * σε μία κορυφή τους.
 *
 * Δεν είναι δυνατή η αλλαγή των θέσεων εμποδίων, ρομπότ και στόχου όπως και του
 * είδους του αλγόριθμου, ενόσω η αναζήτηση είναι σε εξέλιξη.
 * 
 * Συνιστάται η αποφυγή σχεδίασης βελών προς προκατόχους σε πλέγματα μεγάλης διάστασης.
 */

public class Maze41 {

    public static JFrame mazeFrame;  // Η κύρια φόρμα του προγράμματος
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        int width  = 693;
        int height = 545;
        mazeFrame = new JFrame("Maze 4.1");
        mazeFrame.setContentPane(new MazePanel(width,height));
        mazeFrame.pack();
        mazeFrame.setResizable(false);

        // τοποθετούμε τη φόρμα στο κέντρο της οθόνης
        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
        double screenWidth = screenSize.getWidth();
        double ScreenHeight = screenSize.getHeight();
        int x = ((int)screenWidth-width)/2;
        int y = ((int)ScreenHeight-height)/2;

        mazeFrame.setLocation(x,y);
        mazeFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        mazeFrame.setVisible(true);
    } // end main()
    
    /**
     * Αυτή η κλάση καθορίζει το περιεχόμενο της κύριας φόρμας
     * και περιέχει όλη την λειτουργικότητα του προγράμματος.
     */
    public static class MazePanel extends JPanel {
        
        /*
         **********************************************************
         *          Ένθετες κλάσεις στην MazePanel
         **********************************************************
         */
        
        /**
         * Βοηθητική κλάση που αναπαριστά το κελί του πλέγματος
         */
        private class Cell {
            int row;   // ο αριθμός γραμμής του κελιού (Η γραμμή 0 είναι η πάνω)
            int col;   // ο αριθμός στήλης του κελιού (Η στήλη 0 είναι η αριστερή)
            int g;     // η τιμή της συνάρτησης g των αλγόριθμων Α* και Greedy
            int h;     // η τιμή της συνάρτησης h των αλγόριθμων Α* και Greedy
            int f;     // η τιμή της συνάρτησης f των αλγόριθμων Α* και Greedy
            int dist;  // η απόσταση του κελιού από την αρχική θέση του ρομπότ
                       // δηλαδή η ετικέτα που ενημερώνει ο αλγόριθμος Dijkstra
            Cell prev; // κάθε κατάσταση αντιστοιχεί σε κάποιο cell
                       // και κάθε κατάσταση έχει μια προκάτοχο η οποία
                       // αποθηκεύεται σε αυτή τη μεταβλητή
            
            public Cell(int row, int col){
               this.row = row;
               this.col = col;
            }
        } // end nested class Cell
      
        /**
         * Βοηθητική κλάση που καθορίζει ότι τα κελιά θα ταξινομούνται
         * με βάση το πεδίο τους f
         */
        private class CellComparatorByF implements Comparator<Cell>{
            @Override
            public int compare(Cell cell1, Cell cell2){
                return cell1.f-cell2.f;
            }
        } // end nested class CellComparatorByF
      
        /**
         * Βοηθητική κλάση που καθορίζει ότι τα κελιά θα ταξινομούνται
         * με βάση το πεδίο τους dist
         */
        private class CellComparatorByDist implements Comparator<Cell>{
            @Override
            public int compare(Cell cell1, Cell cell2){
                return cell1.dist-cell2.dist;
            }
        } // end nested class CellComparatorByDist
      
        /**
         * Κλάση που χειρίζεται τις κινήσεις του ποντικιού καθώς "ζωγραφίζουμε"
         * εμπόδια ή μετακινούμε το ρομπότ ή/και τον στόχο.
         */
        private class MouseHandler implements MouseListener, MouseMotionListener {
            private int cur_row, cur_col, cur_val;
            @Override
            public void mousePressed(MouseEvent evt) {
                int row = (evt.getY() - 10) / squareSize;
                int col = (evt.getX() - 10) / squareSize;
                if (row >= 0 && row < rows && col >= 0 && col < columns && !searching && !found) {
                    cur_row = row;
                    cur_col = col;
                    cur_val = grid[row][col];
                    if (cur_val == EMPTY){
                        grid[row][col] = OBST;
                    }
                    if (cur_val == OBST){
                        grid[row][col] = EMPTY;
                    }
                }
                repaint();
            }

            @Override
            public void mouseDragged(MouseEvent evt) {
                int row = (evt.getY() - 10) / squareSize;
                int col = (evt.getX() - 10) / squareSize;
                if (row >= 0 && row < rows && col >= 0 && col < columns && !searching && !found){
                    if ((row*columns+col != cur_row*columns+cur_col) && (cur_val == ROBOT || cur_val == TARGET)){
                        int new_val = grid[row][col];
                        if (new_val == EMPTY){
                            grid[row][col] = cur_val;
                            if (cur_val == ROBOT) {
                                robotStart.row = row;
                                robotStart.col = col;
                            } else {
                                targetPos.row = row;
                                targetPos.col = col;
                            }
                            grid[cur_row][cur_col] = new_val;
                            cur_row = row;
                            cur_col = col;
                            if (cur_val == ROBOT) {
                                robotStart.row = cur_row;
                                robotStart.col = cur_col;
                            } else {
                                targetPos.row = cur_row;
                                targetPos.col = cur_col;
                            }
                            cur_val = grid[row][col];
                        }
                    } else if (grid[row][col] != ROBOT && grid[row][col] != TARGET){
                        grid[row][col] = OBST;            
                    }
                }
                repaint();
            }

            @Override
            public void mouseReleased(MouseEvent evt) { }
            @Override
            public void mouseEntered(MouseEvent evt) { }
            @Override
            public void mouseExited(MouseEvent evt) { }
            @Override
            public void mouseMoved(MouseEvent evt) { }
            @Override
            public void mouseClicked(MouseEvent evt) { }
            
        } // end nested class MouseHandler
        
        /**
         * Όταν ο χρήστης πιέζει ένα κουμπί εκτελεί την αντίστοιχη λειτουργικότητα
         */
        private class ActionHandler implements ActionListener {
            @Override
            public void actionPerformed(ActionEvent evt) {
                String cmd = evt.getActionCommand();
                if (cmd.equals("Καθάρισμα")) {
                    fillGrid();
                    dfs.setEnabled(true);
                    bfs.setEnabled(true);
                    aStar.setEnabled(true);
                    greedy.setEnabled(true);
                    dijkstra.setEnabled(true);
                    diagonal.setEnabled(true);
                    drawArrows.setEnabled(true);
                } else if (cmd.equals("Βήμα - Βήμα") && !found && !endOfSearch) {
                    // Η αρχικοποίηση του Dijkstra πρέπει να γίνει ακριβώς πριν την
                    // έναρξη της αναζήτησης, γιατί τα εμπόδια πρέπει να είναι στη θέση τους.
                    if (!searching && dijkstra.isSelected()) {
                        initializeDijkstra();
                    }
                    searching = true;
                    message.setText(msgSelectStepByStepEtc);
                    dfs.setEnabled(false);
                    bfs.setEnabled(false);
                    aStar.setEnabled(false);
                    greedy.setEnabled(false);
                    dijkstra.setEnabled(false);
                    diagonal.setEnabled(false);
                    drawArrows.setEnabled(false);
                    timer.stop();
                    // Εδώ αποφασίζουμε αν μπορούμε να συνεχίσουμε την
                    // 'Βήμα-Βήμα' αναζήτηση ή όχι
                    // Για την περίπτωση των αλγόριθμων DFS, BFS, A* και Greedy
                    // εδώ έχουμε το 2ο βήμα τους:
                    // 2. Εάν ΑΝΟΙΚΤΕΣ = [], τότε τερμάτισε. Δεν υπάρχει λύση.
                    if ((dijkstra.isSelected() && graph.isEmpty()) ||
                                  (!dijkstra.isSelected() && openSet.isEmpty()) ) {
                        endOfSearch = true;
                        grid[robotStart.row][robotStart.col]=ROBOT;
                        message.setText(msgNoSolution);
                    } else {
                        expandNode();
                        if (found) {
                            plotRoute();
                        }
                    }
                    repaint();
                } else if (cmd.equals("Κίνηση") && !endOfSearch) {
                    if (!searching && dijkstra.isSelected()) {
                        initializeDijkstra();
                    }
                    searching = true;
                    message.setText(msgSelectStepByStepEtc);
                    dfs.setEnabled(false);
                    bfs.setEnabled(false);
                    aStar.setEnabled(false);
                    greedy.setEnabled(false);
                    dijkstra.setEnabled(false);
                    diagonal.setEnabled(false);
                    drawArrows.setEnabled(false);
                    timer.setDelay(delay);
                    timer.start();
                }
            }
        } // end nested class ActionHandler
   
        /**
         * Η κλάση που είναι υπεύθυνη για το animation
         */
        private class RepaintAction implements ActionListener {
            @Override
            public void actionPerformed(ActionEvent evt) {
                // Εδώ αποφασίζουμε αν μπορούμε να συνεχίσουμε ή όχι
                // την αναζήτηση με 'Κίνηση'.
                // Για την περίπτωση των αλγόριθμων DFS, BFS, A* και Greedy
                // εδώ έχουμε το 2ο βήμα τους:
                // 2. Εάν ΑΝΟΙΚΤΕΣ = [], τότε τερμάτισε. Δεν υπάρχει λύση.
                if ((dijkstra.isSelected() && graph.isEmpty()) ||
                              (!dijkstra.isSelected() && openSet.isEmpty()) ) {
                    endOfSearch = true;
                    grid[robotStart.row][robotStart.col]=ROBOT;
                    message.setText(msgNoSolution);
                } else {
                    expandNode();
                    if (found) {
                        timer.stop();
                        endOfSearch = true;
                        plotRoute();
                    }
                }
                repaint();
            }
        } // end nested class RepaintAction
      
        /**
         * Η κλάση που δημιουργεί το AboutBox
         */
        private class AboutBox extends JDialog{

            public AboutBox(Frame parent, boolean modal){
                super(parent, modal);
                // τοποθετούμε το aboutBox στο κέντρο της οθόνης
                Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
                double screenWidth = screenSize.getWidth();
                double ScreenHeight = screenSize.getHeight();
                int width = 450;
                int height = 185;
                int x = ((int)screenWidth-width)/2;
                int y = ((int)ScreenHeight-height)/2;
                setSize(width,height);
                setLocation(x, y);
        
                setResizable(false);
                setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);

                JLabel title = new JLabel("Maze", JLabel.CENTER);
                title.setFont(new Font("Helvetica",Font.PLAIN,24));
                title.setForeground(new java.awt.Color(255, 153, 102));

                JLabel version = new JLabel("Έκδοση: 4.1", JLabel.CENTER);
                version.setFont(new Font("Helvetica",Font.BOLD,14));

                JLabel programmer = new JLabel("Προγραμματιστής: Νίκος Κανάργιας", JLabel.CENTER);
                programmer.setFont(new Font("Helvetica",Font.PLAIN,16));

                JLabel email = new JLabel("E-mail: nkana@tee.gr", JLabel.CENTER);
                email.setFont(new Font("Helvetica",Font.PLAIN,14));

                JLabel sourceCode = new JLabel("Κώδικας και τεκμηρίωση:", JLabel.CENTER);
                sourceCode.setFont(new Font("Helvetica",Font.PLAIN,14));

                JLabel link = new JLabel("https://www.dropbox.com/sh/3tl3vmsd8ebxo24/xKmjl8uXX7", JLabel.CENTER);
                link.setFont(new Font("Helvetica",Font.PLAIN,16));

                JLabel dummy = new JLabel("");

                add(title);
                add(version);
                add(programmer);
                add(email);
                add(sourceCode);
                add(link);
                add(dummy);

                title.     setBounds(5,  0, 430, 30);
                version.   setBounds(5, 30, 430, 20);
                programmer.setBounds(5, 55, 430, 20);
                email.     setBounds(5, 80, 430, 20);
                sourceCode.setBounds(5,105, 430, 20);
                link      .setBounds(5,130, 430, 20);
                dummy.     setBounds(5,155, 430, 20);
            }
        } // end nested class AboutBox

        /**
         * Δημιουργεί έναν τυχαίο, τέλειο (χωρίς κύκλους) λαβύρινθο
         * 
         * Ο κώδικας είναι μια προσαρμογή, με τον αυθεντικό σχολιασμό, της απάντησης που έδωσε ο χρήστης
         * DoubleMx2 στις 25 Αυγούστου σε σχετικό ερώτημα του χρήστη nazar_art στο stackoverflow.com
         * http://stackoverflow.com/questions/18396364/maze-generation-arrayindexoutofboundsexception
         */
        private class MyMaze {
            private int dimensionX, dimensionY; // dimension of maze
            private int gridDimensionX, gridDimensionY; // dimension of output grid
            private char[][] mazeGrid; // output grid
            private Cell[][] cells; // 2d array of Cells
            private Random random = new Random(); // The random object

            // initialize with x and y the same
            public MyMaze(int aDimension) {
                // Initialize
                this(aDimension, aDimension);
            }
            // constructor
            public MyMaze(int xDimension, int yDimension) {
                dimensionX = xDimension;
                dimensionY = yDimension;
                gridDimensionX = xDimension * 2 + 1;
                gridDimensionY = yDimension * 2 + 1;
                mazeGrid = new char[gridDimensionX][gridDimensionY];
                init();
                generateMaze();
            }

            private void init() {
                // create cells
                cells = new Cell[dimensionX][dimensionY];
                for (int x = 0; x < dimensionX; x++) {
                    for (int y = 0; y < dimensionY; y++) {
                        cells[x][y] = new Cell(x, y, false); // create cell (see Cell constructor)
                    }
                }
            }

            // inner class to represent a cell
            private class Cell {
                int x, y; // coordinates
                // cells this cell is connected to
                ArrayList<Cell> neighbors = new ArrayList<>();
                // impassable cell
                boolean wall = true;
                // if true, has yet to be used in generation
                boolean open = true;
                // construct Cell at x, y
                Cell(int x, int y) {
                    this(x, y, true);
                }
                // construct Cell at x, y and with whether it isWall
                Cell(int x, int y, boolean isWall) {
                    this.x = x;
                    this.y = y;
                    this.wall = isWall;
                }
                // add a neighbor to this cell, and this cell as a neighbor to the other
                void addNeighbor(Cell other) {
                    if (!this.neighbors.contains(other)) { // avoid duplicates
                        this.neighbors.add(other);
                    }
                    if (!other.neighbors.contains(this)) { // avoid duplicates
                        other.neighbors.add(this);
                    }
                }
                // used in updateGrid()
                boolean isCellBelowNeighbor() {
                    return this.neighbors.contains(new Cell(this.x, this.y + 1));
                }
                // used in updateGrid()
                boolean isCellRightNeighbor() {
                    return this.neighbors.contains(new Cell(this.x + 1, this.y));
                }
                // useful Cell equivalence
                @Override
                public boolean equals(Object other) {
                    if (!(other instanceof Cell)) return false;
                    Cell otherCell = (Cell) other;
                    return (this.x == otherCell.x && this.y == otherCell.y);
                }

                // should be overridden with equals
                @Override
                public int hashCode() {
                    // random hash code method designed to be usually unique
                    return this.x + this.y * 256;
                }

            }
            // generate from upper left (In computing the y increases down often)
            private void generateMaze() {
                generateMaze(0, 0);
            }
            // generate the maze from coordinates x, y
            private void generateMaze(int x, int y) {
                generateMaze(getCell(x, y)); // generate from Cell
            }
            private void generateMaze(Cell startAt) {
                // don't generate from cell not there
                if (startAt == null) return;
                startAt.open = false; // indicate cell closed for generation
                ArrayList<Cell> cellsList = new ArrayList<>();
                cellsList.add(startAt);

                while (!cellsList.isEmpty()) {
                    Cell cell;
                    // this is to reduce but not completely eliminate the number
                    //   of long twisting halls with short easy to detect branches
                    //   which results in easy mazes
                    if (random.nextInt(10)==0)
                        cell = cellsList.remove(random.nextInt(cellsList.size()));
                    else cell = cellsList.remove(cellsList.size() - 1);
                    // for collection
                    ArrayList<Cell> neighbors = new ArrayList<>();
                    // cells that could potentially be neighbors
                    Cell[] potentialNeighbors = new Cell[]{
                        getCell(cell.x + 1, cell.y),
                        getCell(cell.x, cell.y + 1),
                        getCell(cell.x - 1, cell.y),
                        getCell(cell.x, cell.y - 1)
                    };
                    for (Cell other : potentialNeighbors) {
                        // skip if outside, is a wall or is not opened
                        if (other==null || other.wall || !other.open) continue;
                        neighbors.add(other);
                    }
                    if (neighbors.isEmpty()) continue;
                    // get random cell
                    Cell selected = neighbors.get(random.nextInt(neighbors.size()));
                    // add as neighbor
                    selected.open = false; // indicate cell closed for generation
                    cell.addNeighbor(selected);
                    cellsList.add(cell);
                    cellsList.add(selected);
                }
                updateGrid();
            }
            // used to get a Cell at x, y; returns null out of bounds
            public Cell getCell(int x, int y) {
                try {
                    return cells[x][y];
                } catch (ArrayIndexOutOfBoundsException e) { // catch out of bounds
                    return null;
                }
            }
            // draw the maze
            public void updateGrid() {
                char backChar = ' ', wallChar = 'X', cellChar = ' ';
                // fill background
                for (int x = 0; x < gridDimensionX; x ++) {
                    for (int y = 0; y < gridDimensionY; y ++) {
                        mazeGrid[x][y] = backChar;
                    }
                }
                // build walls
                for (int x = 0; x < gridDimensionX; x ++) {
                    for (int y = 0; y < gridDimensionY; y ++) {
                        if (x % 2 == 0 || y % 2 == 0)
                            mazeGrid[x][y] = wallChar;
                    }
                }
                // make meaningful representation
                for (int x = 0; x < dimensionX; x++) {
                    for (int y = 0; y < dimensionY; y++) {
                        Cell current = getCell(x, y);
                        int gridX = x * 2 + 1, gridY = y * 2 + 1;
                        mazeGrid[gridX][gridY] = cellChar;
                        if (current.isCellBelowNeighbor()) {
                            mazeGrid[gridX][gridY + 1] = cellChar;
                        }
                        if (current.isCellRightNeighbor()) {
                            mazeGrid[gridX + 1][gridY] = cellChar;
                        }
                    }
                }
                
                // Δημιουργούμε ένα καθαρό πλέγμα ...
                searching = false;
                endOfSearch = false;
                fillGrid();
                // ... και αντιγράφουμε μέσα του τις θέσεις των εμποδίων
                // που δημιούργησε ο αλγόριθμος κατασκευής του λαβύρινθου
                for (int x = 0; x < gridDimensionX; x++) {
                    for (int y = 0; y < gridDimensionY; y++) {
                        if (mazeGrid[x][y] == wallChar && grid[x][y] != ROBOT && grid[x][y] != TARGET){
                            grid[x][y] = OBST;
                        }
                    }
                }
            }
        } // end nested class MyMaze
        
        /*
         **********************************************************
         *          Σταθερές της κλάσης MazePanel
         **********************************************************
         */
        
        private final static int
            INFINITY = Integer.MAX_VALUE, // Η αναπαράσταση του άπειρου
            EMPTY    = 0,  // κενό κελί
            OBST     = 1,  // κελί με εμπόδιο
            ROBOT    = 2,  // η θέση του ρομπότ
            TARGET   = 3,  // η θέση του στόχου
            FRONTIER = 4,  // κελιά του μετώπου αναζήτησης (ΑΝΟΙΚΤΈΣ καταστάσεις)
            CLOSED   = 5,  // κελιά κλειστών καταστάσεων
            ROUTE    = 6;  // κελιά που σχηματίζουν τη διαδρομή ρομπότ-στόχος
        
        // Μηνύματα προς τον χρήστη
        private final static String
            msgDrawAndSelect =
                "\"Σχεδιάστε\" εμπόδια και επιλέξτε 'Βήμα-Βήμα' ή 'Κίνηση'",
            msgSelectStepByStepEtc =
                "Επιλέξτε 'Βήμα-Βήμα' ή 'Κίνηση' ή 'Καθάρισμα'",
            msgNoSolution =
                "Δεν υπάρχει διαδρομή για τον στόχο !!!";

        /*
         **********************************************************
         *          Μεταβλητές της κλάσης MazePanel
         **********************************************************
         */
        
        JTextField rowsField, columnsField;
        
        int rows    = 41,           // Ο αριθμός των γραμμών του πλέγματος
            columns = 41,           // Ο αριθμός των στηλών του πλέγματος
            squareSize = 500/rows;  // Το μέγεθος του κελιού σε pixels
        

        int arrowSize = squareSize/2; // Το μέγεθος της μύτης του βέλους
                                      // που δείχνει το προκάτοχο κελί
        ArrayList<Cell> openSet   = new ArrayList();// το σύνολο ανοικτών καταστάσεων
        ArrayList<Cell> closedSet = new ArrayList();// το σύνολο κλειστών καταστάσεων
        ArrayList<Cell> graph     = new ArrayList();// το σύνολο των κορυφών του γράφου
                                                    // που εξερευνά ο αλγόριθμος Dijkstra
         
        Cell robotStart; // η αρχική θέση του ρομπότ
        Cell targetPos;  // η θέση του στόχου
      
        JLabel message;  // μήνυμα προς τον χρήστη
        
        // τα κουμπιά για την επιλογή του αλγόριθμου
        JRadioButton dfs, bfs, aStar, greedy, dijkstra;
        
        // ο slider για την ρύθμιση της ταχύτητας του animation
        JSlider slider;
        
        // Επιτρέπονται διαγώνιες κινήσεις;
        JCheckBox diagonal;
        // Σχεδίαση βελών προς προκατόχους
        JCheckBox drawArrows;

        int[][] grid;        // το πλέγμα
        boolean found;       // flag ότι βρέθηκε ο στόχος
        boolean searching;   // flag ότι η αναζήτηση είναι σε εξέλιξη
        boolean endOfSearch; // flag ότι η αναζήτηση έφθασε στο τέρμα
        int delay;           // ο χρόνος της καθυστέρησης σε msec του animation
        int expanded;        // ο αριθμός των κόμβων που έχουν αναπτυχθεί
        
        // το αντικείμενο που ελέγχει το animation
        RepaintAction action = new RepaintAction();
        
        // ο Timer που ρυθμίζει την ταχύτητα εκτέλεσης του animation
        Timer timer;
      
        /**
         * Ο δημιουργός του panel
         * @param width το πλάτος του panel.
         * @param height το ύψος panel.
         */
        public MazePanel(int width, int height) {
      
            setLayout(null);
            
            MouseHandler listener = new MouseHandler();
            addMouseListener(listener);
            addMouseMotionListener(listener);

            setBorder(BorderFactory.createMatteBorder(2,2,2,2,Color.blue));

            setPreferredSize( new Dimension(width,height) );

            grid = new int[rows][columns];

            // Δημιουργούμε τα περιεχόμενα του panel

            message = new JLabel(msgDrawAndSelect, JLabel.CENTER);
            message.setForeground(Color.blue);
            message.setFont(new Font("Helvetica",Font.PLAIN,16));

            JLabel rowsLbl = new JLabel("Πλήθος γραμμών (5-83):", JLabel.RIGHT);
            rowsLbl.setFont(new Font("Helvetica",Font.PLAIN,13));

            rowsField = new JTextField();
            rowsField.setText(Integer.toString(rows));

            JLabel columnsLbl = new JLabel("Πλήθος στηλών (5-83):", JLabel.RIGHT);
            columnsLbl.setFont(new Font("Helvetica",Font.PLAIN,13));

            columnsField = new JTextField();
            columnsField.setText(Integer.toString(columns));

            JButton resetButton = new JButton("Νέο πλέγμα");
            resetButton.addActionListener(new ActionHandler());
            resetButton.setBackground(Color.lightGray);
            resetButton.setToolTipText
                    ("Καθαρίζει και επανασχεδιάζει το πλέγμα με βάση τις δοθείσες διαστάσεις του");
            resetButton.addActionListener(new java.awt.event.ActionListener() {
                @Override
                public void actionPerformed(java.awt.event.ActionEvent evt) {
                    resetButtonActionPerformed(evt);
                }
            });

            JButton mazeButton = new JButton("Λαβύρινθος");
            mazeButton.addActionListener(new ActionHandler());
            mazeButton.setBackground(Color.lightGray);
            mazeButton.setToolTipText
                    ("Δημιουργεί έναν τυχαίο λαβύρινθο");
            mazeButton.addActionListener(new java.awt.event.ActionListener() {
                @Override
                public void actionPerformed(java.awt.event.ActionEvent evt) {
                    mazeButtonActionPerformed(evt);
                }
            });

            JButton clearButton = new JButton("Καθάρισμα");
            clearButton.addActionListener(new ActionHandler());
            clearButton.setBackground(Color.lightGray);
            clearButton.setToolTipText
                    ("Πρώτο κλικ: καθάρισμα αναζήτησης, Δεύτερο κλικ: καθάρισμα εμποδίων");

            JButton stepButton = new JButton("Βήμα - Βήμα");
            stepButton.addActionListener(new ActionHandler());
            stepButton.setBackground(Color.lightGray);
            stepButton.setToolTipText
                    ("Η αναζήτηση γίνεται βήμα-βήμα για κάθε κλικ");

            JButton animationButton = new JButton("Κίνηση");
            animationButton.addActionListener(new ActionHandler());
            animationButton.setBackground(Color.lightGray);
            animationButton.setToolTipText
                    ("Η αναζήτηση γίνεται αυτόματα");

            JLabel velocity = new JLabel("Ταχύτητα", JLabel.CENTER);
            velocity.setFont(new Font("Helvetica",Font.PLAIN,10));
            
            slider = new JSlider(0,1000,500); // αρχική τιμή καθυστέρησης 500 msec
            slider.setToolTipText
                    ("Ρυθμίζει την καθυστέρηση σε κάθε βήμα (0 μέχρι 1 sec)");
            
            delay = 1000-slider.getValue();
            slider.addChangeListener(new ChangeListener() {
                @Override
                public void stateChanged(ChangeEvent evt) {
                    JSlider source = (JSlider)evt.getSource();
                    if (!source.getValueIsAdjusting()) {
                        delay = 1000-source.getValue();
                    }
                }
            });
            
            // ButtonGroup που συγχρονίζει τα πέντε RadioButtons
            // που επιλέγουν τον αλγόριθμο, έτσι ώστε ένα μόνο από
            // αυτά να μπορεί να επιλεγεί ανά πάσα στιγμή
            ButtonGroup algoGroup = new ButtonGroup();

            dfs = new JRadioButton("DFS");
            dfs.setToolTipText("Αλγόριθμος αναζήτησης πρώτα σε βάθος");
            algoGroup.add(dfs);
            dfs.addActionListener(new ActionHandler());

            bfs = new JRadioButton("BFS");
            bfs.setToolTipText("Αλγόριθμος αναζήτησης πρώτα σε πλάτος");
            algoGroup.add(bfs);
            bfs.addActionListener(new ActionHandler());

            aStar = new JRadioButton("A*");
            aStar.setToolTipText("Αλγόριθμος αναζήτησης Α*");
            algoGroup.add(aStar);
            aStar.addActionListener(new ActionHandler());

            greedy = new JRadioButton("Greedy");
            greedy.setToolTipText("Αλγόριθμος άπληστης αναζήτησης");
            algoGroup.add(greedy);
            greedy.addActionListener(new ActionHandler());

            dijkstra = new JRadioButton("Dijkstra");
            dijkstra.setToolTipText("Αλγόριθμος του Dijkstra");
            algoGroup.add(dijkstra);
            dijkstra.addActionListener(new ActionHandler());

            JPanel algoPanel = new JPanel();
            algoPanel.setBorder(javax.swing.BorderFactory.
                    createTitledBorder(javax.swing.BorderFactory.createEtchedBorder(),
                    "Αλγόριθμοι", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION,
                    javax.swing.border.TitledBorder.TOP, new java.awt.Font("Helvetica", 0, 14)));
            
            dfs.setSelected(true);  // επιλέγουμε αρχικά τον DFS
            
            diagonal = new
                    JCheckBox("Διαγώνιες κινήσεις");
            diagonal.setToolTipText("Επιτρέπονται και διαγώνιες κινήσεις");

            drawArrows = new
                    JCheckBox("Βέλη προς προκατόχους");
            drawArrows.setToolTipText("Σχεδίαση βελών προς προκατόχους καταστάσεις");

            JLabel robot = new JLabel("Ρομπότ", JLabel.CENTER);
            robot.setForeground(Color.red);
            robot.setFont(new Font("Helvetica",Font.PLAIN,14));

            JLabel target = new JLabel("Στόχος", JLabel.CENTER);
            target.setForeground(Color.GREEN);
            target.setFont(new Font("Helvetica",Font.PLAIN,14));
         
            JLabel frontier = new JLabel("Μέτωπο", JLabel.CENTER);
            frontier.setForeground(Color.blue);
            frontier.setFont(new Font("Helvetica",Font.PLAIN,14));

            JLabel closed = new JLabel("Κλειστές", JLabel.CENTER);
            closed.setForeground(Color.CYAN);
            closed.setFont(new Font("Helvetica",Font.PLAIN,14));

            JButton aboutButton = new JButton("Σχετικά με το Maze");
            aboutButton.addActionListener(new ActionHandler());
            aboutButton.setBackground(Color.lightGray);
            aboutButton.addActionListener(new java.awt.event.ActionListener() {
                @Override
                public void actionPerformed(java.awt.event.ActionEvent evt) {
                    aboutButtonActionPerformed(evt);
                }
            });

            // προσθέτουμε τα περιεχόμενα στο panel
            add(message);
            add(rowsLbl);
            add(rowsField);
            add(columnsLbl);
            add(columnsField);
            add(resetButton);
            add(mazeButton);
            add(clearButton);
            add(stepButton);
            add(animationButton);
            add(velocity);
            add(slider);
            add(dfs);
            add(bfs);
            add(aStar);
            add(greedy);
            add(dijkstra);
            add(algoPanel);
            add(diagonal);
            add(drawArrows);
            add(robot);
            add(target);
            add(frontier);
            add(closed);
            add(aboutButton);

            // ρυθμίζουμε τα μεγέθη και τις θέσεις τους
            message.setBounds(0, 515, 500, 23);
            rowsLbl.setBounds(520, 5, 140, 25);
            rowsField.setBounds(665, 5, 25, 25);
            columnsLbl.setBounds(520, 35, 140, 25);
            columnsField.setBounds(665, 35, 25, 25);
            resetButton.setBounds(520, 65, 170, 25);
            mazeButton.setBounds(520, 95, 170, 25);
            clearButton.setBounds(520, 125, 170, 25);
            stepButton.setBounds(520, 155, 170, 25);
            animationButton.setBounds(520, 185, 170, 25);
            velocity.setBounds(520, 215, 170, 10);
            slider.setBounds(520, 225, 170, 25);
            dfs.setBounds(530, 270, 70, 25);
            bfs.setBounds(600, 270, 70, 25);
            aStar.setBounds(530, 295, 70, 25);
            greedy.setBounds(600, 295, 85, 25);
            dijkstra.setBounds(530, 320, 85, 25);
            algoPanel.setLocation(520,250);
            algoPanel.setSize(170, 100);
            diagonal.setBounds(520, 355, 170, 25);
            drawArrows.setBounds(520, 380, 170, 25);
            robot.setBounds(520, 465, 80, 25);
            target.setBounds(605, 465, 80, 25);
            frontier.setBounds(520, 485, 80, 25);
            closed.setBounds(605, 485, 80, 25);
            aboutButton.setBounds(520, 515, 170, 25);

            // δημιουργούμε τον timer
            timer = new Timer(delay, action);
            
            // δίνουμε στα κελιά του πλέγματος αρχικές τιμές
            // εδώ γίνεται και το πρώτο βήμα των αλγόριθμων
            fillGrid();

        } // end constructor

        /**
         * Λειτουργία που εκτελείται αν ο χρήστης πιέσει το κουμπί "Νέο Πλέγμα"
         */
        private void resetButtonActionPerformed(java.awt.event.ActionEvent evt) {                                           
            initializeGrid(false);
        } // end resetButtonActionPerformed()
    
        /**
         * Λειτουργία που εκτελείται αν ο χρήστης πιέσει το κουμπί "Λαβύριθμος"
         */
        private void mazeButtonActionPerformed(java.awt.event.ActionEvent evt) {
            initializeGrid(true);
        } // end mazeButtonActionPerformed()
    
        /**
         * Δημιουργεί ένα νέο καθαρό πλέγμα ή ένα νέο λαβύρινθο
         */
        private void initializeGrid(Boolean makeMaze) {                                           
            int oldRows = rows;
            int oldColumns = columns;
            try {
                if (!rowsField.getText().isEmpty()){
                    rows = Integer.parseInt(rowsField.getText());
                } else {
                    JOptionPane.showMessageDialog(this,
                            "Το πεδίο \"Πλήθος γραμμών\" \nδέχεται μόνο αριθμούς μεταξύ 5 και 83",
                            "Πρόβλημα", JOptionPane.ERROR_MESSAGE);
                    rows = oldRows;
                    return;
                }
            } catch (NumberFormatException ex) {
                JOptionPane.showMessageDialog(this,
                        "Το πεδίο \"Πλήθος γραμμών\" \nδέχεται μόνο αριθμούς μεταξύ 5 και 83",
                        "Πρόβλημα", JOptionPane.ERROR_MESSAGE);
                rows = oldRows;
                return;
            }
            if (rows < 5 || rows > 83) {
                JOptionPane.showMessageDialog(this,
                        "Το πεδίο \"Πλήθος γραμμών\" \nδέχεται τιμές μεταξύ 5 και 83",
                        "Πρόβλημα", JOptionPane.ERROR_MESSAGE);
                rows = oldRows;
                return;
            }
            try {
                if (!columnsField.getText().isEmpty()){
                    columns = Integer.parseInt(columnsField.getText());
                } else {
                    JOptionPane.showMessageDialog(this,
                            "Το πεδίο \"Πλήθος στηλών\" \nδέχεται μόνο αριθμούς μεταξύ 5 και 83",
                            "Πρόβλημα", JOptionPane.ERROR_MESSAGE);
                    columns = oldColumns;
                    return;
                }
            } catch (NumberFormatException ex) {
                JOptionPane.showMessageDialog(this,
                        "Το πεδίο \"Πλήθος στηλών\" \nδέχεται μόνο αριθμούς μεταξύ 5 και 83",
                        "Πρόβλημα", JOptionPane.ERROR_MESSAGE);
                columns = oldColumns;
                return;
            }
            if (columns < 5 || columns > 83) {
                JOptionPane.showMessageDialog(this,
                        "Το πεδίο \"Πλήθος στηλών\" \nδέχεται τιμές μεταξύ 5 και 83",
                        "Πρόβλημα", JOptionPane.ERROR_MESSAGE);
                columns = oldColumns;
                return;
            }
            squareSize = 500/(rows > columns ? rows : columns);
            arrowSize = squareSize/2;
            // ο λαβύρινθος πρέπει να έχει περιττό αριθμό γραμμών και στηλών
            if (makeMaze && rows % 2 == 0) {
                rows -= 1;
            }
            if (makeMaze && columns % 2 == 0) {
                columns -= 1;
            }
            grid = new int[rows][columns];
            robotStart = new Cell(rows-2,1);
            targetPos = new Cell(1,columns-2);
            dfs.setEnabled(true);
            dfs.setSelected(true);
            bfs.setEnabled(true);
            aStar.setEnabled(true);
            greedy.setEnabled(true);
            dijkstra.setEnabled(true);
            diagonal.setSelected(false);
            diagonal.setEnabled(true);
            drawArrows.setSelected(false);
            drawArrows.setEnabled(true);
            slider.setValue(500);
            if (makeMaze) {
                MyMaze maze = new MyMaze(rows/2,columns/2);
            } else {
                fillGrid();
            }
        } // end initializeGrid()
    
        
        /**
         * Λειτουργία που εκτελείται αν ο χρήστης πιέσει το κουμπί "Σχετικά με το Maze"
         */
        private void aboutButtonActionPerformed(java.awt.event.ActionEvent evt) {                                           
            AboutBox aboutBox = new AboutBox(mazeFrame,true);
            aboutBox.setVisible(true);
        } // end aboutButtonActionPerformed()
    
        /**
         * Επεκτείνει ένα κόμβο και δημιουργεί τους διαδόχους του
         */
        private void expandNode(){
            // Τον αλγόριθμο του Dijkstra τον χειριζόμαστε ξεχωριστά
            if (dijkstra.isSelected()){
                Cell u;
                // 11: while Q is not empty:
                if (graph.isEmpty()){
                    return;
                }
                // 12:  u := vertex in Q (graph) with smallest distance in dist[] ;
                // 13:  remove u from Q (graph);
                u = graph.remove(0);
                // Προσθέτουμε την κορυφή u στις κλειστές
                closedSet.add(u);
                // Αν βρέθηκε ο στόχος ...
                if (u.row == targetPos.row && u.col == targetPos.col){
                    found = true;
                    return;
                }
                // Καταμετρούμε τους κόμβους που έχουμε αναπτύξει.
                expanded++;
                // Ενημερώνουμε το χρώμα του κελιού
                grid[u.row][u.col] = CLOSED;
                // 14: if dist[u] = infinity:
                if (u.dist == INFINITY){
                    // ... τότε δεν υπάρχει λύση.
                    // 15: break;
                    return;
                // 16: end if
                } 
                // Δημιουργούμε τους γείτονες της u
                ArrayList<Cell> neighbors = createSuccesors(u, false);
                // 18: for each neighbor v of u:
                for (Cell v: neighbors){
                    // 20: alt := dist[u] + dist_between(u, v) ;
                    int alt = u.dist + distBetween(u,v);
                    // 21: if alt < dist[v]:
                    if (alt < v.dist){
                        // 22: dist[v] := alt ;
                        v.dist = alt;
                        // 23: previous[v] := u ;
                        v.prev = u;
                        // Ενημέρωσε το χρώμα του κελιού
                        grid[v.row][v.col] = FRONTIER;
                        // 24: decrease-key v in Q;
                        // (ταξινομούμε την λίστα των κόμβων ως προς dist)
                        Collections.sort(graph, new CellComparatorByDist());
                    }
                }
            // Ο χειρισμός των υπόλοιπων αλγόριθμων
            } else {
                Cell current;
                if (dfs.isSelected() || bfs.isSelected()) {
                    // Εδώ έχουμε το 3ο βήμα των αλγόριθμων DFS και BFS
                    // 3. Αφαίρεσε την πρώτη κατάσταση Si, από τις ΑΝΟΙΚΤΕΣ ....
                    current = openSet.remove(0);
                } else {
                    // Εδώ έχουμε το 3ο βήμα των αλγόριθμων Α* και Greedy
                    // 3. Αφαίρεσε την κατάσταση Si, από την λίστα ΑΝΟΙΚΤΕΣ,
                    //    για την οποία f(Si) <= f(Sj) για όλες τις άλλες
                    //    ανοικτές καταστάσεις Sj ...
                    // (ταξινομούμε πρώτα τη λίστα ΑΝΟΙΚΤΕΣ κατά αύξουσα σειρά ως προς f)
                    Collections.sort(openSet, new CellComparatorByF());
                    current = openSet.remove(0);
                }
                // ... και πρόσθεσέ την στις ΚΛΕΙΣΤΕΣ.
                closedSet.add(0,current);
                // Ενημέρωσε το χρώμα του κελιού
                grid[current.row][current.col] = CLOSED;
                // Αν ο επιλεγμένος κόμβος είναι ο στόχος ...
                if (current.row == targetPos.row && current.col == targetPos.col) {
                    // ... τότε τερμάτισε κλπ
                    Cell last = targetPos;
                    last.prev = current.prev;
                    closedSet.add(last);
                    found = true;
                    return;
                }
                // Καταμετρούμε τους κόμβους που έχουμε αναπτύξει.
                expanded++;
                // Εδώ έχουμε το 4ο βήμα των αλγόριθμων
                // 4. Δημιούργησε τις διαδόχους της Si, με βάση τις ενέργειες που μπορούν
                //    να εφαρμοστούν στην Si.
                //    Η κάθε διάδοχος έχει ένα δείκτη προς την Si, ως την προκάτοχό της.
                //    Στην περίπτωση των αλγόριθμων DFS και BFS οι διάδοχοι δεν πρέπει
                //    να ανήκουν ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ.
                ArrayList<Cell> succesors;
                succesors = createSuccesors(current, false);
                // Εδώ έχουμε το 5ο βήμα των αλγόριθμων
                // 5. Για κάθε διάδοχο της Si, ...
                for (Cell cell: succesors){
                    // ... αν τρέχουμε τον DFS ...
                    if (dfs.isSelected()) {
                        // ... πρόσθεσε τον διάδοχο στην αρχή της λίστας ΑΝΟΙΚΤΕΣ
                        openSet.add(0, cell);
                        // Ενημέρωσε το χρώμα του κελιού
                        grid[cell.row][cell.col] = FRONTIER;
                    // ... αν τρέχουμε τον ΒFS ...
                    } else if (bfs.isSelected()){
                        // ... πρόσθεσε τον διάδοχο στο τέλος της λίστας ΑΝΟΙΚΤΕΣ
                        openSet.add(cell);
                        // Ενημέρωσε το χρώμα του κελιού
                        grid[cell.row][cell.col] = FRONTIER;
                    // ... αν τρέχουμε τους αλγόριθμους Α* ή Greedy (Βήμα 5 αλγόριθμου Α*) ...
                    } else if (aStar.isSelected() || greedy.isSelected()){
                        // ... υπολόγισε την τιμή f(Sj)...
                        int dxg = current.col-cell.col;
                        int dyg = current.row-cell.row;
                        int dxh = targetPos.col-cell.col;
                        int dyh = targetPos.row-cell.row;
                        if (diagonal.isSelected()){
                            // Με διαγώνιες κινήσεις υπολογίζουμε
                            // το 1000-πλάσιο των ευκλείδιων αποστάσεων
                            if (greedy.isSelected()) {
                                // ειδικά για τον Greedy ...
                                cell.g = 0;
                            } else {
                                cell.g = current.g+(int)((double)1000*Math.sqrt(dxg*dxg + dyg*dyg));
                            }
                            cell.h = (int)((double)1000*Math.sqrt(dxh*dxh + dyh*dyh));
                        } else {
                            // Χωρίς διαγώνιες κινήσεις υπολογίζουμε
                            // τις αποστάσεις Manhattan
                            if (greedy.isSelected()) {
                                // ειδικά για τον Greedy ...
                                cell.g = 0;
                            } else {
                                cell.g = current.g+Math.abs(dxg)+Math.abs(dyg);
                            }
                            cell.h = Math.abs(dxh)+Math.abs(dyh);
                        }
                        cell.f = cell.g+cell.h;
                        // ... αν η Sj δεν ανήκει ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        int openIndex   = isInList(openSet,cell);
                        int closedIndex = isInList(closedSet,cell);
                        if (openIndex == -1 && closedIndex == -1) {
                            // ... τότε πρόσθεσε την Sj στις ΑΝΟΙΚΤΕΣ ...
                            // ... με τιμή αξιολόγησης f(Sj)
                            openSet.add(cell);
                            // Ενημέρωσε το χρώμα του κελιού
                            grid[cell.row][cell.col] = FRONTIER;
                        // Αλλιώς ...
                        } else {
                            // ... αν ανήκε στις ΑΝΟΙΚΤΕΣ, τότε ...
                            if (openIndex > -1){
                                // ... σύγκρινε την νέα τιμή αξιολόγισής της με την παλαιά.
                                // Αν παλαιά <= νέα ...
                                if (openSet.get(openIndex).f <= cell.f) {
                                    // ... απόβαλε το νέο κόμβο με την κατάσταση Sj
                                    // (δηλαδή μην κάνεις τίποτε για αυτόν τον κόμβο).
                                // Διαφορετικά, ...
                                } else {
                                    // ... αφαίρεσε το στοιχείο (Sj,παλαιά) από τη λίστα
                                    // στην οποία ανήκει ...
                                    openSet.remove(openIndex);
                                    // ... και πρόσθεσε το στοιχείο (Sj,νέα) στις ΑΝΟΙΚΤΕΣ
                                    openSet.add(cell);
                                    // Ενημέρωσε το χρώμα του κελιού
                                    grid[cell.row][cell.col] = FRONTIER;
                                }
                            // ... αν ανήκε στις ΚΛΕΙΣΤΕΣ, τότε ...
                            } else {
                                // ... σύγκρινε την νέα τιμή αξιολόγισής της με την παλαιά.
                                // Αν παλαιά <= νέα ...
                                if (closedSet.get(closedIndex).f <= cell.f) {
                                    // ... απόβαλε το νέο κόμβο με την κατάσταση Sj
                                    // (δηλαδή μην κάνεις τίποτε για αυτόν τον κόμβο).
                                // Διαφορετικά, ...
                                } else {
                                    // ... αφαίρεσε το στοιχείο (Sj,παλαιά) από τη λίστα
                                    // στην οποία ανήκει ...
                                    closedSet.remove(closedIndex);
                                    // ... και πρόσθεσε το στοιχείο (Sj,νέα) στις ΑΝΟΙΚΤΕΣ
                                    openSet.add(cell);
                                    // Ενημέρωσε το χρώμα του κελιού
                                    grid[cell.row][cell.col] = FRONTIER;
                                }
                            }
                        }
                    }
                }
            }
        } //end expandNode()
        
        /**
         * Δημιουργεί τους διαδόχους μιας κατάστασης/κελιού
         * 
         * @param current το κελί του οποίου ζητούμε τους διαδόχους
         * @param makeConnected flag που δηλώνει ότι μας ενδιαφέρουν μόνο οι συντεταγμένες των
         *                      κελιών χωρίς την ετικέτα τους dist (αφορά μόνον τον Dijkstra)
         * @return οι διάδοχοι του κελιού με μορφή λίστας
         */
        private ArrayList<Cell> createSuccesors(Cell current, boolean makeConnected){
            int r = current.row;
            int c = current.col;
            // Δημιουργούμε μια κενή λίστα για τους διαδόχους του τρέχοντος κελιού.
            ArrayList<Cell> temp = new ArrayList<>();
            // Με διαγώνιες κινήσεις η προτεραιότητα είναι:
            // 1:Πάνω 2:Πάνω-δεξιά 3:Δεξιά 4:Κάτω-δεξιά
            // 5:Κάτω 6:Κάτω-αριστερά 7:Αριστερά 8:Πάνω-αριστερά
            
            // Χωρίς διαγώνιες κινήσεις η προτεραιότητα είναι:
            // 1:Πάνω 2:Δεξιά 3:Κάτω 4:Αριστερά 
            
            // Αν δεν βρισκόμαστε στο πάνω όριο του πλέγματος
            // και το πάνω κελί δεν είναι εμπόδιο ...
            if (r > 0 && grid[r-1][c] != OBST &&
                    // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ή τον Greedy) 
                    // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                    ((aStar.isSelected() || greedy.isSelected() || dijkstra.isSelected()) ? true :
                          isInList(openSet,new Cell(r-1,c)) == -1 &&
                          isInList(closedSet,new Cell(r-1,c)) == -1)) {
                Cell cell = new Cell(r-1,c);
                // Στην περίπτωση του αλγόριθμου Dijkstra δεν μπορούμε να προσθέσουμε
                // στην λίστα με τους διαδόχους το "γυμνό" κελί που μόλις δημιουργήσαμε.
                // Το κελί πρέπει να συνοδεύεται από την ετικέτα του dist,
                // γι' αυτό πρέπει να το εντοπίσουμε μέσα στην λίστα graph
                // και από εκεί να το αντιγράψουμε στην λίστα των διαδόχων.
                // Το flag makeConnected είναι απαραίτητο για να είναι δυνατόν να
                // συνεργαστεί η παρούσα μέθοδος createSuccesors() με την
                // μέθοδο findConnectedComponent() η οποία δημιουργεί κατά την
                // αρχικοποίηση του Dijkstra την συνεκτική συνιστώσα.
                if (dijkstra.isSelected()){
                    if (makeConnected) {
                        temp.add(cell);
                    } else {
                        int graphIndex = isInList(graph,cell);
                        if (graphIndex > -1) {
                            temp.add(graph.get(graphIndex));
                        }
                    }
                } else {
                    // ... ενημέρωσε τον δείκτη του πάνω κελιού να δείχνει το τρέχον ...
                    cell.prev = current;
                    // ... και πρόσθεσε το πάνω κελί στους διαδόχους του τρέχοντος. 
                    temp.add(cell);
                 }
            }
            if (diagonal.isSelected()){
                // Αν δεν βρισκόμαστε ούτε στο πάνω ούτε στο δεξιό όριο του πλέγματος
                // και το πάνω-δεξί κελί δεν είναι εμπόδιο ...
                if (r > 0 && c < columns-1 && grid[r-1][c+1] != OBST &&
                        // ... και ένα από τα πάνω ή δεξιό κελιά δεν είναι εμπόδια ...
                        // (επειδή δεν είναι λογικό να επιτρέψουμε να περάσει
                        //  το ρομπότ από μία σχισμή)
                        (grid[r-1][c] != OBST || grid[r][c+1] != OBST) &&
                        // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ή τον Greedy) 
                        // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        ((aStar.isSelected() || greedy.isSelected() || dijkstra.isSelected()) ? true :
                              isInList(openSet,new Cell(r-1,c+1)) == -1 &&
                              isInList(closedSet,new Cell(r-1,c+1)) == -1)) {
                    Cell cell = new Cell(r-1,c+1);
                    if (dijkstra.isSelected()){
                        if (makeConnected) {
                            temp.add(cell);
                        } else {
                            int graphIndex = isInList(graph,cell);
                            if (graphIndex > -1) {
                                temp.add(graph.get(graphIndex));
                            }
                        }
                    } else {
                        // ... ενημέρωσε τον δείκτη του πάνω-δεξιού κελιού να δείχνει το τρέχον ...
                        cell.prev = current;
                        // ... και πρόσθεσε το πάνω-δεξί κελί στους διαδόχους του τρέχοντος. 
                        temp.add(cell);
                    }
                }
            }
            // Αν δεν βρισκόμαστε στο δεξί όριο του πλέγματος
            // και το δεξί κελί δεν είναι εμπόδιο ...
            if (c < columns-1 && grid[r][c+1] != OBST &&
                    // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ή τον Greedy) 
                    // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                    ((aStar.isSelected() || greedy.isSelected() || dijkstra.isSelected())? true :
                          isInList(openSet,new Cell(r,c+1)) == -1 &&
                          isInList(closedSet,new Cell(r,c+1)) == -1)) {
                Cell cell = new Cell(r,c+1);
                if (dijkstra.isSelected()){
                    if (makeConnected) {
                        temp.add(cell);
                    } else {
                        int graphIndex = isInList(graph,cell);
                        if (graphIndex > -1) {
                            temp.add(graph.get(graphIndex));
                        }
                    }
                } else {
                    // ... ενημέρωσε τον δείκτη του δεξιού κελιού να δείχνει το τρέχον ...
                    cell.prev = current;
                    // ... και πρόσθεσε το δεξί κελί στους διαδόχους του τρέχοντος. 
                    temp.add(cell);
                }
            }
            if (diagonal.isSelected()){
                // Αν δεν βρισκόμαστε ούτε στο κάτω ούτε στο δεξιό όριο του πλέγματος
                // και το κάτω-δεξί κελί δεν είναι εμπόδιο ...
                if (r < rows-1 && c < columns-1 && grid[r+1][c+1] != OBST &&
                        // ... και ένα από τα κάτω ή δεξιό κελιά δεν είναι εμπόδια ...
                        (grid[r+1][c] != OBST || grid[r][c+1] != OBST) &&
                        // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ή τον Greedy) 
                        // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        ((aStar.isSelected() || greedy.isSelected() || dijkstra.isSelected()) ? true :
                              isInList(openSet,new Cell(r+1,c+1)) == -1 &&
                              isInList(closedSet,new Cell(r+1,c+1)) == -1)) {
                    Cell cell = new Cell(r+1,c+1);
                    if (dijkstra.isSelected()){
                        if (makeConnected) {
                            temp.add(cell);
                        } else {
                            int graphIndex = isInList(graph,cell);
                            if (graphIndex > -1) {
                                temp.add(graph.get(graphIndex));
                            }
                        }
                    } else {
                        // ... ενημέρωσε τον δείκτη του κάτω-δεξιού κελιού να δείχνει το τρέχον ...
                        cell.prev = current;
                        // ... και πρόσθεσε το κάτω-δεξί κελί στους διαδόχους του τρέχοντος. 
                        temp.add(cell);
                    }
                }
            }
            // Αν δεν βρισκόμαστε στο κάτω όριο του πλέγματος
            // και το κάτω κελί δεν είναι εμπόδιο ...
            if (r < rows-1 && grid[r+1][c] != OBST &&
                    // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ή τον Greedy) 
                    // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                    ((aStar.isSelected() || greedy.isSelected() || dijkstra.isSelected()) ? true :
                          isInList(openSet,new Cell(r+1,c)) == -1 &&
                          isInList(closedSet,new Cell(r+1,c)) == -1)) {
                Cell cell = new Cell(r+1,c);
                if (dijkstra.isSelected()){
                    if (makeConnected) {
                        temp.add(cell);
                    } else {
                        int graphIndex = isInList(graph,cell);
                        if (graphIndex > -1) {
                            temp.add(graph.get(graphIndex));
                        }
                    }
                } else {
                    // ... ενημέρωσε τον δείκτη του κάτω κελιού να δείχνει το τρέχον ...
                    cell.prev = current;
                    // ... και πρόσθεσε το κάτω κελί στους διαδόχους του τρέχοντος.
                    temp.add(cell);
                }
            }
            if (diagonal.isSelected()){
                // Αν δεν βρισκόμαστε ούτε στο κάτω ούτε στο αριστερό όριο του πλέγματος
                // και το κάτω-αριστερό κελί δεν είναι εμπόδιο ...
                if (r < rows-1 && c > 0 && grid[r+1][c-1] != OBST &&
                        // ... και ένα από τα κάτω ή αριστερό κελιά δεν είναι εμπόδια ...
                        (grid[r+1][c] != OBST || grid[r][c-1] != OBST) &&
                        // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ή τον Greedy) 
                        // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        ((aStar.isSelected() || greedy.isSelected() || dijkstra.isSelected()) ? true :
                              isInList(openSet,new Cell(r+1,c-1)) == -1 &&
                              isInList(closedSet,new Cell(r+1,c-1)) == -1)) {
                    Cell cell = new Cell(r+1,c-1);
                    if (dijkstra.isSelected()){
                        if (makeConnected) {
                            temp.add(cell);
                        } else {
                            int graphIndex = isInList(graph,cell);
                            if (graphIndex > -1) {
                                temp.add(graph.get(graphIndex));
                            }
                        }
                    } else {
                        // ... ενημέρωσε τον δείκτη του κάτω-αριστερού κελιού να δείχνει το τρέχον ...
                        cell.prev = current;
                        // ... και πρόσθεσε το κάτω-αριστερό κελί στους διαδόχους του τρέχοντος.
                        temp.add(cell);
                    }
                }
            }
            // Αν δεν βρισκόμαστε στο αριστερό όριο του πλέγματος
            // και το αριστερό κελί δεν είναι εμπόδιο ...
            if (c > 0 && grid[r][c-1] != OBST && 
                    // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ή τον Greedy) 
                    // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                    ((aStar.isSelected() || greedy.isSelected() || dijkstra.isSelected()) ? true :
                          isInList(openSet,new Cell(r,c-1)) == -1 &&
                          isInList(closedSet,new Cell(r,c-1)) == -1)) {
                Cell cell = new Cell(r,c-1);
                if (dijkstra.isSelected()){
                    if (makeConnected) {
                        temp.add(cell);
                    } else {
                        int graphIndex = isInList(graph,cell);
                        if (graphIndex > -1) {
                            temp.add(graph.get(graphIndex));
                        }
                    }
                } else {
                    // ... ενημέρωσε τον δείκτη του αριστερού κελιού να δείχνει το τρέχον ...
                    cell.prev = current;
                    // ... και πρόσθεσε το αριστερό κελί στους διαδόχους του τρέχοντος.
                    temp.add(cell);
                }
            }
            if (diagonal.isSelected()){
                // Αν δεν βρισκόμαστε ούτε στο πάνω ούτε στο αριστερό όριο του πλέγματος
                // και το πάνω-αριστερό κελί δεν είναι εμπόδιο ...
                if (r > 0 && c > 0 && grid[r-1][c-1] != OBST &&
                        // ... και ένα από τα πάνω ή αριστερό κελιά δεν είναι εμπόδια ...
                        (grid[r-1][c] != OBST || grid[r][c-1] != OBST) &&
                        // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ή τον Greedy) 
                        // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        ((aStar.isSelected() || greedy.isSelected() || dijkstra.isSelected()) ? true :
                              isInList(openSet,new Cell(r-1,c-1)) == -1 &&
                              isInList(closedSet,new Cell(r-1,c-1)) == -1)) {
                    Cell cell = new Cell(r-1,c-1);
                    if (dijkstra.isSelected()){
                        if (makeConnected) {
                            temp.add(cell);
                        } else {
                            int graphIndex = isInList(graph,cell);
                            if (graphIndex > -1) {
                                temp.add(graph.get(graphIndex));
                            }
                        }
                    } else {
                        // ... ενημέρωσε τον δείκτη του πάνω-αριστερού κελιού να δείχνει το τρέχον ...
                        cell.prev = current;
                        // ... και πρόσθεσε το πάνω-αριστερό κελί στους διαδόχους του τρέχοντος.
                        temp.add(cell);
                    }
                }
            }
            // Επειδή στον αλγόριθμο DFS τα κελιά προστίθενται ένα-ένα στην
            // αρχή της λίστας ΑΝΟΙΚΤΕΣ, αντιστρέφουμε την σειρά των διαδόχων
            // που σχηματίστηκε, ώστε ο διάδοχος που αντιστοιχεί στην υψηλότερη
            // προτεραιότητα, να βρεθεί πρώτος στην λίστα.
            // Για τους Greedy, A* και Dijkstra δεν υπάρχει ζήτημα, γιατί η λίστα
            // ταξινομείται ως προς f ή dist πριν την εξαγωγή του πρώτου στοιχείου της.
            if (dfs.isSelected()){
                Collections.reverse(temp);
            }
            return temp;
        } // end createSuccesors()
        
        /**
         * Επιστρέφει τον δείκτη του κελιού current στη λίστα list
         *
         * @param list η λίστα μέσα στην οποία αναζητάμε
         * @param current το κελί που αναζητάμε
         * @return ο δείκτης το κελιού μέσα στη λίστα
         * αν το κελί δεν βρεθεί επιστρέφει -1
         */
        private int isInList(ArrayList<Cell> list, Cell current){
            int index = -1;
            for (int i = 0 ; i < list.size(); i++) {
                if (current.row == list.get(i).row && current.col == list.get(i).col) {
                    index = i;
                    break;
                }
            }
            return index;
        } // end isInList()
        
        /**
         * Επιστρέφει το προκάτοχο κελί του κελιού current της λίστας list
         *
         * @param list η λίστα μέσα στην οποία αναζητάμε
         * @param current το κελί που αναζητάμε
         * @return το κελί που αντιστοιχεί στον προκάτοχο του current
         */
        private Cell findPrev(ArrayList<Cell> list, Cell current){
            int index = isInList(list, current);
            return list.get(index).prev;
        } // end findPrev()
        
        /**
         * Επιστρέφει την απόσταση μεταξύ δύο κελιών
         *
         * @param u το ένα κελί
         * @param v το άλλο κελί
         * @return η απόσταση μεταξύ των κελιών u και v
         */
        private int distBetween(Cell u, Cell v){
            int dist;
            int dx = u.col-v.col;
            int dy = u.row-v.row;
            if (diagonal.isSelected()){
                // Με διαγώνιες κινήσεις υπολογίζουμε
                // το 1000-πλάσιο των ευκλείδιων αποστάσεων
                dist = (int)((double)1000*Math.sqrt(dx*dx + dy*dy));
            } else {
                // Χωρίς διαγώνιες κινήσεις υπολογίζουμε
                // τις αποστάσεις Manhattan
                dist = Math.abs(dx)+Math.abs(dy);
            }
            return dist;
        } // end distBetween()
        
        /**
         * Υπολογίζει την διαδρομή από τον στόχο προς την αρχική θέση
         * του ρομπότ και μετρά τα αντίστοιχα βήματα
         * και την απόσταση που διανύθηκε.
         */
        private void plotRoute(){
            searching = false;
            endOfSearch = true;
            int steps = 0;
            double distance = 0;
            int index = isInList(closedSet,targetPos);
            Cell cur = closedSet.get(index);
            grid[cur.row][cur.col]= TARGET;
            do {
                steps++;
                if (diagonal.isSelected()) {
                    int dx = cur.col-cur.prev.col;
                    int dy = cur.row-cur.prev.row;
                    distance += Math.sqrt(dx*dx + dy*dy);
                } else { 
                    distance++;
                }
                cur = cur.prev;
                grid[cur.row][cur.col] = ROUTE;
            } while (!(cur.row == robotStart.row && cur.col == robotStart.col));
            grid[robotStart.row][robotStart.col]=ROBOT;
            String msg;
            msg = String.format("Αναπτύχθηκαν κόμβοι: %d, Βήματα: %d, Απόσταση: %.3f",
                     expanded,steps,distance); 
            message.setText(msg);
          
        } // end plotRoute()
        
        /**
         * Δίνει αρχικές τιμές στα κελιά του πλέγματος
         * Με το πρώτο κλικ στο κουμπί 'Καθάρισμα' μηδενίζει τα στοιχεία
         * της τυχόν αναζήτησης που είχε εκτελεστεί (Μέτωπο, Κλειστές, Διαδρομή) 
         * και αφήνει ανέπαφα τα εμπόδια και τις θέσεις ρομπότ και στόχου
         * προκειμένου να είναι δυνατή η εκτέλεση άλλου αλγόριθμου
         * με τα ίδια δεδομένα.
         * Με το δεύτερο κλικ αφαιρεί και τα εμπόδια.
         */
        private void fillGrid() {
            if (searching || endOfSearch){ 
                for (int r = 0; r < rows; r++) {
                    for (int c = 0; c < columns; c++) {
                        if (grid[r][c] == FRONTIER || grid[r][c] == CLOSED || grid[r][c] == ROUTE) {
                            grid[r][c] = EMPTY;
                        }
                        if (grid[r][c] == ROBOT){
                            robotStart = new Cell(r,c);
                        }
                        if (grid[r][c] == TARGET){
                            targetPos = new Cell(r,c);
                        }
                    }
                }
                searching = false;
            } else {
                for (int r = 0; r < rows; r++) {
                    for (int c = 0; c < columns; c++) {
                        grid[r][c] = EMPTY;
                    }
                }
                robotStart = new Cell(rows-2,1);
                targetPos = new Cell(1,columns-2);
            }
            if (aStar.isSelected() || greedy.isSelected()){
                robotStart.g = 0;
                robotStart.h = 0;
                robotStart.f = 0;
            }
            expanded = 0;
            found = false;
            searching = false;
            endOfSearch = false;
         
            // Το πρώτο βήμα των υπόλοιπων αλγόριθμων γίνεται εδώ
            // 1. ΑΝΟΙΚΤΕΣ:= [So], ΚΛΕΙΣΤΕΣ:= []
            openSet.removeAll(openSet);
            openSet.add(robotStart);
            closedSet.removeAll(closedSet);
         
            grid[targetPos.row][targetPos.col] = TARGET; 
            grid[robotStart.row][robotStart.col] = ROBOT;
            message.setText(msgDrawAndSelect);
            timer.stop();
            repaint();
            
        } // end fillGrid()

        /**
         * Προσθέτει στην λίστα που περιέχει τους κόμβους του γραφήματος μόνο
         * εκείνα τα κελιά τα οποία ανήκουν στην ίδια συνεκτική συνιστώσα με τον κόμβο v.
         * Πρόκειται για μια αναζήτηση σε πλάτος με αφετηρία τον κόμβο v.
         * 
         * @param v ο κόμβος αφετηρία
         */
        private void findConnectedComponent(Cell v){
            Stack<Cell> stack;
            stack = new Stack();
            ArrayList<Cell> succesors;
            stack.push(v);
            graph.add(v);
            while(!stack.isEmpty()){
                v = stack.pop();
                succesors = createSuccesors(v, true);
                for (Cell c: succesors) {
                    if (isInList(graph, c) == -1){
                        stack.push(c);
                        graph.add(c);
                    }
                }
            }
        } // end findConnectedComponent()
        
        /**
         * Αρχικοποίηση του αλγόριθμου Dijkstra
         * 
         * Αν προσέξουμε τον ψευδοκώδικα της Wikipedia παρατηρούμε ότι
         * ο αλγόριθμος εξακολουθεί να ψάχνει για τον στόχο ενόσω υπάρχουν ακόμη
         * κόμβοι στην ουρά Q.
         * Μόνον όταν θα εξαντληθεί η ουρά και δεν έχει βρεθεί ο στόχος μπορεί να
         * δώσει απάντηση ότι δεν υπάρχει λύση.
         * Ως γνωστόν, ο αλγόριθμος μοντελοποιεί το πρόβλημα σαν ένα συνδεδεμένο
         * γράφο. Αυτονόητο είναι πως λύση δεν υπάρχει μόνον όταν το γράφημα δεν
         * είναι συνεκτικό και ο στόχος βρίσκεται σε διαφορετική συνεκτική συνιστώσα
         * από αυτήν της αρχικής θέσης του ρομπότ.
         * Για να είναι λοιπόν δυνατή η αρνητική απάντηση από τον αλγόριθμο
         * θα πρέπει η αναζήτηση να γίνει ΜΟΝΟΝ στην συνεκτική συνιστώσα στην οποία
         * ανήκει η αρχική θέση του ρομπότ.
         */
        private void initializeDijkstra() {
            // Δημιουργούμε πρώτα την συνεκτική συνιστώσα
            // στην οποία ανήκει η αρχική θέση του ρομπότ.
            graph.removeAll(graph);
            findConnectedComponent(robotStart);
            // Εδώ γίνεται η αρχικοποίηση του αλγόριθμου Dijkstra
            // 2: for each vertex v in Graph;
            for (Cell v: graph) {
                // 3: dist[v] := infinity ;
                v.dist = INFINITY;
                // 5: previous[v] := undefined ;
                v.prev = null;
            }
            // 8: dist[source] := 0;
            graph.get(isInList(graph,robotStart)).dist = 0;
            // 9: Q := the set of all nodes in Graph;
            // Εμείς, αντί για τη μεταβλητή Q θα χρησιμοποιήσουμε
            // την ίδια τη λίστα graph την οποία αρχικοποιήσαμε ήδη.
            
            // Ταξινομούμε την λίστα των κόμβων ως προς dist.
            Collections.sort(graph, new CellComparatorByDist());
            // Αρχικοποιούμε την λίστα των κλειστών κόμβων
            closedSet.removeAll(closedSet);
        } // end initializeDijkstra()

        /**
         * Ζωγραφίζει το πλέγμα
         */
        @Override
        public void paintComponent(Graphics g) {

            super.paintComponent(g);  // Γεμίζει το background χρώμα.

            g.setColor(Color.DARK_GRAY);
            g.fillRect(10, 10, columns*squareSize+1, rows*squareSize+1);

            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < columns; c++) {
                    if (grid[r][c] == EMPTY) {
                        g.setColor(Color.WHITE);
                    } else if (grid[r][c] == ROBOT) {
                        g.setColor(Color.RED);
                    } else if (grid[r][c] == TARGET) {
                        g.setColor(Color.GREEN);
                    } else if (grid[r][c] == OBST) {
                        g.setColor(Color.BLACK);
                    } else if (grid[r][c] == FRONTIER) {
                        g.setColor(Color.BLUE);
                    } else if (grid[r][c] == CLOSED) {
                        g.setColor(Color.CYAN);
                    } else if (grid[r][c] == ROUTE) {
                        g.setColor(Color.YELLOW);
                    }
                    g.fillRect(11 + c*squareSize, 11 + r*squareSize, squareSize - 1, squareSize - 1);
                }
            }
           
            
            if (drawArrows.isSelected()) {
                // Ζωγραφίζουμε όλα τα βέλη από κάθε ανοικτή ή κλειστή κατάσταση
                // προς την προκάτοχό της.
                for (int r = 0; r < rows; r++) {
                    for (int c = 0; c < columns; c++) {
                        // Αν το τρέχον κελί είναι ο στόχος και έχει βρεθεί λύση
                        // ή είναι κελί της διαδρομής προς στο στόχο
                        // ή είναι ανοικτή κατάσταση,
                        // ή κλειστή αλλά όχι η αρχική θέση του ρομπότ
                        if ((grid[r][c] == TARGET && found)  || grid[r][c] == ROUTE  || 
                                grid[r][c] == FRONTIER || (grid[r][c] == CLOSED &&
                                !(r == robotStart.row && c == robotStart.col))){
                            // Η ουρά του βέλους είναι το τρέχον κελί, ενώ
                            // η κορυφή του βέλους είναι το προκάτοχο κελί.
                            Cell head;
                            if (grid[r][c] == FRONTIER){
                                if (dijkstra.isSelected()){
                                    head = findPrev(graph,new Cell(r,c));
                                } else {
                                    head = findPrev(openSet,new Cell(r,c));
                                }
                            } else {
                                head = findPrev(closedSet,new Cell(r,c));
                            }
                            // Οι συντεταγμένες του κέντρου του τρέχοντος κελιού
                            int tailX = 11+c*squareSize+squareSize/2;
                            int tailY = 11+r*squareSize+squareSize/2;
                            // Οι συντεταγμένες του κέντρου του προκάτοχου κελιού
                            int headX = 11+head.col*squareSize+squareSize/2;
                            int headY = 11+head.row*squareSize+squareSize/2;
                            // Αν το τρέχον κελί είναι ο στόχος
                            // ή είναι κελί της διαδρομής προς το στόχο ...
                            if (grid[r][c] == TARGET  || grid[r][c] == ROUTE){
                                // ... σχεδίασε ένα κόκκινο βέλος προς την κατεύθυνση του στόχου.
                                g.setColor(Color.RED);
                                drawArrow(g,tailX,tailY,headX,headY);
                            // Αλλιώς ...
                            } else {
                                // ... σχεδίασε ένα μαύρο βέλος προς το προκάτοχο κελί.
                                g.setColor(Color.BLACK);
                                drawArrow(g,headX,headY,tailX,tailY);
                            }
                        }
                    }
                }
            }
        } // end paintComponent()
        
        /**
         * Ζωγραφίζει ένα βέλος από το σημείο (x2,y2) προς το σημείο (x1,y1)
         */
        private void drawArrow(Graphics g1, int x1, int y1, int x2, int y2) {
            Graphics2D g = (Graphics2D) g1.create();

            double dx = x2 - x1, dy = y2 - y1;
            double angle = Math.atan2(dy, dx);
            int len = (int) Math.sqrt(dx*dx + dy*dy);
            AffineTransform at = AffineTransform.getTranslateInstance(x1, y1);
            at.concatenate(AffineTransform.getRotateInstance(angle));
            g.transform(at);

            // Εμείς ζωγραφίζουμε ένα οριζόντιο βέλος μήκους len
            // που καταλήγει στο σημείο (0,0) με τις δύο αιχμές μήκους arrowSize
            // να σχηματίζουν γωνίες 20 μοιρών με τον άξονα του βέλους ...
            g.drawLine(0, 0, len, 0);
            g.drawLine(0, 0, (int)(arrowSize*Math.sin(70*Math.PI/180)) , (int)(arrowSize*Math.cos(70*Math.PI/180)));
            g.drawLine(0, 0, (int)(arrowSize*Math.sin(70*Math.PI/180)) , -(int)(arrowSize*Math.cos(70*Math.PI/180)));
            // ... και η κλάση AffineTransform αναλαμβάνει τα υπόλοιπα !!!!!!
            // Πώς να μην θαυμάσει κανείς αυτήν την Java !!!!
        } // end drawArrow()
        
    } // end nested classs MazePanel
  
} // end class Maze