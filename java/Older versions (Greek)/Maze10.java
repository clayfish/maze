package maze10;

import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import javax.swing.*;
import javax.swing.event.*;

/**
 *
 * @author Νίκος Κανάργιας (ΑΘΗ-1 2012-13 ΠΛΗ24, ΠΛΗ30, ΠΛΗ31)
 * 
 * Το πρόγραμμα υλοποιεί λέξη προς λέξη τους αλγορίθμους DFS, BFS και A*
 * όπως ακριβώς περιγράφονται στο βιβλίο "Τεχνητή Νοημοσύνη και Έμπειρα Συστήματα"
 * της Ε. Κεραυνού, ΠΑΤΡΑ 2000.
 * Με βάση της παραμέτρους του δημιουργού της κλάσης MazePanel υπολογίζονται
 * τα μεγέθη και οι θέσεις των διαφόρων στοιχείων του panel.
 * Μπορεί ακόμη να μεταβληθεί το μέγεθος του κελιού δίνοντας την επιθυμητή τιμή
 * στην σταθερά SQUARE_SIZE.
 */
public class Maze10 {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        JFrame window = new JFrame("Πώς βρίσκει το στόχο το ρομπότ...;");
        // Μπορείτε να δώσετε διαφορετικές διαστάσεις για το πάνελ.
        // Το πλάτος δεν μπορεί να είναι λιγότερο από 400 pixels
        // και το ύψος δεν μπορεί να είναι λιγότερο από 250 pixels
        window.setContentPane( new MazePanel(400,400) );
        window.pack();
        window.setResizable(false);
        window.setLocation(100,100);
        window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        window.setVisible(true);
    }
    
    /**
     * Αυτή η κλάση καθορίζει το περιχόμενο του panel
     * και περιέχει όλη την λειτουργικότητα του προγράμματος.
     */
    public static class MazePanel extends JPanel {
        
        /*
         **********************************************************
         *          Ένθετες κλάσεις στην MazePanel
         **********************************************************
         */
        
        /**
         * Βοηθητική κλάση που αναπαριστά το κελί του δωματίου
         */
        private class Cell {
            int row;   // ο αριθμός γραμμής του κελιού (Η γραμμή 0 είναι η πάνω)
            int col;   // ο αριθμός στήλης του κελιού (Η στήλη 0 είναι η αριστερή)
            int g;     // η τιμή της συνάρτησης g του αλγορίθμου Α*
            int h;     // η τιμή της συνάρτησης h του αλγορίθμου Α*
            int f;     // η τιμή της συνάρτησης f του αλγορίθμου Α*
            Cell prev; // κάθε κατάσταση αντιστοιχεί σε κάποιο cell
                       // και κάθε κατάσταση έχει μια προκάτοχο η οποία
                       // αποθηκεύεται σε αυτή τη μεταβλητή
            
            public Cell(int row, int col){
               this.row = row;
               this.col = col;
            }
        } // end nested class Cell
      
        /**
         * Βοηθητική κλάση που καθορίζει ότι τα κελιά θα ταξινομούνται με βάση
         * μεταβλητή τους f
         */
        private class CellComparator implements Comparator<Cell>{
            @Override
            public int compare(Cell cell1, Cell cell2){
                return cell1.f -cell2.f;
            }
        } // end nested class CellComparator
      
        /**
         * Κλάση που χειρίζεται τις κινήσεις του ποντικιού καθώς "ζωγραφίζουμε"
         * εμπόδια ή μετακινούμε το ρομπότ ή/και τον στόχο.
         */
        private class MouseHandler implements MouseListener, MouseMotionListener {
            private int cur_row, cur_col, cur_val;
            @Override
            public void mousePressed(MouseEvent evt) {
                int row = (evt.getY() - 10) / SQUARE_SIZE;
                int col = (evt.getX() - 10) / SQUARE_SIZE;
                if (row >= 0 && row < rows && col >= 0 && col < columns && !searching) {
                    cur_row = row;
                    cur_col = col;
                    cur_val = room[row][col];
                    if (cur_val == EMPTY){
                        room[row][col] = OBST;
                    }
                    if (cur_val == OBST){
                        room[row][col] = EMPTY;
                    }
                }
                repaint();
            }

            @Override
            public void mouseDragged(MouseEvent evt) {
                int row = (evt.getY() - 10) / SQUARE_SIZE;
                int col = (evt.getX() - 10) / SQUARE_SIZE;
                if (row >= 0 && row < rows && col >= 0 && col < columns&& !searching){
                    if ((row*columns+col != cur_row*columns+cur_col) && (cur_val == ROBOT || cur_val == TARGET)){
                        int new_val = room[row][col];
                        room[row][col] = cur_val;
                        if (cur_val == ROBOT) {
                            robotStart.row = row;
                            robotStart.col = col;
                        } else {
                            targetPos.row = row;
                            targetPos.col = col;
                        }
                        room[cur_row][cur_col] = new_val;
                        cur_row = row;
                        cur_col = col;
                        if (cur_val == ROBOT) {
                            robotStart.row = cur_row;
                            robotStart.col = cur_col;
                        } else {
                            targetPos.row = cur_row;
                            targetPos.col = cur_col;
                        }
                        cur_val = room[row][col];
                    } else if (room[row][col] != ROBOT && room[row][col] != TARGET){
                        room[row][col] = OBST;            
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
                } else if (cmd.equals("Βήμα - Βήμα") && !found && !endOfSearch) {
                    searching = true;
                    message.setText("Επιλέξτε 'Βήμα-Βήμα' ή 'Κίνηση' ή 'Καθάρισμα'");
                    dfs.setEnabled(false);
                    bfs.setEnabled(false);
                    aStar.setEnabled(false);
                    timer.stop();
                    // Στην 'Βήμα-Βήμα' αναζήτηση εδώ έχουμε το 2ο βήμα των αλγορίθμων
                    // 2. Εάν ΑΝΟΙΚΤΕΣ = [], τότε τερμάτισε. Δεν υπάρχει λύση.
                    if (openSet.isEmpty()) {
                        endOfSearch = true;
                        message.setText("Δεν υπάρχει διαδρομή για τον στόχο !!!");
                    } else {
                        expandNode();
                        if (found) {
                            plotRoute();
                        }
                    }
                    repaint();
                } else if (cmd.equals("Κίνηση") && !endOfSearch) {
                    searching = true;
                    message.setText("Επιλέξτε 'Βήμα-Βήμα' ή 'Κίνηση' ή 'Καθάρισμα'");
                    dfs.setEnabled(false);
                    bfs.setEnabled(false);
                    aStar.setEnabled(false);
                    timer.setDelay(delay);
                    timer.start();
                }
            }
        } // end nested class ActionHandler
   
        
        /**
         * Η κλάση του είναι υπεύθυνη για το animation
         */
        private class RepaintAction implements ActionListener {
            @Override
            public void actionPerformed(ActionEvent evt) {
                expandNode();
                repaint();
                if (found || openSet.isEmpty()){
                    timer.stop();
                    endOfSearch = true;

                    // Στην αναζήτηση με 'Κίνηση' εδώ έχουμε το 2ο βήμα των αλγορίθμων
                    // 2. Εάν ΑΝΟΙΚΤΕΣ = [], τότε τερμάτισε. Δεν υπάρχει λύση.
                    if (openSet.isEmpty()) {
                        message.setText("Δεν υπάρχει διαδρομή για τον στόχο !!!");
                    } else {
                        plotRoute();
                    }
                }
            }
        } // end nested class RepaintAction
      
        /*
         **********************************************************
         *          Σταθερές της κλάσης MazePanel
         **********************************************************
         */
        
        private final static int
            SQUARE_SIZE = 12,  // Το μέγεθος του κελιού
                               // μπορεί να μεταβληθεί σε μια λογική τιμή 
            EMPTY    = 0,  // κενό κελί
            OBST     = 1,  // κελί με εμπόδιο
            ROBOT    = 2,  // η θέση του ρομπότ
            TARGET   = 3,  // η θέση του στόχου
            FRONTIER = 4,  // κελιά του μετώπου αναζήτησης (ΑΝΟΙΚΤΈΣ καταστάσεις)
            CLOSED   = 5,  // κελιά κλειστών καταστάσεων
            ROUTE    = 6;  // κελιά που σχηματίζουν τη διαδρομή ρομπότ-στόχος

        /*
         **********************************************************
         *          Μεταβλητές της κλάσης MazePanel
         **********************************************************
         */
        
        ArrayList<Cell> openSet = new ArrayList();  // το σύνολο ανοικτών καταστάσεων
        ArrayList<Cell> closedSet = new ArrayList();// το σύνολο κλειστών καταστάσεων
         
        Cell robotStart ; // η αρχική θέση του ρομπότ
        Cell targetPos ;  // η θέση του στόχου
      
        // Διάφορα μηνύματα πρός τον χρήστη
        JLabel message; 
        JLabel programer;
        JLabel robot;
        JLabel target;
        JLabel frontier;
        JLabel closed;
        JLabel velocity;
        
        // τα κουμπιά για την επιλογή του αλγορίθμου
        JRadioButton dfs, bfs, aStar;
        
        // ο slider για την ρύθμιση της ταχύτητας του animation
        JSlider slider;

        // Οι παρακάτω μεταβλητές εξαρτώνται από το μέγεθος του παραθύρου
        int rows;     // Ο αριθμός των γραμμών του δωματίου
        int columns;  // Ο αριθμός των στηλών του δωματίου

        int[][] room;        // το δωμάτιο
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
         * Από τις τιμές του πλάτους και του ύψους προσδιορίζεται
         * η τοποθέτηση των περιεχομένων του panel
         */
        public MazePanel(int width, int height) {
      
            setLayout(null);
            
            MouseHandler listener = new MouseHandler();
            addMouseListener(listener);
            addMouseMotionListener(listener);

            setBorder(BorderFactory.createMatteBorder(2,2,2,2,Color.blue));
         
            // το πλάτος του panel έχει ελάχιστη τιμή 400 pixels και
            // μέγιστη τιμή το πλάτος της οθόνης μείον 20 pixels
            // το ύψος του panel έχει ελάχιστη τιμή 250 pixels και
            // μέγιστη τιμή το ύψος της οθόνης μείον 40 pixels
            Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
            double screenWidth = screenSize.getWidth();
            double ScreenHeight = screenSize.getHeight();
            if (width < 400) {
                width = 400;
            } else {
                width = Math.min((int)screenWidth-20, width);
            }
            if (height < 250) {
                height = 250;
            } else {
                height = Math.min((int)ScreenHeight-40, height);
            }
         
            setPreferredSize( new Dimension(width,height) );

            // προσδιορίζουμε των αριθμό των γραμμών και των στηλών του δωματίου
            rows = (height - 170) / SQUARE_SIZE ;
            columns = (width - 20) / SQUARE_SIZE;
         
            room = new int[rows][columns];

            // Δημιουργούμε τα περιεχόμενα του panel

            message = new JLabel("\"Σχεδιάστε\" εμπόδια και επιλέξτε 'Βήμα-Βήμα' ή 'Κίνηση'", JLabel.CENTER);
            message.setForeground(Color.blue);
            message.setFont(new Font("Helvetica",Font.PLAIN,14));

            programer = new JLabel("Προγραμματιστής: Νίκος Κανάργιας  ver. 1.0", JLabel.CENTER);
            programer.setForeground(Color.red);
            programer.setFont(new Font("Helvetica",Font.PLAIN,12));

            robot = new JLabel("Ρομπότ", JLabel.CENTER);
            robot.setForeground(Color.red);
            robot.setFont(new Font("Helvetica",Font.PLAIN,14));

            target = new JLabel("Στόχος", JLabel.CENTER);
            target.setForeground(Color.GREEN);
            target.setFont(new Font("Helvetica",Font.PLAIN,14));
         
            frontier = new JLabel("Μέτωπο", JLabel.CENTER);
            frontier.setForeground(Color.blue);
            frontier.setFont(new Font("Helvetica",Font.PLAIN,14));

            closed = new JLabel("Κλειστές", JLabel.CENTER);
            closed.setForeground(Color.CYAN);
            closed.setFont(new Font("Helvetica",Font.PLAIN,14));

            JButton clearButton = new JButton("Καθάρισμα");
            clearButton.addActionListener(new ActionHandler());
            clearButton.setBackground(Color.lightGray);
            clearButton.setToolTipText("Πρώτο κλικ: καθάρισμα αναζήτησης, Δεύτερο κλικ: καθάρισμα εμποδίων");

            JButton stepButton = new JButton("Βήμα - Βήμα");
            stepButton.addActionListener(new ActionHandler());
            stepButton.setBackground(Color.lightGray);
            stepButton.setToolTipText("Η πορεία προς τον στόχο γίνεται βήμα-βήμα για κάθε κλικ.");

            JButton animationButton = new JButton("Κίνηση");
            animationButton.addActionListener(new ActionHandler());
            animationButton.setBackground(Color.lightGray);
            animationButton.setToolTipText("Η πορεία προς τον στόχο γίνεται αυτόματα.");

            velocity = new JLabel("Ταχύτητα", JLabel.CENTER);
            velocity.setFont(new Font("Helvetica",Font.PLAIN,10));
            
            // αρχική τιμή καθυστέρησης 500 msec
            slider = new JSlider(0,1000,500);
            slider.setToolTipText("Ρυθμίζει την καθυστέρηση σε κάθε βήμα (0 μέχρι 1 sec).");
            
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
            
            // ButtonGroup που συγχρονίζει τα τρία RadioButtons
            // που επιλέγουν τον αλγόριθμο, έτσι ώστε ένα μόνο από
            // αυτά να μπορεί να επιλεγεί ανά πάσα στιγμή
            ButtonGroup algoGroup = new ButtonGroup();

            dfs = new JRadioButton("DFS");
            dfs.setToolTipText("Αλγόριθμος αναζήτησης σε βάθος");
            algoGroup.add(dfs);
            dfs.addActionListener(new ActionHandler());
            add(dfs);

            bfs = new JRadioButton("BFS");
            bfs.setToolTipText("Αλγόριθμος αναζήτησης σε πλάτος");
            algoGroup.add(bfs);
            bfs.addActionListener(new ActionHandler());
            add(bfs);

            aStar = new JRadioButton("A*");
            aStar.setToolTipText("Αλγόριθμος αναζήτησης Α*");
            algoGroup.add(aStar);
            aStar.addActionListener(new ActionHandler());
            add(aStar);

            dfs.setSelected(true);  // επιλέγουμε αρχικά τον DFS

            // προσθέτουμε τα περιεχόμενα στο panel
            add(message);
            add(robot);
            add(target);
            add(frontier);
            add(closed);
            add(programer);
            add(clearButton);
            add(stepButton);
            add(animationButton);
            add(dfs);
            add(bfs);
            add(aStar);
            add(velocity);
            add(slider);

            // ρυθμίζουμε τα μεγέθη και τις θέσεις τους
            message.setBounds(15, height-155, width-30, 23);
            robot.setBounds(15, height-85, (width-50)*5/20, 23);
            target.setBounds(15+(width-60)*5/20, height-85, (width-60)*5/20, 23);
            frontier.setBounds(15, height-55, (width-60)*5/20, 23);
            closed.setBounds(15+(width-60)*5/20, height-55, (width-60)*5/20, 23);
            clearButton.setBounds(15, height-120, (width-60)*10/20, 28);
            stepButton.setBounds(30+(width-60)*10/20, height-120, (width-60)*7/20, 28);
            animationButton.setBounds(30+(width-60)*10/20, height-85, (width-60)*7/20, 28);
            dfs.setBounds(45+(width-60)*17/20, height-120, (width-60)*3/20, 28);
            bfs.setBounds(45+(width-60)*17/20, height-85, (width-60)*3/20, 28);
            aStar.setBounds(45+(width-60)*17/20, height-50, (width-60)*3/20, 28);
            velocity.setBounds(30+(width-60)*10/20, height-60, (width-60)*7/20, 23);
            slider.setBounds(30+(width-60)*10/20, height-42, (width-60)*7/20, 23);
            programer.setBounds(15, height-15, width-30, 23);

            // δημιουργούμε τον timer
            timer = new Timer(delay, action);
            
            // δίνουμε το κελιά του δωματίου αρχικές τιμές
            fillGrid();

        } // end constructor

        /**
         * Επεκτείνει ένα κόμβο και δημιουργεί τους διαδόχους του
         */
        private void expandNode(){
            Cell current = null;
            if (dfs.isSelected() || bfs.isSelected()) {
                // Εδώ έχουμε το 3ο βήμα των αλγορίθμων DFS και BFS
                // 3. Αφαίρεσε την πρώτη κατάσταση Si, από τις ΑΝΟΙΚΤΕΣ ....
                current = openSet.remove(0);
            }
            if (aStar.isSelected()) {
                // Εδώ έχουμε το 3ο βήμα του αλγορίθμου Α*
                // 3. Αφαίρεσε την κατάσταση Si, από την λίστα ΑΝΟΙΚΤΕΣ,
                //    για την οποία f(Si) <= f(Sj) για όλες τις άλλες
                //    ανοικτές καταστάσεις Sj ...
                // (ταξινομούμε πρώτα τη λίστα ΑΝΟΙΚΤΕΣ κατά αύξουσα σειρά ως προς f)
                Collections.sort(openSet, new CellComparator());
                current = openSet.remove(0);
            }
            // ... και πρόσθεσέ την στις ΚΛΕΙΣΤΕΣ.
            closedSet.add(0,current);
            // Ενημέρωσε το χρώμα του κελιού
            room[current.row][current.col] = CLOSED;
            // Καταμετρούμε τους κόμβους που έχουμε αναπτύξει.
            expanded++;
            ArrayList<Cell> succesors;
            if (dfs.isSelected() || bfs.isSelected()) {
                // Εδώ έχουμε το 4ο βήμα των αλγορίθμων DFS και BFS
                // 4. Δημιούργησε τις διαδόχους της Si, που δεν ανήκουν ήδη στις ΑΝΟΙΚΤΕΣ
                //    ή ΚΛΕΙΣΤΕΣ, με βάση τις ενέργειες που μπορούν να εφαρμοστούν στην Si.
                //    Η κάθε διάδοχος έχει ένα δείκτη προς την Si, ως την προκάτοχό της.
                succesors = succesors_DFS_BFS(current);
            } else {
                // Εδώ έχουμε το 4ο βήμα του αλγορίθμου A*
                // 4. Δημιούργησε τις διαδόχους της Si,
                //    Η κάθε διάδοχος έχει ένα δείκτη προς την Si, ως την προκάτοχό της.
                succesors = succesors_aStar(current);
            }
            // Εδώ έχουμε το 5ο βήμα των αλγορίθμων DFS και BFS
            // 5. Για κάθε διάδοχο της Si, ...
            for (Cell cell: succesors){
                // ... εάν η Sg ανήκει στους διαδόχους της Si, ...
                if (cell.row == targetPos.row && cell.col == targetPos.col){
                    // ... τότε τερμάτισε κλπ
                    Cell last = targetPos;
                    last.prev = cell.prev;
                    closedSet.add(last);
                    found = true;
                    break;
                // Διαφορετικά ...
                } else {
                    // ... αν τρέχουμε τον DFS ...
                    if (dfs.isSelected()) {
                        // ... πρόσθεσε τον διάδοχο στην αρχή της λίστας ΑΝΟΙΚΤΕΣ
                        openSet.add(0, cell);
                        // Ενημέρωσε το χρώμα του κελιού
                        room[cell.row][cell.col] = FRONTIER;
                    // ... αν τρέχουμε τον ΒFS ...
                    } else if (bfs.isSelected()){
                        // ... πρόσθεσε τον διάδοχο στο τέλος της λίστας ΑΝΟΙΚΤΕΣ
                        openSet.add(cell);
                        // Ενημέρωσε το χρώμα του κελιού
                        room[cell.row][cell.col] = FRONTIER;
                    // ... αν τρέχουμε τον Α* ...
                    } else if (aStar.isSelected()){
                        // ... υπολόγισε την τιμή f(Sj)...
                        cell.g = current.g+1;
                        cell.h = Math.abs(targetPos.row-cell.row)+
                                 Math.abs(targetPos.col-cell.col);
                        cell.f = cell.g+cell.h;
                        // ... αν η Sj δεν ανήκει ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        int openIndex   = isInList(openSet,cell);
                        int closedIndex = isInList(closedSet,cell);
                        if (openIndex == -1 && closedIndex == -1) {
                            // ... τότε πρόσθεσε την Sj στις ΑΝΟΙΚΤΕΣ ...
                            // ... με τιμή αξιολόγησης f(Sj)
                            openSet.add(cell);
                            // Ενημέρωσε το χρώμα του κελιού
                            room[cell.row][cell.col] = FRONTIER;
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
                                    room[cell.row][cell.col] = FRONTIER;
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
                                    room[cell.row][cell.col] = FRONTIER;
                                }
                            }
                        }
                    }
               }
            }
        } //end expandNode()
        
        /**
         * Δημιουργεί τους διαδόχους μιας κατάστασης/κελιού
         * για τους αλγορίθμους DFS και BFS
         * 
         * @param current το κελί του οποίου ζητούμε τους διαδόχους
         * @return οι διάδοχοι του κελιού με μορφή λίστας
         */
        private ArrayList<Cell> succesors_DFS_BFS(Cell current){
            int r = current.row;
            int c = current.col;
            // Δημιουργούμε μια κενή λίστα για τους διαδόχους του τρέχοντος κελιού.
            ArrayList<Cell> temp = new ArrayList<>();
            // Αν δεν βρισκόμαστε στο αριστερό όριο του δωματίου
            // και το αριστερό κελί δεν είναι εμπόδιο και δεν ανήκει ήδη
            // ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
            if (c > 0 && (room[r][c-1] != OBST && 
                          isInList(openSet,new Cell(r,c-1)) == -1 &&
                          isInList(closedSet,new Cell(r,c-1)) == -1)) {
                Cell cell = new Cell(r,c-1);
                // ... ενημέρωσε τον δείκτη του αριστερού κελιού να δείχνει το τρέχον ...
                cell.prev = current;
                // ... και πρόσθεσε το αριστερό κελί στους διαδόχους του τρέχοντος.
                temp.add(cell);
            }
            // Αν δεν βρισκόμαστε στο κάτω όριο του δωματίου
            // και το κάτω κελί δεν είναι εμπόδιο και δεν ανήκει ήδη
            // ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
            if (r < rows-1 && (room[r+1][c] != OBST &&
                          isInList(openSet,new Cell(r+1,c)) == -1 &&
                          isInList(closedSet,new Cell(r+1,c)) == -1)) {
                Cell cell = new Cell(r+1,c);
                // ... ενημέρωσε τον δείκτη του αριστερού κελιού να δείχνει το τρέχον ...
                cell.prev = current;
                // ... και πρόσθεσε το κάτω κελί στους διαδόχους του τρέχοντος.
                temp.add(cell);
            }
            // Αν δεν βρισκόμαστε στο δεξί όριο του δωματίου
            // και το δεξί κελί δεν είναι εμπόδιο και δεν ανήκει ήδη
            // ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
            if (c < columns-1 && (room[r][c+1] != OBST &&
                          isInList(openSet,new Cell(r,c+1)) == -1 &&
                          isInList(closedSet,new Cell(r,c+1)) == -1)) {
                Cell cell = new Cell(r,c+1);
                // ... ενημέρωσε τον δείκτη του αριστερού κελιού να δείχνει το τρέχον ...
                cell.prev = current;
                // ... και πρόσθεσε το δεξί κελί στους διαδόχους του τρέχοντος. 
                temp.add(cell);
            }
            // Αν δεν βρισκόμαστε στο πάνω όριο του δωματίου
            // και το πάνω κελί δεν είναι εμπόδιο και δεν ανήκει ήδη
            // ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
            if (r > 0 && (room[r-1][c] != OBST &&
                          isInList(openSet,new Cell(r-1,c)) == -1 &&
                          isInList(closedSet,new Cell(r-1,c)) == -1)) {
                Cell cell = new Cell(r-1,c);
                // ... ενημέρωσε τον δείκτη του αριστερού κελιού να δείχνει το τρέχον ...
                cell.prev = current;
                // ... και πρόσθεσε το πάνω κελί στους διαδόχους του τρέχοντος. 
                temp.add(cell);
            }
           return temp;
        } // end succesors_DFS_BFS()
        
        /**
         * Δημιουργεί τους διαδόχους μιας κατάστασης/κελιού
         * για τον αλγόριθμο Α*
         * 
         * @param current το κελί του οποίου ζητούμε τους διαδόχους
         * @return οι διάδοχοι του κελιού με μορφή λίστας
         */
        private ArrayList<Cell> succesors_aStar(Cell current){
            int r = current.row;
            int c = current.col;
            // Δημιουργούμε μια κενή λίστα για τους διαδόχους του τρέχοντος κελιού.
            ArrayList<Cell> temp = new ArrayList<>();
            // Αν δεν βρισκόμαστε στο αριστερό όριο του δωματίου
            // και το αριστερό κελί δεν είναι εμπόδιο ...
            if (c > 0 && (room[r][c-1] != OBST)) {
                Cell cell = new Cell(r,c-1);
                // ... ενημέρωσε τον δείκτη του αριστερού κελιού να δείχνει το τρέχον ...
                cell.prev = current;
                // ... και πρόσθεσε το αριστερό κελί στους διαδόχους του τρέχοντος. 
                temp.add(cell);
            }
            // Αν δεν βρισκόμαστε στο κάτω όριο του δωματίου
            // και το κάτω κελί δεν είναι εμπόδιο ...
            if (r < rows-1 && (room[r+1][c] != OBST)) {
                Cell cell = new Cell(r+1,c);
                // ... ενημέρωσε τον δείκτη του αριστερού κελιού να δείχνει το τρέχον ...
                cell.prev = current;
                // και πρόσθεσε το κάτω κελί στους διαδόχους του τρέχοντος. 
                temp.add(cell);
            }
            // Αν δεν βρισκόμαστε στο δεξί όριο του δωματίου
            // και το δεξί κελί δεν είναι εμπόδιο ...
            if (c < columns-1 && (room[r][c+1] != OBST)) {
                Cell cell = new Cell(r,c+1);
                // ... ενημέρωσε τον δείκτη του αριστερού κελιού να δείχνει το τρέχον ...
                cell.prev = current;
                // και πρόσθεσε το δεξί κελί στους διαδόχους του τρέχοντος. 
                temp.add(cell);
            }
            // Αν δεν βρισκόμαστε στο πάνω όριο του δωματίου
            // και το πάνω κελί δεν είναι εμπόδιο ...
            if (r > 0 && (room[r-1][c] != OBST)) {
                Cell cell = new Cell(r-1,c);
                // ... ενημέρωσε τον δείκτη του αριστερού κελιού να δείχνει το τρέχον ...
                cell.prev = current;
                // ... και πρόσθεσε το πάνω κελί στους διαδόχους του τρέχοντος. 
                temp.add(cell);
            }
           return temp;
        } // end succesors_aStar()

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
            Cell prev = list.get(0).prev;
            for (int i = 0 ; i < list.size(); i++) {
                if (current.row == list.get(i).row && current.col == list.get(i).col) {
                    prev = list.get(i).prev;
                    break;
                }
            }
            return prev;
        } // end findPrev()
        
        /**
         * Υπολογίζει την διαδρομή από τον στόχο προς την αρχική θέση
         * του ρομπότ και μετρά τα αντίστοιχα βήματα
         */
        private void plotRoute(){
            searching = false;
            endOfSearch = true;
            System.out.println();
            int steps = 0;
            Cell cur = targetPos;
            room[cur.row][cur.col]= TARGET;
            do {
                steps++;
                cur = findPrev(closedSet,cur);
                room[cur.row][cur.col] = ROUTE;
            } while (cur != robotStart);
            room[robotStart.row][robotStart.col]=ROBOT;
            message.setText("Αναπτύχθηκαν "+expanded+" κόμβοι. Χρειάστηκαν "+steps+" βήματα !!!");
          
        } // end plotRoute()
        
        /**
         * Δίνει αρχικές τιμές στα κελιά του δωματίου
         * Με το πρώτο κλικ στο κουμπί 'Καθάρισμα' μηδενίζει τα στοιχεία
         * της τυχόν αναζήτησης που είχε εκτελεστεί (Μέτωπο, Κλειστές, Διαδρομή) 
         * και αφήνει ανέπαφα τα εμπόδια και τις θέσεις ρομπότ και στόχου
         * προκειμένου να είναι δυνατή η εκτέλεση άλλου αλγορίθμου
         * με τα ίδια δεδομένα.
         * Με το δεύτερο κλικ αφαιρεί και τα εμπόδια.
         */
        private void fillGrid() {
            if (searching || endOfSearch){ 
                for (int r = 0; r < rows; r++) {
                    for (int c = 0; c < columns; c++) {
                        if (room[r][c] == FRONTIER || room[r][c] == CLOSED || room[r][c] == ROUTE) {
                            room[r][c] = EMPTY;
                        }
                        if (room[r][c] == ROBOT){
                            robotStart = new Cell(r,c);
                        }
                        if (room[r][c] == TARGET){
                            targetPos = new Cell(r,c);
                        }
                    }
                }
                searching = false;
            } else {
                for (int r = 0; r < rows; r++) {
                    for (int c = 0; c < columns; c++) {
                        room[r][c] = EMPTY;
                    }
                }
                robotStart = new Cell(rows-3,2);
                targetPos = new Cell(2,columns-3);
            }
            if (aStar.isSelected()){
                robotStart.g = 0;
                robotStart.h = 0;
                robotStart.f = 0;
            }
            expanded = 0;
            found = false;
            searching = false;
            endOfSearch = false;
         
            openSet.removeAll(openSet);
            
            // Το πρώτο βήμα των αλγορίθμων γίνεται εδώ
            // 1. ΑΝΟΙΚΤΕΣ:= [So], ΚΛΕΙΣΤΕΣ:= []
            openSet.add(robotStart);
            closedSet.removeAll(closedSet);
         
            room[targetPos.row][targetPos.col] = TARGET; 
            room[robotStart.row][robotStart.col] = ROBOT;
            message.setText("\"Σχεδιάστε\" εμπόδια και επιλέξτε 'Βήμα-Βήμα' ή 'Κίνηση'");
            timer.stop();
            repaint();
            
        } // end fillGrid()

        /**
         * Ζωγραφίζει το δωμάτιο
         */
        @Override
        public void paintComponent(Graphics g) {

            super.paintComponent(g);  // Γεμίζει το background χρώμα.

            g.setColor(Color.white);
            g.fillRect(10, 10, columns*SQUARE_SIZE, rows*SQUARE_SIZE);

            g.setColor(Color.black);
            for (int i = 0; i <= rows; i++){
                g.drawLine(10, 10 + i*SQUARE_SIZE, columns*SQUARE_SIZE + 10, 10 + i*SQUARE_SIZE);
            }
            for (int i = 0; i <= columns; i++) {
                g.drawLine(10 + i*SQUARE_SIZE, 10, 10 + i*SQUARE_SIZE, rows*SQUARE_SIZE + 10);
            }

            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < columns; c++) {
                    if (room[r][c] == EMPTY) {
                        g.setColor(Color.WHITE);
                    } else if (room[r][c] == ROBOT) {
                        g.setColor(Color.RED);
                    } else if (room[r][c] == TARGET) {
                        g.setColor(Color.GREEN);
                    } else if (room[r][c] == OBST) {
                        g.setColor(Color.BLACK);
                    } else if (room[r][c] == FRONTIER) {
                        g.setColor(Color.blue);
                    } else if (room[r][c] == CLOSED) {
                        g.setColor(Color.CYAN);
                    } else if (room[r][c] == ROUTE) {
                        g.setColor(Color.YELLOW);
                    }
                    g.fillRect(11 + c*SQUARE_SIZE, 11 + r*SQUARE_SIZE, SQUARE_SIZE - 1, SQUARE_SIZE - 1);
                }

            }
        } // end paintComponent();

    } // end nested classs MazePanel
  
} // end class Maze
