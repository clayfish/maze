using System;
using System.Collections.Generic;
using System.Collections;
using System.Drawing;
using System.Windows.Forms;

/*
 *
 * @author Nikos Kanargias
 * E-mail: nkana@tee.gr
 * @version 5.2
 * 
 * The software solves and visualizes the robot motion planning problem,
 * by implementing variants of DFS, BFS and A* algorithms, as described
 * by E. Keravnou in her book: "Artificial Intelligence and Expert Systems",
 * Hellenic Open University,  Patra 2000 (in Greek)
 * as well as the Greedy search algorithm, as a special case of A*.
 * 
 * The software also implements Dijkstra's algorithm,
 * as just described in the relevant article in Wikipedia.
 * http://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
 * 
 * The superiority of  A* and Dijkstra's algorithms against the other three becomes obvious.
 * 
 * The user can change the number of the grid cells, indicating
 * the desired number of rows and columns.
 * 
 * The user can add as many obstacles he/she wants, as he/she
 * would "paint" free curves with a drawing program.
 * 
 * Individual obstacles can be removed by clicking them.
 * 
 * The position of the robot and/or the target can be changed by dragging with the mouse.
 * 
 * Jump from search in "Step-by-Step" way to "Animation" way and vice versa is done
 * by pressing the corresponding button, even when the search is in progress.
 * 
 * The speed of a search can be changed, even if the search is in progress.
 * It is sufficient to place the slider "Delay" in the new desired position
 * and then press the "Animation" button.
 * 
 * The application considers that the robot itself has some volume.
 * Therefore it can’t move diagonally to a free cell passing between two obstacles
 * adjacent to one apex.
 *
 * When 'Step-by-Step' or 'Animation' search is underway it is not possible to change
 * the position of obstacles, robot and target, as well as the search algorithm.
 * 
 * When 'Real-Time' search is underway the position of obstacles, robot and target
 * can be changed.
 * 
 * Advisable not to draw arrows to predecessors in large grids.
 */

namespace Maze52
{
    /// <summary>
    /// The main form of the application 
    /// </summary>
    public partial class MainForm : Form
    {
        /// <summary>
        /// Creates a random, perfect (without cycles) maze
        /// </summary>
        private class MyMaze
        {
            // The code of the class is an adaptation, with the original commentary, of the answer given
            // by user DoubleMx2 on August 25'13 to a question posted by user nazar_art at stackoverflow.com:
            // http://stackoverflow.com/questions/18396364/maze-generation-arrayindexoutofboundsexception

            private int dimensionX, dimensionY; // dimension of maze
            public int gridDimensionX, gridDimensionY; // dimension of output grid
            public char[,] mazeGrid; // output grid
            private Cell[,] cells; // 2d array of Cells
            private Random random = new Random(); // The random object

            // constructor
            public MyMaze(int xDimension, int yDimension)
            {
                dimensionX = xDimension;              // dimension of maze
                dimensionY = yDimension;
                gridDimensionX = xDimension * 2 + 1;  // dimension of output grid
                gridDimensionY = yDimension * 2 + 1;
                mazeGrid = new char[gridDimensionX, gridDimensionY];
                Init();
                GenerateMaze();
            }

            private void Init()
            {
                // create cells
                cells = new Cell[dimensionX, dimensionY];
                for (int x = 0; x < dimensionX; x++)
                    for (int y = 0; y < dimensionY; y++)
                        cells[x, y] = new Cell(x, y, false); // create cell (see Cell constructor)
            }

            // inner class to represent a cell
            public class Cell
            {
                public int x, y; // coordinates
                // cells this cell is connected to
                ArrayList neighbors = new ArrayList();
                // impassable cell
                public bool wall = true;
                // if true, has yet to be used in generation
                public bool open = true;
                // construct Cell at x, y
                public Cell(int x, int y)
                {
                    this.x = x;
                    this.y = y;
                    wall = true;
                }
                // construct Cell at x, y and with whether it isWall
                public Cell(int x, int y, bool isWall)
                {
                    this.x = x;
                    this.y = y;
                    wall = isWall;
                }
                // add a neighbor to this cell, and this cell as a neighbor to the other
                public void AddNeighbor(Cell other)
                {
                    if (!this.neighbors.Contains(other))
                        // avoid duplicates
                        this.neighbors.Add(other);
                    if (!other.neighbors.Contains(this))
                        // avoid duplicates
                        other.neighbors.Add(this);
                }
                // used in updateGrid()
                public bool IsCellBelowNeighbor()
                {
                    return this.neighbors.Contains(new Cell(this.x, this.y + 1));
                }
                // used in updateGrid()
                public bool IsCellRightNeighbor()
                {
                    return this.neighbors.Contains(new Cell(this.x + 1, this.y));
                }
                
                // useful Cell equivalence
                public override bool Equals(Object other)
                {
                    //if (!(other instanceof Cell)) return false;
                    if (other.GetType() != typeof(Cell))
                        return false;
                    Cell otherCell = (Cell)other;
                    return (x == otherCell.x) && (y == otherCell.y);
                }

                // should be overridden with equals
                public override int GetHashCode()
                {
                    // random hash code method designed to be usually unique
                    return x + y * 256;
                }
                
            }
            // generate from upper left (In computing the y increases down often)
            private void GenerateMaze()
            {
                GenerateMaze(0, 0);
            }
            // generate the maze from coordinates x, y
            private void GenerateMaze(int x, int y)
            {
                GenerateMaze(GetCell(x, y)); // generate from Cell
            }
            private void GenerateMaze(Cell startAt)
            {
                // don't generate from cell not there
                if (startAt == null) return;
                startAt.open = false; // indicate cell closed for generation
                var cellsList = new ArrayList { startAt };

                while (cellsList.Count > 0)
                {
                    Cell cell;
                    // this is to reduce but not completely eliminate the number
                    // of long twisting halls with short easy to detect branches
                    // which results in easy mazes
                    if (random.Next(10) == 0)
                    {
                        cell = (Cell)cellsList[random.Next(cellsList.Count)];
                        cellsList.RemoveAt(random.Next(cellsList.Count));
                    }

                    else
                    {
                        cell = (Cell)cellsList[cellsList.Count - 1];
                        cellsList.RemoveAt(cellsList.Count - 1);
                    }
                    // for collection
                    ArrayList neighbors = new ArrayList();
                    // cells that could potentially be neighbors
                    Cell[] potentialNeighbors = new Cell[]{
                        GetCell(cell.x + 1, cell.y),
                        GetCell(cell.x, cell.y + 1),
                        GetCell(cell.x - 1, cell.y),
                        GetCell(cell.x, cell.y - 1)
                    };
                    foreach (Cell other in potentialNeighbors)
                    {
                        // skip if outside, is a wall or is not opened
                        if (other == null || other.wall || !other.open)
                            continue;
                        neighbors.Add(other);
                    }
                    if (neighbors.Count == 0) continue;
                    // get random cell
                    Cell selected = (Cell)neighbors[random.Next(neighbors.Count)];
                    // add as neighbor
                    selected.open = false; // indicate cell closed for generation
                    cell.AddNeighbor(selected);
                    cellsList.Add(cell);
                    cellsList.Add(selected);
                }
                UpdateGrid();
            }
            // used to get a Cell at x, y; returns null out of bounds
            public Cell GetCell(int x, int y)
            {
                try
                {
                    return cells[x, y];
                }
                catch (IndexOutOfRangeException)
                { // catch out of bounds
                    return null;
                }
            }
            // draw the maze
            public void UpdateGrid()
            {
                char backChar = ' ', wallChar = 'X', cellChar = ' ';
                // fill background
                for (int x = 0; x < gridDimensionX; x++)
                    for (int y = 0; y < gridDimensionY; y++)
                        mazeGrid[x, y] = backChar;
                // build walls
                for (int x = 0; x < gridDimensionX; x++)
                    for (int y = 0; y < gridDimensionY; y++)
                        if (x % 2 == 0 || y % 2 == 0)
                            mazeGrid[x, y] = wallChar;
                // make meaningful representation
                for (int x = 0; x < dimensionX; x++)
                    for (int y = 0; y < dimensionY; y++)
                    {
                        Cell current = GetCell(x, y);
                        int gridX = x * 2 + 1, gridY = y * 2 + 1;
                        mazeGrid[gridX, gridY] = cellChar;
                        if (current.IsCellBelowNeighbor())
                            mazeGrid[gridX, gridY + 1] = cellChar;
                        if (current.IsCellRightNeighbor())
                            mazeGrid[gridX + 1, gridY] = cellChar;
                    }
            }
        } // end nested class MyMaze

        /// <summary>
        /// Helper class that represents the cell of the grid
        /// </summary>
        private class Cell
        {
            public int row;     // the row number of the cell(row 0 is the top)
            public int col;     // the column number of the cell (Column 0 is the left)
            public double g;    // the value of the function g of A* and Greedy algorithms
            public double h;    // the value of the function h of A* and Greedy algorithms
            public double f;    // the value of the function h of A* and Greedy algorithms
            public double dist; // the distance of the cell from the initial position of the robot
                                // Ie the label that updates the Dijkstra's algorithm
            public Cell prev;   // Each state corresponds to a cell
                                // and each state has a predecessor which
                                // is stored in this variable

            public Cell(int row, int col)
            {
                this.row = row;
                this.col = col;
            }
            
        } // end nested class Cell

        /*
         **********************************************************
         *          Constants of class MazePanel
         **********************************************************
         */
        const int INFINITY = Int32.MaxValue; // The representation of the infinite
        const int EMPTY = 0;      // empty cell
        const int OBST = 1;       // cell with obstacle
        const int ROBOT = 2;      // the position of the robot
        const int TARGET = 3;     // the position of the target
        const int FRONTIER = 4;   // cells that form the frontier (OPEN SET)
        const int CLOSED = 5;     // cells that form the CLOSED SET
        const int ROUTE = 6;      // cells that form the robot-to-target path

        const String MSG_DRAW_AND_SELECT =
            "\"Paint\" obstacles, then click 'Real-Time' or 'Step-by-Step' or 'Animation'";
        const String MSG_SELECT_STEP_BY_STEP_ETC =
            "Click 'Step-by-Step' or 'Animation' or 'Clear'";
        const String MSG_NO_SOLUTION =
            "There is no path to the target !!!";

        /*
         **********************************************************
         *          Variables of class MazePanel
         **********************************************************
         */
        int rows;        // the number of rows of the grid
        int columns;     // the number of columns of the grid
        int squareSize;  // the cell size in pixels
        int arrowSize;   // the size of the tip of the arrow
                         // pointing the predecessor cell

        List<Cell> openSet =   new List<Cell>(); // the OPEN SET
        List<Cell> closedSet = new List<Cell>(); // the CLOSED SET
        List<Cell> graph =     new List<Cell>(); // the set of vertices of the graph
                                                 // to be explored by Dijkstra's algorithm

        Cell robotStart;  // the initial position of the robot
        Cell targetPos;   // the position of the target

        int[,] grid;      // the grid
        bool realTime;    // Solution is displayed instantly
        bool found;       // flag that the goal was found
        bool searching;   // flag that the search is in progress
        bool endOfSearch; // flag that the search came to an end
        bool animation;   // flag that the animation is running
        int delay;        // time delay of animation (in msec)
        int expanded;     // the number of nodes that have been expanded

        bool mouse_down = false;
        int cur_row, cur_col, cur_val;

        /// <summary>
        /// Constructor
        /// </summary>
        public MainForm()
        {
            InitializeComponent();
            dfs.Checked = true;
            message.Text = MSG_DRAW_AND_SELECT;
            InitializeGrid(false);
        } // end MainForm constructor

        /// <summary>
        /// Handles mouse's clicks as we add or remove obstacles
        /// </summary>
        private void MainForm_MouseDown(object sender, MouseEventArgs e)
        {
            mouse_down = true;
            int row = (e.Y - 10) / squareSize;
            int col = (e.X - 10) / squareSize;
            if (row >= 0 && row < rows && col >= 0 && col < columns)
            {
                if (realTime ? true : !found && !searching)
                {
                    if (realTime)
                        FillGrid();
                    cur_row = row;
                    cur_col = col;
                    cur_val = grid[row, col];
                    if (cur_val == EMPTY)
                        grid[row, col] = OBST;
                    if (cur_val == OBST)
                        grid[row, col] = EMPTY;
                    if (realTime && dijkstra.Checked)
                        InitializeDijkstra();
                }
                if (realTime)
                    RealTime_action();
                else
                    Invalidate();
            }
        } // end MainForm_MouseDown

        /// <summary>
        /// Handles mouse's movements as we "paint" obstacles or move the robot and/or target.
        /// </summary>
        private void MainForm_MouseMove(object sender, MouseEventArgs e)
        {
            if (!mouse_down)
                return;
            int row = (e.Y - 10) / squareSize;
            int col = (e.X - 10) / squareSize;
            if (row >= 0 && row < rows && col >= 0 && col < columns)
            {
                if (realTime ? true : !found && !searching)
                {
                    if (realTime)
                        FillGrid();
                    if (!(row == cur_row && col == cur_col) && (cur_val == ROBOT || cur_val == TARGET))
                    {
                        int new_val = grid[row, col];
                        if (new_val == EMPTY)
                        {
                            grid[row, col] = cur_val;
                            if (cur_val == ROBOT)
                            {
                                robotStart.row = row;
                                robotStart.col = col;
                            }
                            else
                            {
                                targetPos.row = row;
                                targetPos.col = col;
                            }
                            grid[cur_row, cur_col] = new_val;
                            cur_row = row;
                            cur_col = col;
                            cur_val = grid[row, col];
                        }
                    }
                    else if (grid[row, col] != ROBOT && grid[row, col] != TARGET)
                        grid[row, col] = OBST;
                    if (realTime && dijkstra.Checked)
                        InitializeDijkstra();
                }
                if (realTime)
                    RealTime_action();
                else
                    Invalidate();
            }
        } // end MainForm_MouseMove

        /// <summary>
        /// When the user releases the mouse
        /// </summary>
        private void MainForm_MouseUp(object sender, MouseEventArgs e)
        {
            mouse_down = false;
        } // end MainForm_MouseUp

        /// <summary>
        /// Creates a new clean grid or a new maze
        /// </summary>
        /// <param name="makeMaze">Flag for maze creation</param>
        private void InitializeGrid(bool makeMaze)
        {
            rows = (int)this.rowsSpinner.Value;
            columns = (int)this.columnsSpinner.Value;
            // the maze must have an odd number of rows and columns
            if (makeMaze && rows % 2 == 0)
                rows -= 1;
            if (makeMaze && columns % 2 == 0)
                columns -= 1;
            squareSize = (int)(500 / (rows > columns ? rows : columns));
            arrowSize = (int)(squareSize / 2);
            grid = new int[rows, columns];
            robotStart = new Cell(rows - 2, 1);
            targetPos = new Cell(1, columns - 2);
            FillGrid();
            if (makeMaze)
            {
                MyMaze maze = new MyMaze(rows / 2, columns / 2);
                for (int x = 0; x < maze.gridDimensionX; x++)
                    for (int y = 0; y < maze.gridDimensionY; y++)
                        if (maze.mazeGrid[x, y] == 'X')
                            grid[x, y] = OBST;


            }
             Invalidate(); // forces the repainting of the grid
        } // end InitializeGrid

        /// <summary>
        /// Gives initial values ​​for the cells in the grid.
        /// </summary>
        private void FillGrid()
        {
            /*
             * With the first click on button 'Clear' clears the data
             * of any search was performed (Frontier, Closed Set, Route)
             * and leaves intact the obstacles and the robot and target positions
             * in order to be able to run another algorithm
             * with the same data.
             * With the second click removes any obstacles also.
             */
            if (searching || endOfSearch || realTime)
            {
                for (int r = 0; r < rows; r++)
                    for (int c = 0; c < columns; c++)
                    {
                        if (grid[r, c] == FRONTIER || grid[r, c] == CLOSED || grid[r, c] == ROUTE)
                            grid[r, c] = EMPTY;
                        if (grid[r, c] == ROBOT)
                            robotStart = new Cell(r, c);
                        if (grid[r, c] == TARGET)
                            targetPos = new Cell(r, c);
                    }
            }
            else
            {
                for (int r = 0; r < rows; r++)
                    for (int c = 0; c < columns; c++)
                        grid[r, c] = EMPTY;
                robotStart = new Cell(rows - 2, 1);
                targetPos = new Cell(1, columns - 2);
            }
            if (aStar.Checked || greedy.Checked)
            {
                robotStart.g = 0;
                robotStart.h = 0;
                robotStart.f = 0;
            }
            expanded = 0;
            found = false;
            searching = (realTime ? true : false);
            endOfSearch = false;

            // The first step of DFS, BFS, A* and Greedy algorithms is here
            // 1. OPEN SET: = [So], CLOSED SET: = []
            openSet.Clear();
            openSet.Add(robotStart);
            closedSet.Clear();

            if (!realTime)
            {
                grid[targetPos.row, targetPos.col] = TARGET;
                grid[robotStart.row, robotStart.col] = ROBOT;
                message.Text = MSG_DRAW_AND_SELECT;
            }
        } // end FillGrid

        /// <summary>
        /// Enables radio buttons and checkboxes
        /// </summary>
        private void EnableRadiosAndChecks()
        {
            slider.Enabled = true;
            dfs.Enabled = true;
            bfs.Enabled = true;
            aStar.Enabled = true;
            greedy.Enabled = true;
            dijkstra.Enabled = true;
            diagonal.Enabled = true;
            drawArrows.Enabled = true;
        } // end EnableRadiosAndChecks

        /// <summary>
        /// Disables radio buttons and checkboxes
        /// </summary>
        private void DisableRadiosAndChecks()
        {
            slider.Enabled = false;
            dfs.Enabled = false;
            bfs.Enabled = false;
            aStar.Enabled = false;
            greedy.Enabled = false;
            dijkstra.Enabled = false;
            diagonal.Enabled = false;
            drawArrows.Enabled = false;
        } // end DisableRadiosAndChecks

        /// <summary>
        /// When the user clicks the "Clear" button
        /// </summary>
        private void ResetButton_Click(object sender, EventArgs e)
        {
            animation = false;
            realTime = false;
            RealTimeButton.Enabled = true;
            RealTimeButton.ForeColor = Color.Black;
            StepButton.Enabled = true;
            AnimationButton.Enabled = true;
            EnableRadiosAndChecks();
            InitializeGrid(false);
        } // end  ResetButton_Click

        /// <summary>
        /// When the user clicks the "Maze" button
        /// </summary>
        private void MazeButton_Click(object sender, EventArgs e)
        {
            animation = false;
            realTime = false;
            RealTimeButton.Enabled = true;
            RealTimeButton.ForeColor = Color.Black;
            StepButton.Enabled = true;
            AnimationButton.Enabled = true;
            EnableRadiosAndChecks();
            InitializeGrid(true);
        }// end MazeButton_Click

        /// <summary>
        /// When the user clicks the "Clear" button
        /// </summary>
        private void ClearButton_Click(object sender, EventArgs e)
        {
            animation = false;
            realTime = false;
            RealTimeButton.Enabled = true;
            RealTimeButton.ForeColor = Color.Black;
            StepButton.Enabled = true;
            AnimationButton.Enabled = true;
            EnableRadiosAndChecks();
            FillGrid();
            Invalidate();
        } // end ClearButton_Click

        /// <summary>
        /// When the user clicks the "Real-Time" button
        /// </summary>
        private void RealTimeButton_Click(object sender, EventArgs e)
        {
            if (realTime)
                return;
            realTime = true;
            searching = true;
            // The Dijkstra's initialization should be done just before the
            // start of search, because obstacles must be in place.
            if (dijkstra.Checked)
                InitializeDijkstra();
            RealTimeButton.ForeColor = Color.Red;
            DisableRadiosAndChecks();
            RealTime_action();
        } // end RealTimeButton_Click

        /// <summary>
        /// Action performed during real-time search
        /// </summary>
        private void RealTime_action()
        {
            do
                CheckTermination();
            while (!endOfSearch);
        } // end RealTime_action

        /// <summary>
        /// When the user clicks the "Step-by-Step" button
        /// </summary>
        private void StepButton_Click(object sender, EventArgs e)
        {
            animation = false;
            timer.Stop();
            if (found || endOfSearch)
                return;
            if (!searching && dijkstra.Checked)
                InitializeDijkstra();
            searching = true;
            message.Text = MSG_SELECT_STEP_BY_STEP_ETC;
            RealTimeButton.Enabled = false;
            DisableRadiosAndChecks();
            slider.Enabled = true;
            CheckTermination();
            Invalidate();
        } // end StepButton_Click

        /// <summary>
        /// When the user clicks the "Animation" button
        /// </summary>
        private void AnimationButton_Click(object sender, EventArgs e)
        {
            animation = true;
            if (!searching && dijkstra.Checked)
                InitializeDijkstra();
            searching = true;
            message.Text = MSG_SELECT_STEP_BY_STEP_ETC;
            RealTimeButton.Enabled = false;
            DisableRadiosAndChecks();
            slider.Enabled = true;
            delay = slider.Value;
            timer.Stop();
            timer.Interval = delay;
            timer.Start();
            Animation_action();
        } // end AnimationButton_Click

        /// <summary>
        /// Action performed during animated search
        /// </summary>
        private void Animation_action()
        {
            if (animation)
            {
                CheckTermination();
                Invalidate();
                if (endOfSearch)
                {
                    animation = false;
                    timer.Stop();
                }
            }
        } // end Animation_action

        /// <summary>
        /// Action performed after the time interval of the timer
        /// </summary>
        private void Timer_Tick(object sender, EventArgs e)
        {
            Animation_action();
        } // end Timer_Tick

        /// <summary>
        /// When the user clicks the "About Maze" button
        /// </summary>
        private void AboutButton_Click(object sender, EventArgs e)
        {
            AboutBox aboutForm = new AboutBox();
            aboutForm.ShowDialog();
        } // end AboutButton_Click

        /// <summary>
        /// Checks if we have reached the end of search
        /// </summary>
        private void CheckTermination()
        {
            // Here we decide whether we can continue the search or not.

            // In the case of Dijkstra's algorithm
            // here we check condition of step 11:
            // 11. while Q is not empty.

            // In the case of DFS, BFS, A* and Greedy algorithms
            // here we have the second step:
            // 2. If OPEN SET = [], then terminate. There is no solution.
            if ((dijkstra.Checked && graph.Count==0) ||
                          (!dijkstra.Checked && openSet.Count==0))
            {
                endOfSearch = true;
                grid[robotStart.row,robotStart.col] = ROBOT;
                message.Text = MSG_NO_SOLUTION;
                StepButton.Enabled = false;
                AnimationButton.Enabled = false;
                Invalidate();
            }
            else
            {
                ExpandNode();
                if (found)
                {
                    endOfSearch = true;
                    PlotRoute();
                    StepButton.Enabled = false;
                    AnimationButton.Enabled = false;
                    slider.Enabled= false;
                    Invalidate();
                }
            }
        } // end CheckTermination

        /// <summary>
        /// Expands a node and creates its successors
        /// </summary>
        private void ExpandNode()
        {
            if (dijkstra.Checked) // Dijkstra's algorithm to handle separately
            {
                //Cell u;
                // 11: while Q is not empty:
                if (graph.Count==0)
                    return;
                // 12:  u := vertex in Q (graph) with smallest distance in dist[] ;
                // 13:  remove u from Q (graph);
                Cell u = (Cell)graph[0];
                graph.RemoveAt(0);
                // Add vertex u in closed set
                closedSet.Add(u);
                // If target has been found ...
                if (u.row == targetPos.row && u.col == targetPos.col)
                {
                    found = true;
                    return;
                }
                // Counts nodes that have expanded.
                expanded++;
                // Update the color of the cell
                grid[u.row,u.col] = CLOSED;
                // 14: if dist[u] = infinity:
                if (u.dist == INFINITY)
                {
                    // ... then there is no solution.
                    // 15: break;
                    return;
                } // 16: end if
                // Create the neighbors of u
                List<Cell> neighbors = CreateSuccesors(u, false);
                // 18: for each neighbor v of u:
                foreach (Cell v in neighbors)
                { 
                    // 20: alt := dist[u] + dist_between(u, v) ;
                    double alt = u.dist + DistBetween(u, v);
                    // 21: if alt < dist[v]:
                    if (alt < v.dist)
                    {
                        // 22: dist[v] := alt ;
                        v.dist = alt;
                        // 23: previous[v] := u ;
                        v.prev = u;
                        // Update the color of the cell
                        grid[v.row,v.col] = FRONTIER;
                        // 24: decrease-key v in Q;
                        // (sort list of nodes with respect to dist)
                        graph.Sort((r1, r2) => r1.dist.CompareTo(r2.dist));
                    }
                }
            }
            else // The handling of the other four algorithms
            {
                Cell current;
                if (dfs.Checked || bfs.Checked)
                {
                    // Here is the 3rd step of the algorithms DFS and BFS
                    // 3. Remove the first state, Si, from OPEN SET ...
                    current = (Cell)openSet[0];
                    openSet.RemoveAt(0);
                }
                else
                {
                    // Here is the 3rd step of the algorithms A* and Greedy
                    // 3. Remove the first state, Si, from OPEN SET,
                    // for which f(Si) ≤ f(Sj) for all other
                    // open states Sj  ...
                    // (sort first OPEN SET list with respect to 'f')
                    openSet.Sort((r1, r2) => r1.f.CompareTo(r2.f));
                    current = (Cell)openSet[0];
                    openSet.RemoveAt(0);
                }
                // ... and add it to CLOSED SET.
                closedSet.Insert(0, current);
                // Update the color of the cell
                grid[current.row,current.col] = CLOSED;
                // If the selected node is the target ...
                if (current.row == targetPos.row && current.col == targetPos.col)
                {
                    // ... then terminate etc
                    Cell last = targetPos;
                    last.prev = current.prev;
                    closedSet.Add(last);
                    found = true;
                    return;
                }
                // Count nodes that have been expanded.
                expanded++;
                // Here is the 4rd step of the algorithms
                // 4. Create the successors of Si, based on actions
                //    that can be implemented on Si.
                //    Each successor has a pointer to the Si, as its predecessor.
                //    In the case of DFS and BFS algorithms, successors should not
                //    belong neither to the OPEN SET nor the CLOSED SET.
                List<Cell> succesors = CreateSuccesors(current, false);
                // Here is the 5th step of the algorithms
                // 5. For each successor of Si, ...
                foreach (Cell cell in succesors)
                {    
                    if (dfs.Checked) // ... if we are running DFS ...
                    {
                        // ... add the successor at the beginning of the list OPEN SET
                        openSet.Insert(0, cell);
                        // Update the color of the cell
                        grid[cell.row,cell.col] = FRONTIER;
                        // ... if we are runnig BFS ...
                    }
                    else if (bfs.Checked)
                    {
                        // ... add the successor at the end of the list OPEN SET
                        openSet.Add(cell);
                        // Update the color of the cell
                        grid[cell.row,cell.col] = FRONTIER;
                    }
                    // ... if we are running A* or Greedy algorithms (step 5 of A* algorithm) ...
                    else if (aStar.Checked || greedy.Checked)
                    {
                        // ... calculate the value f(Sj) ...
                        int dxg = current.col - cell.col;
                        int dyg = current.row - cell.row;
                        int dxh = targetPos.col - cell.col;
                        int dyh = targetPos.row - cell.row;
                        if (diagonal.Checked)
                        {
                            // with diagonal movements 
                            // calculate the Euclidean distance
                            if (greedy.Checked)
                            {
                                // especially for the Greedy ...
                                cell.g = 0;
                            }
                            else
                            {
                                cell.g = current.g + Math.Sqrt(dxg * dxg + dyg * dyg);
                            }
                            cell.h = Math.Sqrt(dxh * dxh + dyh * dyh);
                        }
                        else
                        {
                            // without diagonal movements
                            // calculate the Manhattan distance
                            if (greedy.Checked)
                            {
                                // especially for the Greedy ...
                                cell.g = 0;
                            }
                            else
                            {
                                cell.g = current.g + Math.Abs(dxg) + Math.Abs(dyg);
                            }
                            cell.h = Math.Abs(dxh) + Math.Abs(dyh);
                        }
                        cell.f = cell.g + cell.h;
                         int openIndex = IsInList(openSet, cell);
                        int closedIndex = IsInList(closedSet, cell);
                        // ... If Sj is neither in the OPEN SET nor in the CLOSED SET states ...
                        if (openIndex == -1 && closedIndex == -1)
                        {
                            // ... then add Sj in the OPEN SET ...
                            // ... evaluated as f(Sj)
                            openSet.Add(cell);
                            // Update the color of the cell
                            grid[cell.row,cell.col] = FRONTIER;
                            // Else ...
                        }
                        else
                        {
                            // ... if already belongs to the OPEN SET, then ...
                            if (openIndex > -1)
                            {
                                // ... compare the new value assessment with the old one. 
                                // If old <= new ...
                                Cell openSetCell = (Cell)openSet[openIndex];
                                if (openSetCell.f <= cell.f)
                                {
                                    // ... then eject the new node with state Sj.
                                    // (ie do nothing for this node).
                                    // Else, ...
                                }
                                else
                                {
                                    // ... remove the element (Sj, old) from the list
                                    // to which it belongs ...
                                    openSet.RemoveAt(openIndex);
                                    // ... and add the item (Sj, new) to the OPEN SET.
                                    openSet.Add(cell);
                                    // Update the color of the cell
                                    grid[cell.row,cell.col] = FRONTIER;
                                }
                                // ... if already belongs to the CLOSED SET, then ...
                            }
                            else
                            {
                                // ... compare the new value assessment with the old one. 
                                // If old <= new ...
                                Cell closedSetCell = (Cell)closedSet[closedIndex];
                                if (closedSetCell.f <= cell.f)
                                {
                                    // ... then eject the new node with state Sj.
                                    // (ie do nothing for this node).
                                    // Else, ...
                                }
                                else
                                {
                                    // ... remove the element (Sj, old) from the list
                                    // to which it belongs ...
                                    closedSet.RemoveAt(closedIndex);
                                    // ... and add the item (Sj, new) to the OPEN SET.
                                    openSet.Add(cell);
                                    // Update the color of the cell
                                    grid[cell.row,cell.col] = FRONTIER;
                                }
                            }
                        }
                    }
                }
            }
        } //end ExpandNode()

        /// <summary>
        /// Creates the successors of a state/cell
        /// </summary>
        /// <param name="current">The cell for which we ask successors</param>
        /// <param name="makeConnected">
        /// Flag that indicates that we are interested only on the coordinates 
        /// of cells and not on the label 'dist' (concerns only Dijkstra's)</param>
        /// <returns>The successors of the cell as a list</returns>
        private List<Cell> CreateSuccesors(Cell current, bool makeConnected)
        {
            int r = current.row;
            int c = current.col;
            // We create an empty list for the successors of the current cell.
            List<Cell> temp = new List<Cell>();
            // With diagonal movements priority is:
            // 1: Up 2: Up-right 3: Right 4: Down-right
            // 5: Down 6: Down-left 7: Left 8: Up-left

            // Without diagonal movements the priority is:
            // 1: Up 2: Right 3: Down 4: Left

            // If not at the topmost limit of the grid
            // and the up-side cell is not an obstacle ...
            if (r > 0 && grid[r - 1,c] != OBST &&
                    // ... and (only in the case are not running the A* or Greedy)
                    // not already belongs neither to the OPEN SET nor to the CLOSED SET ...
                    ((aStar.Checked || greedy.Checked || dijkstra.Checked) ? true :
                          IsInList(openSet, new Cell(r - 1, c)) == -1 &&
                          IsInList(closedSet, new Cell(r - 1, c)) == -1))
            {
                Cell cell = new Cell(r - 1, c);
                // In the case of Dijkstra's algorithm we can not append to
                // the list of successors the "naked" cell we have just created.
                // The cell must be accompanied by the label 'dist',
                // so we need to track it down through the list 'graph'
                // and then copy it back to the list of successors.
                // The flag makeConnected is necessary to be able
                // the present method createSuccesors() to collaborate
                // with the method findConnectedComponent(), which creates
                // the connected component when Dijkstra's initializes.
                if (dijkstra.Checked)
                {
                    if (makeConnected)
                        temp.Add(cell);
                    else
                    {
                        int graphIndex = IsInList(graph, cell);
                        if (graphIndex > -1)
                            temp.Add(graph[graphIndex]);
                    }
                }
                else
                {
                    // ... update the pointer of the up-side cell so it points the current one ...
                    cell.prev = current;
                    // ... and add the up-side cell to the successors of the current one. 
                    temp.Add(cell);
                }
            }
            if (diagonal.Checked)
            {
                // If we are not even at the topmost nor at the rightmost border of the grid
                // and the up-right-side cell is not an obstacle ...
                if (r > 0 && c < columns - 1 && grid[r - 1,c + 1] != OBST &&
                        // ... and one of the upper side or right side cells are not obstacles ...
                        // (because it is not reasonable to allow 
                        // the robot to pass through a "slot")                        
                        (grid[r - 1,c] != OBST || grid[r,c + 1] != OBST) &&
                        // ... and (only in the case are not running the A* or Greedy)
                        // not already belongs neither to the OPEN SET nor CLOSED SET ...
                        ((aStar.Checked || greedy.Checked || dijkstra.Checked) ? true :
                              IsInList(openSet, new Cell(r - 1, c + 1)) == -1 &&
                              IsInList(closedSet, new Cell(r - 1, c + 1)) == -1))
                {
                    Cell cell = new Cell(r - 1, c + 1);
                    if (dijkstra.Checked)
                    {
                        if (makeConnected)
                            temp.Add(cell);
                        else
                        {
                            int graphIndex = IsInList(graph, cell);
                            if (graphIndex > -1)
                                temp.Add(graph[graphIndex]);
                        }
                    }
                    else
                    {
                        // ... update the pointer of the up-right-side cell so it points the current one ...
                        cell.prev = current;
                        // ... and add the up-right-side cell to the successors of the current one. 
                        temp.Add(cell);
                    }
                }
            }
            // If not at the rightmost limit of the grid
            // and the right-side cell is not an obstacle ...
            if (c < columns - 1 && grid[r,c + 1] != OBST &&
                    // ... and (only in the case are not running the A* or Greedy)
                    // not already belongs neither to the OPEN SET nor to the CLOSED SET ...
                    ((aStar.Checked || greedy.Checked || dijkstra.Checked) ? true :
                          IsInList(openSet, new Cell(r, c + 1)) == -1 &&
                          IsInList(closedSet, new Cell(r, c + 1)) == -1))
            {
                Cell cell = new Cell(r, c + 1);
                if (dijkstra.Checked)
                {
                    if (makeConnected)
                        temp.Add(cell);
                    else
                    {
                        int graphIndex = IsInList(graph, cell);
                        if (graphIndex > -1)
                            temp.Add(graph[graphIndex]);
                    }
                }
                else
                {
                    // ... update the pointer of the right-side cell so it points the current one ...
                    cell.prev = current;
                    // ... and add the right-side cell to the successors of the current one. 
                    temp.Add(cell);
                }
            }
            if (diagonal.Checked)
            {
                // If we are not even at the lowermost nor at the rightmost border of the grid
                // and the down-right-side cell is not an obstacle ...
                if (r < rows - 1 && c < columns - 1 && grid[r + 1,c + 1] != OBST &&
                        // ... and one of the down-side or right-side cells are not obstacles ...
                        (grid[r + 1,c] != OBST || grid[r,c + 1] != OBST) &&
                        // ... and (only in the case are not running the A* or Greedy)
                        // not already belongs neither to the OPEN SET nor to the CLOSED SET ...
                        ((aStar.Checked || greedy.Checked || dijkstra.Checked) ? true :
                              IsInList(openSet, new Cell(r + 1, c + 1)) == -1 &&
                              IsInList(closedSet, new Cell(r + 1, c + 1)) == -1))
                {
                    Cell cell = new Cell(r + 1, c + 1);
                    if (dijkstra.Checked)
                    {
                        if (makeConnected)
                            temp.Add(cell);
                        else
                        {
                            int graphIndex = IsInList(graph, cell);
                            if (graphIndex > -1)
                                temp.Add(graph[graphIndex]);
                        }
                    }
                    else
                    {
                        // ... update the pointer of the downr-right-side cell so it points the current one ...
                        cell.prev = current;
                        // ... and add the down-right-side cell to the successors of the current one. 
                        temp.Add(cell);
                    }
                }
            }
            // If not at the lowermost limit of the grid
            // and the down-side cell is not an obstacle ...
            if (r < rows - 1 && grid[r + 1,c] != OBST &&
                    // ... and (only in the case are not running the A* or Greedy)
                    // not already belongs neither to the OPEN SET nor to the CLOSED SET ...
                    ((aStar.Checked || greedy.Checked || dijkstra.Checked) ? true :
                          IsInList(openSet, new Cell(r + 1, c)) == -1 &&
                          IsInList(closedSet, new Cell(r + 1, c)) == -1))
            {
                Cell cell = new Cell(r + 1, c);
                if (dijkstra.Checked)
                {
                    if (makeConnected)
                        temp.Add(cell);
                    else
                    {
                        int graphIndex = IsInList(graph, cell);
                        if (graphIndex > -1)
                            temp.Add(graph[graphIndex]);
                    }
                }
                else
                {
                    // ... update the pointer of the down-side cell so it points the current one ...
                    cell.prev = current;
                    // ... and add the down-side cell to the successors of the current one. 
                    temp.Add(cell);
                }
            }
            if (diagonal.Checked)
            {
                // If we are not even at the lowermost nor at the leftmost border of the grid
                // and the down-left-side cell is not an obstacle ...
                if (r < rows - 1 && c > 0 && grid[r + 1,c - 1] != OBST &&
                        // ... and one of the down-side or left-side cells are not obstacles ...
                        (grid[r + 1,c] != OBST || grid[r,c - 1] != OBST) &&
                        // ... and (only in the case are not running the A* or Greedy)
                        // not already belongs neither to the OPEN SET nor to the CLOSED SET ...
                        ((aStar.Checked || greedy.Checked || dijkstra.Checked) ? true :
                              IsInList(openSet, new Cell(r + 1, c - 1)) == -1 &&
                              IsInList(closedSet, new Cell(r + 1, c - 1)) == -1))
                {
                    Cell cell = new Cell(r + 1, c - 1);
                    if (dijkstra.Checked)
                    {
                        if (makeConnected)
                            temp.Add(cell);
                        else
                        {
                            int graphIndex = IsInList(graph, cell);
                            if (graphIndex > -1)
                                temp.Add(graph[graphIndex]);
                        }
                    }
                    else
                    {
                        // ... update the pointer of the down-left-side cell so it points the current one ...
                        cell.prev = current;
                        // ... and add the down-left-side cell to the successors of the current one. 
                        temp.Add(cell);
                    }
                }
            }
            // If not at the leftmost limit of the grid
            // and the left-side cell is not an obstacle ...
            if (c > 0 && grid[r,c - 1] != OBST &&
                    // ... and (only in the case are not running the A* or Greedy)
                    // not already belongs neither to the OPEN SET nor to the CLOSED SET ...
                    ((aStar.Checked || greedy.Checked || dijkstra.Checked) ? true :
                          IsInList(openSet, new Cell(r, c - 1)) == -1 &&
                          IsInList(closedSet, new Cell(r, c - 1)) == -1))
            {
                Cell cell = new Cell(r, c - 1);
                if (dijkstra.Checked)
                {
                    if (makeConnected)
                        temp.Add(cell);
                    else
                    {
                        int graphIndex = IsInList(graph, cell);
                        if (graphIndex > -1)
                            temp.Add(graph[graphIndex]);
                    }
                }
                else
                {
                    // ... update the pointer of the left-side cell so it points the current one ...
                    cell.prev = current;
                    // ... and add the left-side cell to the successors of the current one. 
                    temp.Add(cell);
                }
            }
            if (diagonal.Checked)
            {
                // If we are not even at the topmost nor at the leftmost border of the grid
                // and the up-left-side cell is not an obstacle ...
                if (r > 0 && c > 0 && grid[r - 1,c - 1] != OBST &&
                        // ... and one of the up-side or left-side cells are not obstacles ...
                        (grid[r - 1,c] != OBST || grid[r,c - 1] != OBST) &&
                        // ... and (only in the case are not running the A* or Greedy)
                        // not already belongs neither to the OPEN SET nor to the CLOSED SET ...
                        ((aStar.Checked || greedy.Checked || dijkstra.Checked) ? true :
                              IsInList(openSet, new Cell(r - 1, c - 1)) == -1 &&
                              IsInList(closedSet, new Cell(r - 1, c - 1)) == -1))
                {
                    Cell cell = new Cell(r - 1, c - 1);
                    if (dijkstra.Checked)
                    {
                        if (makeConnected)
                            temp.Add(cell);
                        else
                        {
                            int graphIndex = IsInList(graph, cell);
                            if (graphIndex > -1)
                                temp.Add(graph[graphIndex]);
                        }
                    }
                    else
                    {
                        // ... update the pointer of the up-left-side cell so it points the current one ...
                        cell.prev = current;
                        // ... and add the up-left-side cell to the successors of the current one. 
                        temp.Add(cell);
                    }
                }
            }
            // When DFS algorithm is in use, cells are added one by one at the beginning of the
            // OPEN SET list. Because of this, we must reverse the order of successors formed,
            // so the successor corresponding to the highest priority, to be placed
            // the first in the list.
            // For the Greedy, A* and Dijkstra's no issue, because the list is sorted
            // according to 'f' or 'dist' before extracting the first element of.
            if (dfs.Checked)
                temp.Reverse();

            return temp;
        } // end CreateSuccesors()

        /// <summary>
        /// Returns the distance between two cells
        /// </summary>
        /// <param name="u">The first cell</param>
        /// <param name="v">The other cell</param>
        /// <returns>The distance between the cells u and v</returns>
        private double DistBetween(Cell u, Cell v)
        {
            double dist;
            int dx = u.col - v.col;
            int dy = u.row - v.row;
            if (diagonal.Checked)
            {
                // with diagonal movements 
                // calculate the Euclidean distance
                dist = Math.Sqrt(dx * dx + dy * dy);
            }
            else
            {
                // without diagonal movements
                // calculate the Manhattan distance
                dist = Math.Abs(dx) + Math.Abs(dy);
            }
            return dist;
        } // end DistBetween()

        /// <summary>
        /// Returns the index of the cell 'current' in the list 'list'
        /// </summary>
        /// <param name="list">The list in which we seek</param>
        /// <param name="current">The cell we are looking for</param>
        /// <returns>The index of the cell in the list. If the cell is not found returns -1</returns>
        private int IsInList(List<Cell> list, Cell current)
        {
            int index = -1;
            for (int i = 0; i < list.Count; i++)
            {
                Cell listItem = (Cell)list[i];
                if (current.row == listItem.row && current.col == listItem.col)
                {
                    index = i;
                    break;
                }
            }
            return index;
        } // end IsInList()

        /// <summary>
        /// Returns the predecessor of cell 'current' in list 'list'
        /// </summary>
        /// <param name="list">The list in which we seek</param>
        /// <param name="current">The cell we are looking for</param>
        /// <returns>The predecessor of cell 'current'</returns>
        private Cell FindPrev(List<Cell> list, Cell current)
        {
            int index = IsInList(list, current);
            Cell cell = (Cell)list[index];
            return cell.prev;
        } // end FindPrev()

        /// <summary>
        /// Calculates the path from the target to the initial position of the robot, 
        /// counts the corresponding steps and measures the distance traveled.
        /// </summary>
        private void PlotRoute()
        {
            int steps = 0;
            double distance = 0;
            int index = IsInList(closedSet, targetPos);
            Cell cur = (Cell)closedSet[index];
            grid[cur.row, cur.col] = TARGET;
            do
            {
                steps++;
                if (diagonal.Checked)
                {
                    int dx = cur.col - cur.prev.col;
                    int dy = cur.row - cur.prev.row;
                    distance += Math.Sqrt(dx * dx + dy * dy);
                }
                else
                    distance++;
                cur = cur.prev;
                grid[cur.row, cur.col] = ROUTE;
            } while (!(cur.row == robotStart.row && cur.col == robotStart.col));
            grid[robotStart.row, robotStart.col] = ROBOT;
            String msg;
            msg = String.Format("Nodes expanded: {0}, Steps: {1}, Distance: {2:N3}",
                     expanded, steps, distance);
            message.Text = msg;

        } // end PlotRoute()

        /// <summary>
        /// Appends to the list containing the nodes of the graph only
        /// the cells belonging to the same connected component with node v.
        /// This is a Breadth First Search of the graph starting from node v.
        /// </summary>
        /// <param name="v">The starting node</param>
        private void FindConnectedComponent(Cell v)
        {
            Stack<Cell> stack = new Stack<Cell>();
            List<Cell> succesors;
            stack.Push(v);
            graph.Add(v);
            while (!(stack.Count == 0))
            {
                v = stack.Pop();
                succesors = CreateSuccesors(v, true);
                foreach (Cell c in succesors)
                    if (IsInList(graph, c) == -1)
                    {
                        stack.Push(c);
                        graph.Add(c);
                    }
            }
        } // end FindConnectedComponent()

        /// <summary>
        /// Initialization of Dijkstra's algorithm
        /// </summary>
        private void InitializeDijkstra()
        {
            /*
             * When one thinks of Wikipedia pseudocode, observe that the
             * algorithm is still looking for his target while there are still
             * nodes in the queue Q.
             * Only when we run out of queue and the target has not been found,
             * can answer that there is no solution .
             * As is known, the algorithm models the problem as a connected graph.
             * It is obvious that no solution exists only when the graph is not
             * connected and the target is in a different connected component
             * of this initial position of the robot.
             * To be thus possible negative response from the algorithm,
             * should search be made ONLY in the coherent component to which the
             * initial position of the robot belongs.
             */

            // First create the connected component
            // to which the initial position of the robot belongs.
            graph.Clear();
            FindConnectedComponent(robotStart);
            // Here is the initialization of Dijkstra's algorithm 
            // 2: for each vertex v in Graph;
            foreach (Cell v in graph)
            {
                // 3: dist[v] := infinity ;
                v.dist = INFINITY;
                // 5: previous[v] := undefined ;
                v.prev = null;
            }
            // 8: dist[source] := 0;
            graph[IsInList(graph, robotStart)].dist = 0;
            // 9: Q := the set of all nodes in Graph;
            // Instead of the variable Q we will use the list
            // 'graph' itself, which has already been initialised.            

            // Sorts the list of nodes with respect to 'dist'.
            graph.Sort((r1, r2) => r1.dist.CompareTo(r2.dist));
            // Initializes the list of closed nodes
            closedSet.Clear();
        } // end InitializeDijkstra

        /// <summary>
        /// Repaints the grid
        /// </summary>
        private void MainForm_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            Brush brush;
            Rectangle rect;
            brush = new SolidBrush(Color.DarkGray);
            // Fills the background color.
            rect = new Rectangle(10, 10, columns * squareSize + 1, rows * squareSize + 1);
            g.FillRectangle(brush, rect);
            brush.Dispose();
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < columns; c++)
                {
                    if (grid[r, c] == EMPTY)
                        brush = new SolidBrush(Color.White);
                    else if (grid[r, c] == ROBOT)
                        brush = new SolidBrush(Color.Red);
                    else if (grid[r, c] == TARGET)
                        brush = new SolidBrush(Color.Green);
                    else if (grid[r, c] == OBST)
                        brush = new SolidBrush(Color.Black);
                    else if (grid[r, c] == FRONTIER)
                        brush = new SolidBrush(Color.Blue);
                    else if (grid[r, c] == CLOSED)
                        brush = new SolidBrush(Color.Cyan);
                    else if (grid[r, c] == ROUTE)
                        brush = new SolidBrush(Color.Yellow);
                    rect = new Rectangle(11 + c * squareSize, 11 + r * squareSize, squareSize - 1, squareSize - 1);
                    g.FillRectangle(brush, rect);
                    brush.Dispose();
                }

            if (drawArrows.Checked)
            {
                // We draw all arrows from each open or closed state
                // to its predecessor.
                for (int r = 0; r < rows; r++)
                    for (int c = 0; c < columns; c++)
                        // If the current cell is the goal and the solution has been found,
                        // or belongs in the route to the target,
                        // or is an open state,
                        // or is a closed state but not the initial position of the robot
                        if ((grid[r, c] == TARGET && found) || grid[r, c] == ROUTE ||
                                grid[r, c] == FRONTIER || (grid[r, c] == CLOSED &&
                                !(r == robotStart.row && c == robotStart.col)))
                        {
                            // The tail of the arrow is the current cell, while
                            // the arrowhead is the predecessor cell.
                            Cell head;
                            if (grid[r, c] == FRONTIER)
                                if (dijkstra.Checked)
                                    head = FindPrev(graph, new Cell(r, c));
                                else
                                    head = FindPrev(openSet, new Cell(r, c));
                            else
                                head = FindPrev(closedSet, new Cell(r, c));

                            // The coordinates of the center of the current cell
                            int tailX = 11 + c * squareSize + squareSize / 2;
                            int tailY = 11 + r * squareSize + squareSize / 2;
                            // The coordinates of the center of the predecessor cell
                            int headX = 11 + head.col * squareSize + squareSize / 2;
                            int headY = 11 + head.row * squareSize + squareSize / 2;
                            int thickness = squareSize > 25 ? 2 : 1;

                            // If the current cell is the target
                            // or belongs to the path to the target ...
                            if (grid[r, c] == TARGET || grid[r, c] == ROUTE)
                            {
                                // ... draw a red arrow directing to the target.
                                DrawArrow(g, Color.Red, thickness, tailX, tailY, headX, headY);
                                // Else ...
                            }
                            else
                            {
                                // ... draw a black arrow to the predecessor cell.
                                DrawArrow(g, Color.Black, thickness, headX, headY, tailX, tailY);
                            }
                        }
            }

        } // end MainForm_Paint

        /// <summary>
        /// Draws an arrow of specified thickness and color from point (x2,y2) to point (x1,y1)
        /// </summary>
        /// <param name="g">The graphics object</param>
        /// <param name="color">The color of the arrow</param>
        /// <param name="thickness">The thickness of the arrow</param>
        /// <param name="x1">The x coordinate of point 1</param>
        /// <param name="y1">The y coordinate of point 1</param>
        /// <param name="x2">The x coordinate of point 2</param>
        /// <param name="y2">The y coordinate of point 2</param>
        private void DrawArrow(Graphics g, Color color, int thickness, int x1, int y1, int x2, int y2)
        {
            // We calculate the matrix of the affine transformation
            System.Drawing.Drawing2D.Matrix matrix = new System.Drawing.Drawing2D.Matrix();

            double dx = x2 - x1, dy = y2 - y1;
            float angle = (float)(Math.Atan2(dy, dx) * 180 / Math.PI);
            int len = (int)Math.Sqrt(dx * dx + dy * dy);
            matrix.Translate(x1, y1);  // move the tip of the arrow to point (x1,y1)
            matrix.Rotate(angle);      // rotate the arrow 'angle' degrees

            // We draw an horizontal arrow 'len' in length
            // that ends at the point (0,0) with two tips 'arrowSize' in length
            // which form 20 degrees angles with the axis of the arrow ...
            System.Drawing.Drawing2D.GraphicsPath myPath = new System.Drawing.Drawing2D.GraphicsPath();

            myPath.AddLine(0, 0, len, 0);
            myPath.AddLine(0, 0, (int)(arrowSize * Math.Sin(70 * Math.PI / 180)), (int)(arrowSize * Math.Cos(70 * Math.PI / 180)));
            myPath.AddLine(0, 0, (int)(arrowSize * Math.Sin(70 * Math.PI / 180)), -(int)(arrowSize * Math.Cos(70 * Math.PI / 180)));
            myPath.Transform(matrix);

            Pen myPen = new Pen(color, thickness);
            g.DrawPath(myPen, myPath);
            // ... and affine transformation handles the rest !!!!!!
        } // end DrawArrow

    } // end class MainForm

} // end namespace Maze52
