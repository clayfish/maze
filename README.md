# Maze
Originally developed by Nikos Kanargias. https://goo.gl/tRaLfe and https://youtu.be/0ol_PptA7rM

## v5.0.1
The main addition in v5.0.1 is a slightly different but much more efficient algorithm for the Real-Time search.

I also tried to make the code more readable.

## v5.0.0
The main addition in v5.0.0 is the ability to search in Real-Time mode.

After starting the search in this mode the user can add or remove obstacles, or move the target and/or the robot and get the solution almost immediately.

While this mode is in effect, the label or the corresponding button is displayed in red font.

The idea is to start a full-speed animation search every time the user alters the positions of obstacles, target or robot and to display only the solution (or lack of solution) of the problem, but not the intermediate steps.
