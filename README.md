# CSCB58-Project: RoadCrosser 2037

Roadcrosser 2037 is a 2D pixel based action game
implemented using Verilog. The player is controlled
using KEY[3:0]. KEY[3] is up. KEY[2] is down. KEY[1]
is left. KEY[0] is right. SW[9] is an active low game 
reset. Before the game starts, the player is allowed to 
set the number of lives between 1 and 15 (Setting life to 0 will
set life to 1 instead.). This is followed by the number
of cars of type 1, type 2, and type 3 respectively. 
Type 1 cars are the fastest moving cars and type 3 cars
are the slowest moving cars. The number of cars of each 
type can be set between 0 and 15. The numbers are set
using SW[3:0]. After each setting, the player needs
to press and release KEY[0] to proceed to setting the
next number. The game starts after setting lives, number
of cars of type 1, number of cars of type 2, and number
of cars of type 3. During the game, the goal of the player
is to move to the top of the screen while maintaining 
positive life points. Each collision with a car will
decrease the player's life by 1. The game ends either
when the player reaches the top of the screen or loses
all life points. The number of lives of the player
is displayed on HEX2. The score of the player is displayed
on HEX0 and HEX1. The score is based on the current
distance of the player away from the bottom of the screen.
This distance is between 0 and 119. Once the game ends,
the player is allowed to start a new game by setting the
game settings mentioned above again. The score from the
previous game is not cleared until the next game begins.


