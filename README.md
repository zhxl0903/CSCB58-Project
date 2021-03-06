# CSCB58-Project: RoadCrosser 2037

Roadcrosser 2037 is a 2D pixel based action game implemented using Verilog. 

## Basic Controls
Player is controlled using KEY[3:0]. KEY[3] is up; KEY[2] is down; KEY[1] is left; and KEY[0] is 
right. SW[9] is an active low game reset. 

## Starting a game
Before game starts, player is allowed to set number of lives between 1 and 15 (Setting life to 0
will set life to 1 instead.). This is followed by number of cars of type 1, type 2, and type 3 
respectively. Type 1 cars are the fastest moving cars and type 3 cars are the slowest moving cars.
Number of cars of each type can be set between 0 and 15. Numbers are set using SW[3:0]. After each 
setting, player needs to press and release KEY[0] to proceed to setting next number. LEDR light 
status indicators have been added as a bonus feature to aid player in setting each game option.

## Gameplay
This game starts after setting lives, number of cars of type 1, number of cars of type 2, and number
of cars of type 3. During this game, the goal of player is to move to the top of the screen while 
maintaining positive life points. Each collision with a car will decrease the player's life by 1. 
The game ends either when the player reaches the top of the screen or loses all life points. Moving
left or right to either end of the screen will make the player appear at the other end. This looped
motion is supported as long as player is above the VGA on screen HEX display panels.

## Score and Lives Display
Number of lives of player is displayed on HEX2. Score of player is displayed on HEX0 and HEX1. Score
is based on current distance of player away from bottom of the screen. This distance is between 0h 
and 77h. Once game ends, player is allowed to start a new game by setting the game settings mentioned
above again. Score from previous game is not cleared until next game begins. Score and lives are also 
displayed on screen at bottom right corner using VGA HEX display panels.

## Source Code
See RoadCrosser.v for this game's source code.


