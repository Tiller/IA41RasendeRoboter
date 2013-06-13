/*


  D�finition de l'IA du joueur artificiel de Rasende Roboter


*/



% PROBLEMES : Si un support n'est pas bon, tous les points apr�s le point de support seront dans la closedList alors qu''on pourrait les atteindre par ailleur probablement
% Si plusieurs supports, les suivants ne prennent pas en compte la nouvelle position des anciens supports

:- module( decision, [
	init/1,
	move/2
] ).


init(_).


move([S1, S2, S3, S4, NumCible, BX, BY, GX, GY, YX, YY, RX, RY], Path2):-
	Robot is (NumCible - 1) / 4,
	floor(Robot, RobotId),
	
	nb_setval(scenario, [S1, S2, S3, S4]),
	nb_setval(support, 1),
	
	listeCibles([S1, S2, S3, S4], L),
	nth0(NumCible, L, Target),
	
	writeln(' '),
	time(a_star([[[BX, BY], [GX, GY], [YX, YY], [RX, RY]], RobotId, Target], Path, Helps, Supports)),
	
	changePath(Path, Path2),
	
	length(Path, LenPath),
	NbMoves is LenPath / 2,
	write('+ OK ('),
	write(NbMoves),
	writeln(' moves)'),
	write('Helped by: '),
	writeln(Helps),
	write('Support needed: '),
	beautifySupport(Supports, BSupports),
	writeln(BSupports),
	write('Path: '),
	writeln(Path),
	!.

move([S1, S2, S3, S4, NumCible, BX, BY, GX, GY, YX, YY, RX, RY], []):- 
	writeln('- Fail'), 
	write('Scenario: '),
	writeln([S1, S2, S3, S4]),
	write('Cible: '),
	writeln(NumCible),
	
	listeCibles([S1, S2, S3, S4], L),
	nth0(NumCible, L, Target),
	
	write('Cible coords: '),
	writeln(Target),
	write('RobotsPos: '),
	writeln([[BX, BY], [GX, GY], [YX, YY], [RX, RY]]),
	!.

beautifySupport([], []):- !.
beautifySupport([[Case|_]|R], [Case|R2]):- beautifySupport(R, R2).




listeCibles([0,0,0,0], [[7,5],[6,1],[9,10],[13,5],[6,13],[11,2],[5,4],[1,10],[14,13],[4,9],[9,1],[9,14],[1,3],[12,9],[2,14],[2,5],[10,7]]).
listeCibles([0,0,0,1], [[7,5],[6,1],[12,9],[13,5],[6,13],[11,2],[5,4],[1,10],[14,13],[4,9],[9,1],[9,12],[1,3],[11,14],[2,14],[2,5],[10,7]]).
listeCibles([0,0,1,0], [[7,5],[6,1],[9,10],[13,5],[3,9],[11,2],[5,4],[6,14],[14,13],[1,13],[9,1],[9,14],[1,3],[12,9],[5,11],[2,5],[10,7]]).
listeCibles([0,0,1,1], [[7,5],[6,1],[12,9],[13,5],[3,9],[11,2],[5,4],[6,14],[14,13],[1,13],[9,1],[9,12],[1,3],[11,14],[5,11],[2,5],[10,7]]).
listeCibles([0,1,0,0], [[7,5],[6,1],[9,10],[11,2],[6,13],[13,6],[5,4],[1,10],[14,13],[4,9],[10,7],[9,14],[1,3],[12,9],[2,14],[2,5],[14,1]]).
listeCibles([0,1,0,1], [[7,5],[6,1],[12,9],[11,2],[6,13],[13,6],[5,4],[1,10],[14,13],[4,9],[10,7],[9,12],[1,3],[11,14],[2,14],[2,5],[14,1]]).
listeCibles([0,1,1,0], [[7,5],[6,1],[9,10],[11,2],[3,9],[13,6],[5,4],[6,14],[14,13],[1,13],[10,7],[9,14],[1,3],[12,9],[5,11],[2,5],[14,1]]).
listeCibles([0,1,1,1], [[7,5],[6,1],[12,9],[11,2],[3,9],[13,6],[5,4],[6,14],[14,13],[1,13],[10,7],[9,12],[1,3],[11,14],[5,11],[2,5],[14,1]]).
listeCibles([1,0,0,0], [[3,7],[5,6],[9,10],[13,5],[6,13],[11,2],[1,3],[1,10],[14,13],[4,9],[9,1],[9,14],[6,4],[12,9],[2,14],[2,1],[10,7]]).
listeCibles([1,0,0,1], [[3,7],[5,6],[12,9],[13,5],[6,13],[11,2],[1,3],[1,10],[14,13],[4,9],[9,1],[9,12],[6,4],[11,14],[2,14],[2,1],[10,7]]).
listeCibles([1,0,1,0], [[3,7],[5,6],[9,10],[13,5],[3,9],[11,2],[1,3],[6,14],[14,13],[1,13],[9,1],[9,14],[6,4],[12,9],[5,11],[2,1],[10,7]]).
listeCibles([1,0,1,1], [[3,7],[5,6],[12,9],[13,5],[3,9],[11,2],[1,3],[6,14],[14,13],[1,13],[9,1],[9,12],[6,4],[11,14],[5,11],[2,1],[10,7]]).
listeCibles([1,1,0,0], [[3,7],[5,6],[9,10],[11,2],[6,13],[13,6],[1,3],[1,10],[14,13],[4,9],[10,7],[9,14],[6,4],[12,9],[2,14],[2,1],[14,1]]).
listeCibles([1,1,0,1], [[3,7],[5,6],[12,9],[11,2],[6,13],[13,6],[1,3],[1,10],[14,13],[4,9],[10,7],[9,12],[6,4],[11,14],[2,14],[2,1],[14,1]]).
listeCibles([1,1,1,0], [[3,7],[5,6],[9,10],[11,2],[3,9],[13,6],[1,3],[6,14],[14,13],[1,13],[10,7],[9,14],[6,4],[12,9],[5,11],[2,1],[14,1]]).
listeCibles([1,1,1,1], [[3,7],[5,6],[12,9],[11,2],[3,9],[13,6],[1,3],[6,14],[14,13],[1,13],[10,7],[9,12],[6,4],[11,14],[5,11],[2,1],[14,1]]).

% Cases having an obstacle to their right
listeObstaclesHorizontaux([0,0,0,0], [[3,0],[10,0],[9,1],[5,1],[11,2],[1,3],[2,5],[4,4],[7,5],[12,5],[12,9],[9,7],[6,7],[6,8],[8,7],[8,8],[8,10],[3,9],[1,10],[2,14],[3,15],[5,13],[8,14],[10,15],[14,13]]).
listeObstaclesHorizontaux([0,0,0,1], [[3,0],[5,1],[9,1],[10,0],[11,2],[1,3],[4,4],[12,5],[7,5],[2,5],[6,7],[6,8],[8,7],[8,8],[9,7],[12,9],[3,9],[1,10],[5,13],[2,14],[3,15],[8,12],[11,14],[13,13],[13,15]]).
listeObstaclesHorizontaux([0,0,1,0], [[3,0],[5,1],[9,1],[10,0],[11,2],[1,3],[4,4],[2,5],[12,5],[9,7],[8,7],[8,8],[6,7],[6,8],[3,9],[4,11],[0,13],[4,15],[6,14],[8,14],[10,15],[8,10],[12,9],[14,13],[7,5]]).
listeObstaclesHorizontaux([0,0,1,1], [[3,0],[10,0],[9,1],[5,1],[1,3],[4,4],[2,5],[7,5],[11,2],[12,5],[9,7],[8,7],[8,8],[6,7],[6,8],[12,9],[3,9],[4,11],[0,13],[4,15],[6,14],[8,12],[13,13],[13,15],[11,14]]).
listeObstaclesHorizontaux([0,1,0,0], [[3,0],[9,0],[13,1],[10,2],[5,1],[1,3],[4,4],[2,5],[7,5],[10,7],[13,6],[8,7],[8,8],[6,7],[6,8],[3,9],[1,10],[2,14],[3,15],[5,13],[8,14],[10,15],[8,10],[14,13],[12,9]]).
listeObstaclesHorizontaux([0,1,0,1], [[3,0],[9,0],[13,1],[10,2],[5,1],[1,3],[4,4],[2,5],[7,5],[6,7],[6,8],[8,7],[8,8],[10,7],[13,6],[12,9],[13,13],[13,15],[11,14],[8,12],[5,13],[3,15],[2,14],[1,10],[3,9]]).
listeObstaclesHorizontaux([0,1,1,0], [[3,0],[5,1],[9,0],[10,2],[13,1],[1,3],[2,5],[4,4],[7,5],[6,7],[6,8],[8,7],[8,8],[10,7],[13,6],[12,9],[8,10],[3,9],[4,11],[0,13],[6,14],[8,14],[14,13],[4,15],[10,15]]).
listeObstaclesHorizontaux([0,1,1,1], [[3,0],[5,1],[9,0],[10,2],[13,1],[13,6],[10,7],[12,9],[8,7],[8,8],[6,7],[6,8],[7,5],[4,4],[1,3],[2,5],[3,9],[4,11],[0,13],[6,14],[8,12],[11,14],[13,13],[13,15],[4,15]]).
listeObstaclesHorizontaux([1,0,0,0], [[4,0],[10,0],[11,2],[12,5],[5,4],[2,1],[0,3],[6,7],[6,8],[5,6],[8,7],[8,8],[3,7],[3,9],[1,10],[2,14],[3,15],[5,13],[10,15],[8,14],[14,13],[12,9],[8,10],[9,7],[9,1]]).
listeObstaclesHorizontaux([1,0,0,1], [[4,0],[2,1],[0,3],[5,4],[5,6],[3,7],[3,9],[1,10],[2,14],[3,15],[5,13],[8,12],[6,8],[6,7],[8,7],[8,8],[9,7],[9,1],[10,0],[11,2],[12,5],[12,9],[13,13],[13,15],[11,14]]).
listeObstaclesHorizontaux([1,0,1,0], [[4,0],[10,0],[9,1],[11,2],[12,5],[5,4],[2,1],[0,3],[3,7],[5,6],[6,7],[6,8],[8,7],[8,8],[9,7],[12,9],[8,10],[4,11],[3,9],[0,13],[4,15],[6,14],[8,14],[10,15],[14,13]]).
listeObstaclesHorizontaux([1,0,1,1], [[4,0],[10,0],[9,1],[11,2],[12,5],[5,4],[5,6],[0,3],[3,7],[3,9],[0,13],[4,11],[6,14],[8,12],[13,13],[13,15],[11,14],[12,9],[9,7],[8,7],[8,8],[6,7],[6,8],[4,15],[2,1]]).
listeObstaclesHorizontaux([1,1,0,0], [[4,0],[9,0],[10,2],[13,1],[13,6],[10,7],[8,7],[8,8],[6,7],[6,8],[5,6],[5,4],[0,3],[3,7],[3,9],[1,10],[2,14],[3,15],[5,13],[8,14],[10,15],[14,13],[12,9],[8,10],[2,1]]).
listeObstaclesHorizontaux([1,1,0,1], [[4,0],[9,0],[10,2],[13,1],[13,6],[10,7],[5,4],[0,3],[5,6],[3,7],[3,9],[1,10],[2,14],[3,15],[5,13],[8,12],[11,14],[13,15],[13,13],[12,9],[6,7],[6,8],[8,7],[8,8],[2,1]]).
listeObstaclesHorizontaux([1,1,1,0], [[4,0],[2,1],[0,3],[5,4],[5,6],[3,7],[3,9],[0,13],[4,15],[6,14],[4,11],[8,10],[8,14],[10,15],[14,13],[12,9],[10,7],[13,6],[13,1],[10,2],[9,0],[6,7],[6,8],[8,7],[8,8]]).
listeObstaclesHorizontaux([1,1,1,1], [[2,1],[4,0],[0,3],[5,4],[5,6],[9,0],[10,2],[13,1],[13,6],[10,7],[12,9],[13,13],[13,15],[11,14],[8,12],[6,14],[4,15],[4,11],[3,9],[0,13],[3,7],[6,7],[6,8],[8,7],[8,8]]).

% Cases having an obstacle to their bottom
listeObstaclesVerticaux([0,0,0,0], [[0,3],[0,6],[1,2],[6,1],[9,0],[11,2],[15,3],[13,5],[15,9],[10,6],[12,8],[7,5],[5,3],[2,5],[4,9],[9,10],[14,13],[9,13],[6,12],[2,14],[1,9],[7,6],[8,6],[7,8],[8,8]]).
listeObstaclesVerticaux([0,0,0,1], [[9,0],[15,1],[6,1],[11,2],[15,3],[5,3],[0,3],[1,2],[2,5],[7,5],[13,5],[10,6],[8,6],[7,6],[0,6],[7,8],[8,8],[12,8],[4,9],[1,9],[2,14],[6,12],[9,11],[11,14],[14,13]]).
listeObstaclesVerticaux([0,0,1,0], [[6,1],[9,0],[11,2],[15,3],[13,5],[7,5],[5,3],[1,2],[0,2],[2,5],[0,6],[7,6],[8,6],[7,8],[8,8],[9,10],[15,9],[12,8],[10,6],[14,13],[9,13],[6,13],[5,10],[3,9],[1,13]]).
listeObstaclesVerticaux([0,0,1,1], [[6,1],[9,0],[11,2],[15,1],[15,3],[5,3],[1,2],[0,2],[0,6],[2,5],[7,5],[13,5],[10,6],[8,6],[7,6],[7,8],[8,8],[12,8],[9,11],[5,10],[3,9],[14,13],[11,14],[6,13],[1,13]]).
listeObstaclesVerticaux([0,1,0,0], [[6,1],[11,2],[14,0],[15,3],[7,5],[5,3],[1,2],[0,3],[0,6],[2,5],[4,9],[1,9],[7,6],[8,6],[7,8],[8,8],[10,6],[12,8],[15,9],[13,6],[14,13],[9,10],[9,13],[6,12],[2,14]]).
listeObstaclesVerticaux([0,1,0,1], [[6,1],[14,0],[15,1],[15,3],[11,2],[13,6],[10,6],[8,6],[7,6],[7,8],[8,8],[7,5],[5,3],[2,5],[1,2],[0,3],[0,6],[4,9],[1,9],[2,14],[6,12],[9,11],[11,14],[14,13],[12,8]]).
listeObstaclesVerticaux([0,1,1,0], [[0,2],[1,2],[6,1],[11,2],[14,0],[15,3],[10,6],[13,6],[7,5],[5,3],[2,5],[0,6],[7,6],[8,6],[7,8],[8,8],[9,10],[3,9],[5,10],[1,13],[6,13],[9,13],[15,9],[12,8],[14,13]]).
listeObstaclesVerticaux([0,1,1,1], [[0,2],[1,2],[6,1],[5,3],[11,2],[15,1],[15,3],[14,0],[13,6],[10,6],[8,6],[7,6],[7,5],[7,8],[8,8],[12,8],[14,13],[11,14],[9,11],[5,10],[6,13],[1,13],[3,9],[0,6],[2,5]]).
listeObstaclesVerticaux([1,0,0,0], [[0,3],[1,3],[0,4],[2,1],[6,3],[9,0],[11,2],[15,3],[13,5],[12,8],[15,9],[10,6],[8,6],[7,6],[7,8],[8,8],[9,10],[4,9],[5,5],[3,7],[1,9],[2,14],[6,12],[9,13],[14,13]]).
listeObstaclesVerticaux([1,0,0,1], [[2,1],[0,3],[1,3],[0,4],[6,3],[9,0],[15,1],[15,3],[11,2],[13,5],[10,6],[12,8],[7,6],[8,6],[7,8],[8,8],[5,5],[4,9],[3,7],[1,9],[2,14],[6,12],[9,11],[14,13],[11,14]]).
listeObstaclesVerticaux([1,0,1,0], [[0,2],[1,3],[2,1],[6,3],[9,0],[11,2],[15,3],[13,5],[15,9],[12,8],[10,6],[8,6],[7,6],[7,8],[8,8],[5,5],[0,4],[3,7],[3,9],[5,10],[1,13],[6,13],[9,13],[9,10],[14,13]]).
listeObstaclesVerticaux([1,0,1,1], [[2,1],[0,2],[1,3],[0,4],[6,3],[5,5],[3,7],[3,9],[1,13],[5,10],[6,13],[9,11],[11,14],[14,13],[12,8],[10,6],[13,5],[15,3],[15,1],[11,2],[9,0],[8,6],[7,6],[7,8],[8,8]]).
listeObstaclesVerticaux([1,1,0,0], [[2,1],[0,3],[1,3],[0,4],[6,3],[5,5],[7,6],[8,6],[7,8],[8,8],[10,6],[13,6],[15,3],[14,0],[11,2],[15,9],[12,8],[9,10],[14,13],[9,13],[6,12],[4,9],[2,14],[1,9],[3,7]]).
listeObstaclesVerticaux([1,1,0,1], [[0,3],[1,3],[0,4],[2,1],[6,3],[11,2],[15,1],[15,3],[14,0],[13,6],[10,6],[12,8],[14,13],[11,14],[9,11],[6,12],[4,9],[1,9],[2,14],[3,7],[5,5],[7,6],[8,6],[7,8],[8,8]]).
listeObstaclesVerticaux([1,1,1,0], [[7,8],[8,8],[7,6],[8,6],[2,1],[0,2],[1,3],[0,4],[6,3],[11,2],[14,0],[15,3],[15,9],[12,8],[13,6],[10,6],[14,13],[9,13],[9,10],[6,13],[5,10],[1,13],[3,7],[3,9],[5,5]]).
listeObstaclesVerticaux([1,1,1,1], [[0,2],[1,3],[2,1],[6,3],[0,4],[3,7],[5,5],[3,9],[5,10],[1,13],[6,13],[9,11],[11,14],[14,13],[12,8],[10,6],[13,6],[15,3],[15,1],[14,0],[11,2],[7,6],[8,6],[7,8],[8,8]]).

obstacleHorizontal(Pos):- 
	nb_getval(scenario, Scenario),
	listeObstaclesHorizontaux(Scenario, L), member(Pos, L), !.

obstacleVertical(Pos):- 
	nb_getval(scenario, Scenario),
	listeObstaclesVerticaux(Scenario, L), member(Pos, L), !.

direction(1, right).
direction(3, left).
direction(2, up).
direction(4, down).

inverseDirection(H, [up, down]):- member(H, [right, left]), !.
inverseDirection(V, [left, right]):- member(V, [up, down]).

accessible1Robot(From, Dir):- prochaineCase(From, Dir, To), not(prochaineCase(To, Dir, _)), !.
accessible1Robot(From, Dir):- prochaineCase(From, Dir, To), inverseDirection(Dir, InvDir), member(InvDir1, InvDir), member(InvDir2, InvDir), InvDir1 \= InvDir2, prochaineCase(To, InvDir1, _), not(prochaineCase(To, InvDir2, _)), !.

prochaineCase([FromX, FromY], right, [ToX, FromY]):- FromX < 15, ToX is FromX + 1, not(obstacleHorizontal([FromX, FromY])), !.
prochaineCase([FromX, FromY], left, [ToX, FromY]):- FromX > 0, ToX is FromX - 1, not(obstacleHorizontal([ToX, FromY])), !.
prochaineCase([FromX, FromY], down, [FromX, ToY]):- FromY < 15, ToY is FromY + 1, not(obstacleVertical([FromX, FromY])), !.
prochaineCase([FromX, FromY], up, [FromX, ToY]):- FromY > 0, ToY is FromY - 1, not(obstacleVertical([FromX, ToY])), !.

prochaineCase(RobotsPos, From, Dir, To):- prochaineCase(From, Dir, To), not(member(To, RobotsPos)), !.

destination(RobotsPos, From, Dir, From):- not(prochaineCase(RobotsPos, From, Dir, _)), !.
destination(RobotsPos, From, Dir, To):- prochaineCase(RobotsPos, From, Dir, Temp), destination(RobotsPos, Temp, Dir, To), !.

destinations(RobotsPos, From, Dir, Tos):- bagof(To, destination(RobotsPos, From, Dir, To), Tos).

destinationWithHelp(RobotsPos, From, Dir, To, Support):- prochaineCase(RobotsPos, From, Dir, To), accessible1Robot(To, Dir), prochaineCase(RobotsPos, To, Dir, Support).
destinationWithHelp(RobotsPos, From, Dir, To, Support):- prochaineCase(RobotsPos, From, Dir, Tmp), destinationWithHelp(RobotsPos, Tmp, Dir, To, Support).














changePath([], []):- !.
changePath([Robot, Dir | R], [Robot, NewDir | R2]):- changePath(R, R2), direction(NewDir, Dir).



% State = [[BPos, GPos, YPos, RPos], Robot, Target]

a_star(InitialState, Path, Helps, Supports):-
	nb_setval(closedList, []),
	nb_setval(openList, [[InitialState, [], [], [], 0, 0]]),
	
	buildPath(Path, Helps, Supports).



distance_manhattan([RobotsPos, Robot, Target], H) :-
	nth0(Robot, RobotsPos, Pos),
	distance_manhattan_simple(Pos, Target, H).
	
	
	
distance_manhattan_simple([RobotX, RobotY], [TargetX, TargetY], H) :-
	DX is RobotX - TargetX,
	DY is RobotY - TargetY,
	abs(DX, AX),
	abs(DY, AY),
	H is AX + AY.

	
testeuh(V):-
	nb_setval(scenario, [0, 0, 0, 0]),
	nb_setval(support, V),
	time(a_star([[[10, 6], [5, 4], [9, 1], [2, 5]], 0, [6,13]], Path, _, _)),
	!.

buildPath(FinalPath, Helps, Supports):-
	getBestNodeFromOpenList([State, Path, Helps, Supports, _, _]),
	[RobotsPos, Robot, Target] = State,
	member(Target, RobotsPos),
	
	tryThenFinally(
		(
			theEnd(Robot, Path, Helps, Supports, FinalPath)
		),
		(
			deleteBestNodeFromOpenList
		)
	),
	!.



theEnd(_, Path, _, [], Path):- !.




theEnd(Robot, Path, Helps, Supports, FinalPath):-
	append(Helps, [Robot], BannedRobots),
	needSupport(Path, Supports, BannedRobots, FinalPath),
	!.




buildPath(FinalPath, FinalHelps, FinalSupports):-
	extractBestNodeFromOpenList([State, Path, Helps, Supports, G, _]),
	getAllAccessibleStates(State, AccessibleStatesList), 
	insertAllStatesInOpenList(AccessibleStatesList, Path, Helps, Supports, G), 
	buildPath(FinalPath, FinalHelps, FinalSupports),
	!.


needSupport([], _, _, []):- !.

needSupport([PRobot, PDir, support|R], [[Target,RobotsPos]|Supports], BannedRobots, FinalPath):-
	!,
	length(RobotsPos, NbRobot),
	Nb1 is NbRobot - 1,
	between(0, Nb1, RobotId),
	not(member(RobotId, BannedRobots)),
	
	% Probleme si plusieurs supports : Il faut m�j la pos des robots ayant fait supports avant
	
	nb_getval(openList, OpenList),
	nb_getval(closedList, ClosedList),
	nb_setval(support, 0),
	
	tryThenFinally(
		(
			a_star([RobotsPos, RobotId, Target], Path, _, _)
		),
		(
			nb_setval(openList, OpenList),
			nb_setval(closedList, ClosedList),
			nb_setval(support, 1)
		)
	),
	
	
	append(BannedRobots, [RobotId], BannedRobots2),
	
	needSupport([PRobot, PDir|R], Supports, BannedRobots2, OPath),
	
	append(Path, OPath, FinalPath).

	
needSupport([Robot, Dir|R], Supports, BannedRobots, [Robot,Dir|R2]):-
	needSupport(R, Supports, BannedRobots, R2),
	!.

 

	


tryThenFinally(Goal, Finally) :- Goal, ignore(Finally), !.
tryThenFinally(_, Finally) :- ignore(Finally), fail, !.

insertAllStatesInOpenList([], _, _, _, _):- !.

insertAllStatesInOpenList([Element|R], Path, Helps, Supports, G) :- 
	insertStateInOpenList(Element, Path, Helps, Supports, G),
	!,
	insertAllStatesInOpenList(R, Path, Helps, Supports, G).

insertAllStatesInOpenList([_|R], Path, Helps, Supports, G) :-
	insertAllStatesInOpenList(R, Path, Helps, Supports, G).

	
	
insertStateInOpenList([State, [Robot, Dir, Support]], Path, Helps, Supports, G) :-
	!,
	G1 is G + 15,
	insertSIOL([State, [Robot, Dir, Support]], Path, Helps, Supports, G1).


insertStateInOpenList(Node, Path, Helps, Supports, G) :-
	insertSIOL(Node, Path, Helps, Supports, G).

	
	
	
insertSIOL([State, Move], Path, Helps, Supports, G) :-
	[RobotsPos, Robot, Target] = State,
	[RId, Dir|_] = Move,
	
	nb_getval(closedList, ClosedList),
	not(member(State, ClosedList)),
	
	nb_getval(openList, OpenList),
	
	nth0(Robot, RobotsPos, RPos),
	updateHelps(Helps, RobotsPos, RPos, Dir, NewHelps),
	
	updateSupports(Supports, Move, NewSupports),
	length(NewSupports, NbSupports),
	NbSupports =< 2,
	
	distance_manhattan(State, H),
	G1 is G + 1,
	appendMove(Path, Move, NextPath),
	F is G1 + H,
	
	insertIn(OpenList, State, NextPath, NewHelps, NewSupports, G1, H, F, NewOpenList),
	nb_setval(openList, NewOpenList).



appendMove(Path, [Robot, Dir, _], NextPath):-
	append(Path, [Robot, Dir, support], NextPath),
	!.

appendMove(Path, Move, NextPath):-
	append(Path, Move, NextPath),
	!.
	
updateHelps(Helps, RobotsPos, To, Dir, [Robot|Helps]):-
	prochaineCase(To, Dir, RPos),
	nth0(Robot, RobotsPos, RPos),
	!.

updateHelps(Helps, _, _, _, Helps).



updateSupports(Supports, [_, _, Support], NewSupports):- 
	append(Supports, [Support], NewSupports),
	!.

updateSupports(Supports, _, Supports).






insertIn([], State, NextPath, Helps, Supports, G, H, _, [[State, NextPath, Helps, Supports, G, H]]).

insertIn([[OState, OPath, OHelps, OSupports, OG, OH]|R], State, NextPath, Helps, Supports, G, H, F, [[State, NextPath, Helps, Supports, G, H],[OState, OPath, OHelps, OSupports, OG, OH]|R]):-
	OF is OG + OH,
	OF > F,
	!.

insertIn([E|R], State, NextPath, Helps, Supports, G, H, F, [E|R2]):-
	insertIn(R, State, NextPath, Helps, Supports, G, H, F, R2).

	
	
	
	
	
getBestNodeFromOpenList(Node):-
	nb_getval(openList, [Node|_]).
	

	
extractBestNodeFromOpenList(Node):-
	nb_getval(openList, [Node|List]),
	nb_setval(openList, List),
	
	[State|_] = Node,
	
	nb_getval(closedList, ClosedList),
	nb_setval(closedList, [State|ClosedList]).


	
deleteBestNodeFromOpenList:-
	nb_getval(openList, [_|List]),
	nb_setval(openList, List).
	
	

	
getAccessibleState([RobotsPos, -1, Target], [[RobotsPos2, Robot, Target], [Robot, Dir|R]]):-
	!,
	length(RobotsPos, LenRobots),
	LenRobotsMinusOne is LenRobots - 1, 
	between(0, LenRobotsMinusOne, Robot),
	
	getAccessibleState([RobotsPos, Robot, Target], [[RobotsPos2, Robot, Target], [Robot, Dir|R]]).
	
	
getAccessibleState([RobotsPos, Robot, Target], [[RobotsPos2, Robot, Target], [Robot, Dir]]):-
	nth0(Robot, RobotsPos, From),
	direction(_, Dir),
	
	destination(RobotsPos, From, Dir, To),
	To \= From,
	
	setRobotPos(RobotsPos, Robot, To, RobotsPos2).
	
	
getAccessibleState([RobotsPos, Robot, Target], [[RobotsPos2, Robot, Target], [Robot, Dir, [Support,RobotsPos]]]):-
	nb_getval(support, 1),
	
	nth0(Robot, RobotsPos, From),
	direction(_, Dir),
	
	destinationWithHelp(RobotsPos, From, Dir, To, Support),
	To \= From,
	
	setRobotPos(RobotsPos, Robot, To, RobotsPos2).


setRobotPos([], _, _, []).
setRobotPos([_|R], 0, Pos, [Pos|L]):- !, setRobotPos(R, -1, [], L).
setRobotPos([T|R], N, Pos, [T|L]):- N1 is N - 1, setRobotPos(R, N1, Pos, L).



getAllAccessibleStates(State, AccessibleStatesList):- 
	bagof(NextState, getAccessibleState(State, NextState), AccessibleStatesList).
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	