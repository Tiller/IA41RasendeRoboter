/*
  Définition de l'IA du joueur artificiel de Rasende Roboter
*/
:- module( decision, [
	init/1,
	move/2
] ).


init(_):-
	nb_setval(supportCost, 30).


move([S1, S2, S3, S4, NumCible, BX, BY, GX, GY, YX, YY, RX, RY], Path2):-
	Robot is (NumCible - 1) / 4,
	floor(Robot, RobotId),

	nb_setval(scenario, [S1, S2, S3, S4]),
	nb_setval(forbiddenRobots, []),
	nb_setval(support, 1),

	targets([S1, S2, S3, S4], L),
	nth0(NumCible, L, Target),

	writeln(' '),
	time(
		a_star([[[BX, BY], [GX, GY], [YX, YY], [RX, RY]], RobotId, Target], Path, _)
	),

	translatePath(Path, Path2),

	length(Path, LenPath),
	NbMoves is LenPath / 2,
	write('+ OK ('),
	write(NbMoves),
	writeln(' moves)'),

	!.


move([S1, S2, S3, S4, NumCible, BX, BY, GX, GY, YX, YY, RX, RY], []):-
	targets([S1, S2, S3, S4], L),
	nth0(NumCible, L, Target),

	writeln('- Fail'),
	write('-   Scenario: '),
	writeln([S1, S2, S3, S4]),
	write('-   Cible: '),
	write(NumCible),
	write(' = '),
	writeln(Target),
	write('-   RobotsPos: '),
	writeln([[BX, BY], [GX, GY], [YX, YY], [RX, RY]]),

	!.



a_star(InitialState, Path, RobotsPos):-
	nb_setval(closedList, []),
	nb_setval(openList, [[InitialState, [], 0, 0, []]]),
	buildPath(Path, RobotsPos).






buildPath(NewPath, RobotsPos2):-
	getBestNodeFromOpenList([State, Path, G, _, Support]),
	[RobotsPos, _, Target] = State,
	member(Target, RobotsPos),

	ifFail(
		(
			searchForSupport(State, Path, G, Support, NewState, NewPath, _)
		),
		(
			deleteBestNodeFromOpenList
		)
	),
	[RobotsPos2, _, _] = NewState,

	!.



buildPath(FinalPath, RobotsPos):-
	extractBestNodeFromOpenList([State, Path, G, _, Support]),

	ignore((
		searchForSupport(State, Path, G, Support, NewState, NewPath, NewG),

		getAllAccessibleStates(NewState, AccessibleStatesList),
		insertAllStatesInOpenList(AccessibleStatesList, NewPath, NewG)
	)),

	buildPath(FinalPath, RobotsPos),
	!.






searchForSupport(State, Path, G, [], State, Path, G):- !.

searchForSupport([FakeRobotsPos, Robot, Target], Path, G, [PrevRobotsPos, Support], NewState, NewPath, NewG):-
	supportPath([PrevRobotsPos, Robot, Support], SupportPath, RobotsPos),
	!,

	nth0(Robot, FakeRobotsPos, RPos),
	setRobotPos(RobotsPos, Robot, RPos, RobotsPos2),

	NewState = [RobotsPos2, Robot, Target],

	insertSupportInPath(Path, SupportPath, NewPath),

	length(SupportPath, LengthPath),
	nb_getval(supportCost, SupportCost),
	NewG is G - SupportCost + LengthPath / 2.


supportPath([RobotsPos, Robot, Support], SupportPath, FinalRobotsPos):-
	nb_getval(openList, OpenList),
	nb_getval(closedList, ClosedList),
	nb_getval(forbiddenRobots, ForbiddenRobots),

	nb_setval(forbiddenRobots, [Robot|ForbiddenRobots]),
	nb_setval(support, 0),


	tryAndDo(
		(
			a_star([RobotsPos, -1, Support], SupportPath, FinalRobotsPos)
		),
		(
			nb_setval(openList, OpenList),
			nb_setval(closedList, ClosedList),
			nb_setval(forbiddenRobots, ForbiddenRobots),
			nb_setval(support, 1)
		)
	).










getAccessibleState([RobotsPos, -1, Target], [[RobotsPos2, Robot, Target], [Robot, Dir, Support]]):-
	!,
	nb_getval(forbiddenRobots, ForbiddenRobots),

	length(RobotsPos, LenRobots),
	LenRobotsMinusOne is LenRobots - 1,
	between(0, LenRobotsMinusOne, Robot),

	not(member(Robot, ForbiddenRobots)),

	getAccessibleState([RobotsPos, Robot, Target], [[RobotsPos2, Robot, Target], [Robot, Dir, Support]]).



getAccessibleState([RobotsPos, Robot, Target], [[RobotsPos2, Robot, Target], [Robot, Dir, []]]):-
	nth0(Robot, RobotsPos, From),
	direction(_, Dir),

	destination(RobotsPos, From, Dir, To),
	To \= From,

	setRobotPos(RobotsPos, Robot, To, RobotsPos2).



getAccessibleState([RobotsPos, Robot, Target], [[RobotsPos2, Robot, Target], [Robot, Dir, [RobotsPos, Support]]]):-
	nb_getval(support, 1),

	nth0(Robot, RobotsPos, From),
	direction(_, Dir),

	destinationWithSupport(RobotsPos, From, Dir, To, Support),
	To \= From,


	setRobotPos(RobotsPos, Robot, To, RobotsPos2).



getAllAccessibleStates(State, AccessibleStatesList):-
	bagof(NextState, getAccessibleState(State, NextState), AccessibleStatesList).










insertAllStatesInOpenList([], _, _):- !.

insertAllStatesInOpenList([Element|R], Path, G) :-
	preInsertStateInOpenList(Element, Path, G),
	!,
	insertAllStatesInOpenList(R, Path, G).

insertAllStatesInOpenList([_|R], Path, G) :-
	insertAllStatesInOpenList(R, Path, G).


preInsertStateInOpenList([State, [Robot, Dir, []]], Path, G):-
	!,
	nb_getval(closedList, ClosedList),
	not(member(State, ClosedList)),

	insertStateInOpenList(State, [Robot, Dir], Path, G, []).



preInsertStateInOpenList([State, [Robot, Dir, Support]], Path, G):-
	nb_getval(closedList, ClosedList),
	not(member(State, ClosedList)),

	nb_getval(supportCost, SupportCost),
	G2 is G + SupportCost,

	insertStateInOpenList(State, [Robot, Dir], Path, G2, Support).


insertStateInOpenList(State, Move, Path, G, Support) :-
	manhattanDistance(State, H),
	G1 is G + 1,
	append(Path, Move, NextPath),
	F is G1 + H,

	nb_getval(openList, OpenList),
	insertInOpenList(OpenList, State, NextPath, G1, H, F, Support, NewOpenList),
	nb_setval(openList, NewOpenList).




insertInOpenList([], State, NextPath, G, H, _, Support, [[State, NextPath, G, H, Support]]).
insertInOpenList([[OState, OPath, OG, OH, OSupport]|R], State, NextPath, G, H, F, Support, [[State, NextPath, G, H, Support],[OState, OPath, OG, OH, OSupport]|R]):-
	OF is OG + OH,
	OF > F,
	!.

insertInOpenList([E|R], State, NextPath, G, H, F, Support, [E|R2]):-
	insertInOpenList(R, State, NextPath, G, H, F, Support, R2).







getBestNodeFromOpenList(Node):-
	nb_getval(openList, [Node|_]).


deleteBestNodeFromOpenList:-
	nb_getval(openList, [_|List]),
	nb_delete(openList),
	nb_setval(openList, List).


extractBestNodeFromOpenList(Node):-
	nb_getval(openList, [Node|List]),
	nb_delete(openList),
	nb_setval(openList, List),

	[State|_] = Node,
	nb_getval(closedList, ClosedList),
	nb_setval(closedList, [State|ClosedList]).






setRobotPos([], _, _, []).

setRobotPos([_|R], 0, Pos, [Pos|L]):-
	!,
	setRobotPos(R, -1, [], L).

setRobotPos([T|R], N, Pos, [T|L]):-
	N1 is N - 1,
	setRobotPos(R, N1, Pos, L).


translatePath([], []):- !.
translatePath([Robot, Dir | R], [Robot, NewDir | R2]):-
	translatePath(R, R2),
	direction(NewDir, Dir).

insertSupportInPath([R,D], Path, NewPath):-
	append(Path, [R, D], NewPath),
	!.

insertSupportInPath([R,D|P], Path, [R,D|NewPath]):-
	insertSupportInPath(P, Path, NewPath),
	!.


manhattanDistance([RobotsPos, Robot, Target], H) :-
	nth0(Robot, RobotsPos, Pos),
	manhattanDistanceSimple(Pos, Target, H).




manhattanDistanceSimple([RobotX, RobotY], [TargetX, TargetY], H) :-
	DX is RobotX - TargetX,
	DY is RobotY - TargetY,
	abs(DX, AX),
	abs(DY, AY),
	H is AX + AY.


tryAndDo(Goal, Finally) :-
	Goal,
	ignore(Finally),
	!.

tryAndDo(_, Finally) :-
	ignore(Finally),
	fail.

ifFail(Cond, _):-
	Cond,
	!.

ifFail(_, Todo):-
	Todo,
	fail.



direction(1, right).
direction(3, left).
direction(2, up).
direction(4, down).

reverseDirection(H, [up, down]):- member(H, [right, left]), !.
reverseDirection(V, [left, right]):- member(V, [up, down]).



horizontalObstacle(Pos):-
	nb_getval(scenario, Scenario),
	horizontalObstacles(Scenario, L), member(Pos, L), !.

verticalObstacle(Pos):-
	nb_getval(scenario, Scenario),
	verticalObstacles(Scenario, L), member(Pos, L), !.


accessibleWithSupport(From, Dir):-
	nextSquare(From, Dir, To),
	not(nextSquare(To, Dir, _)),
	!.

accessibleWithSupport(From, Dir):-
	nextSquare(From, Dir, To),
	reverseDirection(Dir, InvDir),

	member(InvDir1, InvDir),
	member(InvDir2, InvDir),
	InvDir1 \= InvDir2,

	nextSquare(To, InvDir1, _),
	not(nextSquare(To, InvDir2, _)),
	!.


nextSquare([FromX, FromY], right, [ToX, FromY]):-
	FromX < 15,
	ToX is FromX + 1,
	not(horizontalObstacle([FromX, FromY])),
	!.

nextSquare([FromX, FromY], left, [ToX, FromY]):-
	FromX > 0,
	ToX is FromX - 1,
	not(horizontalObstacle([ToX, FromY])),
	!.

nextSquare([FromX, FromY], down, [FromX, ToY]):-
	FromY < 15,
	ToY is FromY + 1,
	not(verticalObstacle([FromX, FromY])),
	!.

nextSquare([FromX, FromY], up, [FromX, ToY]):-
	FromY > 0,
	ToY is FromY - 1,
	not(verticalObstacle([FromX, ToY])),
	!.

nextSquare(RobotsPos, From, Dir, To):-
	nextSquare(From, Dir, To),
	not(member(To, RobotsPos)),
	!.

destination(RobotsPos, From, Dir, From):-
	not(nextSquare(RobotsPos, From, Dir, _)),
	!.

destination(RobotsPos, From, Dir, To):-
	nextSquare(RobotsPos, From, Dir, Temp),
	destination(RobotsPos, Temp, Dir, To),
	!.

destinations(RobotsPos, From, Dir, Tos):-
	bagof(To, destination(RobotsPos, From, Dir, To), Tos).

destinationWithSupport(RobotsPos, From, Dir, To, Support):-
	nextSquare(RobotsPos, From, Dir, To),
	accessibleWithSupport(To, Dir),
	nextSquare(RobotsPos, To, Dir, Support).

destinationWithSupport(RobotsPos, From, Dir, To, Support):-
	nextSquare(RobotsPos, From, Dir, Tmp),
	destinationWithSupport(RobotsPos, Tmp, Dir, To, Support).



targets([0,0,0,0], [[7,5],[6,1],[9,10],[13,5],[6,13],[11,2],[5,4],[1,10],[14,13],[4,9],[9,1],[9,14],[1,3],[12,9],[2,14],[2,5],[10,7]]).
targets([0,0,0,1], [[7,5],[6,1],[12,9],[13,5],[6,13],[11,2],[5,4],[1,10],[14,13],[4,9],[9,1],[9,12],[1,3],[11,14],[2,14],[2,5],[10,7]]).
targets([0,0,1,0], [[7,5],[6,1],[9,10],[13,5],[3,9],[11,2],[5,4],[6,14],[14,13],[1,13],[9,1],[9,14],[1,3],[12,9],[5,11],[2,5],[10,7]]).
targets([0,0,1,1], [[7,5],[6,1],[12,9],[13,5],[3,9],[11,2],[5,4],[6,14],[14,13],[1,13],[9,1],[9,12],[1,3],[11,14],[5,11],[2,5],[10,7]]).
targets([0,1,0,0], [[7,5],[6,1],[9,10],[11,2],[6,13],[13,6],[5,4],[1,10],[14,13],[4,9],[10,7],[9,14],[1,3],[12,9],[2,14],[2,5],[14,1]]).
targets([0,1,0,1], [[7,5],[6,1],[12,9],[11,2],[6,13],[13,6],[5,4],[1,10],[14,13],[4,9],[10,7],[9,12],[1,3],[11,14],[2,14],[2,5],[14,1]]).
targets([0,1,1,0], [[7,5],[6,1],[9,10],[11,2],[3,9],[13,6],[5,4],[6,14],[14,13],[1,13],[10,7],[9,14],[1,3],[12,9],[5,11],[2,5],[14,1]]).
targets([0,1,1,1], [[7,5],[6,1],[12,9],[11,2],[3,9],[13,6],[5,4],[6,14],[14,13],[1,13],[10,7],[9,12],[1,3],[11,14],[5,11],[2,5],[14,1]]).
targets([1,0,0,0], [[3,7],[5,6],[9,10],[13,5],[6,13],[11,2],[1,3],[1,10],[14,13],[4,9],[9,1],[9,14],[6,4],[12,9],[2,14],[2,1],[10,7]]).
targets([1,0,0,1], [[3,7],[5,6],[12,9],[13,5],[6,13],[11,2],[1,3],[1,10],[14,13],[4,9],[9,1],[9,12],[6,4],[11,14],[2,14],[2,1],[10,7]]).
targets([1,0,1,0], [[3,7],[5,6],[9,10],[13,5],[3,9],[11,2],[1,3],[6,14],[14,13],[1,13],[9,1],[9,14],[6,4],[12,9],[5,11],[2,1],[10,7]]).
targets([1,0,1,1], [[3,7],[5,6],[12,9],[13,5],[3,9],[11,2],[1,3],[6,14],[14,13],[1,13],[9,1],[9,12],[6,4],[11,14],[5,11],[2,1],[10,7]]).
targets([1,1,0,0], [[3,7],[5,6],[9,10],[11,2],[6,13],[13,6],[1,3],[1,10],[14,13],[4,9],[10,7],[9,14],[6,4],[12,9],[2,14],[2,1],[14,1]]).
targets([1,1,0,1], [[3,7],[5,6],[12,9],[11,2],[6,13],[13,6],[1,3],[1,10],[14,13],[4,9],[10,7],[9,12],[6,4],[11,14],[2,14],[2,1],[14,1]]).
targets([1,1,1,0], [[3,7],[5,6],[9,10],[11,2],[3,9],[13,6],[1,3],[6,14],[14,13],[1,13],[10,7],[9,14],[6,4],[12,9],[5,11],[2,1],[14,1]]).
targets([1,1,1,1], [[3,7],[5,6],[12,9],[11,2],[3,9],[13,6],[1,3],[6,14],[14,13],[1,13],[10,7],[9,12],[6,4],[11,14],[5,11],[2,1],[14,1]]).

% Cases having an obstacle to their right
horizontalObstacles([0,0,0,0], [[3,0],[10,0],[9,1],[5,1],[11,2],[1,3],[2,5],[4,4],[7,5],[12,5],[12,9],[9,7],[6,7],[6,8],[8,7],[8,8],[8,10],[3,9],[1,10],[2,14],[3,15],[5,13],[8,14],[10,15],[14,13]]).
horizontalObstacles([0,0,0,1], [[3,0],[5,1],[9,1],[10,0],[11,2],[1,3],[4,4],[12,5],[7,5],[2,5],[6,7],[6,8],[8,7],[8,8],[9,7],[12,9],[3,9],[1,10],[5,13],[2,14],[3,15],[8,12],[11,14],[13,13],[13,15]]).
horizontalObstacles([0,0,1,0], [[3,0],[5,1],[9,1],[10,0],[11,2],[1,3],[4,4],[2,5],[12,5],[9,7],[8,7],[8,8],[6,7],[6,8],[3,9],[4,11],[0,13],[4,15],[6,14],[8,14],[10,15],[8,10],[12,9],[14,13],[7,5]]).
horizontalObstacles([0,0,1,1], [[3,0],[10,0],[9,1],[5,1],[1,3],[4,4],[2,5],[7,5],[11,2],[12,5],[9,7],[8,7],[8,8],[6,7],[6,8],[12,9],[3,9],[4,11],[0,13],[4,15],[6,14],[8,12],[13,13],[13,15],[11,14]]).
horizontalObstacles([0,1,0,0], [[3,0],[9,0],[13,1],[10,2],[5,1],[1,3],[4,4],[2,5],[7,5],[10,7],[13,6],[8,7],[8,8],[6,7],[6,8],[3,9],[1,10],[2,14],[3,15],[5,13],[8,14],[10,15],[8,10],[14,13],[12,9]]).
horizontalObstacles([0,1,0,1], [[3,0],[9,0],[13,1],[10,2],[5,1],[1,3],[4,4],[2,5],[7,5],[6,7],[6,8],[8,7],[8,8],[10,7],[13,6],[12,9],[13,13],[13,15],[11,14],[8,12],[5,13],[3,15],[2,14],[1,10],[3,9]]).
horizontalObstacles([0,1,1,0], [[3,0],[5,1],[9,0],[10,2],[13,1],[1,3],[2,5],[4,4],[7,5],[6,7],[6,8],[8,7],[8,8],[10,7],[13,6],[12,9],[8,10],[3,9],[4,11],[0,13],[6,14],[8,14],[14,13],[4,15],[10,15]]).
horizontalObstacles([0,1,1,1], [[3,0],[5,1],[9,0],[10,2],[13,1],[13,6],[10,7],[12,9],[8,7],[8,8],[6,7],[6,8],[7,5],[4,4],[1,3],[2,5],[3,9],[4,11],[0,13],[6,14],[8,12],[11,14],[13,13],[13,15],[4,15]]).
horizontalObstacles([1,0,0,0], [[4,0],[10,0],[11,2],[12,5],[5,4],[2,1],[0,3],[6,7],[6,8],[5,6],[8,7],[8,8],[3,7],[3,9],[1,10],[2,14],[3,15],[5,13],[10,15],[8,14],[14,13],[12,9],[8,10],[9,7],[9,1]]).
horizontalObstacles([1,0,0,1], [[4,0],[2,1],[0,3],[5,4],[5,6],[3,7],[3,9],[1,10],[2,14],[3,15],[5,13],[8,12],[6,8],[6,7],[8,7],[8,8],[9,7],[9,1],[10,0],[11,2],[12,5],[12,9],[13,13],[13,15],[11,14]]).
horizontalObstacles([1,0,1,0], [[4,0],[10,0],[9,1],[11,2],[12,5],[5,4],[2,1],[0,3],[3,7],[5,6],[6,7],[6,8],[8,7],[8,8],[9,7],[12,9],[8,10],[4,11],[3,9],[0,13],[4,15],[6,14],[8,14],[10,15],[14,13]]).
horizontalObstacles([1,0,1,1], [[4,0],[10,0],[9,1],[11,2],[12,5],[5,4],[5,6],[0,3],[3,7],[3,9],[0,13],[4,11],[6,14],[8,12],[13,13],[13,15],[11,14],[12,9],[9,7],[8,7],[8,8],[6,7],[6,8],[4,15],[2,1]]).
horizontalObstacles([1,1,0,0], [[4,0],[9,0],[10,2],[13,1],[13,6],[10,7],[8,7],[8,8],[6,7],[6,8],[5,6],[5,4],[0,3],[3,7],[3,9],[1,10],[2,14],[3,15],[5,13],[8,14],[10,15],[14,13],[12,9],[8,10],[2,1]]).
horizontalObstacles([1,1,0,1], [[4,0],[9,0],[10,2],[13,1],[13,6],[10,7],[5,4],[0,3],[5,6],[3,7],[3,9],[1,10],[2,14],[3,15],[5,13],[8,12],[11,14],[13,15],[13,13],[12,9],[6,7],[6,8],[8,7],[8,8],[2,1]]).
horizontalObstacles([1,1,1,0], [[4,0],[2,1],[0,3],[5,4],[5,6],[3,7],[3,9],[0,13],[4,15],[6,14],[4,11],[8,10],[8,14],[10,15],[14,13],[12,9],[10,7],[13,6],[13,1],[10,2],[9,0],[6,7],[6,8],[8,7],[8,8]]).
horizontalObstacles([1,1,1,1], [[2,1],[4,0],[0,3],[5,4],[5,6],[9,0],[10,2],[13,1],[13,6],[10,7],[12,9],[13,13],[13,15],[11,14],[8,12],[6,14],[4,15],[4,11],[3,9],[0,13],[3,7],[6,7],[6,8],[8,7],[8,8]]).

% Cases having an obstacle to their bottom
verticalObstacles([0,0,0,0], [[0,3],[0,6],[1,2],[6,1],[9,0],[11,2],[15,3],[13,5],[15,9],[10,6],[12,8],[7,5],[5,3],[2,5],[4,9],[9,10],[14,13],[9,13],[6,12],[2,14],[1,9],[7,6],[8,6],[7,8],[8,8]]).
verticalObstacles([0,0,0,1], [[9,0],[15,1],[6,1],[11,2],[15,3],[5,3],[0,3],[1,2],[2,5],[7,5],[13,5],[10,6],[8,6],[7,6],[0,6],[7,8],[8,8],[12,8],[4,9],[1,9],[2,14],[6,12],[9,11],[11,14],[14,13]]).
verticalObstacles([0,0,1,0], [[6,1],[9,0],[11,2],[15,3],[13,5],[7,5],[5,3],[1,2],[0,2],[2,5],[0,6],[7,6],[8,6],[7,8],[8,8],[9,10],[15,9],[12,8],[10,6],[14,13],[9,13],[6,13],[5,10],[3,9],[1,13]]).
verticalObstacles([0,0,1,1], [[6,1],[9,0],[11,2],[15,1],[15,3],[5,3],[1,2],[0,2],[0,6],[2,5],[7,5],[13,5],[10,6],[8,6],[7,6],[7,8],[8,8],[12,8],[9,11],[5,10],[3,9],[14,13],[11,14],[6,13],[1,13]]).
verticalObstacles([0,1,0,0], [[6,1],[11,2],[14,0],[15,3],[7,5],[5,3],[1,2],[0,3],[0,6],[2,5],[4,9],[1,9],[7,6],[8,6],[7,8],[8,8],[10,6],[12,8],[15,9],[13,6],[14,13],[9,10],[9,13],[6,12],[2,14]]).
verticalObstacles([0,1,0,1], [[6,1],[14,0],[15,1],[15,3],[11,2],[13,6],[10,6],[8,6],[7,6],[7,8],[8,8],[7,5],[5,3],[2,5],[1,2],[0,3],[0,6],[4,9],[1,9],[2,14],[6,12],[9,11],[11,14],[14,13],[12,8]]).
verticalObstacles([0,1,1,0], [[0,2],[1,2],[6,1],[11,2],[14,0],[15,3],[10,6],[13,6],[7,5],[5,3],[2,5],[0,6],[7,6],[8,6],[7,8],[8,8],[9,10],[3,9],[5,10],[1,13],[6,13],[9,13],[15,9],[12,8],[14,13]]).
verticalObstacles([0,1,1,1], [[0,2],[1,2],[6,1],[5,3],[11,2],[15,1],[15,3],[14,0],[13,6],[10,6],[8,6],[7,6],[7,5],[7,8],[8,8],[12,8],[14,13],[11,14],[9,11],[5,10],[6,13],[1,13],[3,9],[0,6],[2,5]]).
verticalObstacles([1,0,0,0], [[0,3],[1,3],[0,4],[2,1],[6,3],[9,0],[11,2],[15,3],[13,5],[12,8],[15,9],[10,6],[8,6],[7,6],[7,8],[8,8],[9,10],[4,9],[5,5],[3,7],[1,9],[2,14],[6,12],[9,13],[14,13]]).
verticalObstacles([1,0,0,1], [[2,1],[0,3],[1,3],[0,4],[6,3],[9,0],[15,1],[15,3],[11,2],[13,5],[10,6],[12,8],[7,6],[8,6],[7,8],[8,8],[5,5],[4,9],[3,7],[1,9],[2,14],[6,12],[9,11],[14,13],[11,14]]).
verticalObstacles([1,0,1,0], [[0,2],[1,3],[2,1],[6,3],[9,0],[11,2],[15,3],[13,5],[15,9],[12,8],[10,6],[8,6],[7,6],[7,8],[8,8],[5,5],[0,4],[3,7],[3,9],[5,10],[1,13],[6,13],[9,13],[9,10],[14,13]]).
verticalObstacles([1,0,1,1], [[2,1],[0,2],[1,3],[0,4],[6,3],[5,5],[3,7],[3,9],[1,13],[5,10],[6,13],[9,11],[11,14],[14,13],[12,8],[10,6],[13,5],[15,3],[15,1],[11,2],[9,0],[8,6],[7,6],[7,8],[8,8]]).
verticalObstacles([1,1,0,0], [[2,1],[0,3],[1,3],[0,4],[6,3],[5,5],[7,6],[8,6],[7,8],[8,8],[10,6],[13,6],[15,3],[14,0],[11,2],[15,9],[12,8],[9,10],[14,13],[9,13],[6,12],[4,9],[2,14],[1,9],[3,7]]).
verticalObstacles([1,1,0,1], [[0,3],[1,3],[0,4],[2,1],[6,3],[11,2],[15,1],[15,3],[14,0],[13,6],[10,6],[12,8],[14,13],[11,14],[9,11],[6,12],[4,9],[1,9],[2,14],[3,7],[5,5],[7,6],[8,6],[7,8],[8,8]]).
verticalObstacles([1,1,1,0], [[7,8],[8,8],[7,6],[8,6],[2,1],[0,2],[1,3],[0,4],[6,3],[11,2],[14,0],[15,3],[15,9],[12,8],[13,6],[10,6],[14,13],[9,13],[9,10],[6,13],[5,10],[1,13],[3,7],[3,9],[5,5]]).
verticalObstacles([1,1,1,1], [[0,2],[1,3],[2,1],[6,3],[0,4],[3,7],[5,5],[3,9],[5,10],[1,13],[6,13],[9,11],[11,14],[14,13],[12,8],[10,6],[13,6],[15,3],[15,1],[14,0],[11,2],[7,6],[8,6],[7,8],[8,8]]).

