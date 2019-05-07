// =================================================================================
// Solution to maze problem in Scientific American, December 1996:
// Maze with linked nodes, with arcs labeled 'Yes', 'No', and
// self-referential rules in the boxes. The goal is to put a 'pencil'
// in box 1 and one in box 7 then reach a 'goal state' with either
// one of the pencils.
// =================================================================================

#include <stdlib.h>
#include <iostream>
#include <assert.h>

typedef int BOOL;

static const int TRUE  = 1;
static const int FALSE = 0;

// =================================================================================
// Stack tracer size
// =================================================================================

const int TRACE_ENTRIES = 300;

// =================================================================================
// Codes for special maze points
// =================================================================================

const int GOAL_MAZEPOINT        = 100;
const int ILLEGAL_MAZEPOINT     = 101;
const int START_PENCIL_0        = 1;
const int START_PENCIL_1        = 7;

// =================================================================================
// Codes for paths
// =================================================================================

const int PATH_YES    = 0; // Yes has been taken (no choice)
const int PATH_NO     = 1; // No has been taken (no choice)
const int PATH_LUGNUT = 2; // both Yes & LUGNUT can be taken (choice)
const int PATH_NONE   = 3; // in case of deadly embrace - no exit

// =================================================================================
// Codes for maze points
// =================================================================================

const int MAZEPOINT_COUNT            = 16;
const int MAZEPOINT[MAZEPOINT_COUNT] = {1,2,5,7,9,15,25,26,35,40,50,55,60,61,65,75};

// =================================================================================
// Description of a state in the state space: pencil 0 is on some mazepoint, pencil
// 2 is on some mazepoint, rule 60 is activated (or not), and pencil 0 or pencil 1
// or both moved in the last round.
// =================================================================================
// GetMazePointIndex(int x): Get the index of mazepoint 'x' in the 'MAZEPOINT'
//                           array. (ILLEGAL_MAZEPOINT & GOAL_MAZEPOINT do not have
//                           any mazepoint index, of course).
// Pencil(int pencil):       Get the mazepoint which 'pencil' (0 or 1) is at. This 
//                           is either a value from the 'MAZEPOINT' array or else
//                           'ILLEGAL_MAZEPOINT' or 'GOAL_MAZEPOINT'.
// MovementP(int pencil):    Did 'pencil' move in the last round?
// Rule60P(void) :           Is rule 60 active?
// IllegalP(void):           Is this an illegal state (any pencil at
//                           'ILLEGAL_MAZEPOINT')?
// GoalP(void):              Is this a goal state (any pencil at 'GOAL_MAZEPOINT')?
// SetPencil(int p,int mp):  Set pencil 'p' (0 or 1) to mazepoint 'mp'. 'mp' must 
//                           be either a value from the 'MAZEPOINT' array or else
//                           'ILLEGAL_MAZEPOINT' or 'GOAL_MAZEPOINT'.
// SetRule60(BOOL x):        Set or unset rule 60.
// SetMovement(int p,BOOL x):Record movement for pencil 'p'.
// =================================================================================
// Internal:
// A pencil's position is stored internally as the index into the MAZEPOINT array,
// except for ILLEGAL_MAZEPOINT and GOAL_MAZEPOINT, which are stored 'as is'.
// =================================================================================

class State {

private:

  unsigned char pencil[2];
  unsigned char rule60active;
  unsigned char pencilMoved[2];

public:

  static int GetMazePointIndex(int mp);

  State(void);
  State(const State &old);
  State &operator=(const State &old);
  ~State(void);

  int  Pencil(int pencil)    const;
  BOOL MovementP(int pencil) const;
  BOOL Rule60P(void)         const;

  BOOL IllegalP(void)        const;
  BOOL GoalP(void)           const;

  void SetPencil(int pencil,int mp);
  void SetMovement(int pencil,BOOL x);
  void SetRule60(BOOL x);

};

std::ostream &operator<<(std::ostream &os,const State &s);

// =================================================================================
// Description of a state transition:
//
// Given a 'current state', one can either choose to follow
// the rule at the maze point given by pencil 0, or one can follow the rule in 
// the maze point given by pencil 1. In case of a (rare) nondeterministic choice,
// an alternate state can be the result for pencil 0, or else an alternate state can
// be the result for pencil 1.
//
// Flags indicate whether the alternate states are valid or don't exist.
//
// A 'visited' counter tells when the Transition was visited by the
// state search algorithm (it's 0 if no visit has taken place)
// =================================================================================
// SetCurrentState(const State &current):
//   set the 'current' state (i.e. the state at which this transition may be 
//   applied). The 'current' state gives the last 3 dimensions of the state space
//   position.
// SetNextState(int chosenPencil,const State &next):
//   set the next state that can be reached by applying the rule pointed to by
//   pencil 'chosenPencil' (0 or 1), given the 'current' state. The
//   application of such a rule corresponds to traversal of an arc in the 5-d
//   state space. 
// SetAltNextState(int chosenPencil,const State &next):
//   set the next alternate state that can be reached by applying the rule pointed
//   to by pencil 'chosenPencil' (0 or 1), given the and 'current' state, in
//   case of a nondeterministic choice possibility. Calling this function 
//   automatically sets the 'valid' flag for that alternate state to 'TRUE'.
// CurrentState(void),NextState(int chosenPencil),
// AltNextState(int chosenPencil),AltNextValidP(int chosenPencil):
//   These function retrieve the named states or check whether the alternate 
//   next states exist.
// =================================================================================

class Transition {

private:

  State         current;
  State         next[2];
  State         altNext[2];
  int           visited;
  unsigned char altNextValid[2];

public:

  Transition(void);
  Transition(const Transition &old);
  Transition &operator=(const Transition &old);
  ~Transition(void);

  void  SetCurrentState(const State &current);
  void  SetNextState(int chosenPencil,const State &next);
  void  SetAltNextState(int chosenPencil,const State &next);
  
  State CurrentState(void)              const;
  State NextState(int chosenPencil)     const; 
  State AltNextState(int chosenPencil)  const;
  BOOL  AltNextValidP(int chosenPencil) const;

  void  SetVisited(int x);
  int   Visited(void) const;

};

std::ostream &operator<<(std::ostream &os,const Transition &t);

// =================================================================================
// =================================================================================

class Searcher {

  Transition *space;

  static long TotalStates(void);
  static long ComputeIndex(const State &current);

  void EnumerateTransitions(void);
  void DetermineNextStates(int chosenPencil,Transition &trs,int &pathTaken);

  void RecTraverse(long index,int depth,int &maxDepth,State *stackTrace);

  void DumpStackTrace(State *stackTrace,int depth,std::ostream &os);

  static BOOL HasBoxRedTextOrGreenTextP(int x);
  static BOOL HasBoxGreenTextOrGreenWordP(int x);
  static BOOL HasBoxRedWordOrGreenWordP(int x);
  static BOOL HasBoxOddNumberP(int x);
  static BOOL HasBoxWordWordP(int x);
  static BOOL HasBoxMultipleOfFiveNumberP(int x);
  static BOOL DoesBoxReferToCowsP(int x);
  static int  GetYesPath(int x);
  static BOOL HasBoxIfSentenceP(int x);
  
  // undefined and cannot be called

  Searcher(const Searcher &old);
  Searcher &operator=(const Searcher &old);

public:

  Searcher(void);
  ~Searcher(void);

  void StartTraversal(void);
  void DumpTransitions(std::ostream &os);
};

// =================================================================================
// Set everything in motion
// =================================================================================

int main(void) {
  Searcher x;
  x.StartTraversal();
  x.DumpTransitions(std::cout);
  return 0;
}

// =================================================================================
// Definitions for "State";
// =================================================================================

int State::GetMazePointIndex(int m) {
  int i;
  for (i=0;i<MAZEPOINT_COUNT;i++) {
    if (MAZEPOINT[i]==m) break;
  }
  if (i==MAZEPOINT_COUNT) {
    std::cerr << "State::GetMazePointIndex(): Illegal maze point value passed";
    std::cerr << std::endl << std::flush;
    abort();
  }
  return i;
}

State::State(void) {
  pencil[0] = ILLEGAL_MAZEPOINT;
  pencil[1] = ILLEGAL_MAZEPOINT;
  pencilMoved[0] = 0;
  pencilMoved[1] = 0;
  rule60active = 0;
}

State::State(const State &old) {
  (*this) = old;
}

State &State::operator=(const State &old) {
  pencil[0] = old.pencil[0];
  pencil[1] = old.pencil[1];
  pencilMoved[0] = old.pencilMoved[0];
  pencilMoved[1] = old.pencilMoved[1];
  rule60active = old.rule60active;
  return (*this);
}

State::~State(void) {
}

int State::Pencil(int p) const {
  if (p<0 || 1<p) {
    std::cerr << "State::Pencil(): Illegal pencil index passed";
    std::cerr << std::endl << std::flush;
    abort();
  }
  if (pencil[p]==ILLEGAL_MAZEPOINT ||
      pencil[p]==GOAL_MAZEPOINT) {
    return pencil[p];
  }
  else {
    return MAZEPOINT[pencil[p]];
  }
}

BOOL State::MovementP(int p) const {
  if (p<0 || 1<p) {
    std::cerr << "State::MovementP(): Illegal pencil index passed";
    std::cerr << std::endl << std::flush;
    abort();
  }
  return pencilMoved[p]==1;
}

BOOL State::Rule60P(void) const {
  return (rule60active==1);
}

void State::SetPencil(int p,int x) {
  if (p<0 || 1<p) {
    std::cerr << "State::SetPencil(): Illegal pencil index passed";
    std::cerr << std::endl << std::flush;
    abort();
  }
  if (x==ILLEGAL_MAZEPOINT ||
      x==GOAL_MAZEPOINT) {
    pencil[p] = x;
  }
  else {
    pencil[p] = (unsigned char)GetMazePointIndex(x);
  }
}

void State::SetMovement(int p,BOOL x) {
  if (p<0 || 1<p) {
    std::cerr << "State::SetMovement(): Illegal pencil index passed";
    std::cerr << std::endl << std::flush;
    abort();
  }
  if (x) pencilMoved[p] = 1; else pencilMoved[p] = 0;
}

void State::SetRule60(BOOL x) {
  if (x) rule60active = 1; else rule60active = 0;
}

BOOL State::IllegalP(void) const {
  return (pencil[0]==ILLEGAL_MAZEPOINT ||
          pencil[1]==ILLEGAL_MAZEPOINT);
}

BOOL State::GoalP(void) const {
  return (pencil[0]==GOAL_MAZEPOINT ||
          pencil[1]==GOAL_MAZEPOINT);
}

std::ostream &operator<<(std::ostream &os,const State &s) {
  os << "(";
  if (s.MovementP(0)) os << "m"; else os << ".";
  if (s.MovementP(1)) os << "m,"; else os << ".,";
  if (s.Pencil(0)==ILLEGAL_MAZEPOINT) {
    os << "XX";
  }
  else if (s.Pencil(0)==GOAL_MAZEPOINT) {
    os << "GG";
  }
  else {
    os.width(2);
    os << s.Pencil(0);
  }
  os << ",";
  if (s.Pencil(1)==ILLEGAL_MAZEPOINT) {
    os << "XX";
  }
  else if (s.Pencil(1)==GOAL_MAZEPOINT) {
    os << "GG";
  }
  else {
    os.width(2);
    os << s.Pencil(1);
  }
  os << ",";
  if (s.Rule60P()) os << "*)"; else os << " )";
  return os;
}

// =================================================================================
// Definitions for "Transition"
// =================================================================================

Transition::Transition(void) {
  visited         = 0;
  altNextValid[0] = 0;
  altNextValid[1] = 0;
}

Transition::Transition(const Transition &old) {
  (*this) = old;
}

Transition &Transition::operator=(const Transition &old) {
  current = old.current;
  next[0] = old.next[0];
  next[1] = old.next[1];
  altNext[0] = old.altNext[0];
  altNext[1] = old.altNext[1];
  altNextValid[0] = old.altNextValid[0];
  altNextValid[1] = old.altNextValid[1];
  visited = old.visited;
  return (*this);
}

Transition::~Transition(void) {
}
  
void Transition::SetCurrentState(const State &currentIn) {
  current = currentIn;
}

void Transition::SetNextState(int chosenPencil,const State &nextIn) {
  if (chosenPencil<0 || 1<chosenPencil) {
    std::cerr << "Transition::SetNextState(): Illegal pencil index passed";
    std::cerr << std::endl << std::flush;
    abort();
  }
  next[chosenPencil] = nextIn;
}

void Transition::SetAltNextState(int chosenPencil,const State &nextIn) {
  if (chosenPencil<0 || 1<chosenPencil) {
    std::cerr << "Transition::SetAltNextState(): Illegal pencil index passed";
    std::cerr << std::endl << std::flush;
    abort();
  }
  altNext[chosenPencil]      = nextIn;
  altNextValid[chosenPencil] = 1;
}

void Transition::SetVisited(int x) {
  visited = x;
}

int Transition::Visited(void) const {
  return visited;
}
  
State Transition::CurrentState(void) const {
  return current;
}

State Transition::NextState(int chosenPencil) const {
  if (chosenPencil<0 || 1<chosenPencil) {
    std::cerr << "Transition::NextState(): Illegal pencil index passed";
    std::cerr << std::endl << std::flush;
    abort();
  }
  return next[chosenPencil];
}
  
State Transition::AltNextState(int chosenPencil) const {
  if (chosenPencil<0 || 1<chosenPencil) {
    std::cerr << "Transition::AltNextState(): Illegal pencil index passed";
    std::cerr << std::endl << std::flush;
    abort();
  }
  return altNext[chosenPencil];  
}

BOOL Transition::AltNextValidP(int chosenPencil) const {
  if (chosenPencil<0 || 1<chosenPencil) {
    std::cerr << "Transition::AltNextValidP(): Illegal pencil index passed";
    std::cerr << std::endl << std::flush;
    abort();
  }
  return altNextValid[chosenPencil];  
}

std::ostream &operator<<(std::ostream &os,const Transition &t) {
  State cur  = t.CurrentState();
  os << cur << " -> ";
  os << t.NextState(0);
  if (t.AltNextValidP(0)) {
    os << " or " << t.AltNextState(0);
  }
  os << " (p0) & " << t.NextState(1);
  if (t.AltNextValidP(1)) {
    os << " or " << t.AltNextState(1);
  }
  os << " (p1) ";
  if (t.Visited()>0) {
    os << " visited: " << t.Visited();
  }
  return os;
}

// =================================================================================
// Definitions for "Searcher"
// =================================================================================

Searcher::Searcher(void) {
  std::cerr << "Allocating state space..." << std::endl << std::flush;
  space = new Transition[TotalStates()];
  if (space==NULL) {
    std::cerr << "Not enough memory" << std::endl << std::flush;
    abort();
  }
  EnumerateTransitions();
}

Searcher::~Searcher(void) {
  delete[] space;
}

long Searcher::TotalStates(void) {
  // This yields the total number of states to check out;
  // a point in the state space is given by the position
  // of pencil 0, the position of pencil 1 and two booleans
  // indicating whether pencil 0 or pencil 1 moved in the
  // last round and the state of rule 60. That's all the
  // information one needs.
  // If one did not want to distinguish state pencil positions
  // (7,1) and (1,7) for example, one could half the
  // state space. But it's not sure whether this is 
  // desirable...
  // Thus we consider:
  long result = (long)MAZEPOINT_COUNT *  // pencil 0
                (long)MAZEPOINT_COUNT *  // pencil 1
                2 *                      // pencil 0 movement
                2 *                      // pencil 1 movement
                2;                       // rule 60 active
  return result;
}

long Searcher::ComputeIndex(const State &current) {
  long result = 0;
  // this yields the index of the state array
  //
  // pencil 0,movement indexing ... range is 2
  result = result * 2;
  if (current.MovementP(0)) result++;
  // pencil 1,movement indexing ... range is 2
  result = result * 2;
  if (current.MovementP(1)) result++;
  // pencil 0,current indexing ... range is MAZEPOINT_COUNT; result is first
  // multiplied by the current range
  result = result * (long)MAZEPOINT_COUNT ;
  result = result + (long)State::GetMazePointIndex(current.Pencil(0));
  // pencil 1,current indexing ... range is MAZEPOINT_COUNT; result is first
  // multiplied by the current range
  result = result * (long)MAZEPOINT_COUNT ;
  result = result + (long)State::GetMazePointIndex(current.Pencil(1));
  // rule60,current boolean indexing ... range is 2; result is first
  // multiplied by the current range
  result = result * 2;
  if (current.Rule60P()) result++;
  return result;
}

void Searcher::EnumerateTransitions(void) {
  BOOL pencil0mv=TRUE;
  do {
    BOOL pencil1mv=TRUE;
    do {
      for (int curP0=0;curP0<MAZEPOINT_COUNT;curP0++) {
        for (int curP1=0;curP1<MAZEPOINT_COUNT;curP1++) {
          BOOL curR60=TRUE;
          do {
            State current;
            current.SetPencil(0,MAZEPOINT[curP0]);
            current.SetPencil(1,MAZEPOINT[curP1]);
            current.SetRule60(curR60);
            current.SetMovement(0,pencil0mv);
            current.SetMovement(1,pencil1mv);
            {
              long index = ComputeIndex(current);
              assert(index<TotalStates());
	            space[index].SetCurrentState(current);
              // compute next states
              int pathTaken;
              DetermineNextStates(0,space[index],pathTaken);
              DetermineNextStates(1,space[index],pathTaken);
            }
            curR60=!curR60;
          } while (!curR60);
        }
      }
      pencil1mv = !pencil1mv;
    } while (!pencil1mv);
    pencil0mv = !pencil0mv;
  } while (!pencil0mv);
}

void Searcher::DumpTransitions(std::ostream &os) {
  BOOL pencil0mv=TRUE;
  do {
    BOOL pencil1mv=TRUE;
    do {
      for (int curP0=0;curP0<MAZEPOINT_COUNT;curP0++) {
        for (int curP1=0;curP1<MAZEPOINT_COUNT;curP1++) {
          BOOL curR60=TRUE;
          do {
            State current;
            current.SetPencil(0,MAZEPOINT[curP0]);
            current.SetPencil(1,MAZEPOINT[curP1]);
            current.SetRule60(curR60);
            current.SetMovement(0,pencil0mv);
            current.SetMovement(1,pencil1mv);
            long index = ComputeIndex(current);
            assert(index<TotalStates());
            if (space[index].Visited()>0) {
	            os << space[index] << std::endl << std::flush;
            }
            curR60=!curR60;
          } while (!curR60);
        }
      }
      pencil1mv = !pencil1mv;
    } while (!pencil1mv);
    pencil0mv = !pencil0mv;
  } while (!pencil0mv);
}

void Searcher::DumpStackTrace(State *stackTrace,int depth,std::ostream &os) {
  if (depth>37) return;
  os << "---- Stack trace, depth " << depth << std::endl;
  for (int i=0;i<depth;i++) {
    os << stackTrace[i] << std::endl;
  }
}

// ---------------------------------------------------------------------------------
// Various decisions
// ---------------------------------------------------------------------------------

BOOL Searcher::HasBoxRedTextOrGreenTextP(int x) {
  return (x==7  || x==26 || x==61 || x==25 ||
          x==50 || x==60 || x==9  || x==40);
}

BOOL Searcher::HasBoxGreenTextOrGreenWordP(int x) {
  return (x==60 || x==5  || x==25 || x==2 ||
          x==65 || x==40 || x==1);
}

BOOL Searcher::HasBoxRedWordOrGreenWordP(int x) {
  return (x==5 || x==25  || x==2 || x==60 ||
          x==1 || x==40  || x==65);
}

BOOL Searcher::HasBoxOddNumberP(int x) {
  return (x%2==1);
}

BOOL Searcher::HasBoxWordWordP(int x) {
  return (x==35 || x==5);
}

BOOL Searcher::HasBoxMultipleOfFiveNumberP(int x) {
  return (x%5==0);
}

BOOL Searcher::DoesBoxReferToCowsP(int x) {
  return x==50;
}

int Searcher::GetYesPath(int x) {
  switch (x) {
  case 1:  return 2;
  case 2:  return 7;
  case 5:  return 25;
  case 7:  return 26;
  case 9:  return 2;
  case 15: return 5;
  case 25: return 7;
  case 26: return 61;
  case 35: return 40;
  case 40: return 65;
  case 50: return GOAL_MAZEPOINT;
  case 55: return 15;
  case 60: return 25;
  case 61: return 1;
  case 65: return 75;
  case 75: return 1;
  default:
    std::cerr << "GetYesPath(): No such state!" << std::endl << std::flush;
    abort();
    return 0; // keeps compiler happy
  }
}

BOOL Searcher::HasBoxIfSentenceP(int x) {
  return (x==61 || x==26 || x==65);
}

// ---------------------------------------------------------------------------------
// Determine the 'next states' for the passed transition if the rule in the box of
// 'chosenPencil' is considered. 'pathTaken' will take a code telling whether
// 'YES', 'NO' or 'LUGNUT' (in the single nondeterministic choice) has been
// taken
// ---------------------------------------------------------------------------------

void Searcher::DetermineNextStates(int chosenPencil,Transition &trs,int &pathTaken) {
  State current     = trs.CurrentState();
  State next        = current;
  State altNext     = current;
  next.SetMovement(0,FALSE);
  next.SetMovement(1,FALSE);
  int   otherPencil = (chosenPencil+1)%2;
  if (current.IllegalP() || current.GoalP()) {
    std::cerr << "Searcher::DetermineNextStates(): not a normal current state" << std::endl << std::flush;
    abort();
  }
  switch (current.Pencil(chosenPencil)) {
  case 1:
    // Box 1:
    // "Does the other pencil point to a box that has either red
    //  text or green text?"
    // Yes -> Box 2
    // No  -> Box 9
    if (HasBoxRedTextOrGreenTextP(current.Pencil(otherPencil))) {
      next.SetPencil(chosenPencil,2);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      next.SetPencil(chosenPencil,9);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_NO;
    }
    break;
  case 2:
    // Box 2:
    // "Does the other pencil point to a box that has green text
    // or has the word "green"?
    // Yes -> Box 7
    // No  -> Box 15
    if (HasBoxGreenTextOrGreenWordP(current.Pencil(otherPencil))) {
      next.SetPencil(chosenPencil,7);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      next.SetPencil(chosenPencil,15);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_NO;
    }
    break;
  case 5:
    // Box 5:
    // "Does the other pencil point to text that has the word "red" or 
    // the word "green"?
    // Yes -> Box 25
    // No  -> Box 2
    if (HasBoxRedWordOrGreenWordP(current.Pencil(otherPencil))) {
      next.SetPencil(chosenPencil,25);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      next.SetPencil(chosenPencil,2);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_NO;
    }
    break;
  case 7:
    // Box 7: (red text)
    // "Is the other pencil in a box whose number is an odd number?"
    // Yes -> Box 26
    // No  -> Box 5
    if (current.Rule60P()) {
      // ignore red text, just move through 'Yes'
      next.SetPencil(chosenPencil,26);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      // work off rule as usual
      if (HasBoxOddNumberP(current.Pencil(otherPencil))) {
        next.SetPencil(chosenPencil,26);
        next.SetMovement(chosenPencil,TRUE);
        trs.SetNextState(chosenPencil,next);
        pathTaken = PATH_YES;
      }
      else {
        next.SetPencil(chosenPencil,5);
        next.SetMovement(chosenPencil,TRUE);
        trs.SetNextState(chosenPencil,next);
        pathTaken = PATH_NO;
      }
    }
    break;
  case 9:
    // Box 9: (red text)
    // "On the last turn, did you move the other pencil?"
    // Yes -> 2
    // No  -> 35
    if (current.Rule60P()) {
      // ignore red text, just move through 'Yes'
      next.SetPencil(chosenPencil,2);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      // work off rule as usual
      if (current.MovementP(otherPencil)) {
        next.SetPencil(chosenPencil,2);
        next.SetMovement(chosenPencil,TRUE);
        trs.SetNextState(chosenPencil,next);
        pathTaken = PATH_YES;
      }
      else {
        next.SetPencil(chosenPencil,35);
        next.SetMovement(chosenPencil,TRUE);
        trs.SetNextState(chosenPencil,next);
        pathTaken = PATH_NO;
      }
    }
    break;
  case 15:
    // Box 15: 
    // "Is the other pencil in a box whose number is evenly divisible by 5?"
    // Yes -> 5
    // No  -> 40
    if (HasBoxMultipleOfFiveNumberP(current.Pencil(otherPencil))) {
      next.SetPencil(chosenPencil,5);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      next.SetPencil(chosenPencil,40);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_NO;
    }
    break;
  case 25:
    // Box 25: (red text)
    // "Does the other pencil point to a box that has either
    //  red text or green text?"
    // Yes -> 7
    // No  -> 50
    if (current.Rule60P()) {
      // ignore red text, just move through 'Yes'
      next.SetPencil(chosenPencil,7);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      // work off rule as usual
      if (HasBoxRedTextOrGreenTextP(current.Pencil(otherPencil))) {
        next.SetPencil(chosenPencil,7);
        next.SetMovement(chosenPencil,TRUE);
        trs.SetNextState(chosenPencil,next);
        pathTaken = PATH_YES;
      }
      else {
        next.SetPencil(chosenPencil,50);
        next.SetMovement(chosenPencil,TRUE);
        trs.SetNextState(chosenPencil,next);
        pathTaken = PATH_NO;
      }
    }
    break;
  case 26:
    // Box 26: (red text)
    // "If you had chosen the other pencil, would it exit on a
    // path marked "NO"?"
    // Yes -> 61
    // No  -> 55
    if (current.Rule60P()) {
      // ignore red text, just move through 'Yes'
      next.SetPencil(chosenPencil,61);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      // work off rule as usual
      // this is a recursive call, which need not be feasible
      if (current.Pencil(otherPencil)==26) {
        // deadly embrace
        next.SetPencil(chosenPencil,ILLEGAL_MAZEPOINT);
        next.SetPencil(otherPencil,ILLEGAL_MAZEPOINT);
        trs.SetNextState(chosenPencil,next);
        pathTaken = PATH_NONE;
      }
      else {
        // call the function recursively on a temporary transition
        int        otherPath;
        Transition tmp = trs;
        DetermineNextStates(otherPencil,tmp,otherPath);
        if (otherPath==PATH_NO) {
          next.SetPencil(chosenPencil,61);
          next.SetMovement(chosenPencil,TRUE);
          trs.SetNextState(chosenPencil,next);
          pathTaken = PATH_YES;
        }
        else {
          next.SetPencil(chosenPencil,55);
          next.SetMovement(chosenPencil,TRUE);
          trs.SetNextState(chosenPencil,next);
          pathTaken = PATH_NO;
        }
      }
    }
    break;
  case 35:
    // Box 35:
    // "Does the other pencil point to text that has the word "word"?
    // Yes -> 40
    // No  -> 1
    if (HasBoxWordWordP(current.Pencil(otherPencil))) {
      next.SetPencil(chosenPencil,40);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      next.SetPencil(chosenPencil,1);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_NO;
    }
    break;
  case 40:
    // Box 40: (red text)
    // "Is the text in this box green?" (this is a question about rule 60,i.e.
    // it prepares toggling rule 60)
    // Yes -> 65
    // No  -> 60
    if (current.Rule60P()) {
      // ignore red text, just move through 'Yes'
      next.SetPencil(chosenPencil,65);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      // work off rule as usual
      next.SetPencil(chosenPencil,60);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_NO;
    }
    break;
  case 50:
    // Box 50: (red text)
    // "Does the other pencil point to text that refers to cows?"
    // Yes -> GOAL
    // No  -> 26
    if (current.Rule60P()) {
      // ignore red text, just move through 'Yes'
      next.SetPencil(chosenPencil,GOAL_MAZEPOINT);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      // work off rule as usual
      if (DoesBoxReferToCowsP(current.Pencil(otherPencil))) {
        next.SetPencil(chosenPencil,GOAL_MAZEPOINT);
        next.SetMovement(chosenPencil,TRUE);
        trs.SetNextState(chosenPencil,next);
        pathTaken = PATH_YES;
      }
      else {
        next.SetPencil(chosenPencil,26);
        next.SetMovement(chosenPencil,TRUE);
        trs.SetNextState(chosenPencil,next);
        pathTaken = PATH_NO;
      }
    }
    break;
  case 55:
    // Box 55:
    // "Free choice: Exit either on the path marked "Yes" or on the
    //  path marked "LUGNUT""
    // Yes    -> 15
    // Lugnut -> 7
    // ...this means an alternate state comes on
    next.SetPencil(chosenPencil,15);
    next.SetMovement(chosenPencil,TRUE);
    trs.SetNextState(chosenPencil,next);
    next.SetPencil(chosenPencil,7);
    trs.SetAltNextState(chosenPencil,next);
    pathTaken = PATH_LUGNUT;
    break;
  case 60:
    // Box 60: (green text)
    // "Until further notice, make this change in the rules: If you choose
    //  a pencil that points to a red text, ignore what the text says. Just exit
    //  on the path marked "yes". Now exit from this box on the path marked
    //  "Yes"".
    // Yes -> 25
    next.SetPencil(chosenPencil,25);
    next.SetMovement(chosenPencil,TRUE);
    next.SetRule60(TRUE);
    trs.SetNextState(chosenPencil,next);
    pathTaken = PATH_YES;
    break;
  case 61:
    // Box 61: (red text)
    // "If you choose this box, ignore the text the other pencil points to.
    //  Move the other pencil on the path marked "Yes". Then move this 
    //  pencil on the path marked "yes".
    // Yes -> 1
    if (current.Rule60P()) {
      // ignore red text, just move through 'Yes'
      next.SetPencil(chosenPencil,1);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      // work off rule as usual
      next.SetPencil(chosenPencil,1);
      next.SetMovement(chosenPencil,TRUE);
      next.SetPencil(otherPencil,GetYesPath(current.Pencil(otherPencil)));
      next.SetMovement(otherPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    break;
  case 65:
    // Box 65:
    // "If the rule stated in green in Box 60 is now in effect, cancel
    //  that rule. Until further notice, when you choose a box with red
    //  text, follow what the text says. Now exit from this box on the
    //  path marked "Yes"."
    // Yes -> 75
    next.SetPencil(chosenPencil,75);
    next.SetRule60(FALSE);
    next.SetMovement(chosenPencil,TRUE);
    trs.SetNextState(chosenPencil,next);
    pathTaken = PATH_YES;
    break;
  case 75:
    // Box 75:
    // "Does the other pencil point to text that begins "If"?"
    // Yes -> 1
    // No  -> 50
    if (HasBoxIfSentenceP(current.Pencil(otherPencil))) {
      next.SetPencil(chosenPencil,1);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_YES;
    }
    else {
      next.SetPencil(chosenPencil,50);
      next.SetMovement(chosenPencil,TRUE);
      trs.SetNextState(chosenPencil,next);
      pathTaken = PATH_NO;
    }
    break;
  default:
    std::cerr << "Searcher::DetermineNextStates(): No such state" << std::endl << std::flush;
    abort();
  }
}

// ---------------------------------------------------------------------------------
// Recursively traverse the state space
// ---------------------------------------------------------------------------------

void Searcher::StartTraversal(void) {
  // allocate a stack tracer of TRACE_ENTRIES entries (empirical result, otherwise
  // a linked list would have to be created)
  State *stackTrace = new State[TRACE_ENTRIES];
  if (stackTrace==NULL) {
    std::cerr << "Out of memory" << std::endl << std::flush;
    abort();
  }
  // searching may start
  State start;
  int  maxDepth = 0;
  start.SetPencil(0,START_PENCIL_0);
  start.SetPencil(1,START_PENCIL_1);
  RecTraverse(ComputeIndex(start),1,maxDepth,stackTrace);
  std::cout << "The maximal search depth encountered is " << maxDepth << std::endl << std::flush;
  delete[] stackTrace;
}

void Searcher::RecTraverse(long index,int depth,int &maxDepth,State *stackTrace) {
  if (space[index].Visited()>0 && space[index].Visited()<=depth) {
    // we have been here earlier
    return;
  }
  else {
    // store the current depth here
    space[index].SetVisited(depth);
    // record a maximal depth value
    if (maxDepth<depth) maxDepth=depth;
    // record the current state
    if (depth<=TRACE_ENTRIES) {
      stackTrace[depth-1] = space[index].CurrentState();
    }
    // test all possible movements from here...
    for (int pencil=0;pencil<2;pencil++) {
      if (space[index].NextState(pencil).IllegalP()) {
        // no use continuing
        std::cerr << "Illegal state encountered!" << std::endl << std::flush;
        return;
      }
      else if (space[index].NextState(pencil).GoalP()) {
        // no use continuing
        std::cerr << "Goal state encountered at " << depth << "!" << std::endl << std::flush;
        // dump this
        DumpStackTrace(stackTrace,depth,std::cout);
        return;
      }
      else {
        long nextState = ComputeIndex(space[index].NextState(pencil));  
        RecTraverse(nextState,depth+1,maxDepth,stackTrace);
      }
      if (space[index].AltNextValidP(pencil)) {
        if (space[index].AltNextState(pencil).IllegalP()) {
          // no use continuing
          std::cerr << "Illegal state encountered!" << std::endl << std::flush;
          return;
        }
        else if (space[index].AltNextState(pencil).GoalP()) {
          // no use continuing
          std::cerr << "Goal state encountered at " << depth << "!" << std::endl << std::flush;
          // dump this
          DumpStackTrace(stackTrace,depth,std::cout);
          return;
        }
        else {
         long altNextState = ComputeIndex(space[index].AltNextState(pencil));  
         RecTraverse(altNextState,depth+1,maxDepth,stackTrace);
        }
      }
    }
  }
}
