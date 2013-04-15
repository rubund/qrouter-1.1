/*--------------------------------------------------------------*/
/* qrouter.h -- general purpose autorouter                     	*/
/*--------------------------------------------------------------*/
/*            Steve Beccue and Tim Edwards			*/
/*            Copyright (C) 2003 - 2013 - All Rights Reserved   */
/*--------------------------------------------------------------*/

#ifndef QROUTER_H

#define OGRID(x, y, layer) ((int)((x) + ((y) * NumChannelsX[(layer)])))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#define TRUE    1
#define FALSE   0

#ifndef _SYS_TYPES_H
#ifndef u_char
typedef unsigned char  u_char;
#endif
#ifndef u_short
typedef unsigned short u_short;
#endif
#ifndef u_int
typedef unsigned int   u_int;
#endif
#ifndef u_long
typedef unsigned long  u_long;
#endif
#endif /* _SYS_TYPES_H */

/* Maximum number of route layers */
#define MAX_LAYERS    9

/* Maximum number of pins in a gate */
#define MAX_GATE_NODES 64

/* Cell name (and othe names) max length */
#define MAX_NAME_LEN    64

/* Max reasonable line length */
#define MAXLINE         256

#define  CONFIGFILENAME        "route.cfg" 

// define possible gate orientations

#define  MNONE   0
#define  MX      1
#define  MY      2

// define search directions

#define NORTH	0
#define SOUTH	1
#define EAST	2
#define WEST	3
#define UP	4
#define DOWN	5

typedef struct proute_ PROUTE;

struct proute_ {        // partial route
  int pred;             // predecessor proute index
  int cost;		// cost of route coming from predecessor
  int layer;            // load into struct seg_
  int x1, y1;
  u_char flags; 	// values PR_PROCESSED and PR_CONFLICT
};

// Bit values for "flags" in PROUTE

#define PR_PROCESSED 1
#define PR_CONFLICT  2

// Linked string list

typedef struct string_ *STRING;

struct string_ {
   STRING next;
   char *name;
};

/* Path segment information */

enum segtype_ {ST_WIRE, ST_VIA};

typedef struct seg_ *SEG;

struct seg_ {
   SEG next;
   int layer;
   int x1, y1, x2, y2;
   int segtype;
};

/* DSEG is like a SEG, but coordinates are in microns (therefore type double)	*/
/* Used for gate node and obstruction positions.				*/

typedef struct dseg_ *DSEG;

struct dseg_ {
   DSEG   next;
   int    layer;
   double x1, y1, x2, y2;
};


/* POINT is an integer point in three dimensions (layer giving the	*/
/* vertical dimension).							*/

typedef struct point_ *POINT;

struct point_ {
  POINT next; 
  int layer;
  int x1, y1;
};

/* DPOINT is a point location with  coordinates given *both* as an	*/
/* integer (for the grid-based routing) and as a physical dimension	*/
/* (microns).								*/

typedef struct dpoint_ *DPOINT;

struct dpoint_ {
   DPOINT next;
   int layer;
   double x, y;
   int gridx, gridy;
};

typedef struct route_ *ROUTE;
typedef struct node_ *NODE;

struct route_ {
  ROUTE next;
  int   netnum;
  NODE  node;		// pointer to any node connected by this route
  SEG   segments;
  int   output;         // 0 if not output, 1 if already output
                        // prevents duplicate output
};

struct node_ {
  NODE    next;
  int     nodenum;                 // node ordering within its net
  DPOINT  taps;			   // point position for node taps
  DPOINT  extend;		   // point position within halo of the tap
  char    *netname;   		   // name of net this node belongs to
  int     netnum;                  // number of net this node belongs to
  ROUTE   routes;		   // routes for this node
  int     numnodes;		   // number of nodes on this net
};

// these are instances of gates in the netlist.  The description of a 
// given gate is gateinfo.  The same structure is used for both the
// gateinfo and the instances.

typedef struct gate_ *GATE;

struct gate_ {
    GATE next;
    char *gatename;     	     // e.g. x112
    char *gatetype;     	     // e.g. nd02d1
    int  nodes;                      // number of nodes on this gate
    char *node[MAX_GATE_NODES];	     // names of the pins on this gate
    int    netnum[MAX_GATE_NODES];   // net number connected to each pin
    NODE   noderec[MAX_GATE_NODES];  // node record for each pin
    DSEG   taps[MAX_GATE_NODES];     // list of gate node locations and layers
    DSEG   obs;			     // list of obstructions in gate
    double width, height;
    double placedX;                 
    double placedY;
    int orient;
    int rotate;
    int glue;
    int vert;                       // gluev flag - text is vertical
    int placegridx, placegridy;
    int placewidth, placeheight;
    int gatenum;                    // just an integer refering to this instance
};

// The list of nodes per net

typedef struct nodelist_ *NODELIST;

struct nodelist_ {
   NODELIST next;
   NODE node;
};

// Structure for a network to be routed

typedef struct net_ *NET;
typedef struct netlist_ *NETLIST;

struct net_ {
   NET  next;
   int  netnum;		// a unique number for this net
   char *netname;
   NODELIST netnodes;	// list of nodes connected to the net
   int  numnodes;	// number of nodes connected to the net
   u_char flags;	// flags for this net (see below)
   int  netorder;	// to be assigned by route strategy (critical
			// nets first, then order by number of nodes).
   NETLIST noripup;	// list of nets that have been ripped up to
			// route this net.  This will not be allowed
			// a second time, to avoid looping.
};

// Flags used by NET "flags" record

#define NET_PENDING 1	// pending being placed on "abandoned" list

// List of nets, used to maintain a list of failed routes

struct netlist_ {
   NETLIST next;
   int idx;
   NET net;
};

#define MAXRT		10000000		// "Infinite" route cost
#define PRindMAX	((u_int)0x10000000)	// Good to 8192 x 8192 x 4
#define RTFLAG		((u_int)0x80000000)
#define SRCFLAG		((u_int)0x40000000)	// Location is a source node
#define PINOBSTRUCTMASK	((u_int)0x60000000) 	// takes values from below

// The following values are added to the Obs[] structure for unobstructed
// route positions close to a terminal, but not close enough to connect
// directly.  They describe which direction to go to reach the terminal.

#define STUBROUTE_NS	((u_int)0x20000000)  // route north or south to reach terminal
#define STUBROUTE_EW	((u_int)0x40000000)  // route east or west to reach terminal
#define STUBROUTE_X	((u_int)0x60000000)  // diagonal---not routable

extern STRING DontRoute;
extern STRING CriticalNet;
extern GATE   GateInfo;		// standard cell macro information
extern NET    CurNet;
extern NETLIST FailedNets;	// nets that have failed the first pass
extern NETLIST Abandoned;	// nets that have failed the second pass

extern GATE   Nlgates;
extern NET    Nlnets;
extern NODE   Nlnodes;

extern u_int *Obs[MAX_LAYERS];		// obstructions by layer, y, x
extern u_int *Obs2[MAX_LAYERS]; 	// working copy of Obs 
extern float *Stub[MAX_LAYERS];		// stub route distances to pins
extern NODE  *Nodeloc[MAX_LAYERS];	// nodes are attached to grid points
					// for reverse lookup
extern NODE  *Nodesav[MAX_LAYERS];	// copy of Nodeloc used for restoring
					// Nodeloc after net rip-up
extern PROUTE  *Pr;			// put this in the Obs2 array
extern DSEG  UserObs;			// user-defined obstruction layers

extern int   Numnodes;
extern int   Numnets;
extern int   Numgates;
extern int   Numpins;
extern int   Verbose;
extern int   Loops;
extern int   PRind;
extern int   Debug;
extern int   CurrentPass;

NET    getnettoroute();
void   dosecondstage();
int    doroute(NET net, u_char stage);
int    route_segs(ROUTE rt, u_char stage);
ROUTE  createemptyroute();
void   emit_routes(char *filename, double oscale);

void   createMask();
void   fillMask();
void   setMask();
void   expandMask();

void   pathstart(FILE *cmd, int layer, int x, int y, u_char special, double oscale);
void   pathto(FILE *cmd, int x, int y, int vertical, int horizontal);
void   pathvia(FILE *cmd, int layer, int x, int y);

void   helpmessage();

void   print_nets(char *filename);
void   print_routes(char *filename);
void   print_nlgates(char *filename);

#define QROUTER_H
#endif 

/* end of qrouter.h */
