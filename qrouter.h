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

#define NORTH	(u_char)1
#define SOUTH	(u_char)2
#define EAST	(u_char)3
#define WEST	(u_char)4
#define UP	(u_char)5
#define DOWN	(u_char)6

// define a structure containing x, y, and layer

typedef struct gridp_ GRIDP;

struct gridp_ {
   int x;
   int y;
   int lay;
   u_int cost;
};

typedef struct proute_ PROUTE;

struct proute_ {        // partial route
   u_char flags; 	// values PR_PROCESSED and PR_CONFLICT
   union {
      u_int cost;	// cost of route coming from predecessor
      u_int net;	// net number at route point
   } prdata;
};

// Bit values for "flags" in PROUTE

#define PR_PRED_DMASK	0x07		// Mask for directional bits

#define PR_PRED_NONE	0x00		// This node does not have a predecessor
#define PR_PRED_N	0x01		// Predecessor is north
#define PR_PRED_S	0x02		// Predecessor is south
#define PR_PRED_E	0x03		// Predecessor is east
#define PR_PRED_W	0x04		// Predecessor is west
#define PR_PRED_U	0x05		// Predecessor is up
#define PR_PRED_D	0x06		// Predecessor is down

#define PR_PROCESSED	0x08		// Tag to avoid visiting more than once
#define PR_CONFLICT	0x10		// Two nets collide here during stage 2
#define PR_SOURCE	0x20		// This is a source node
#define PR_TARGET	0x40		// This is a target node
#define PR_COST		0x80		// if 1, use prdata.cost, not prdata.net

// Linked string list

typedef struct string_ *STRING;

struct string_ {
   STRING next;
   char *name;
};

/* Path segment information */

#define  ST_WIRE		0x01
#define	 ST_VIA			0x02
#define	 ST_OFFSET_START	0x04	/* (x1, y1) is offset from grid */
#define	 ST_OFFSET_END		0x08	/* (x2, y2) is offset from grid */

typedef struct seg_ *SEG;

struct seg_ {
   SEG next;
   int layer;
   int x1, y1, x2, y2;
   u_char segtype;
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

// Structure for a network to be routed

typedef struct net_ *NET;
typedef struct netlist_ *NETLIST;

struct net_ {
   NET  next;
   int  netnum;		// a unique number for this net
   char *netname;
   NODE netnodes;	// list of nodes connected to the net
   int  numnodes;	// number of nodes connected to the net
   u_char flags;	// flags for this net (see below)
   int  netorder;	// to be assigned by route strategy (critical
			// nets first, then order by number of nodes).
   NETLIST noripup;	// list of nets that have been ripped up to
			// route this net.  This will not be allowed
			// a second time, to avoid looping.
   ROUTE   routes;	// routes for this net
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

// The following values are added to the Obs[] structure for unobstructed
// route positions close to a terminal, but not close enough to connect
// directly.  They describe which direction to go to reach the terminal.
// The Stub[] vector indicates the distance needed to reach the terminal.
// The OFFSET_TAP flag marks a position that is inside a terminal but
// which needs to be adjusted in one direction to avoid a close obstruction.
// The Stub[] vector indicates the distance needed to avoid the obstruction.
//
// The maximum number of nets must not overrun the area used by flags, so
// the maximum number of nets is 0x7fffff, or 8,388,607 nets

#define PINOBSTRUCTMASK	((u_int)0xe0000000)  // takes values from below
#define STUBROUTE_NS	((u_int)0x20000000)  // route north or south to reach terminal
#define STUBROUTE_EW	((u_int)0x40000000)  // route east or west to reach terminal
#define STUBROUTE_X	((u_int)0x60000000)  // diagonal---not routable
#define OFFSET_TAP	((u_int)0x80000000)  // position needs to be offset
#define NO_NET		((u_int)0x10000000)  // indicates a non-routable obstruction
#define BLOCKED_N	((u_int)0x08000000)  // grid point cannot be routed from the N
#define BLOCKED_S	((u_int)0x04000000)  // grid point cannot be routed from the S
#define BLOCKED_E	((u_int)0x02000000)  // grid point cannot be routed from the E
#define BLOCKED_W	((u_int)0x01000000)  // grid point cannot be routed from the W
#define BLOCKED_MASK	((u_int)0x0f000000)
#define ROUTED_NET	((u_int)0x00800000)  // indicates position occupied by a routed
					     // net
#define NETNUM_MASK	((u_int)0x107fffff)  // Mask for the net number field
					     // (includes NO_NET)

#define MAX_NETNUMS	((u_int)0x007fffff)  // Maximum net number

// Definitions used along with the NO_NET bit.
#define OBSTRUCT_MASK	((u_int)0x0000000f)  // Tells where obstruction is
#define OBSTRUCT_N	((u_int)0x00000008)  // relative to the grid point.
#define OBSTRUCT_S	((u_int)0x00000004)  // Stub[] contains distance of
#define OBSTRUCT_E	((u_int)0x00000002)  // obstruction to grid point.
#define OBSTRUCT_W	((u_int)0x00000001)

extern STRING DontRoute;
extern STRING CriticalNet;
extern GATE   GateInfo;		// standard cell macro information
extern NET    CurNet;
extern NETLIST FailedNets;	// nets that have failed the first pass
extern NETLIST Abandoned;	// nets that have failed the second pass

extern GATE   Nlgates;
extern NET    Nlnets;

extern u_int  *Obs[MAX_LAYERS];		// obstructions by layer, y, x
extern PROUTE *Obs2[MAX_LAYERS]; 	// working copy of Obs 
extern float  *Obsinfo[MAX_LAYERS];	// temporary detailed obstruction info
extern float  *Stub[MAX_LAYERS];	// stub route distances to pins
extern NODE   *Nodeloc[MAX_LAYERS];	// nodes are attached to grid points
					// for reverse lookup
extern NODE   *Nodesav[MAX_LAYERS];	// copy of Nodeloc used for restoring
					// Nodeloc after net rip-up
extern DSEG  UserObs;			// user-defined obstruction layers

extern u_char needblockX[MAX_LAYERS];
extern u_char needblockY[MAX_LAYERS];

extern int   Numnets;
extern int   Numgates;
extern int   Numpins;
extern int   Verbose;

extern char *vddnet;
extern char *gndnet;

extern int    set_num_channels();
extern int    allocate_obs_array();

NET    getnettoroute();
void   dosecondstage();
int    doroute(NET net, u_char stage);
int    route_segs(NET net, ROUTE rt, u_char stage);
ROUTE  createemptyroute();
void   emit_routes(char *filename, double oscale);

void   createMask();
void   fillMask();
void   setMask();
void   expandMask();

void   pathstart(FILE *cmd, int layer, int x, int y, u_char special, double oscale);
void   pathto(FILE *cmd, int x, int y, int horizontal, int lastx, int lasty);
void   pathvia(FILE *cmd, int layer, int x, int y, int lastx, int lasty);

void   helpmessage();

void   print_nets(char *filename);
void   print_routes(char *filename);
void   print_nlgates(char *filename);

#define QROUTER_H
#endif 

/* end of qrouter.h */
