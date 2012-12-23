/*--------------------------------------------------------------*/
/*  qrouter.c -- general purpose autorouter                     */
/*  uses magic to automatically generate layout			*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, June 2011, based on code by Steve	*/
/* Beccue, 2003							*/
/*--------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

#define  MAIN

#include "qrouter.h"
#include "config.h"
#include "node.h"
#include "maze.h"
#include "lef.h"

int  Pathon = -1;
int  TotalRoutes = 0;

FILE *Failfptr = NULL;
FILE *CNfptr = NULL;

NET     Nlnets;		// list of nets in the design
NET	CurNet;		// current net to route, used by 2nd stage
STRING  DontRoute;      // a list of nets not to route (e.g., power)
STRING  CriticalNet;    // list of critical nets to route first
GATE    GateInfo;       // standard cell macro information
GATE    Nlgates;	// gate instance information
NODE    Nlnodes;	// information about gate terminals (pins)
NETLIST FailedNets;	// list of nets that failed to route
NETLIST Abandoned;	// list of nets that will never route

u_int *Obs[MAX_LAYERS];      // net obstructions in layer
u_int *Obs2[MAX_LAYERS];     // used for pt->pt routes on layer
float *Stub[MAX_LAYERS];     // used for stub routing to pins
NODE  *Nodeloc[MAX_LAYERS];  // nodes are here. . .
NODE  *Nodesav[MAX_LAYERS];  // . . . and here (but not to be altered)
PROUTE  Pr[PRindMAX + 1];    // put this in the Obs2 array
DSEG  UserObs;		     // user-defined obstruction layers

int   Numnodes = 0;
int   Numnets = 0;
int   Numgates = 0;
int   Numpins = 0;
int   Verbose = 0;
int   PRind = 0;
int   Debug = 0;                 // print level for debug information
int   CurrentPass = 0;           // Current Pass

/*--------------------------------------------------------------*/
/* Open the "failed" and "cn" (critical nets) files.		*/
/*--------------------------------------------------------------*/

void openFailFile()
{
   Failfptr = fopen("failed", "w");
   if (!Failfptr) {
      fprintf(stderr, "Could not open file \"failed\"\n");
   }

   CNfptr = fopen("cn", "w");
   if (!CNfptr) {
      fprintf(stderr, "Could not open file \"cn\".\n" );
   }
}
    
/*--------------------------------------------------------------*/
/* main - program entry point, parse command line		*/
/*								*/
/*   ARGS: argc (count) argv, command line 			*/
/*   RETURNS: to OS						*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

main(argc, argv)
   int	argc;
   char	*argv[];
{
   int	i, j;
   int length, width;
   FILE *l, *configFILEptr, *fptr;
   u_int u;
   static char configdefault[] = CONFIGFILENAME;
   char *configfile = configdefault;
   char *infofile = NULL;
   char *dotptr, *sptr;
   char DEFfilename[256];
   char Filename[256];

   NET net;
    
   Filename[0] = 0;
   DEFfilename[0] = 0;

   while ((i = getopt(argc, argv, "c:d:i:hv:")) != -1) {
      switch (i) {
	 case 'c':
	    configfile = strdup(optarg);
	    break;
	 case 'd':
	    Debug = atoi(optarg);
	    break;
	 case 'v':
	    Verbose = atoi(optarg);
	    break;
	 case 'i':
	    infofile = strdup(optarg);
	    break;
	 default:
	    fprintf(stderr, "bad switch %d\n", i);
	 case 'h':
	    helpmessage();
	    exit(0);
	    break;
      }
   }

   configFILEptr = fopen(configfile, "r");

   if (!configFILEptr) {
      char *configtmp;
      configtmp = malloc(strlen(configfile) + strlen(LIBDIR) + 2);
      sprintf(configtmp, "%s/%s", LIBDIR, configfile);
      if (configfile != configdefault) free(configfile);
      configfile = configtmp;
      configFILEptr = fopen(configfile, "r" );
   }

   if (configFILEptr) {
       read_config(configFILEptr);
   }
   else {
      fprintf(stderr, "Could not open %s!\n", configfile );
   }
   if (configfile != configdefault) free(configfile);

   if (infofile != NULL) {
      FILE *infoFILEptr;

      infoFILEptr = fopen(infofile, "w" );
      if (infoFILEptr != NULL) {
	 // fprintf(infoFILEptr, "INFO qrouter %s\n", QVERSION);
      
         /* Print information about route layers, and exit */
         for (i = 0; i < Num_layers; i++) {
	    fprintf(infoFILEptr, "%s %g %g %g %s\n",
			LefGetRouteName(i), LefGetRoutePitch(i),
			LefGetRouteOffset(i), LefGetRouteWidth(i),
			(LefGetRouteOrientation(i) == 1) ? "horizontal"
			: "vertical");
	 }
	 fclose(infoFILEptr);
      }
      exit(0);
   }

   if (optind < argc) {

      /* process remaining commandline strings */

      strcpy( Filename, argv[optind] );
      dotptr = strrchr(Filename, '.');
      if (dotptr != NULL) *dotptr = '\0';
      sprintf(DEFfilename, "%s.def", Filename);
   }
   else {
      fprintf(stderr, "No netlist file!\n");
      helpmessage();
      exit(1);
   }

   read_def_config(DEFfilename);
   read_def_placement(DEFfilename);
   read_def_netlist(DEFfilename);
   create_netorder();

   for (i = 0; i < Num_layers; i++) {
      if (PitchX[i] == 0.0 || PitchY[i] == 0.0) {
	 fprintf(stderr, "Have a 0 pitch for layer %d (of %d).  "
			"Exit.\n", i + 1, Num_layers);
	 exit(-3);
      }
      NumChannelsX[i] = (int)(1.5 + (Xupperbound - Xlowerbound) / PitchX[i]);
      NumChannelsY[i] = (int)(1.5 + (Yupperbound - Ylowerbound) / PitchY[i]);
      printf("Number of x channels for layer %d is %d\n",
		i, NumChannelsX[i]);
      printf("Number of y channels for layer %d is %d\n",
		i, NumChannelsY[i]);
	
      if (NumChannelsX[i] <= 0) {
	 fprintf(stderr, "Something wrong with layer %d x bounds.\n", i);
	 exit(-3);
      }
      if (NumChannelsY[i] <= 0) {
	 fprintf(stderr, "Something wrong with layer %d y bounds.\n", i);
	 exit(-3);
      }
      fflush(stdout);

      Obs[i] = (u_int *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(u_int));
      if (!Obs[i]) {
	 fprintf(stderr, "Out of memory 1.\n");
	 exit(3);
      }

      Obs2[i] = (u_int *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(u_int));
      if (!Obs2[i]) {
	 fprintf( stderr, "Out of memory 3.\n");
	 exit(5);
      }

      Stub[i] = (float *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(float));
      if (!Stub[i]) {
	 fprintf( stderr, "Out of memory 4.\n");
	 exit(6);
      }

      // Nodeloc is the reverse lookup table for nodes

      Nodeloc[i] = (NODE *)calloc(NumChannelsX[i] * NumChannelsY[i],
		sizeof(NODE));
      if (!Nodeloc[i]) {
         fprintf(stderr, "Out of memory 5.\n");
         exit(7);
      }

      Nodesav[i] = (NODE *)calloc(NumChannelsX[i] * NumChannelsY[i],
		sizeof(NODE));
      if (!Nodesav[i]) {
         fprintf(stderr, "Out of memory 6.\n");
         exit(8);
      }
   }
   fflush(stdout);

   fprintf(stderr, "Diagnostic: memory block is %d bytes\n",
		sizeof(u_int) * NumChannelsX[0] * NumChannelsY[0]);

   /* Be sure to create obstructions from gates first, since we don't	*/
   /* want improperly defined or positioned obstruction layers to over-	*/
   /* write our node list.						*/

   create_obstructions_from_gates();
   create_obstructions_from_nodes();
    
   // now we have netlist data, and can use it to get a list of nets.

   FailedNets = (NETLIST)NULL;
   Abandoned = (NETLIST)NULL;
   fflush(stdout);
   fprintf(stderr, "Numnets = %d, Numnodes = %d, Numpins = %d\n",
	     Numnets, Numnodes, Numpins );

   // print_nlgates( "net.details" );
   // print_nodes( "nodes.details" );
   // print_nlnets( "netlist.out" );
   // print_obs( "obs.out" );

   for (i = 0; i < Numnets; i++) {
      net = getnettoroute(i);
      if (net != NULL) doroute(net, (u_char)0);
   }

   fflush(stdout);
   fprintf(stdout, "\n----------------------------------------------\n");
   fprintf(stdout, "Progress: ");
   fprintf(stdout, "Total routing loops completed: %d\n", TotalRoutes);
   if (FailedNets == (NETLIST)NULL && Abandoned == (NETLIST)NULL)
      fprintf(stdout, "No failed routes!\n");
   else {
      if (FailedNets != (NETLIST)NULL)
          fprintf(stdout, "Failed net routes: %d\n", FailedNets->idx);
      if (Abandoned != (NETLIST)NULL)
          fprintf(stdout, "Abandoned net routes: %d\n", Abandoned->idx);
   }
   fprintf(stdout, "----------------------------------------------\n");

   dosecondstage();

   // Finish up by writing the routes to an annotated DEF file
    
   emit_routes(DEFfilename);

   fprintf(stdout, "----------------------------------------------\n");
   fprintf(stdout, "Final: ");
   if (FailedNets == (NETLIST)NULL && Abandoned == (NETLIST)NULL)
      fprintf(stdout, "No failed routes!\n");
   else {
      if (FailedNets != (NETLIST)NULL) {
         fprintf(stdout, "Failed net routes: %d\n", FailedNets->idx);
         fprintf(stdout, "See file \"failed\" for list of failing nets.\n");
      }
      if (Abandoned != (NETLIST)NULL) {
         fprintf(stdout, "Abandoned net routes: %d\n", Abandoned->idx);
         fprintf(stdout, "See file \"failed\" for list of abandoned nets.\n");
      }
   }
   fprintf(stdout, "----------------------------------------------\n");
   exit(0);

} /* main() */

/*--------------------------------------------------------------*/
/* pathstart - begin a DEF format route path           		*/
/*								*/
/* 	If "special" is true, then this path is in a		*/
/*	SPECIALNETS section, in which each route specifies	*/
/*	a width.						*/
/*--------------------------------------------------------------*/

void pathstart(FILE *cmd, int layer, int x, int y, u_char special)
{
   if (Pathon == 1) {
      fprintf( stderr, "pathstart():  Major error.  Started a new "
		"path while one is in progress!\n"
		"Doing it anyway.\n" );
   }

   if (layer >= 0) {
      if (Pathon == -1)
	 fprintf(cmd, "+ ROUTED ");
      else
	 fprintf(cmd, "\n  NEW ");
      if (special)
         fprintf(cmd, "%s %d ( %d %d ) ", CIFLayer[layer],
			(int)(100 * LefGetViaWidth(layer, layer, 0) + 0.5),
			x, y);
      else
         fprintf(cmd, "%s ( %d %d ) ", CIFLayer[layer], x, y);
   }
   Pathon = 1;

} /* pathstart() */

/*--------------------------------------------------------------*/
/* pathto  - continue a path to the next point        		*/
/*								*/
/*   ARGS: coordinate pair					*/
/*   RETURNS: 							*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

void pathto(FILE *cmd, int x, int y, int horizontal, int vertical)
{
    if (Pathon <= 0) {
	fprintf(stderr, "pathto():  Major error.  Added to a "
		"non-existent path!\n"
		"Doing it anyway.\n");
    }
    fprintf(cmd, "( ");
    if (vertical)
	fprintf(cmd, "* ");
    else
	fprintf(cmd, "%d ", x);

    if (horizontal)
	fprintf(cmd, "* ");
    else
	fprintf(cmd, "%d ", y);

    fprintf(cmd, ") ");

} /* pathto() */

/*--------------------------------------------------------------*/
/* pathvia  - add a via to a path               		*/
/*								*/
/*   ARGS: coord						*/
/*   RETURNS: 							*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

void pathvia(FILE *cmd, int layer, int x, int y)
{
   char *s;

  s = Via[layer];
  if (Pathon <= 0) {
     if (Pathon == -1)
	fprintf(cmd, "+ ROUTED ");
     else 
	fprintf(cmd, "\n  NEW ");
     fprintf(cmd, "%s ( %d %d ) ", CIFLayer[layer], x, y);
  }
  fprintf(cmd, "%s ", s);
  Pathon = 0;

} /* pathvia() */

/*--------------------------------------------------------------*/
/* print_nets - print the nets list - created from Nlgates list */
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Sat July 26		*/
/*--------------------------------------------------------------*/

void print_nets(char *filename)
{
   FILE *o;
   GATE g;
   int i;
   DSEG drect;

   if (!strcmp(filename, "stdout")) {
	o = stdout;
   } else {
	o = fopen(filename, "w");
   }
   if (!o) {
	fprintf(stderr, "route:print_nets.  Couldn't open output file\n");
	return;
   }

   for (g = Nlgates; g; g = g->next) {
      fprintf(o, "%s: %s: nodes->", g->gatename, g->gatetype);
      for (i = 0; i < g->nodes; i++) {
	 // This prints the first tap position only.
	 drect = g->taps[i];
	 fprintf( o, "%s(%g,%g) ", g->node[i], drect->x1, drect->y1);
      }
   }
   fprintf( o, "\n");
} /* print_nets() */

/*--------------------------------------------------------------*/
/* print_routes - print the routes list				*/
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Sat July 26		*/
/*--------------------------------------------------------------*/

void print_routes( char *filename )
{
    FILE *o;
    GATE g;
    int i;

    if( !strcmp( filename, "stdout" ) ) {
	o = stdout;
    } else {
	o = fopen( filename, "w" );
    }
    if( !o ) {
	fprintf( stderr, "route:print_routes.  Couldn't open output file\n" );
	return;
    }

    for (g = Nlgates; g; g = g->next) {
	fprintf( o, "%s: %s: nodes->", g->gatename, g->gatetype );
	for( i = 0 ; i < g->nodes; i++ ) {
	    fprintf( o, "%s ", g->node[i] );
	}
	fprintf(o, "\n");
    }
} /* print_routes() */

/*--------------------------------------------------------------*/
/* print_nlgates - print the nlgate list			*/
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Wed July 23		*/
/*--------------------------------------------------------------*/

void print_nlgates( char *filename )
{
    FILE *o;
    GATE g;
    int i;
    DSEG drect;

    if( !strcmp( filename, "stdout" ) ) {
	o = stdout;
    } else {
	o = fopen( filename, "w" );
    }
    if( !o ) {
	fprintf( stderr, "route:print_nlgates.  Couldn't open output file\n" );
	return;
    }

    for (g = Nlgates; g; g = g->next) {
	fprintf( o, "%s: %s: nodes->", g->gatename, g->gatetype );
	for( i = 0 ; i < g->nodes; i++ ) {
	    // This prints the first tap position only.
	    drect = g->taps[i];
	    fprintf( o, "%s(%g,%g)", g->node[i], drect->x1, drect->y1);
	}
        fprintf(o, "\n");
    }
} /* print_nlgates() */

/*--------------------------------------------------------------*/
/* Routine to read cell placement information from		*/
/* a DEF file COMPONENTS list (as generated by place2def2.tcl)	*/
/*--------------------------------------------------------------*/

void read_def_placement(char *filename)
{
   FILE *pp;
   GATE gate, gateginfo;
   int comps = 0, pins = 0;
   int ncells, npins;
   int curlayer = 0;
   char line[MAXLINE+1], inst[128], macro[128], orient[8];
   char netname[128], pinname[128], layname[32];
   char *ppos;
   int OK;
   int i, j;
   int mxflag, myflag, gateindex;
   double maxx, maxy, minx, miny, x, y, tmp, scalefac = 1.0;
   double pinx0, piny0, pinx1, piny1, hwidth;
   DSEG drect, newrect;

   maxx = maxy = -1E8;
   minx = miny = 1E8;

   pp = fopen(filename, "r");
   if (!pp) {
      fprintf(stderr, "Couldn't open placement file %s\n", filename);
      exit(-4);
   }
   else if (Verbose > 0) {
      fprintf(stdout, "route:read_def_placement:  Opened %s for reading.\n",
			filename);
   }

   gateindex = 1;

   while (!feof(pp)) {
      mxflag = myflag = 0;
      for (j = 0; j < MAXLINE; j++) line[j] = 0;
      for (j = 0; j < MAXLINE - 1; j++ ) {
	 line[j] = fgetc(pp);
	 if (line[j] == '\n') break;
      }
      if (comps == 0 && pins == 0) {
	 i = sscanf(line, "COMPONENTS %d ;", &ncells);
         if (i == 1) comps = 1;
	 else {
	    if (!strncmp(line, "UNITS", 5)) {
	       i = sscanf(line + 6, "DISTANCE MICRONS %lg", &scalefac);
	       if (i != 1) scalefac = 1.0;
	    }
	    else {
	       i = sscanf(line, "PINS %d ;", &npins);
	       if (i == 1) {
		  pins = 1;
		  Numpins = 0;
	       }
	    }
	 }
      }
      else if (comps == 1) {
	 if (!strncmp(line, "END COMPONENTS", 14)) {
	    comps = 0;
	 }
         else if (line[0] == '-') {
	    i = sscanf(line + 2, "%s %s + PLACED ( %lg %lg ) %s ;",
			inst, macro, &x, &y, &orient);
	    if (i == 5) {
	       if (orient[0] == 'F' && orient[1] == 'N')
		  mxflag = 1;
	       else if (orient[0] == 'F' && orient[1] == 'S')
		  myflag = 1;
	       else if (orient[0] == 'S') {	// Rotated 180, or flipped both
		  myflag = 1;
		  mxflag = 1;
	       }
	    }

	    gate = (GATE)malloc(sizeof(struct gate_));
	    gate->gatename = strdup(inst);
	    gate->gatetype = strdup(macro);
	    gate->gatenum = gateindex;
	    if (Numgates < gateindex) Numgates = gateindex;
	    gateindex++;

	    gate->placedX = x / scalefac;
	    gate->placedY = y / scalefac;
	    gate->orient = MNONE;
	    if (mxflag) gate->orient |= MX;
	    if (myflag) gate->orient |= MY;
	    // find the (x, y) of the nodes from ginfo
	    OK = 0;
	    for (gateginfo = GateInfo; gateginfo; gateginfo = gateginfo->next) {
	       if (!strcasecmp(gateginfo->gatetype, gate->gatetype)) {
		  OK = 1;
		  break;
	       }
	    }
	    if (!OK) {
	       fprintf(stderr, "route:read_def_placement: I couldn't find"
			" the pin locations for gate %s\nAdd it to the"
			" route.cfg file.\n", macro);
	       return;
	    }
	    gate->width = gateginfo->width;
	    gate->height = gateginfo->height;
	    gate->nodes = gateginfo->nodes;
	    gate->obs = (DSEG)NULL;

	    for (i = 0; i < gate->nodes; i++) {
	       /* Let the node names point to the master cell;  this is	just	*/
	       /* diagnostic;  allows us, for instance, to identify vdd and gnd	*/
	       /* nodes, so we don't complain about them being disconnected.	*/

	       gate->node[i] = gateginfo->node[i];	/* copy pointer, not string */
	       gate->taps[i] = (DSEG)NULL;

	       /* Make a copy of the gate nodes and adjust for instance position */
	       for (drect = gateginfo->taps[i]; drect; drect = drect->next) {
		  newrect = (DSEG)malloc(sizeof(struct dseg_));
		  *newrect = *drect;
		  newrect->next = gate->taps[i];
		  gate->taps[i] = newrect;
	       }

	       for (drect = gate->taps[i]; drect; drect = drect->next) {
		  // handle offset from the gate origin
		  drect->x1 -= gateginfo->placedX;
		  drect->x2 -= gateginfo->placedX;
		  drect->y1 -= gateginfo->placedY;
		  drect->y2 -= gateginfo->placedY;

	          // handle rotations and orientations here
		  if (gate->orient & MX) {
		     tmp = drect->x1;
		     drect->x1 = -drect->x2;
		     drect->x1 += gate->placedX + gateginfo->width;
		     drect->x2 = -tmp;
		     drect->x2 += gate->placedX + gateginfo->width;
		  }
		  else {
		     drect->x1 += gate->placedX;
		     drect->x2 += gate->placedX;
		  }
		  if (gate->orient & MY) {
		     tmp = drect->y1;
		     drect->y1 = -drect->y2;
		     drect->y1 += gate->placedY + gateginfo->height;
		     drect->y2 = -tmp;
		     drect->y2 += gate->placedY + gateginfo->height;
		  }
		  else {
		     drect->y1 += gate->placedY;
		     drect->y2 += gate->placedY;
		  }

		  if (drect->x1 > maxx) maxx = drect->x1;
		  else if (drect->x1 < minx) minx = drect->x1;
		  if (drect->x2 > maxx) maxx = drect->x2;
		  else if (drect->x2 < minx) minx = drect->x2;

		  if (drect->y1 > maxy) maxy = drect->y1;
		  else if (drect->y1 < miny) miny = drect->y1;
		  if (drect->y2 > maxy) maxy = drect->y2;
		  else if (drect->y2 < miny) miny = drect->y2;
	       }
	    }

	    /* Make a copy of the gate obstructions and adjust for instance position */
	    for (drect = gateginfo->obs; drect; drect = drect->next) {
	       newrect = (DSEG)malloc(sizeof(struct dseg_));
	       *newrect = *drect;
	       newrect->next = gate->obs;
	       gate->obs = newrect;
	    }

	    for (drect = gate->obs; drect; drect = drect->next) {
	       drect->x1 -= gateginfo->placedX;
	       drect->x2 -= gateginfo->placedX;
	       drect->y1 -= gateginfo->placedY;
	       drect->y2 -= gateginfo->placedY;

	       // handle rotations and orientations here
	       if (gate->orient & MX) {
		  tmp = drect->x1;
		  drect->x1 = -drect->x2;
		  drect->x1 += gate->placedX + gateginfo->width;
		  drect->x2 = -tmp;
		  drect->x2 += gate->placedX + gateginfo->width;
	       }
	       else {
		  drect->x1 += gate->placedX;
		  drect->x2 += gate->placedX;
	       }
	       if (gate->orient & MY) {
		  tmp = drect->y1;
		  drect->y1 = -drect->y2;
		  drect->y1 += gate->placedY + gateginfo->height;
		  drect->y2 = -tmp;
		  drect->y2 += gate->placedY + gateginfo->height;
	       }
	       else {
		  drect->y1 += gate->placedY;
		  drect->y2 += gate->placedY;
	       }
	    }
	    gate->next = Nlgates;
	    Nlgates = gate;
	 }
      }
      else if (pins == 1) {
	 if (!strncmp(line, "END PINS", 8)) {
	    pins = 2;
	 }
         else if (line[0] == '-') {
	    strcpy(layname, CIFLayer[0]);
	    strcpy(orient, "N");
	    pinx1 = piny1 = LefGetRouteWidth(0) / 2.0;
	    pinx0 = piny0 = -pinx1;
	    i = sscanf(line + 2, "%s", pinname);
	    if (i == 1) {
	       if ((ppos = strchr(line + 2, '+')) != NULL) {
		  // NET name will override pin name as the name of
		  // the net, though normally they will be the same.
		  i = sscanf(ppos + 2, "NET %s", netname);
		  if (i == 0) strcpy(netname, pinname);
	       }
	    }
	 }
         else if ((ppos = strchr(line, '+')) != NULL) {
	    i = sscanf(ppos + 2, "LAYER %s", layname);
	    if (i == 0) {
	       i = sscanf(ppos + 2, "PLACED ( %lg %lg ) %s", &x, &y, orient);
	       if (i == 3) {
		  gate = (GATE)malloc(sizeof(struct gate_));
		  gate->gatetype = (char *)malloc(4);
		  strcpy(gate->gatetype, "pin");
		  Numpins++;
		  gate->gatename = strdup(netname);
		  gate->width = (pinx1 - pinx0);
		  gate->height = (piny1 - pinx0);
		  gate->node[0] = strdup(netname);
		  gate->placedX = x / scalefac;
		  gate->placedY = y / scalefac;

		  drect = (DSEG)malloc(sizeof(struct dseg_));
		  gate->taps[0] = drect;
		  drect->next = (DSEG)NULL;

		  hwidth = LefGetRouteWidth(curlayer) / 2.0;
		  drect->x1 = gate->placedX - hwidth;
		  drect->y1 = gate->placedY - hwidth;
		  drect->x2 = gate->placedX + hwidth;
		  drect->y2 = gate->placedY + hwidth;
		  drect->layer = curlayer;
		  gate->obs = (DSEG)NULL;
		  gate->nodes = 1;
		  gate->glue = TRUE;
		  gate->vert = FALSE;
		  gate->next = Nlgates;
		  Nlgates = gate;
	       }
	    }
	    else {
	       i = sscanf(ppos + 2, "LAYER %*s ( %lg %lg ) ( %lg %lg )",
			&pinx0, &piny0, &pinx1, &piny1);
	       pinx0 /= scalefac;
	       piny0 /= scalefac;
	       pinx1 /= scalefac;
	       piny1 /= scalefac;
	       curlayer = LefFindLayerNum(layname);
	       if (curlayer < 0) curlayer = 0;
	    }
	 }
      }
      else if (pins == 2) {
         break;
      }
   }
   fclose(pp);
   printf("Max nodes are at (%g,%g) or (%d,%d) grids\n",
		maxx, maxy, (int)ceil(maxx/PitchX[0]),
		(int)ceil(maxy/PitchY[0]));

   /* Expand bounds if necessary to fit the placement 		*/
   /* (TRACKS should be used to increase the route area---	*/
   /* this code just ensures a minimum area to route in case	*/
   /* of bad or missing TRACKS and/or DIEAREA values)		*/

   if (maxx > Xupperbound) Xupperbound = maxx;
   if (maxy > Yupperbound) Yupperbound = maxy;
   if (minx < Xlowerbound) Xlowerbound = minx;
   if (minx < Ylowerbound) Ylowerbound = miny;
}

/*--------------------------------------------------------------*/
/* Read DEF file for configuration information			*/
/*--------------------------------------------------------------*/

void read_def_config(char *filename)
{
   FILE *pp;
   char line[MAXLINE + 1];
   char token[MAX_NAME_LEN], layer[MAX_NAME_LEN];
   char *cptr, corient;
   double  llx, lly, urx, ury;
   double  start, step, scalefac = 1.0;
   int channels, i, j, v, h;

   pp = fopen(filename, "r");
   if (!pp) {
      fprintf(stderr, "Couldn't open DEF file %s\n", filename);
      exit(-4);
   }
   else if (Verbose > 0) {
      fprintf(stdout, "route:read_def_config:  Opened %s for reading.\n", filename);
   }

   v = h = -1;
   while (!feof(pp)) {
      for (j = 0; j < MAXLINE; j++) line[j] = 0;
      for (j = 0; j < MAXLINE - 1; j++ ) {
	 line[j] = fgetc(pp);
	 if (line[j] == '\n') break;
      }
      i = sscanf(line, "%s", token);
      if (i == 1) {
	 if (!strcasecmp(token, "DIEAREA")) {
	    i = 0;
	    cptr = strchr(line, '(');
	    if (cptr != NULL)
	       i = sscanf(cptr + 1, "%lg %lg", &llx, &lly);
	    if (i == 2) {
	       i = 0;
	       cptr = strchr(cptr + 1, '(');
	       if (cptr != NULL)
		  i = sscanf(cptr + 1, "%lg %lg", &urx, &ury);
	    }
	    if (i == 2) {
	       Xlowerbound = llx / scalefac;
	       Ylowerbound = lly / scalefac;
	       Xupperbound = urx / scalefac;
	       Yupperbound = ury / scalefac;
	    }
	 }
	 else if (!strcasecmp(token, "TRACKS")) {
	    i = sscanf(line + 7, "%c %lg DO %d STEP %lg LAYER %s",
			&corient, &start, &channels, &step, layer); 
	    if (i == 5) {
	       for (j = 0; j < Num_layers; j++) {
		  if (!strcasecmp(layer, CIFLayer[j]))
		     break;
	       }
	       if (j == Num_layers) {
		  /* Assume "M1", "M2", or similar */
		  if (tolower(layer[0]) == 'm') {
		     sscanf(layer + 1, "%d", &j);
		     j--;
		     if (j > Num_layers) {
			Num_layers = j;
			strcpy(CIFLayer[j], layer);
		     }
		  }
	       }
	       if (tolower(corient) == 'x') {
		  Vert[j] = 1;
	          PitchX[j] = step / scalefac;
		  v = j;
		  if ((j < Num_layers - 1) && PitchX[j + 1] == 0.0)
		     PitchX[j + 1] = PitchX[j];
		  llx = start;
		  urx = start + step * channels;
		  if (llx < Xlowerbound) Xlowerbound = llx / scalefac;
		  if (urx > Xupperbound) Xupperbound = urx / scalefac;
	       }
	       else {
		  Vert[j] = 0;
	          PitchY[j] = step / scalefac;
		  h = j;
		  if ((j < Num_layers - 1) && PitchY[j + 1] == 0.0)
		     PitchY[j + 1] = PitchY[j];
		  lly = start;
		  ury = start + step * channels;
		  if (lly < Ylowerbound) Ylowerbound = lly / scalefac;
		  if (ury > Yupperbound) Yupperbound = ury / scalefac;
	       }
	    }
	 }
	 else if (!strcasecmp(token, "UNITS")) {
	    i = sscanf(line + 6, "DISTANCE MICRONS %lg",
			&scalefac);
	    if (i != 1) scalefac = 1.0;
	 }
	 else if (!strcasecmp(token, "COMPONENTS")) {
	    break;
	 }
      }
   }

   /* Because the TRACKS statement only covers the pitch of	*/
   /* a single direction, we need to fill in with the pitch	*/
   /* of opposing layers.  For now, we expect all horizontal	*/
   /* routes to be at the same pitch, and all vertical routes	*/
   /* to be at the same pitch.					*/

   if (h == -1) h = v;
   if (v == -1) v = h;
   
   /* This code copied from config.c.  Preferably, all information	*/
   /* available in the DEF file should be taken from the DEF file.	*/

   for (i = 0; i < Num_layers; i++) {
     if (PitchX[i] != 0.0 && PitchX[i] != PitchX[v])
        fprintf(stderr, "Multiple vertical route layers at different"
                " pitches.  I cannot handle this!  Using pitch %g\n",
                PitchX[v]);
     PitchX[i] = PitchX[v];
     if (PitchY[i] != 0.0 && PitchY[i] != PitchY[h])
        fprintf(stderr, "Multiple horizontal route layers at different"
                " pitches.  I cannot handle this!  Using pitch %g\n",
                PitchY[h]);
     PitchY[i] = PitchY[h];
  }

   fclose(pp);
}

/*--------------------------------------------------------------*/
/* Alternative to reading any gate information at all:  Just	*/
/* read the netlist information from the .def file into the	*/
/* nodes and nets arrays.					*/
/*--------------------------------------------------------------*/

void read_def_netlist(char *filename)
{
   FILE *pp;
   int pins = 0, npins;
   int nets = 0, nnets;
   int i, j, idx2, OK;
   char netname[MAX_NAME_LEN], eol[MAX_NAME_LEN];
   char instname[MAX_NAME_LEN], pinname[MAX_NAME_LEN];
   char line[MAXLINE + 1];
   char *cptr;
   NET net;
   NODE node, node2;
   NODELIST nl;
   GATE gateginfo;
   DSEG drect;
   GATE g;
   double dx, dy;
   int gridx, gridy;
   DPOINT dp;
   double home[MAX_LAYERS];

   pp = fopen(filename, "r");
   if (!pp) {
      fprintf(stderr, "Couldn't open DEF file %s\n", filename);
      exit(-4);
   }
   else if (Verbose > 0) {
      fprintf(stdout, "route:read_def_netlist:  Opened %s for reading.\n", filename);
   }

   Nlnets = (NET)NULL;
   Nlnodes = (NODE)NULL;

   // Compute distance for keepout halo for each route layer
   for (i = 0; i < Num_layers; i++) {
      home[i] = LefGetRouteKeepout(i);
   }

   while (!feof(pp)) {
      for (j = 0; j < MAXLINE; j++) line[j] = 0;
      for (j = 0; j < MAXLINE - 1; j++ ) {
	 line[j] = fgetc(pp);
	 if (line[j] == '\n') break;
      }
      if (pins == 0) {
	 i = sscanf(line, "PINS %d ;", &npins);
         if (i == 1) pins = 1;
	 if (npins > 0) {
	    /* Check if GateInfo has an entry for gate "pin" */
	    OK = 0;
	    for (gateginfo = GateInfo; gateginfo; gateginfo = gateginfo->next) {
	       if (!strcasecmp(gateginfo->gatetype, "pin")) {
		  OK = 1;
		  break;
	       }
	    }

	    if (OK == 0) {
	       /* Add a new GateInfo entry for pseudo-gate "pin" */
               gateginfo = (GATE)malloc(sizeof(struct gate_));
	       gateginfo->gatetype = (char *)malloc(4);
               strcpy(gateginfo->gatetype, "pin");
               gateginfo->gatename = NULL;
               gateginfo->width = LefGetRouteWidth(0);
               gateginfo->height = LefGetRouteWidth(0);
               gateginfo->placedX = 0.0;
               gateginfo->placedY = 0.0;
               gateginfo->nodes = 1;
               gateginfo->node[0] = strdup("pin");

	       drect = (DSEG)malloc(sizeof(struct dseg_));
	       gateginfo->taps[0] = drect;
	       drect->next = (DSEG)NULL;

	       drect->x1 = drect->x1 = 0.0;
	       drect->y1 = drect->y1 = 0.0;
	       drect->layer = 0;
               gateginfo->obs = (DSEG)NULL;
	       gateginfo->next = GateInfo;
	       GateInfo = gateginfo;
	    }
	 }
      }
      else if (pins == 1) {
	 if (!strncmp(line, "END PINS", 8)) {
	    pins == 2;
	 }
      }
      if (nets == 0) {
	 i = sscanf(line, "NETS %d ;", &nnets);
         if (i == 1) {
	    nets = 1;
	    Numnets = MIN_NET_NUMBER;
	 }
      }
      else if (nets == 1) {
	 if (!strncmp(line, "END NETS", 8)) {
	    nets == 2;
	 }
	 i = sscanf(line, "- %s", netname);
	 if (i == 1) {
	    nets = 3;
	    idx2 = 0;
	    net = (NET)malloc(sizeof(struct net_));
	    net->netnum = Numnets++;
	    net->netorder = 0;
	    net->netname = strdup(netname);
	    net->netnodes = (NODELIST)NULL;
	    net->noripup = (NETLIST)NULL;

	    net->next = Nlnets;
	    Nlnets = net;
	 }
      }
      else if (nets == 3) {	/* read lines until semicolon */
	 i = 0;
	 cptr = strchr(line, '(');
	 if (cptr != NULL)
	    i = sscanf(cptr, "( %s %s )%s", instname, pinname, eol);
	 if (i >= 2) {
	    if ((i == 3) && (strchr(eol, ';') != NULL)) {
	       nets = 1;
	    }
	    node = (NODE)calloc(1, sizeof(struct node_));
	    node->nodenum = idx2++; 

	    /* Pins are just listed as PIN but the inst name has been	*/
	    /* set equal to the net name when the PIN list was read.	*/
	    /* We have set the name of the pin's node[0] to "pin", so	*/
	    /* swap the two entries.					*/

	    if (!strcasecmp(instname, "pin")) {
		strcpy(instname, pinname);
		strcpy(pinname, "pin");
	    }

	    /* Find the gate and determine the coordinate of the node */

	    for (g = Nlgates; g; g = g->next) {
	       if (!strcasecmp(instname, g->gatename)) {
		  for (gateginfo = GateInfo; gateginfo; gateginfo = gateginfo->next) {
		     if (!strcasecmp(g->gatetype, gateginfo->gatetype)) {
			for (i = 0; i < gateginfo->nodes; i++) {
			   if (!strcasecmp(gateginfo->node[i], pinname)) {
			      node->taps = (DPOINT)NULL;
			      node->extend = (DPOINT)NULL;

			      for (drect = g->taps[i]; drect; drect = drect->next) {

				 // Add all routing gridpoints that fall inside
				 // the rectangle.
				 // Much to do here: (1) routable area should
				 // extend 1/2 route width to each side, as spacing
				 // to obstructions allows. (2) terminals that are
				 // wide enough to route to but not centered on
				 // gridpoints should be marked in some way, and
				 // handled appropriately.

				 gridx = (int)((drect->x1 - Xlowerbound)
						/ PitchX[drect->layer]) - 1;
				 while (1) {
				    dx = (gridx * PitchX[drect->layer]) + Xlowerbound;
				    if (dx > drect->x2 + home[drect->layer]) break;
				    if (dx < drect->x1 - home[drect->layer]) {
				       gridx++;
				       continue;
				    }
				    gridy = (int)((drect->y1 - Ylowerbound)
						/ PitchY[drect->layer]) - 1;
				    while (1) {
				       dy = (gridy * PitchY[drect->layer]) + Ylowerbound;
				       if (dy > drect->y2 + home[drect->layer]) break;
				       if (dy < drect->y1 - home[drect->layer]) {
				          gridy++;
				          continue;
				       }
				       if (dy >= drect->y1 && dx >= drect->x1 &&
						dy <= drect->y2 && dx <= drect->x2) {

					  // Routing grid point is an interior point
					  // of a gate port.  Record the position

				          dp = (DPOINT)malloc(sizeof(struct dpoint_));
					  dp->layer = drect->layer;
					  dp->x = dx;
					  dp->y = dy;
					  dp->gridx = gridx;
					  dp->gridy = gridy;
					  dp->next = node->taps;
					  node->taps = dp;
				       }
				       else {
				          dp = (DPOINT)malloc(sizeof(struct dpoint_));
					  dp->layer = drect->layer;
					  dp->x = dx;
					  dp->y = dy;
					  dp->gridx = gridx;
					  dp->gridy = gridy;
					  dp->next = node->extend;
					  node->extend = dp;
				       }
				       gridy++;
				    }
				    gridx++;
				 }
			      }
			      node->netnum = net->netnum;
			      g->netnum[i] = net->netnum;
			      g->noderec[i] = node;
			      node->netname = net->netname;
			      node->routes = (ROUTE)NULL;
			      node->next = Nlnodes;
			      Nlnodes = node;
			      Numnodes++;
			      break;
			   }
			}
			if (i < gateginfo->nodes) break;
		     }
		  }
		  if (!gateginfo)
		     fprintf(stderr, "Error:  Endpoint %s/%s of net %s"
				" not found\n", instname, pinname, netname);
	       }
	    }
	 }
      }	 
   }

   // Set the number of nodes per net for each node on the net

   for (node = Nlnodes; node; node = node->next) {
      i = 0;
      for (node2 = Nlnodes; node2; node2 = node2->next) {
         if (node->netname == node2->netname) i++;
      }
      node->numnodes = i;
   }

   // Fill in the netnodes list for each net, needed for checking
   // for isolated routed groups within a net.

   for (net = Nlnets; net; net = net->next) {
      for (node = Nlnodes; node; node = node->next) {
	 if (node->netname == net->netname) {
	    nl = (NODELIST)malloc(sizeof(struct nodelist_));
	    nl->node = node;
	    nl->next = net->netnodes;
	    net->netnodes = nl;
	    net->numnodes++;
	 }
      }
   }

   fclose(pp);
}

/*--------------------------------------------------------------*/
/* getnettoroute - get a net to route				*/
/*								*/
/*   ARGS: 							*/
/*   RETURNS: 							*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

NET getnettoroute(int order)
{
  NET net;
  
  fflush(stdout);
  fflush(stderr);

  for (net = Nlnets; net; net = net->next) {
     if (net->netorder == order) {
	if (net->numnodes >= 2) {
	   return net;
        }
     }
  }
  if (Verbose > 0) {
     fprintf(stderr, "getnettoroute():  Fell through\n");
  }
  return NULL;

} /* getnettoroute() */

/*--------------------------------------------------------------*/
/* dosecondstage() ---						*/
/*								*/
/* Second stage:  Rip-up and reroute failing nets.		*/
/* Method:							*/
/* 1) Route a failing net with stage = 1 (other nets become	*/
/*    costs, not blockages, no copying to Obs)			*/
/* 2) If net continues to fail, flag it as unroutable and	*/
/*    remove it from the list.					*/
/* 3) Otherwise, determine the nets with which it collided.	*/
/* 4) Remove all of the colliding nets, and add them to the	*/
/*    FailedNets list						*/
/* 5) Route the original failing net.				*/
/* 6) Continue until all failed nets have been processed.	*/
/*--------------------------------------------------------------*/

void
dosecondstage()
{
   int i, result;
   NET net;
   NETLIST nl, nl2, fn;

   while (FailedNets != NULL) {

      // Diagnostic:  how are we doing?
      i = 0;
      for (nl = FailedNets; nl; nl = nl->next) i++;
      fprintf(stdout, "------------------------------\n");
      fprintf(stdout, "Number of remaining nets: %d\n", i);
      fprintf(stdout, "------------------------------\n");

      net = FailedNets->net;

      // Remove this net from the fail list
      nl2 = FailedNets;
      FailedNets = FailedNets->next;
      free(nl2);

      // Route as much as possible without collisions
      result = doroute(net, (u_char)0);

      if (result != 0) {
	 fprintf(stderr, "Routing net with collisions\n");
         result = doroute(net, (u_char)1);
         if (result != 0) {
	    if (net->noripup != NULL) {
	       if ((net->flags & NET_PENDING) == 0) {
	          // Clear this net's "noripup" list and try again.

	          while (net->noripup) {
	             nl = net->noripup->next;
	             free(net->noripup);
	             net->noripup = nl;
	          }
	          result = doroute(net, (u_char)1);
		  net->flags |= NET_PENDING;	// Next time we abandon it.
	       }
	    }
	 }
         if (result != 0) {
	    // Complete failure to route, even allowing collisions.
	    // Abandon routing this net.
	    fprintf(stderr, "----------------------------------------------\n");
	    fprintf(stderr, "Complete failure on net %s:  Abandoning.\n",
			net->netname);
	    fprintf(stderr, "----------------------------------------------\n");
	    // Add the net to the "abandoned" list
	    nl = (NETLIST)malloc(sizeof(struct netlist_));
	    nl->net = net;
	    if (Abandoned != (NETLIST)NULL)
	       nl->idx = Abandoned->idx + 1;
	    else
	       nl->idx = 1;
	    nl->next = Abandoned;
	    Abandoned = nl;

	    while (FailedNets && (FailedNets->net == net)) {
	       nl = FailedNets->next;
	       free(FailedNets);
	       FailedNets = nl;
	    }
	    continue;
	 }

         // Analyze route for nets with which it collides

         nl = find_colliding(net);
      }
      else
	 nl = (NETLIST)NULL;

      // Remove the colliding nets from the route grid and append
      // them to FailedNets.

      while(nl) {
	 nl2 = nl->next;
         fprintf(stdout, "Ripping up blocking net %s\n", nl->net->netname);
	 ripup_net(nl->net, (u_char)1);
	 for (fn = FailedNets; fn && fn->next != NULL; fn = fn->next);
	 if (fn)
	    fn->next = nl;
	 else
	    FailedNets = nl;

	 // Add nl->net to "noripup" list for this net, so it won't be
	 // routed over again by the net.  Avoids infinite looping in
	 // the second stage.

	 fn = (NETLIST)malloc(sizeof(struct net_));
	 fn->next = net->noripup;
	 net->noripup = fn;
	 fn->net = nl->net;

	 nl->next = (NETLIST)NULL;
	 nl = nl2;
      }

      // Now we copy the net we routed above into Obs
      writeback_all_routes(net);
   }

   // Make sure FailedNets is cleaned up, in case we broke out
   // of the loop with an error condition.

   while (FailedNets) {
      nl = FailedNets->next;
      free(FailedNets);
      FailedNets = nl;
   }
}

/*--------------------------------------------------------------*/
/* doroute - basic route call					*/
/*								*/
/*	stage = 0 is normal routing				*/
/*	stage = 1 is the rip-up and reroute stage		*/
/*								*/
/*   ARGS: two nodes to be connected				*/
/*   RETURNS: 0 on success, -1 on failure			*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

int doroute(NET net, u_char stage)
{
  ROUTE rt1, lrt;
  NODE n1, n2;
  NETLIST nlist;
  int result;

  if (!net) {
     fprintf(stderr, "doroute():  no net to route.\n");
     return 0;
  }

  CurNet = net;				// Global, used by 2nd stage
  n1 = (NODE)(net->netnodes->node);	// Use the first node

  while (1) {	// Keep going until we are unable to route to a terminal

     TotalRoutes++;

     rt1 = createemptyroute();
     rt1->netnum = net->netnum;
     rt1->node = n1;

     if (Verbose > 0) {
        fprintf(stdout,"doroute(): added net %d path start %d\n", 
	       n1->netnum, n1->nodenum);
     }

     // TO-DO:  When failing to route a node, we need to check if (1) all nodes
     // are routed, (2) last node is not routed, or (3) more than one node is
     // not routed.  If (3), then set n1 to the first unrouted node.  Otherwise,
     // a large and critical net may not get routed at all because one of the
     // first legs wouldn't route.

     result = route_segs(rt1, stage);

     if ((result == 0) || (rt1->node == NULL)) {
        // Nodes already routed, nothing to do
	free(rt1);
	return 0;
     }
     n1 = rt1->node;

     if (n1->routes) {
        for (lrt = n1->routes; lrt->next; lrt = lrt->next);
	lrt->next = rt1;
     }
     else {
	n1->routes = rt1;
     }

     if (result < 0) {		// Route failure.
	nlist = (NETLIST)malloc(sizeof(struct netlist_));
	nlist->net = net;
	if (FailedNets != NULL)
	   nlist->idx = FailedNets->idx + 1;
	else
	   nlist->idx = 1;
	nlist->next = FailedNets;
	FailedNets = nlist;
	return -1;
     }
  }
  
} /* doroute() */

/*--------------------------------------------------------------*/
/* route_segs - detailed route from node to node using onestep	*/
/*	method   						*/
/*								*/
/*   ARGS: ROUTE, ready to add segments to do route		*/
/*   RETURNS: NULL if failed, manhattan distance if success	*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

int route_segs(ROUTE rt, u_char stage)
{
  POINT gpoint, glist, gunproc;
  NET  net;
  SEG  seg;
  struct seg_ bbox;
  int  i, j, k;
  int  x, y;
  NODE n1, n2, n2save;
  u_int netnum, dir;
  char filename[32];
  int  dist, max, min, maxcost;
  int  thisnetnum, thisindex, index, pass;
  int  bestX, bestY, bestL;
  int  prind;
  int  result;
  int  rval;
  u_char first = (u_char)1;
  DPOINT n1tap, n2tap;

  // Make Obs2[][] a copy of Obs[][].  Convert pin obstructions to
  // terminal positions for the net being routed.

  for (i = 0; i < Num_layers; i++) {
      for (x = 0; x < NumChannelsX[i]; x++) {
	  for (y = 0; y < NumChannelsY[i]; y++) {
	      netnum = Obs[i][OGRID(x, y, i)];
	      Obs2[i][OGRID(x, y, i)] = netnum;
	      dir = netnum & PINOBSTRUCTMASK;
	      if ((dir != 0) && (dir != STUBROUTE_X)) {
		 if ((netnum & ~PINOBSTRUCTMASK) == rt->netnum)
		    Obs2[i][OGRID(x, y, i)] = rt->netnum;
	      }
	  }
      }
  }

  PRind = 0;  	// Start assigning indexes after this.  Don't confuse Pr[1]
		// with source, Pr[2] with dest:  Source never gets assigned
		// a Pr[], and dest gets its own number unrelated to TARGNETNUM.

  // Pr[0] will be used to hold information about the route cost at target
  // Note that it will be placed at whichever tap has the lowest cost route.

  Pr[0].pred = 0;
  Pr[0].alt = 0;
  Pr[0].flags = (u_char)0;
  Pr[0].cost = MAXRT;

  // Stack of points to search
  glist = (POINT)NULL;
  gunproc = (POINT)NULL;

  // We start at the node referenced by the route structure, and set all
  // of its taps to SRCNETNUM, as well as all connected routes.

  n1 = rt->node;
  bbox.x2 = bbox.y2 = 0;
  bbox.x1 = NumChannelsX[0];
  bbox.y1 = NumChannelsY[0];
  set_node_to_net(n1, SRCNETNUM, &glist, &bbox);

  // Now search for all other nodes on the same net that have not yet been
  // routed, and set all of their taps to TARGNETNUM

  result = 0;
  for (n2 = Nlnodes; n2; n2 = n2->next) {
     if (n2 == n1) continue;
     if (n2->netnum != n1->netnum) continue;
     if (set_node_to_net(n2, TARGNETNUM, NULL, &bbox) == 0) {
	n2save = n2;
	result = 1;
     }
  }

  // Check for the possibility that there is already a route to the target
  if (!result) {
     fprintf(stdout, "Finished routing net %s\n", n1->netname);

     // Remove nodes of the net from Nodeloc so that they will not be
     // used for crossover costing of future routes.

     for (i = 0; i < Num_layers; i++) {
        for (x = 0; x < NumChannelsX[i]; x++) {
	   for (y = 0; y < NumChannelsY[i]; y++) {
	      n1 = Nodeloc[i][OGRID(x, y, i)];
	      if (n1 != (NODE)NULL)
		 if (n1->netnum == rt->netnum)
		    Nodeloc[i][OGRID(x, y, i)] = (NODE)NULL;
	   }
        }
     }

     while (glist) {
	gpoint = glist;
	glist = glist->next;
	free(gpoint);
     }
     return 0;
  }

  // Heuristic:  Set the initial cost beyond which we stop searching.
  // This value is twice the cost of a direct route across the
  // maximum extent of the source to target, divided by the number of
  // nodes in the net.  We purposely set this value low.  It has a
  // severe impact on the total run time of the algorithm.  If the
  // initial max cost is so low that no route can be found, it will
  // be doubled on each pass.

  maxcost = 2 * MAX((bbox.x2 - bbox.x1), (bbox.y2 - bbox.y1)) * SegCost +
		(int)stage * ConflictCost;
  maxcost /= (n1->numnodes - 1);

  netnum = rt->netnum;
  n2 = n2save;

  n1tap = n1->taps;
  n2tap = n2->taps;

  if ((n1tap == NULL && n1->extend == NULL) || (n2tap == NULL && n2->extend == NULL)) {
     printf("Node of net %s has no tap points---unable to route!\n", n1->netname);
     return -1;
  }
  if (n1tap == NULL) n1tap = n1->extend;
  if (n2tap == NULL) n2tap = n2->extend;

  printf("Source node @ %gum %gum layer=%d grid=(%d %d)\n",
	  n1tap->x, n1tap->y, n1tap->layer,
	  n1tap->gridx, n1tap->gridy);
  printf("Dest node @ %gum %gum layer=%d grid=(%d %d)\n",
	  n2tap->x, n2tap->y, n2tap->layer,
	  n2tap->gridx, n2tap->gridy);
  printf("netname = %s, route number %d\n", n1->netname, TotalRoutes );
  fflush(stdout);

  for (pass = 0; pass < Numpasses; pass++) {
    CurrentPass = pass;

    if (!first) {
       fprintf(stdout, "\n");
       first = (u_char)1;
    }
    fprintf(stdout, "Pass %d\n", pass);

    while (gpoint = glist) {

      NPX = gpoint->x1;
      NPY = gpoint->y1;
      CurrentLay = gpoint->layer;
      glist = gpoint->next;

      thisnetnum = Obs2[CurrentLay][OGRID(NPX, NPY, CurrentLay)];
      if (thisnetnum & SRCFLAG) {
	 thisindex = (thisnetnum & ~SRCFLAG);
	 thisnetnum = SRCNETNUM;
      }
      else if (thisnetnum & RTFLAG) {
         thisindex = (thisnetnum & ~RTFLAG);
      }
      else
	 thisindex = 0;

      // ignore grid positions that have already been processed
      if (thisindex && (Pr[thisindex].flags & PR_PROCESSED)) {
	 free(gpoint);
	 continue;
      }

      // if the grid position is the destination, save the position and
      // cost if minimum.

      if (thisnetnum == TARGNETNUM) {

	 if (first) {
	    fprintf(stdout, "Found a route of cost ");
	    first = (u_char)0;
	 }
	 else
	    fprintf(stdout, "|");
	 fprintf(stdout, "%d", Pr[0].cost);
	 fflush(stdout);

	 // This position may be on a route, not at a terminal, so
	 // record it.
	 bestX = NPX;
	 bestY = NPY;
	 bestL = CurrentLay;

	 // If a complete route has been found, then there's no point
	 // in searching paths with a greater cost than this one.
	 if (Pr[0].cost < maxcost) maxcost = Pr[0].cost;

         // Don't continue processing from the target
	 free(gpoint);
	 continue;
      }

      // Quick check:  Limit maximum cost to limit search space
      // Move the point onto the "unprocessed" stack and we'll pick up
      // from this point on the next pass, if needed.

      if (thisnetnum != SRCNETNUM) {
         if (Pr[thisindex].cost > maxcost && Pr[thisindex].cost < MAXRT) {
	    gpoint->next = gunproc;
	    gunproc = gpoint;
	    continue;
	 }
      }
      free(gpoint);

      // check east/west/north/south, and bottom to top

      min = MAXRT;
      if ((NPX + 1) < NumChannelsX[CurrentLay]) {
	if ((result = eval_pt(NPX + 1, NPY, CurrentLay, thisindex, stage)) == 1) {
	   gpoint = (POINT)malloc(sizeof(struct point_));
	   gpoint->x1 = NPX + 1;
	   gpoint->y1 = NPY;
	   gpoint->layer = CurrentLay;
	   gpoint->next = glist;
	   glist = gpoint;
        }
      }

      if ((NPX - 1) >= 0) {
	if ((result = eval_pt(NPX - 1, NPY, CurrentLay, thisindex, stage)) == 1) {
	   gpoint = (POINT)malloc(sizeof(struct point_));
	   gpoint->x1 = NPX - 1;
	   gpoint->y1 = NPY;
	   gpoint->layer = CurrentLay;
	   gpoint->next = glist;
	   glist = gpoint;
        }
      }

      if ((NPY - 1) >= 0) {
	if ((result = eval_pt(NPX, NPY - 1, CurrentLay, thisindex, stage)) == 1) {
	   gpoint = (POINT)malloc(sizeof(struct point_));
	   gpoint->x1 = NPX;
	   gpoint->y1 = NPY - 1;
	   gpoint->layer = CurrentLay;
	   gpoint->next = glist;
	   glist = gpoint;
        }
      }

      if ((NPY + 1) < NumChannelsY[CurrentLay]) {
	if ((result = eval_pt(NPX, NPY + 1, CurrentLay, thisindex, stage)) == 1) {
	   gpoint = (POINT)malloc(sizeof(struct point_));
	   gpoint->x1 = NPX;
	   gpoint->y1 = NPY + 1;
	   gpoint->layer = CurrentLay;
	   gpoint->next = glist;
	   glist = gpoint;
        }
      }
      
      if (CurrentLay > 0) {
	 if ((result = eval_pt(NPX, NPY, CurrentLay - 1, thisindex, stage)) == 1) {
	    gpoint = (POINT)malloc(sizeof(struct point_));
	    gpoint->x1 = NPX;
	    gpoint->y1 = NPY;
	    gpoint->layer = CurrentLay - 1;
	    gpoint->next = glist;
	    glist = gpoint;
	 }
      }

      if (CurrentLay < (Num_layers - 1)) {
	 if ((result = eval_pt(NPX, NPY, CurrentLay + 1, thisindex, stage)) == 1) {
	    gpoint = (POINT)malloc(sizeof(struct point_));
	    gpoint->x1 = NPX;
	    gpoint->y1 = NPY;
	    gpoint->layer = CurrentLay + 1;
	    gpoint->next = glist;
	    glist = gpoint;
	 }
      }

      // Mark this node as processed
      Pr[thisindex].flags |= PR_PROCESSED;

      if (PRind >= PRindMAX) {
	 fprintf(stdout, "\n");
	 fflush(stdout);
	 fprintf(stderr, "route.c:route_segs().  PRind hit "
			"PRindMAX\nThis is a serious error\n");
	 if (!Failfptr) openFailFile();
	 fprintf(Failfptr, "# PRind hit PRindMAX\n");
	 fprintf(Failfptr, "(%g,%g) <==> (%g,%g) net=%s\tRoute=%d\n",
		     n1tap->gridx * PitchX[0], n1tap->gridy * PitchY[0], 
		     n2tap->gridx * PitchX[0], n2tap->gridy * PitchY[0],
		     n1->netname, TotalRoutes);
	 fprintf(CNfptr, "# PRind hit PRindMAX\nRoute Priority\t%s\n",
			n1->netname);
	 fflush(CNfptr);
	 fflush(Failfptr);
	    
	 while (glist) {
	    gpoint = glist;
	    glist = glist->next;
	    free(gpoint);
	 }
	 rval = -1;
	 goto done;
      }
    } // while stack is not empty

    while (glist) {
       gpoint = glist;
       glist = glist->next;
       free(gpoint);
    }

    // If we found a route, save it and return

    if (Pr[0].cost <= maxcost) {
	fprintf(stdout, "\nCommit to a route of cost %d\n", Pr[0].cost);
	NPX = bestX;
	NPY = bestY;
	CurrentLay = bestL;
	Obs2[CurrentLay][OGRID(NPX, NPY, CurrentLay)] = RTFLAG;
	commit_proute(rt, stage);
	rval = 1;
	goto done;	/* route success */
    }

    // Only continue loop to next pass if we failed to find a route.
    // Increase maximum cost for next pass.

    maxcost <<= 1;

    if (gunproc == NULL) break;		// route failure not due to limiting
					// search to maxcost

    // Regenerate the stack of unprocessed nodes
    glist = gunproc;
    gunproc = NULL;
    
  } // pass
  
  if (!first) fprintf(stdout, "\n");
  fflush(stdout);
  fprintf(stderr, "Fell through %d passes\n", pass);
  fprintf(stderr, "(%g,%g) <==> (%g,%g) net=%s\n",
	   n1tap->x, n1tap->y, n2tap->x, n2tap->y, n1->netname);
  if (!Failfptr) openFailFile();
  fprintf(Failfptr, "(%g,%g) <==> (%g,%g) net=%s\tRoute=%d\n",
	   n1tap->x, n1tap->y, n2tap->x, n2tap->y, n1->netname, TotalRoutes);
  fprintf(CNfptr, "Route Priority\t%s\n", n1->netname);
  fflush(CNfptr);
  fflush(Failfptr);
  
  rval = -1;

done:
  
  while (gunproc) {
     gpoint = gunproc;
     gunproc = gunproc->next;
     free(gpoint);
  }
  return rval;
  
} /* route_segs() */

/*--------------------------------------------------------------*/
/* createemptyroute - begin a ROUTE structure			*/
/*								*/
/*   ARGS: a nodes						*/
/*   RETURNS: ROUTE calloc'd and ready to begin			*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

ROUTE createemptyroute()
{
  ROUTE rt;

  rt = (ROUTE)calloc(1, sizeof(struct route_));
  rt->netnum = 0;
  rt->segments = (SEG)NULL;
  rt->node = (NODE)NULL;
  rt->output = FALSE;
  rt->next = (ROUTE)NULL;
  return rt;

} /* createemptyroute() */

/*--------------------------------------------------------------*/
/* emit_routed_net --						*/
/*								*/
/* Core part of emit_routes().  Dumps the DEF format for a	*/
/* complete net route to file Cmd.  If "special" is TRUE, then	*/
/* it looks only for stub routes between a grid point and an	*/
/* off-grid terminal, and dumps only the stub route geometry as	*/
/* a SPECIALNET, which takes a width parameter.  This allows	*/
/* the stub routes to be given the same width as a via, when	*/
/* the via is larger than a route width, to avoid DRC notch	*/
/* errors between the via and the terminal.  The SPECIALNETS	*/
/* are redundant;  all routing information is in the NETS	*/
/* section.  The SPECIALNETS only specify a wider route for the	*/
/* stub route.							*/
/*								*/
/* Return 1 if a stub route was encountered, so that we can	*/
/* determine if it is necessary to write a SPECIALNETS section	*/
/*--------------------------------------------------------------*/

int
emit_routed_net(FILE *Cmd, NET net, u_char special)
{
   SEG seg, saveseg, lastseg;
   ROUTE rt;
   NODE node;
   GATE g;
   u_int dir;
   int i, layer;
   int x, y, x2, y2;
   double dc;
   int horizontal, vertical;
   DPOINT dp1, dp2;
   int stubroute = 0;

   Pathon = -1;

   /* Insert routed net here */
   for (node = Nlnodes; node; node = node->next) {
      if (node->netnum == net->netnum) {
	 for (rt = node->routes; rt; rt = rt->next) {
	    if (rt->segments && !rt->output && rt->node) {
		vertical = horizontal = FALSE;

		// Check first position for terminal offsets
		seg = (SEG)rt->segments;
		lastseg = saveseg = seg;
		if (seg) {
		   dir = Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)];
		   dir &= PINOBSTRUCTMASK;
		   if (dir) {
		      layer = seg->layer;
		      if (special == (u_char)0)
		         fprintf(stdout, "Stub route distance %g to terminal"
				" at %d %d (%d)\n",
				Stub[layer][OGRID(seg->x1, seg->y1, layer)],
				seg->x1, seg->y1, layer);
		      stubroute = 1;
		      dc = Xlowerbound + (double)seg->x1 * PitchX[layer];
		      x = (int)((dc + 1e-4) * 100);
		      if (dir == STUBROUTE_EW)
			 dc += Stub[layer][OGRID(seg->x1, seg->y1, layer)];
		      x2 = (int)((dc + 1e-4) * 100);
		      dc = Ylowerbound + (double)seg->y1 * PitchY[layer];
		      y = (int)((dc + 1e-4) * 100);
		      if (dir == STUBROUTE_NS)
			 dc += Stub[layer][OGRID(seg->x1, seg->y1, layer)];
		      y2 = (int)((dc + 1e-4) * 100);
		      pathstart(Cmd, seg->layer, x2, y2, special);
		      if (dir == STUBROUTE_EW) {
			 vertical = FALSE;
			 horizontal = TRUE;
		      }
		      else {
			 vertical = TRUE;
			 horizontal = FALSE;
		      }
		      pathto(Cmd, x, y, horizontal, vertical);
		   }
		}

		for (seg = rt->segments; seg; seg = seg->next) {
		   layer = seg->layer;

		   // To do: pick up route layer name from lefInfo.
		   // At the moment, technology names don't even match,
		   // and are redundant between CIFLayer[] from the
		   // config file and lefInfo.

		   dc = Xlowerbound + (double)seg->x1 * PitchX[layer];
		   x = (int)((dc + 1e-4) * 100);
		   dc = Ylowerbound + (double)seg->y1 * PitchY[layer];
		   y = (int)((dc + 1e-4) * 100);
		   dc = Xlowerbound + (double)seg->x2 * PitchX[layer];
		   x2 = (int)((dc + 1e-4) * 100);
		   dc = Ylowerbound + (double)seg->y2 * PitchY[layer];
		   y2 = (int)((dc + 1e-4) * 100);
		   switch (seg->segtype) {
		      case ST_WIRE:
			 if (Pathon != 1) {	// 1st point of route seg
			    if (special == (u_char)0)
			       pathstart(Cmd, seg->layer, x, y, (u_char)0);
			    if (x == x2) {
				vertical = TRUE;
				horizontal = FALSE;
			    }
			    else if (y == y2) {
				vertical = FALSE;
				horizontal = TRUE;
			    }
			    else {
				fprintf(stderr, "Warning:  non-"
					"Manhattan wire in route\n");
			    }
			 }
			 rt->output = TRUE;
			 if (horizontal && x == x2) {
			    vertical = TRUE;
			    horizontal = FALSE;
			 }
			 if (vertical && y == y2) {
			    vertical = FALSE;
			    horizontal = TRUE;
			 }
			 if (!(x == x2) && !(y == y2)) {
			    vertical = FALSE;
			    horizontal = FALSE;
			 }
			 if (special == (u_char)0)
			    pathto(Cmd, x2, y2, horizontal, vertical);
			 break;
		      case ST_VIA:
			 rt->output = TRUE;
			 if (special == (u_char)0)
			    pathvia(Cmd, layer, x, y);
			 break;
		      default:
			 break;
		   }
		   lastseg = seg;
		}

		if (special == (u_char)1) Pathon = 0;

		// Check last position for terminal offsets
		if (lastseg && ((lastseg != saveseg)
				|| (lastseg->segtype == ST_WIRE))) {
		    seg = lastseg;
		    dir = Obs[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)];
		    dir &= PINOBSTRUCTMASK;
		    if (dir) {
		       layer = seg->layer;
		       if (special == (u_char)0)
		          fprintf(stdout, "Stub route distance %g to terminal"
				" at %d %d (%d)\n",
				Stub[layer][OGRID(seg->x2, seg->y2, layer)],
				seg->x2, seg->y2, layer);
		       stubroute = 1;
		       dc = Xlowerbound + (double)seg->x2 * PitchX[layer];
		       x = (int)((dc + 1e-4) * 100);
		       if (dir == STUBROUTE_EW)
			  dc += Stub[layer][OGRID(seg->x2, seg->y2, layer)];
		       x2 = (int)((dc + 1e-4) * 100);
		       dc = Ylowerbound + (double)seg->y2 * PitchY[layer];
		       y = (int)((dc + 1e-4) * 100);
		       if (dir == STUBROUTE_NS)
			  dc += Stub[layer][OGRID(seg->x2, seg->y2, layer)];
		       y2 = (int)((dc + 1e-4) * 100);
		       if (dir == STUBROUTE_EW) {
			  vertical = FALSE;
			  horizontal = TRUE;
		       }
		       else {
			  vertical = TRUE;
			  horizontal = FALSE;
		       }
		       if (Pathon != 1) pathstart(Cmd, layer, x, y, special);
		       pathto(Cmd, x2, y2, horizontal, vertical);
		    }
		}

		Pathon = 0;
	    } // if (rt->segments && !rt->output)
	 }
      } // if (node->routes)
   }
   return stubroute;
}

/*--------------------------------------------------------------*/
/* emit_routes - DEF file output from the Nlnodes list		*/
/*								*/
/*  Reads the <project>.def file and rewrites file		*/
/*  <project>_route.def, where each net definition has the	*/
/*  physical route appended.					*/
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Mon Aug 11 2003		*/
/*--------------------------------------------------------------*/

void emit_routes(char *filename)
{
    FILE *Cmd;
    int i, j, numnets;
    char line[MAXLINE + 1], *lptr;
    char netname[MAX_NAME_LEN];
    NET net;
    NODE node;
    ROUTE rt;
    char newDEFfile[256];
    FILE *fdef;
    int stubroute = 0;

    fdef = fopen(filename, "r");
    if (fdef == NULL) {
       fprintf(stderr, "emit_routes(): Cannot open DEF file for reading.\n");
       return;
    } 

    if (!strcmp(filename, "stdout")) {
	Cmd = stdout;
    }
    else {
	char *dotptr;

	strcpy(newDEFfile, filename);
	dotptr = strrchr(newDEFfile, '.');
	strcpy(dotptr, "_route.def");
	Cmd = fopen(newDEFfile, "w");
    }
    if (!Cmd) {
	fprintf(stderr, "emit_routes():  Couldn't open output (routed) DEF file.\n");
	return;
    }

    // Copy DEF file up to NETS line
    numnets = 0;
    while (fgets(line, MAXLINE, fdef) != NULL) {
       lptr = line;
       while (isspace(*lptr)) lptr++;
       if (!strncmp(lptr, "NETS", 4)) {
	  sscanf(lptr + 4, "%d", &numnets);
	  break;
       }
       fputs(line, Cmd);
    }
    fputs(line, Cmd);	// Write the NETS line
    if (numnets != (Numnets - MIN_NET_NUMBER)) {
        fprintf(stderr, "emit_routes():  DEF file has %d nets, but we want"
		" to write %d\n", numnets, Numnets - MIN_NET_NUMBER);
	if (numnets > Numnets) numnets = Numnets;
    }

    for (i = 0; i < numnets; i++) {
       netname[0] == '\0';
       while (fgets(line, MAXLINE, fdef) != NULL) {
	  if ((lptr = strchr(line, ';')) != NULL) {
	     *lptr = '\n';
	     *(lptr + 1) = '\0';
	     break;
	  }
	  else {
             lptr = line;
             while (isspace(*lptr)) lptr++;
	     if (*lptr == '-') {
		lptr++;
                while (isspace(*lptr)) lptr++;
	        sscanf(lptr, "%s", netname);
	     }
	     fputs(line, Cmd);
	  }
       }

       /* Find this net */

       for (net = Nlnets; net; net = net->next) {
	  if (!strcmp(net->netname, netname))
	     break;
       }
       if (!net) {
	  fprintf(stderr, "emit_routes():  Net %s cannot be found.\n",
		netname);

	  /* Dump rest of net and continue---no routing information */
	  *(lptr) = ';';
	  fputs(line, Cmd);
	  continue;
       }
       else {
	  /* Add last net terminal, without the semicolon */
	  fputs(line, Cmd);

	  stubroute += emit_routed_net(Cmd, net, (u_char)0);
	  fprintf(Cmd, ";\n");
       }
    }

    // Finish copying the rest of the NETS section
    while (fgets(line, MAXLINE, fdef) != NULL) {
       lptr = line;
       while (isspace(*lptr)) lptr++;
       fputs(line, Cmd);
       if (!strncmp(lptr, "END", 3)) {
	  break;
       }
    }

    // If there were stub routes, repeat them in SPECIALNETS at the
    // proper width.
    if (stubroute > 0) {

       /* Reset "output" records on routes, for next pass */
       for (node = Nlnodes; node; node = node->next)
	  for (rt = node->routes; rt; rt = rt->next)
	     rt->output = 0;

       fprintf(Cmd, "\nSPECIALNETS %d ;\n", stubroute);
       fprintf(Cmd, "- stubroute1\n");
       net = Nlnets;
       for (i = 0; i < stubroute; i++) {
          for (; net; net = net->next) {
	     j = emit_routed_net(Cmd, net, (u_char)1);
	     if (j > 0 && i < stubroute - 1)
			fprintf(Cmd, " ;\n- stubroute%d\n", i + 1);
		     if (j > 0) break;
		  }
	       }
	       fprintf(Cmd, " ;\nEND SPECIALNETS\n");
	    }    

	    // fprintf(Cmd, "# Autogenerated by qrouter version %s\n", VERSION);

	    // Finish copying the rest of the file
	    while (fgets(line, MAXLINE, fdef) != NULL) {
	       fputs(line, Cmd);
	    }
	    fclose(fdef);
	    fclose(Cmd);

	} /* emit_routes() */

	/*--------------------------------------------------------------*/
	/* helpmessage - tell user how to use the program		*/
	/*								*/
	/*   ARGS: none.						*/
	/*   RETURNS: nothing.						*/
	/*   SIDE EFFECTS: 						*/
	/*--------------------------------------------------------------*/

	void helpmessage()
	{
	    fprintf(stdout, "qrouter - maze router by Tim Edwards\n\n");
	    fprintf(stdout, "usage:  qrouter [-switchs] design_name\n\n");
	    fprintf(stdout, "switches:\n");
	    fprintf(stdout, "\t-c <file>\t\t\tConfiguration file name if not route.cfg.\n");
	    fprintf(stdout, "\t-v <level>\t\t\tVerbose output level.\n");
	    fprintf(stdout, "\t-i <file>\t\t\tPrint route names and pitches and exit.\n");
	    // fprintf(stdout, "\t-d\t\t\tDebug file output.\n");
	    fprintf(stdout, "\n");
	    fprintf(stdout, "%s\n", VERSION);

	} /* helpmessage() */

/* end of qrouter.c */
