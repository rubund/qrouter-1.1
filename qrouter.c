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
NETLIST FailedNets;	// list of nets that failed to route
NETLIST Abandoned;	// list of nets that will never route

u_char *Mask[MAX_LAYERS];    // mask out best area to route, expand as needed
u_int  *Obs[MAX_LAYERS];     // net obstructions in layer
PROUTE *Obs2[MAX_LAYERS];    // used for pt->pt routes on layer
float  *Stub[MAX_LAYERS];    // used for stub routing to pins
float  *Obsinfo[MAX_LAYERS]; // temporary array used for detailed obstruction info
NODE   *Nodeloc[MAX_LAYERS]; // nodes are here. . .
NODE   *Nodesav[MAX_LAYERS]; // . . . and here (but not to be altered)
DSEG   UserObs;		     // user-defined obstruction layers

u_char needblockX[MAX_LAYERS];
u_char needblockY[MAX_LAYERS];

char *vddnet = NULL;
char *gndnet = NULL;

int   Numnets = 0;
int   Numgates = 0;
int   Numpins = 0;
int   Verbose = 0;

int   pwrbus_src;

/*--------------------------------------------------------------*/
/* Open the "failed" and "cn" (critical nets) files.		*/
/*--------------------------------------------------------------*/

void openFailFile()
{
   Failfptr = fopen("failed", "w");
   if (!Failfptr) {
      fprintf(stderr, "Warning: Could not open file \"failed\"\n");
   }

   CNfptr = fopen("cn", "w");
   if (!CNfptr) {
      fprintf(stderr, "Warning: Could not open file \"cn\".\n" );
   }
}
    
/*--------------------------------------------------------------*/
/* Check track pitch and set the number of channels (may be	*/
/* called from DefRead)						*/
/*--------------------------------------------------------------*/

int set_num_channels()
{
   int i;

   if (NumChannelsX[0] != 0) return;	/* Already been called */

   for (i = 0; i < Num_layers; i++) {
      if (PitchX[i] == 0.0 || PitchY[i] == 0.0) {
	 fprintf(stderr, "Have a 0 pitch for layer %d (of %d).  "
			"Exit.\n", i + 1, Num_layers);
	 return (-3);
      }
      NumChannelsX[i] = (int)(1.5 + (Xupperbound - Xlowerbound) / PitchX[i]);
      NumChannelsY[i] = (int)(1.5 + (Yupperbound - Ylowerbound) / PitchY[i]);
      printf("Number of x channels for layer %d is %d\n",
		i, NumChannelsX[i]);
      printf("Number of y channels for layer %d is %d\n",
		i, NumChannelsY[i]);
	
      if (NumChannelsX[i] <= 0) {
	 fprintf(stderr, "Something wrong with layer %d x bounds.\n", i);
	 return(-3);
      }
      if (NumChannelsY[i] <= 0) {
	 fprintf(stderr, "Something wrong with layer %d y bounds.\n", i);
	 return(-3);
      }
      fflush(stdout);
   }
   return 0;
}

/*--------------------------------------------------------------*/
/* Allocate the Obs[] array (may be called from DefRead)	*/
/*--------------------------------------------------------------*/

int allocate_obs_array()
{
   int i;

   if (Obs[0] != NULL) return;	/* Already been called */

   for (i = 0; i < Num_layers; i++) {
      Obs[i] = (u_int *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(u_int));
      if (!Obs[i]) {
	 fprintf(stderr, "Out of memory 4.\n");
	 return(4);
      }
   }
   return 0;
}

/*--------------------------------------------------------------*/
/* countlist ---						*/
/*   Count the number of entries in a simple linked list	*/
/*--------------------------------------------------------------*/

int
countlist(NETLIST net)
{
   NETLIST nptr = net;
   int count = 0;

   while (nptr != NULL) {
      count++;
      nptr = nptr->next;
   }
   return count;
}

/*--------------------------------------------------------------*/
/* main - program entry point, parse command line		*/
/*								*/
/*   ARGS: argc (count) argv, command line 			*/
/*   RETURNS: to OS						*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

int
main(int argc, char *argv[])
{
   int	i, j, result;
   int length, width;
   FILE *l, *configFILEptr, *fptr;
   NETLIST nl;
   u_int u;
   static char configdefault[] = CONFIGFILENAME;
   char *configfile = configdefault;
   char *infofile = NULL;
   char *dotptr, *sptr;
   char DEFfilename[256];
   char Filename[256];
   double oscale, sreq;

   NET net;
    
   Filename[0] = 0;
   DEFfilename[0] = 0;

   while ((i = getopt(argc, argv, "c:i:hv:p:g:")) != -1) {
      switch (i) {
	 case 'c':
	    configfile = strdup(optarg);
	    break;
	 case 'v':
	    Verbose = atoi(optarg);
	    break;
	 case 'i':
	    infofile = strdup(optarg);
	    break;
	 case 'p':
	    vddnet = strdup(optarg);
	    break;
	 case 'g':
	    gndnet = strdup(optarg);
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
      configtmp = malloc(strlen(configfile) + strlen(QROUTER_LIB_DIR) + 2);
      sprintf(configtmp, "%s/%s", QROUTER_LIB_DIR, configfile);
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
	    char *layername = LefGetRouteName(i);
	    if (layername != NULL)
	       fprintf(infoFILEptr, "%s %g %g %g %s\n",
			layername, LefGetRoutePitch(i),
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

   Obs[0] = (u_int *)NULL;
   NumChannelsX[0] = 0;	// This is so we can check if NumChannelsX/Y were
			// set from within DefRead() due to reading in
			// existing nets.

   oscale = (double)DefRead(DEFfilename);
   create_netorder();

   set_num_channels();		// If not called from DefRead()
   allocate_obs_array();	// If not called from DefRead()

   for (i = 0; i < Num_layers; i++) {

      /*
      Mask[i] = (u_char *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(u_char));
      if (!Mask[i]) {
	 fprintf(stderr, "Out of memory 3.\n");
	 exit(3);
      }
      */

      Obsinfo[i] = (float *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(float));
      if (!Obsinfo[i]) {
	 fprintf(stderr, "Out of memory 5.\n");
	 exit(5);
      }

      Stub[i] = (float *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(float));
      if (!Stub[i]) {
	 fprintf( stderr, "Out of memory 6.\n");
	 exit(6);
      }

      // Nodeloc is the reverse lookup table for nodes

      Nodeloc[i] = (NODE *)calloc(NumChannelsX[i] * NumChannelsY[i],
		sizeof(NODE));
      if (!Nodeloc[i]) {
         fprintf(stderr, "Out of memory 7.\n");
         exit(7);
      }

      Nodesav[i] = (NODE *)calloc(NumChannelsX[i] * NumChannelsY[i],
		sizeof(NODE));
      if (!Nodesav[i]) {
         fprintf(stderr, "Out of memory 8.\n");
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
   tap_to_tap_interactions();
   create_obstructions_from_variable_pitch();
   adjust_stub_lengths();
   find_route_blocks();

   // Remove the Obsinfo array, which is no longer needed, and allocate
   // the Obs2 array for costing information

   for (i = 0; i < Num_layers; i++) free(Obsinfo[i]);

   for (i = 0; i < Num_layers; i++) {
      Obs2[i] = (PROUTE *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(PROUTE));
      if (!Obs2[i]) {
         fprintf( stderr, "Out of memory 9.\n");
         exit(9);
      }
   }

   // Fill in needblockX and needblockY, which are used by commit_proute
   // when route layers are too large for the grid size, and grid points
   // around a route need to be marked as blocked whenever something is
   // routed on those layers.

   for (i = 0; i < Num_layers; i++) {
      sreq = LefGetRouteWidth(i) + LefGetRouteSpacing(i);
      needblockX[i] = (sreq > PitchX[i]) ? TRUE : FALSE;
      needblockY[i] = (sreq > PitchY[i]) ? TRUE : FALSE;
   }

   // Now we have netlist data, and can use it to get a list of nets.

   FailedNets = (NETLIST)NULL;
   Abandoned = (NETLIST)NULL;
   fflush(stdout);
   fprintf(stderr, "Numnets = %d, Numpins = %d\n",
	     Numnets - MIN_NET_NUMBER, Numpins );

   // print_nlgates( "net.details" );
   // print_nodes( "nodes.details" );
   // print_nlnets( "netlist.out" );

   for (i = 0; i < Numnets; i++) {
      net = getnettoroute(i);
      if ((net != NULL) && (net->netnodes != NULL))
	 doroute(net, (u_char)0);
   }

   fflush(stdout);
   fprintf(stdout, "\n----------------------------------------------\n");
   fprintf(stdout, "Progress: ");
   fprintf(stdout, "Total routing loops completed: %d\n", TotalRoutes);
   if (FailedNets == (NETLIST)NULL && Abandoned == (NETLIST)NULL)
      fprintf(stdout, "No failed routes!\n");
   else {
      if (FailedNets != (NETLIST)NULL)
          fprintf(stdout, "Failed net routes: %d\n", countlist(FailedNets));
      if (Abandoned != (NETLIST)NULL)
          fprintf(stdout, "Abandoned net routes: %d\n", countlist(Abandoned));
   }
   fprintf(stdout, "----------------------------------------------\n");

   dosecondstage();

   // Finish up by writing the routes to an annotated DEF file
    
   emit_routes(DEFfilename, oscale);

   fprintf(stdout, "----------------------------------------------\n");
   fprintf(stdout, "Final: ");
   if (FailedNets == (NETLIST)NULL && Abandoned == (NETLIST)NULL)
      fprintf(stdout, "No failed routes!\n");
   else {
      if (FailedNets != (NETLIST)NULL) {
         fprintf(stdout, "Failed net routes: %d\n", countlist(FailedNets));
	 fprintf(stdout, "List of failed nets follows:\n");

	 // Make sure FailedNets is cleaned up as we output the failed nets

	 if (Failfptr)
	    fprintf(Failfptr, "\n------------------------------------------\n");
	 if (!Failfptr) openFailFile();

 	 while (FailedNets) {
	    net = FailedNets->net;
	    fprintf(Failfptr, "Final:  Failed to route net %s\n", net->netname);
	    fprintf(stdout, " %s\n", net->netname);
	    nl = FailedNets->next;
	    free(FailedNets);
	    FailedNets = nl;
	 }
	 fprintf(stdout, "\n");
      }
      if (Abandoned != (NETLIST)NULL) {
         fprintf(stdout, "Abandoned net routes: %d\n", countlist(Abandoned));
	 fprintf(stdout, "List of abandoned nets follows:\n");

	 if (Failfptr)
	    fprintf(Failfptr, "\n------------------------------------------\n");
	 if (!Failfptr) openFailFile();

	 // Make sure Abandoned is cleaned up as we output the failed nets

	 while (Abandoned) {
	    net = Abandoned->net;
	    fprintf(Failfptr, "Final:  Abandoned net %s\n", net->netname);
	    fprintf(stdout, " %s\n", net->netname);
	    nl = Abandoned->next;
	    free(Abandoned);
	    Abandoned = nl;
	 }
	 fprintf(stdout, "\n");
      }
   }
   fprintf(stdout, "----------------------------------------------\n");

   if (Failfptr) fclose(Failfptr);
   if (CNfptr) fclose(CNfptr);

   exit(0);

} /* main() */

/*--------------------------------------------------------------*/
/* pathstart - begin a DEF format route path           		*/
/*								*/
/* 	If "special" is true, then this path is in a		*/
/*	SPECIALNETS section, in which each route specifies	*/
/*	a width.						*/
/*--------------------------------------------------------------*/

void pathstart(FILE *cmd, int layer, int x, int y, u_char special, double oscale)
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
			(int)(oscale * LefGetViaWidth(layer, layer, 0) + 0.5),
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

void pathto(FILE *cmd, int x, int y, int horizontal, int lastx, int lasty)
{
    if (Pathon <= 0) {
	fprintf(stderr, "pathto():  Major error.  Added to a "
		"non-existent path!\n"
		"Doing it anyway.\n");
    }

    /* If the route is not manhattan, then it's because an offset
     * was added to the last point, and we need to add a small
     * jog to the route.
     */

    if ((x != lastx) && (y != lasty)) {
	if (horizontal)
	   pathto(cmd, x, y, FALSE, x, lasty);
	else
	   pathto(cmd, x, y, TRUE, lastx, y);
    }

    fprintf(cmd, "( ");
    if (horizontal)
	fprintf(cmd, "%d ", x);
    else
	fprintf(cmd, "* ");

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

void pathvia(FILE *cmd, int layer, int x, int y, int lastx, int lasty)
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
    else {
       if (x != lastx)
	  pathto(cmd, x, y, TRUE, lastx, y);
       else if (y != lasty)
	  pathto(cmd, x, y, FALSE, x, lasty);
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
	// Qrouter will route power and ground nets even if the
	// standard cell power and ground pins are not listed in
	// the nets section.  Because of this, it is okay to have
	// only one node.
	else if (net->numnodes == 1 && (net->netnum == VDD_NET ||
		net->netnum == GND_NET))
	   return net;
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
   int failcount, origcount, result, maxtries, lasttries;
   NET net;
   NETLIST nl, nl2, fn;

   origcount = countlist(FailedNets);
   if (FailedNets)
      maxtries = TotalRoutes + ((origcount < 20) ? 20 : origcount) * 8;
   else
      maxtries = 0;

   while (FailedNets != NULL) {

      // Diagnostic:  how are we doing?
      failcount = countlist(FailedNets);
      fprintf(stdout, "------------------------------\n");
      fprintf(stdout, "Number of remaining nets: %d\n", failcount);
      fprintf(stdout, "------------------------------\n");

      net = FailedNets->net;

      // Remove this net from the fail list
      nl2 = FailedNets;
      FailedNets = FailedNets->next;
      free(nl2);

      // Route as much as possible without collisions
      result = doroute(net, (u_char)0);

      if (result != 0) {
	 fflush(stdout);
	 fprintf(stderr, "Routing net %s with collisions\n", net->netname);
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
	    fflush(stdout);
	    fprintf(stderr, "----------------------------------------------\n");
	    fprintf(stderr, "Complete failure on net %s:  Abandoning.\n",
			net->netname);
	    fprintf(stderr, "----------------------------------------------\n");
	    // Add the net to the "abandoned" list
	    nl = (NETLIST)malloc(sizeof(struct netlist_));
	    nl->net = net;
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
	 if (ripup_net(nl->net, (u_char)1) == TRUE) { 
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
	 }

	 nl->next = (NETLIST)NULL;
	 nl = nl2;
      }

      // Now we copy the net we routed above into Obs
      writeback_all_routes(net);

      // Failsafe---if we have been looping enough times to exceed
      // maxtries (which is set to 8 route attempts per original failed
      // net), then we check progress.  If we have reduced the number
      // of failed nets by half or more, then we have an indication of
      // real progress, and will continue.  If not, we give up.  Qrouter
      // is almost certainly hopelessly stuck at this point.

      if (TotalRoutes >= maxtries) {
	 if (failcount <= (origcount / 2)) {
	    maxtries = TotalRoutes + failcount * 8;
	    origcount = failcount;
	 }
	 else {
	    fprintf(stderr, "\nQrouter is stuck, abandoning remaining routes.\n");
	    break;
	 }
      }
   }
}

/*--------------------------------------------------------------*/
/* createMask() ---						*/
/*								*/
/* Create mask of optimal area to route.			*/
/* For 2-node routes, find the two L-shaped routes between the	*/
/* two closest points of the nodes.				*/
/* For multi-node (>2) routes, find the best trunk line that	*/
/* passes close to all nodes, and generate stems to the closest	*/
/* point on each node.						*/
/*--------------------------------------------------------------*/

void createMask(NET net)
{
  NODE n1, n2;
  int i, j, o, l;
  DPOINT dtap, d1tap, d2tap, mintap;
  int dx, dy, dist, mindist;
  int x1, x2, y1, y2;
  int xcent, ycent, xmin, ymin, xmax, ymax;

  fillMask(0);

  if (net->numnodes == 2) { 

     n1 = (NODE)net->netnodes;
     n2 = (NODE)net->netnodes->next;
     mindist = MAXRT;

     // Simple 2-pass---pick up first tap on n1, find closest tap on n2,
     // then find closest tap on n1.
     d1tap = (n1->taps == NULL) ? n1->extend : n1->taps;
     for (d2tap = (n2->taps == NULL) ? n2->extend : n2->taps; d2tap != NULL;
		d2tap = d2tap->next) {
	dx = d2tap->gridx - d1tap->gridx;
	dy = d2tap->gridy - d1tap->gridy;
	dist = dx * dx + dy * dy;
	if (dist < mindist) {
	   mindist = dist;
	   mintap = d2tap;
	}
     }
     d2tap = mintap;
     mindist = MAXRT;
     for (d1tap = (n1->taps == NULL) ? n1->extend : n1->taps; d1tap != NULL;
		d1tap = d1tap->next) {
	dx = d2tap->gridx - d1tap->gridx;
	dy = d2tap->gridy - d1tap->gridy;
	dist = dx * dx + dy * dy;
	if (dist < mindist) {
	   mindist = dist;
	   mintap = d1tap;
	}
     }
     d1tap = mintap;

     // Now construct a mask of two L-routes from d1tap to d2tap, 3 tracks width

     x1 = (d1tap->gridx < d2tap->gridx) ? d1tap->gridx : d2tap->gridx;
     x2 = (d1tap->gridx < d2tap->gridx) ? d2tap->gridx : d1tap->gridx;
     y1 = (d1tap->gridy < d2tap->gridy) ? d1tap->gridy : d2tap->gridy;
     y2 = (d1tap->gridy < d2tap->gridy) ? d2tap->gridy : d1tap->gridy;

     l = (d1tap->layer < d2tap->layer) ? d1tap->layer : d2tap->layer;

     // Place a track on every tap and extend position
     for (d1tap = n1->taps; d1tap != NULL; d1tap = d1tap->next)
	Mask[d1tap->layer][OGRID(d1tap->x, d1tap->y, d1tap->layer)] = (u_char)1;
     for (d1tap = n1->extend; d1tap != NULL; d1tap = d1tap->next)
	Mask[d1tap->layer][OGRID(d1tap->x, d1tap->y, d1tap->layer)] = (u_char)1;
     for (d2tap = n2->taps; d2tap != NULL; d2tap = d2tap->next)
	Mask[d2tap->layer][OGRID(d2tap->x, d2tap->y, d2tap->layer)] = (u_char)1;
     for (d2tap = n2->extend; d2tap != NULL; d2tap = d2tap->next)
	Mask[d2tap->layer][OGRID(d2tap->x, d2tap->y, d2tap->layer)] = (u_char)1;

     // Find the orientation of the lowest tap layer.  Lay alternate vertical
     // and horizontal tracks according to track orientation.

     for (; l < Num_layers; l++) {
        o = LefGetRouteOrientation(l);
	if (!o) {
           for (i = x1 - 1; i <= x1 + 1; i++)
	      for (j = y1 - 1; j <= y2 + 1; j++)	// Left vertical route
	         Mask[l][OGRID(i, j, l)] = (u_char)1;

           for (i = x2 - 1; i <= x2 + 1; i++)
	      for (j = y1 - 1; j <= y2 + 1; j++)	// Right vertical route
	         Mask[l][OGRID(i, j, l)] = (u_char)1;
	}
	else {
           for (i = x1 - 1; i <= x2 + 1; i++)
	      for (j = y1 - 1; j <= y1 + 1; j++)	// Bottom horizontal route
	         Mask[l][OGRID(i, j, l)] = (u_char)1;

           for (i = x1 - 1; i <= x2 + 1; i++)
	      for (j = y2 - 1; j <= y2 + 1; j++)	// Top horizontal route
	         Mask[l][OGRID(i, j, l)] = (u_char)1;
	}
     }

     fprintf(stdout, "2-port mask edges rectangle (%d %d) to (%d %d)\n",
		x1, y1, x2, y2);
  }
  else {

     // Use the first tap point for each node to get a rough bounding box and
     // centroid of all taps
     xcent = ycent = 0;
     xmax = ymax = -(MAXRT);
     xmin = ymin = MAXRT;
     for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
	dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	xcent += dtap->gridx;
	ycent += dtap->gridy;
	if (dtap->gridx > xmax) xmax = dtap->gridx;
	if (dtap->gridx < xmin) xmin = dtap->gridx;
	if (dtap->gridy > ymax) ymax = dtap->gridy;
	if (dtap->gridy < ymin) ymin = dtap->gridy;
     }
     xcent /= net->numnodes;
     ycent /= net->numnodes;

     if (xmax - xmin > ymax - ymin) {
	// Horizontal trunk
	o = 1;
	ymin = ymax = ycent;
     }
     else {
	o = 0;
	xmin = xmax = xcent;
     }

     // Allow routes at all tap and extension points
     for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
        for (dtap = n1->taps; dtap != NULL; dtap = dtap->next)
	   Mask[dtap->layer][OGRID(dtap->gridx, dtap->gridy, dtap->layer)] = (u_char)1;
        for (dtap = n1->extend; dtap != NULL; dtap = dtap->next)
	   Mask[dtap->layer][OGRID(dtap->gridx, dtap->gridy, dtap->layer)] = (u_char)1;
     }

     for (l = 0; l < Num_layers; l++) {
	// If the layer orientation is the same as the trunk, place mask around
	// trunk line.
	if (o == LefGetRouteOrientation(l)) {
           for (i = xmin - 1; i <= xmax + 1; i++)
	      for (j = ymin - 1; j <= ymax + 1; j++)
	         Mask[l][OGRID(i, j, l)] = (u_char)1;
	}
	else {
           for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
	      dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	      if (o == 1) {	// Horizontal trunk, vertical branches
                 for (i = dtap->gridx - 1; i <= dtap->gridx + 1; i++) {
	            for (j = dtap->gridy - 1; j <= ycent + 1; j++)
	               Mask[l][OGRID(i, j, l)] = (u_char)1;
	            for (j = ycent - 1; j <= dtap->gridy + 1; j++)
	               Mask[l][OGRID(i, j, l)] = (u_char)1;
	         }
	      } 
	      else {		// Vertical trunk, horizontal branches
                 for (j = dtap->gridy - 1; j <= dtap->gridy + 1; j++) {
	            for (i = dtap->gridx - 1; i <= xcent + 1; i++)
	               Mask[l][OGRID(i, j, l)] = (u_char)1;
	            for (i = xcent - 1; i <= dtap->gridx + 1; i++)
	               Mask[l][OGRID(i, j, l)] = (u_char)1;
		 }
	      }
	   }
	}
     }
     fprintf(stdout, "multi-port mask has trunk line (%d %d) to (%d %d)\n",
		xmin, ymin, xmax, ymax);
  }
}

/*--------------------------------------------------------------*/
/* expandMask --- Extend the Mask area by 1 in all directions	*/
/* by recursive search over all active mask positions (set to	*/
/* nonzero).  Value 1 is used for new mask positions; value 2	*/
/* tracks mask positions that have been processed.		*/
/*--------------------------------------------------------------*/

void expandMask(int x, int y, int l) {
   if (Mask[l][OGRID(x, y, l)] == 0) {
      Mask[l][OGRID(x, y, l)] = (u_char)2;
   }
   else if (Mask[l][OGRID(x, y, l)] == 1) {
      Mask[l][OGRID(x, y, l)] = (u_char)2;	// Mark processed
      if (x > 0) expandMask(x - 1, y, l);
      else if (x < NumChannelsX[l] - 1) expandMask(x + 1, y, l);
      if (y > 0) expandMask(x, y - 1, l);
      else if (y < NumChannelsY[l] - 1) expandMask(x, y + 1, l);
      if (l > 0) expandMask(x, y, l - 1);
      else if (l < Num_layers - 1) expandMask(x, y, l + 1);
   }
}

/*--------------------------------------------------------------*/
/* setMask() is the same routine as expandMask() but cleans up,	*/
/* turning all values 2 back into mask value 1.			*/
/*--------------------------------------------------------------*/

void setMask(int x, int y, int l) {
   if (Mask[l][OGRID(x, y, l)] == 2) {
      Mask[l][OGRID(x, y, l)] = (u_char)1;

      if (x > 0) setMask(x - 1, y, l);
      else if (x < NumChannelsX[l] - 1) setMask(x + 1, y, l);
      if (y > 0) setMask(x, y - 1, l);
      else if (y < NumChannelsY[l] - 1) setMask(x, y + 1, l);
      if (l > 0) setMask(x, y, l - 1);
      else if (l < Num_layers - 1) setMask(x, y, l + 1);
   }
}

/*--------------------------------------------------------------*/
/* fillMask() fills the Mask[] array with all 1s as a last	*/
/* resort, ensuring that no valid routes are missed due to a	*/
/* bad guess about the optimal route positions.			*/
/*--------------------------------------------------------------*/

void fillMask(int value) {
   int i;

   for (i = 0; i < Num_layers; i++) {
      memset((void *)Mask[i], value,
		(size_t)(NumChannelsX[i] * NumChannelsY[i] * sizeof(u_char)));
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
  NETLIST nlist;
  int result;

  if (!net) {
     fprintf(stderr, "doroute():  no net to route.\n");
     return 0;
  }

  CurNet = net;				// Global, used by 2nd stage

  if (net->netnum == VDD_NET || net->netnum == GND_NET)
     pwrbus_src = 0;

  while (1) {	// Keep going until we are unable to route to a terminal

     rt1 = createemptyroute();
     rt1->netnum = net->netnum;

     if (Verbose > 0) {
        fprintf(stdout,"doroute(): added net %d path start %d\n", 
	       net->netnum, net->netnodes->nodenum);
     }

     // TO-DO:  When failing to route a node, we need to check if (1) all nodes
     // are routed, (2) last node is not routed, or (3) more than one node is
     // not routed.  If (3), then set n1 to the first unrouted node.  Otherwise,
     // a large and critical net may not get routed at all because one of the
     // first legs wouldn't route.

     result = route_segs(net, rt1, stage);

     if ((result == 0) || (net == NULL)) {
        // Nodes already routed, nothing to do
	free(rt1);
	return 0;
     }

     if (result < 0) {		// Route failure.
	nlist = (NETLIST)malloc(sizeof(struct netlist_));
	nlist->net = net;
	nlist->next = FailedNets;
	FailedNets = nlist;
	free(rt1);
	return -1;
     }

     TotalRoutes++;

     if (net->routes) {
        for (lrt = net->routes; lrt->next; lrt = lrt->next);
	lrt->next = rt1;
     }
     else {
	net->routes = rt1;
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

int route_segs(NET net, ROUTE rt, u_char stage)
{
  POINT gpoint, glist, gunproc;
  SEG  seg;
  struct seg_ bbox;
  int  i, j, k, o;
  int  x, y;
  NODE n1, n2, n2save;
  u_int netnum, dir, forbid;
  char filename[32];
  int  dist, max, min, maxcost;
  int  thisnetnum, thisindex, index, pass;
  GRIDP best, curpt;
  int  result, rval;
  u_char first = (u_char)1;
  u_char do_pwrbus;
  u_char check_order[6];
  DPOINT n1tap, n2tap;
  PROUTE *Pr;

  // Make Obs2[][] a copy of Obs[][].  Convert pin obstructions to
  // terminal positions for the net being routed.

  for (i = 0; i < Num_layers; i++) {
      for (x = 0; x < NumChannelsX[i]; x++) {
	  for (y = 0; y < NumChannelsY[i]; y++) {
	      netnum = Obs[i][OGRID(x, y, i)] & (~BLOCKED_MASK);
	      Pr = &Obs2[i][OGRID(x, y, i)];
	      if (netnum != 0) {
	         Pr->flags = 0;		// Clear all flags
	         Pr->prdata.net = netnum & NETNUM_MASK;
	         dir = netnum & PINOBSTRUCTMASK;
	         if ((dir != 0) && ((dir & STUBROUTE_X) == STUBROUTE_X)) {
		    if ((netnum & NETNUM_MASK) == rt->netnum)
		       Pr->prdata.net = 0;	// STUBROUTE_X not routable
	         }
	      } else {
	         Pr->flags = PR_COST;		// This location is routable
	         Pr->prdata.cost = MAXRT;
	      }
	  }
      }
  }

  best.cost = MAXRT;

  // Stack of points to search
  glist = (POINT)NULL;
  gunproc = (POINT)NULL;

  n1 = net->netnodes;

  if (n1->netnum == VDD_NET || n1->netnum == GND_NET) {
     // The normal method of selecting source and target is not amenable
     // to power bus routes.  Instead, we use the global standard cell
     // power rails as the target, and each net in sequence becomes the
     // sole source node
     
     do_pwrbus = TRUE;
     for (i = 0; i < pwrbus_src; i++) n1 = n1->next;
  }
  else do_pwrbus = FALSE;

  // We start at the node referenced by the route structure, and flag all
  // of its taps as PR_SOURCE, as well as all connected routes.

  bbox.x2 = bbox.y2 = 0;
  bbox.x1 = NumChannelsX[0];
  bbox.y1 = NumChannelsY[0];

  if (n1 != NULL) {
     rval = set_node_to_net(n1, PR_SOURCE, &glist, &bbox, stage);

     if (rval == -2) {
        printf("Node of net %s has no tap points---unable to route!\n", n1->netname);
        return -1;
     }
  }

  // Set associated routes to PR_SOURCE
  if (do_pwrbus == FALSE) {
     rval = set_routes_to_net(net, PR_SOURCE, &glist, &bbox, stage);

     if (rval == -2) {
        printf("Node of net %s has no tap points---unable to route!\n", net->netname);
        return -1;
     }

     // Now search for all other nodes on the same net that have not yet been
     // routed, and flag all of their taps as PR_TARGET

     result = 0;
     for (n2 = n1->next; n2; n2 = n2->next) {
        rval = set_node_to_net(n2, PR_TARGET, NULL, &bbox, stage);
        if (rval == 0) {
	   n2save = n2;
	   result = 1;
        }
        else if (rval == -2) {
           printf("Node of net %s has no tap points---unable to route!\n", n2->netname);
	   if (result == 0) result = -1;
        }
     }

     /* If there's only one node left and it's not routable, then fail. */
     if (result == -1) return -1;
  }
  else {
     pwrbus_src++;
     if ((pwrbus_src > net->numnodes) || (n1 == NULL))
	result = 0;
     else {
        set_powerbus_to_net(n1->netnum);
	result = 1;
     }
  }

  // Check for the possibility that there is already a route to the target
  if (!result) {
     fprintf(stdout, "Finished routing net %s\n", net->netname);

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

  // Generate a search area mask representing the "likely best route".
  // createMask(CurNet);

  // Heuristic:  Set the initial cost beyond which we stop searching.
  // This value is twice the cost of a direct route across the
  // maximum extent of the source to target, divided by the square
  // root of the number of nodes in the net.  We purposely set this
  // value low.  It has a severe impact on the total run time of the
  // algorithm.  If the initial max cost is so low that no route can
  // be found, it will be doubled on each pass.

  if (do_pwrbus)
     maxcost = 20;	// Maybe make this SegCost * row height?
  else {
     maxcost = 1 + 2 * MAX((bbox.x2 - bbox.x1), (bbox.y2 - bbox.y1)) * SegCost +
		(int)stage * ConflictCost;
     maxcost /= (n1->numnodes - 1);
  }

  netnum = rt->netnum;
  n1tap = n1->taps;
  if (!do_pwrbus) {
     n2 = n2save;
     n2tap = n2->taps;
  }

  if (n1tap == NULL && n1->extend == NULL) {
     printf("Node of net %s has no tap points---unable to route!\n", n1->netname);
     return -1;
  }
  if (n1tap == NULL) n1tap = n1->extend;

  if (!do_pwrbus) {
     if (n2tap == NULL && n2->extend == NULL) {
        printf("Node of net %s has no tap points---unable to route!\n", n2->netname);
        return -1;
     }
     if (n2tap == NULL) n2tap = n2->extend;
  }

  printf("Source node @ %gum %gum layer=%d grid=(%d %d)\n",
	  n1tap->x, n1tap->y, n1tap->layer,
	  n1tap->gridx, n1tap->gridy);
  if (!do_pwrbus) {
     printf("Dest node @ %gum %gum layer=%d grid=(%d %d)\n",
	  n2tap->x, n2tap->y, n2tap->layer,
	  n2tap->gridx, n2tap->gridy);
  }
  printf("netname = %s, route number %d\n", n1->netname, TotalRoutes );
  fflush(stdout);

  for (pass = 0; pass < Numpasses; pass++) {

    if (!first) {
       fprintf(stdout, "\n");
       first = (u_char)1;
    }
    fprintf(stdout, "Pass %d", pass + 1);
    fprintf(stdout, " (maxcost is %d)\n", maxcost);

    while (gpoint = glist) {

      glist = gpoint->next;

      curpt.x = gpoint->x1;
      curpt.y = gpoint->y1;
      curpt.lay = gpoint->layer;
	
      Pr = &Obs2[curpt.lay][OGRID(curpt.x, curpt.y, curpt.lay)];

      // ignore grid positions that have already been processed
      if (Pr->flags & PR_PROCESSED) {
	 free(gpoint);
	 continue;
      }

      if (Pr->flags & PR_COST)
	 curpt.cost = Pr->prdata.cost;	// Route points, including target
      else
	 curpt.cost = 0;			// For source tap points

      // if the grid position is the destination, save the position and
      // cost if minimum.

      if (Pr->flags & PR_TARGET) {

 	 if (curpt.cost < best.cost) {
	    if (first) {
	       fprintf(stdout, "Found a route of cost ");
	       first = (u_char)0;
	    }
	    else
	       fprintf(stdout, "|");
	    fprintf(stdout, "%d", curpt.cost);
	    fflush(stdout);

	    // This position may be on a route, not at a terminal, so
	    // record it.
	    best.x = curpt.x;
	    best.y = curpt.y;
	    best.lay = curpt.lay;
	    best.cost = curpt.cost;

	    // If a complete route has been found, then there's no point
	    // in searching paths with a greater cost than this one.
	    if (best.cost < maxcost) maxcost = best.cost;
	 }

         // Don't continue processing from the target
	 free(gpoint);
	 continue;
      }

      if (curpt.cost < MAXRT) {

	 // Severely limit the search space by not processing anything that
	 // is not under the current route mask, which identifies a narrow
	 // "best route" solution.

	 // if (Mask[curpt.lay][OGRID(curpt.x, curpt.y, curpt.lay)] == (u_char)0) {
	 //    gpoint->next = gunproc;
	 //    gunproc = gpoint;
	 //    continue;
	 // }

         // Quick check:  Limit maximum cost to limit search space
         // Move the point onto the "unprocessed" stack and we'll pick up
         // from this point on the next pass, if needed.

	 // else
         if (curpt.cost > maxcost) {
	    gpoint->next = gunproc;
	    gunproc = gpoint;
	    continue;
	 }
      }
      free(gpoint);

      // check east/west/north/south, and bottom to top

      // 1st optimization:  Direction of route on current layer is preferred.
      o = LefGetRouteOrientation(curpt.lay);
      forbid = Obs[curpt.lay][OGRID(curpt.x, curpt.y, curpt.lay)] & BLOCKED_MASK;

      if (o == 1) {			// horizontal routes---check EAST and WEST first
	 check_order[0] = (forbid & BLOCKED_E) ? 0 : EAST;
	 check_order[1] = (forbid & BLOCKED_W) ? 0 : WEST;
	 check_order[2] = UP;
	 check_order[3] = DOWN;
	 check_order[4] = (forbid & BLOCKED_N) ? 0 : NORTH;
	 check_order[5] = (forbid & BLOCKED_S) ? 0 : SOUTH;
      }
      else {				// vertical routes---check NORTH and SOUTH first
	 check_order[0] = (forbid & BLOCKED_N) ? 0 : NORTH;
	 check_order[1] = (forbid & BLOCKED_S) ? 0 : SOUTH;
	 check_order[2] = UP;
	 check_order[3] = DOWN;
	 check_order[4] = (forbid & BLOCKED_E) ? 0 : EAST;
	 check_order[5] = (forbid & BLOCKED_W) ? 0 : WEST;
      }

      min = MAXRT;

      // Check order is from 0 (1st priority) to 5 (last priority).  However, this
      // is a stack system, so the last one placed on the stack is the first to be
      // pulled and processed.  Therefore we evaluate and drop positions to check
      // on the stack in reverse order (5 to 0).

      for (i = 5; i >= 0; i--) {
	 switch (check_order[i]) {
	    case EAST:
               if ((curpt.x + 1) < NumChannelsX[curpt.lay]) {
         	  if ((result = eval_pt(&curpt, PR_PRED_W, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x + 1;
         	     gpoint->y1 = curpt.y;
         	     gpoint->layer = curpt.lay;
         	     gpoint->next = glist;
         	     glist = gpoint;
                   }
               }
	       break;

	    case WEST:
               if ((curpt.x - 1) >= 0) {
         	  if ((result = eval_pt(&curpt, PR_PRED_E, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x - 1;
         	     gpoint->y1 = curpt.y;
         	     gpoint->layer = curpt.lay;
         	     gpoint->next = glist;
         	     glist = gpoint;
                  }
               }
	       break;
         
	    case SOUTH:
               if ((curpt.y - 1) >= 0) {
         	  if ((result = eval_pt(&curpt, PR_PRED_N, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x;
         	     gpoint->y1 = curpt.y - 1;
         	     gpoint->layer = curpt.lay;
         	     gpoint->next = glist;
         	     glist = gpoint;
                   }
               }
	       break;

	    case NORTH:
               if ((curpt.y + 1) < NumChannelsY[curpt.lay]) {
         	  if ((result = eval_pt(&curpt, PR_PRED_S, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x;
         	     gpoint->y1 = curpt.y + 1;
         	     gpoint->layer = curpt.lay;
         	     gpoint->next = glist;
         	     glist = gpoint;
                  }
               }
	       break;
      
	    case DOWN:
               if (curpt.lay > 0) {
         	  if ((result = eval_pt(&curpt, PR_PRED_U, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x;
         	     gpoint->y1 = curpt.y;
         	     gpoint->layer = curpt.lay - 1;
         	     gpoint->next = glist;
         	     glist = gpoint;
         	  }
               }
	       break;
         
	    case UP:
               if (curpt.lay < (Num_layers - 1)) {
         	  if ((result = eval_pt(&curpt, PR_PRED_D, stage)) == 1) {
         	     gpoint = (POINT)malloc(sizeof(struct point_));
         	     gpoint->x1 = curpt.x;
         	     gpoint->y1 = curpt.y;
         	     gpoint->layer = curpt.lay + 1;
         	     gpoint->next = glist;
         	     glist = gpoint;
         	  }
               }
	       break;
            }
         }

      // Mark this node as processed
      Pr->flags |= PR_PROCESSED;

    } // while stack is not empty

    while (glist) {
       gpoint = glist;
       glist = glist->next;
       free(gpoint);
    }

    // If we found a route, save it and return

    if (best.cost <= maxcost) {
	curpt.x = best.x;
	curpt.y = best.y;
	curpt.lay = best.lay;
	if ((rval = commit_proute(rt, &curpt, stage)) != 1) break;
	fprintf(stdout, "\nCommit to a route of cost %d\n", best.cost);
	fprintf(stdout, "Between positions (%d %d) and (%d %d)\n",
		best.x, best.y, curpt.x, curpt.y);
	goto done;	/* route success */
    }

    // Only continue loop to next pass if we failed to find a route.
    // Increase maximum cost for next pass.

    maxcost <<= 1;
    if (maxcost < 0) break;		// Overflowed unsigned integer, we're
					// probably completely hosed long before
					// this.

    if (gunproc == NULL) break;		// route failure not due to limiting
					// search to maxcost

    // Regenerate the stack of unprocessed nodes
    glist = gunproc;
    gunproc = NULL;
    
  } // pass
  
  if (!first) fprintf(stdout, "\n");
  fflush(stdout);
  fprintf(stderr, "Fell through %d passes\n", pass);
  if (!do_pwrbus)
     fprintf(stderr, "(%g,%g) <==> (%g,%g) net=%s\n",
	   n1tap->x, n1tap->y, n2tap->x, n2tap->y, n1->netname);
  if (!Failfptr) openFailFile();
  if (!do_pwrbus)
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
emit_routed_net(FILE *Cmd, NET net, u_char special, double oscale)
{
   SEG seg, saveseg, lastseg;
   ROUTE rt;
   u_int dir1, dir2, tdir;
   int i, layer;
   int x, y, x2, y2;
   double dc;
   int lastx, lasty;
   int horizontal;
   DPOINT dp1, dp2;
   float offset1, offset2;
   int stubroute = 0;
   u_char cancel;

   Pathon = -1;

   /* Insert routed net here */
   for (rt = net->routes; rt; rt = rt->next) {
      if (rt->segments && !rt->output) {
	 horizontal = FALSE;
	 cancel = FALSE;

	 // Check first position for terminal offsets
	 seg = (SEG)rt->segments;
	 lastseg = saveseg = seg;
	 layer = seg->layer;
	 if (seg) {

	    // It is rare but possible to have a stub route off of an
	    // endpoint via, so check this case, and use the layer type
	    // of the via top if needed.

	    if ((seg->segtype & ST_VIA) && seg->next && (seg->next->layer <=
			seg->layer))
	       layer++;

	    dir1 = Obs[layer][OGRID(seg->x1, seg->y1, layer)];
	    dir1 &= PINOBSTRUCTMASK;
	    if (dir1 && !(seg->segtype & (ST_OFFSET_START | ST_OFFSET_END))) {
	       stubroute = 1;
	       if (special == (u_char)0)
		  fprintf(stdout, "Stub route distance %g to terminal"
				" at %d %d (%d)\n",
				Stub[layer][OGRID(seg->x1, seg->y1, layer)],
				seg->x1, seg->y1, layer);

	       dc = Xlowerbound + (double)seg->x1 * PitchX[layer];
	       x = (int)((dc + EPS) * oscale);
	       if (dir1 == STUBROUTE_EW)
		  dc += Stub[layer][OGRID(seg->x1, seg->y1, layer)];
	       x2 = (int)((dc + EPS) * oscale);
	       dc = Ylowerbound + (double)seg->y1 * PitchY[layer];
	       y = (int)((dc + EPS) * oscale);
	       if (dir1 == STUBROUTE_NS)
		  dc += Stub[layer][OGRID(seg->x1, seg->y1, layer)];
	       y2 = (int)((dc + EPS) * oscale);
	       if (dir1 == STUBROUTE_EW) {
		  horizontal = TRUE;

		  // If the gridpoint ahead of the stub has a route
		  // on the same net, and the stub is long enough
		  // to come within a DRC spacing distance of the
		  // other route, then lengthen it to close up the
		  // distance and resolve the error.  (NOTE:  This
		  // unnecessarily stretches routes to cover taps
		  // that have not been routed to.  At least on the
		  // test standard cell set, these rules remove a
		  // handful of DRC errors and don't create any new
		  // ones.  If necessary, a flag can be added to
		  // distinguish routes from taps.

		  if ((x < x2) && (seg->x1 < (NumChannelsX[layer] - 1))) {
		     tdir = Obs[layer][OGRID(seg->x1 + 1, seg->y1, layer)];
		     if ((tdir & ~PINOBSTRUCTMASK) ==
					(net->netnum | ROUTED_NET)) {
			if (Stub[layer][OGRID(seg->x1, seg->y1, layer)] +
					LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	   dc = Xlowerbound + (double)(seg->x1 + 1)
					* PitchX[layer];
		      	   x2 = (int)((dc + EPS) * oscale);
			}
		     }
		  }
		  else if ((x > x2) && (seg->x1 > 0)) {
		     tdir = Obs[layer][OGRID(seg->x1 - 1, seg->y1, layer)];
		     if ((tdir & ~PINOBSTRUCTMASK) ==
					(net->netnum | ROUTED_NET)) {
			if (-Stub[layer][OGRID(seg->x1, seg->y1, layer)] +
					LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	   dc = Xlowerbound + (double)(seg->x1 - 1)
					* PitchX[layer];
		      	   x2 = (int)((dc + EPS) * oscale);
			}
		     }
		  }

		  dc = oscale * 0.5 * LefGetRouteWidth(layer);
		  if (special == (u_char)0) {
		     // Regular nets include 1/2 route width at
		     // the ends, so subtract from the stub terminus
		     if (x < x2) {
			x2 -= dc;
			if (x >= x2) cancel = TRUE;
		     }
		     else {
			x2 += dc;
			if (x <= x2) cancel = TRUE;
		     }
		  }
		  else {
		     // Special nets don't include 1/2 route width
		     // at the ends, so add to the route at the grid
		     if (x < x2)
			x -= dc;
		     else
			x += dc;

		     // Routes that extend for more than one track
		     // without a bend do not need a wide stub
		     if (seg->x1 != seg->x2) cancel = TRUE;
	  	  }
	       }
	       else {
		  horizontal = FALSE;

		  // If the gridpoint ahead of the stub has a route
		  // on the same net, and the stub is long enough
		  // to come within a DRC spacing distance of the
		  // other route, then lengthen it to close up the
		  // distance and resolve the error.

		  if ((y < y2) && (seg->y1 < (NumChannelsY[layer] - 1))) {
		     tdir = Obs[layer][OGRID(seg->x1, seg->y1 + 1, layer)];
		     if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			if (Stub[layer][OGRID(seg->x1, seg->y1, layer)] +
					LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	   dc = Ylowerbound + (double)(seg->y1 + 1)
					* PitchY[layer];
		      	   y2 = (int)((dc + EPS) * oscale);
			}
		     }
		  }
		  else if ((y > y2) && (seg->y1 > 0)) {
		     tdir = Obs[layer][OGRID(seg->x1, seg->y1 - 1, layer)];
		     if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			if (-Stub[layer][OGRID(seg->x1, seg->y1, layer)] +
					LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	   dc = Ylowerbound + (double)(seg->y1 - 1)
					* PitchY[layer];
		      	   y2 = (int)((dc + EPS) * oscale);
			}
		     }
		  }

		  dc = oscale * 0.5 * LefGetRouteWidth(layer);
		  if (special == (u_char)0) {
		     // Regular nets include 1/2 route width at
		     // the ends, so subtract from the stub terminus
		     if (y < y2) {
			y2 -= dc;
			if (y >= y2) cancel = TRUE;
		     }
		     else {
			y2 += dc;
			if (y <= y2) cancel = TRUE;
		     }
		  }
		  else {
		     // Special nets don't include 1/2 route width
		     // at the ends, so add to the route at the grid
		     if (y < y2)
			y -= dc;
		     else
			y += dc;

		     // Routes that extend for more than one track
		     // without a bend do not need a wide stub
		     if (seg->y1 != seg->y2) cancel = TRUE;
		  }
	       }

	       if (cancel == FALSE) {
		  pathstart(Cmd, layer, x2, y2, special, oscale);
		  pathto(Cmd, x, y, horizontal, x2, y2);
	       }
	       lastx = x;
	       lasty = y;
	    }
	 }

	 lastseg = NULL;
	 for (seg = rt->segments; seg; seg = seg->next) {
	    layer = seg->layer;

	    // Check for offset terminals at either point

	    offset1 = 0.0;
	    offset2 = 0.0;
	    dir1 = 0;
	    dir2 = 0;

	    if (seg->segtype & ST_OFFSET_START) {
	       dir1 = Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)] &
				PINOBSTRUCTMASK;
	       if (dir1 == 0 && lastseg) {
		  dir1 = Obs[lastseg->layer][OGRID(lastseg->x2, lastseg->y2,
					lastseg->layer)] & PINOBSTRUCTMASK;
		  offset1 = Stub[lastseg->layer][OGRID(lastseg->x2,
					lastseg->y2, lastseg->layer)];
	       }
	       else
		  offset1 = Stub[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)];

	       // Additional offset for vias vs. plain route layer
	       if (seg->segtype & ST_VIA) {
		  if (offset1 < 0)
		     offset1 -= 0.5 * (LefGetViaWidth(seg->layer, seg->layer, 0)
					- LefGetRouteWidth(seg->layer));
		  else if (offset1 > 0)
		     offset1 += 0.5 * (LefGetViaWidth(seg->layer, seg->layer, 0)
					- LefGetRouteWidth(seg->layer));
	       }

	       if (special == (u_char)0) {
		  if (seg->segtype & ST_VIA)
		     fprintf(stdout, "Offset terminal distance %g to grid"
					" at %d %d (%d)\n", offset1,
					seg->x1, seg->y1, layer);
	       }
	    }
	    if (seg->segtype & ST_OFFSET_END) {
	       dir2 = Obs[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)] &
				PINOBSTRUCTMASK;
	       if (dir2 == 0 && seg->next) {
		  dir2 = Obs[seg->next->layer][OGRID(seg->next->x1,
					seg->next->y1, seg->next->layer)] &
					PINOBSTRUCTMASK;
		  offset2 = Stub[seg->next->layer][OGRID(seg->next->x1,
					seg->next->y1, seg->next->layer)];
	       }
	       else
		  offset2 = Stub[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)];

	       // Additional offset for vias vs. plain route layer
	       if (seg->segtype & ST_VIA) {
		  if (offset2 < 0)
		     offset2 -= 0.5 * (LefGetViaWidth(seg->layer, seg->layer, 0)
					- LefGetRouteWidth(seg->layer));
		  else if (offset2 > 0)
		     offset2 += 0.5 * (LefGetViaWidth(seg->layer, seg->layer, 0)
					- LefGetRouteWidth(seg->layer));
	       }

	       if (special == (u_char)0) {
		  if ((seg->segtype & ST_VIA)
					&& !(seg->segtype & ST_OFFSET_START))
		     fprintf(stdout, "Offset terminal distance %g to grid"
					" at %d %d (%d)\n", offset2,
					seg->x2, seg->y2, layer);
	       }
	    }

	    // To do: pick up route layer name from lefInfo.
	    // At the moment, technology names don't even match,
	    // and are redundant between CIFLayer[] from the
	    // config file and lefInfo.

	    dc = Xlowerbound + (double)seg->x1 * PitchX[layer];
	    if (dir1 == (STUBROUTE_EW | OFFSET_TAP)) dc += offset1;
	    x = (int)((dc + EPS) * oscale);
	    dc = Ylowerbound + (double)seg->y1 * PitchY[layer];
	    if (dir1 == (STUBROUTE_NS | OFFSET_TAP)) dc += offset1;
	    y = (int)((dc + EPS) * oscale);
	    dc = Xlowerbound + (double)seg->x2 * PitchX[layer];
	    if (dir2 == (STUBROUTE_EW | OFFSET_TAP)) dc += offset2;
	    x2 = (int)((dc + EPS) * oscale);
	    dc = Ylowerbound + (double)seg->y2 * PitchY[layer];
	    if (dir2 == (STUBROUTE_NS | OFFSET_TAP)) dc += offset2;
	    y2 = (int)((dc + EPS) * oscale);
	    switch (seg->segtype & ~(ST_OFFSET_START | ST_OFFSET_END)) {
	       case ST_WIRE:
		  if (Pathon != 1) {	// 1st point of route seg
		     if (special == (u_char)0) {
			pathstart(Cmd, seg->layer, x, y, (u_char)0, oscale);
			lastx = x;
			lasty = y;
		     }
		     if (x == x2) {
			horizontal = FALSE;
		     }
		     else if (y == y2) {
			horizontal = TRUE;
		     }
		     else if (Verbose > 0) {
			// NOTE:  This is a development diagnostic.  The
			// occasional non-Manhanhattan route is due to a
			// tap offset and is corrected automatically by
			// making an L-bend in the wire.

		     	fflush(stdout);
			fprintf(stderr, "Warning:  non-Manhattan wire in route"
				" at (%d %d) to (%d %d)\n", x, y, x2, y2);
		     }
		  }
		  rt->output = TRUE;
		  if (horizontal && x == x2) {
		     horizontal = FALSE;
		  }
		  if ((!horizontal) && y == y2) {
		     horizontal = TRUE;
		  }
		  if (!(x == x2) && !(y == y2)) {
		     horizontal = FALSE;
		  }
		  if (special == (u_char)0) {
		     pathto(Cmd, x2, y2, horizontal, lastx, lasty);
		     lastx = x2;
		     lasty = y2;
		  }
		  break;
	       case ST_VIA:
		  rt->output = TRUE;
		  if (special == (u_char)0) {
		     if (lastseg == NULL) {
			// Make sure last position is valid
			lastx = x;
			lasty = y;
		     }
		     pathvia(Cmd, layer, x, y, lastx, lasty);
		     lastx = x;
		     lasty = y;
		  }
		  break;
	       default:
		  break;
	    }
	    lastseg = seg;
	 }

	 // For stub routes, reset the path between terminals, since
	 // the stubs are not connected.
	 if (special == (u_char)1 && Pathon != -1) Pathon = 0;

	 // Check last position for terminal offsets
	 if (lastseg && ((lastseg != saveseg)
				|| (lastseg->segtype & ST_WIRE))) {
	     cancel = FALSE;
	     seg = lastseg;
	     layer = seg->layer;
	     dir2 = Obs[layer][OGRID(seg->x2, seg->y2, layer)];
	     dir2 &= PINOBSTRUCTMASK;
	     if (dir2 && !(seg->segtype & (ST_OFFSET_END | ST_OFFSET_START))) {
		stubroute = 1;
		if (special == (u_char)0)
		   fprintf(stdout, "Stub route distance %g to terminal"
				" at %d %d (%d)\n",
				Stub[layer][OGRID(seg->x2, seg->y2, layer)],
				seg->x2, seg->y2, layer);

		dc = Xlowerbound + (double)seg->x2 * PitchX[layer];
		x = (int)((dc + EPS) * oscale);
		if (dir2 == STUBROUTE_EW)
		   dc += Stub[layer][OGRID(seg->x2, seg->y2, layer)];
		x2 = (int)((dc + EPS) * oscale);
		dc = Ylowerbound + (double)seg->y2 * PitchY[layer];
		y = (int)((dc + EPS) * oscale);
		if (dir2 == STUBROUTE_NS)
		   dc += Stub[layer][OGRID(seg->x2, seg->y2, layer)];
		y2 = (int)((dc + EPS) * oscale);
		if (dir2 == STUBROUTE_EW) {
		   horizontal = TRUE;

		   // If the gridpoint ahead of the stub has a route
		   // on the same net, and the stub is long enough
		   // to come within a DRC spacing distance of the
		   // other route, then lengthen it to close up the
		   // distance and resolve the error.

		   if ((x < x2) && (seg->x2 < (NumChannelsX[layer] - 1))) {
		      tdir = Obs[layer][OGRID(seg->x2 + 1, seg->y2, layer)];
		      if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (Stub[layer][OGRID(seg->x2, seg->y2, layer)] +
					LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	    dc = Xlowerbound + (double)(seg->x2 + 1)
					* PitchX[layer];
		      	    x2 = (int)((dc + EPS) * oscale);
			 }
		      }
		   }
		   else if ((x > x2) && (seg->x2 > 0)) {
		      tdir = Obs[layer][OGRID(seg->x2 - 1, seg->y2, layer)];
		      if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (-Stub[layer][OGRID(seg->x2, seg->y2, layer)] +
					LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	    dc = Xlowerbound + (double)(seg->x2 - 1)
					* PitchX[layer];
		      	    x2 = (int)((dc + EPS) * oscale);
			 }
		      }
		   }

		   dc = oscale * 0.5 * LefGetRouteWidth(layer);
		   if (special == (u_char)0) {
		      // Regular nets include 1/2 route width at
		      // the ends, so subtract from the stub terminus
		      if (x < x2) {
			 x2 -= dc;
			 if (x >= x2) cancel = TRUE;
		      }
		      else {
			 x2 += dc;
			 if (x <= x2) cancel = TRUE;
		      }
		   }
		   else {
		      // Special nets don't include 1/2 route width
		      // at the ends, so add to the route at the grid
		      if (x < x2)
			 x -= dc;
		      else
			 x += dc;

		      // Routes that extend for more than one track
		      // without a bend do not need a wide stub
		      if (seg->x1 != seg->x2) cancel = TRUE;
		   }
		}
		else {
		   horizontal = FALSE;

		   // If the gridpoint ahead of the stub has a route
		   // on the same net, and the stub is long enough
		   // to come within a DRC spacing distance of the
		   // other route, then lengthen it to close up the
		   // distance and resolve the error.

		   if ((y < y2) && (seg->y2 < (NumChannelsY[layer] - 1))) {
		      tdir = Obs[layer][OGRID(seg->x2, seg->y2 + 1, layer)];
		      if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (Stub[layer][OGRID(seg->x2, seg->y2, layer)] +
					LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	    dc = Ylowerbound + (double)(seg->y2 + 1)
					* PitchY[layer];
		      	    y2 = (int)((dc + EPS) * oscale);
			 }
		      }
		   }
		   else if ((y > y2) && (seg->y2 > 0)) {
		      tdir = Obs[layer][OGRID(seg->x2, seg->y2 - 1, layer)];
		      if ((tdir & ~PINOBSTRUCTMASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (-Stub[layer][OGRID(seg->x2, seg->y2, layer)] +
					LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	    dc = Ylowerbound + (double)(seg->y2 - 1)
					* PitchY[layer];
		      	    y2 = (int)((dc + EPS) * oscale);
			 }
		      }
		   }

		   dc = oscale * 0.5 * LefGetRouteWidth(layer);
		   if (special == (u_char)0) {
		      // Regular nets include 1/2 route width at
		      // the ends, so subtract from the stub terminus
		      if (y < y2) {
			 y2 -= dc;
			 if (y >= y2) cancel = TRUE;
		      }
		      else {
			 y2 += dc;
			 if (y <= y2) cancel = TRUE;
		      }
		   }
		   else {
		      // Special nets don't include 1/2 route width
		      // at the ends, so add to the route at the grid
		      if (y < y2)
			 y -= dc;
		      else
			 y += dc;

		      // Routes that extend for more than one track
		      // without a bend do not need a wide stub
		      if (seg->y1 != seg->y2) cancel = TRUE;
		   }
		}
		if (cancel == FALSE) {
		   if (Pathon != 1) {
		      pathstart(Cmd, layer, x, y, special, oscale);
		      lastx = x;
		      lasty = y;
		   }
		   pathto(Cmd, x2, y2, horizontal, lastx, lasty);
		   lastx = x2;
		   lasty = y2;
		}
	    }
	 }
	 if (Pathon != -1) Pathon = 0;

      } // if (rt->segments && !rt->output)
   }
   return stubroute;
}

/*--------------------------------------------------------------*/
/* emit_routes - DEF file output from the list of routes	*/
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

void emit_routes(char *filename, double oscale)
{
    FILE *Cmd;
    int i, j, numnets;
    char line[MAXLINE + 1], *lptr;
    char netname[MAX_NAME_LEN];
    NET net;
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
    if (numnets != (Numnets - MIN_NET_NUMBER + 1)) {
	fflush(stdout);
        fprintf(stderr, "emit_routes():  DEF file has %d nets, but we want"
		" to write %d\n", numnets, Numnets - MIN_NET_NUMBER + 1);
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
		fputs(line, Cmd);
	     }
	     else if (*lptr == '+') {
		lptr++;
                while (isspace(*lptr)) lptr++;
		if (!strncmp(lptr, "ROUTED", 6)) {
		   // This net is being handled by qrouter, so remove
		   // the original routing information
		   while (fgets(line, MAXLINE, fdef) != NULL) {
		      if ((lptr = strchr(line, ';')) != NULL) {
			 *lptr = '\n';
			 *(lptr + 1) = '\0';
			 break;
		      }
		   }
		   break;
		}
		else
		   fputs(line, Cmd);
	     }
	     else
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

	  stubroute += emit_routed_net(Cmd, net, (u_char)0, oscale);
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
       for (net = Nlnets; net; net = net->next)
	  for (rt = net->routes; rt; rt = rt->next)
	     rt->output = 0;

       fprintf(Cmd, "\nSPECIALNETS %d ;\n", stubroute);
       fprintf(Cmd, "- stubroute1\n");
       net = Nlnets;
       for (i = 0; i < stubroute; i++) {
          for (; net; net = net->next) {
	     if (emit_routed_net(Cmd, net, (u_char)1, oscale) > 0) {
		if (i < stubroute - 1) {
		   fprintf(Cmd, " ;\n- stubroute%d\n", i + 2);
		}
		break;
	     }
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
    fprintf(stdout, "\t-p <name>\t\t\tSpecify global power bus name.\n");
    fprintf(stdout, "\t-g <name>\t\t\tSpecify global ground bus name.\n");
    fprintf(stdout, "\n");
    fprintf(stdout, "%s\n", VERSION);

} /* helpmessage() */

/* end of qrouter.c */
