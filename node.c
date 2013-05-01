/*--------------------------------------------------------------*/
/* node.c -- Generation	of detailed network and obstruction	*/
/* information on the routing grid based on the geometry of the	*/
/* layout of the standard cell macros.				*/
/*								*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, June, 2011, based on work by Steve	*/
/* Beccue.							*/
/*--------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "qrouter.h"
#include "node.h"
#include "config.h"
#include "lef.h"

/*--------------------------------------------------------------*/
/* create_netorder --- assign indexes to net->netorder    	*/
/*								*/
/* 	Nets are ordered simply from those with the most nodes	*/
/*	to those with the fewest.  However, any nets marked	*/
/* 	critical in the configuration or critical net files	*/
/*	will be given precedence.				*/
/*								*/
/*  ARGS: none							*/
/*  RETURNS: nothing						*/
/*  SIDE EFFECTS: Nlnets -> netorder is assigned		*/
/*--------------------------------------------------------------*/

void create_netorder()
{
  int i, max;
  NET  net;
  STRING cn;

  i = 1;
  for (cn = CriticalNet; cn; cn = cn->next) {
     fprintf(stdout, "critical net %s\n", cn->name);
     for (net = Nlnets; net; net = net->next) {
	if (!strcmp(net->netname, (char *)cn->name)) {
           net->netorder = i++;
	}
     }
  }

  for (; i <= Numnets; i++) {
     max = 0;

     for (net = Nlnets; net; net = net->next) {
	if (!net->netorder) {
	   if (net->numnodes > max) max = net->numnodes;
	}
     }

     for (net = Nlnets; net; net = net->next) {
	if (!net->netorder) {
	   if (net->numnodes == max) {
	      net->netorder = i;
	      break;
	   }
        }
     }
  }
} /* create_netorder() */

/*--------------------------------------------------------------*/
/* print_nodes - show the nodes list				*/
/*         ARGS: filename to print to
        RETURNS: nothing
   SIDE EFFECTS: none
AUTHOR and DATE: steve beccue      Tue Aug 04  2003
\*--------------------------------------------------------------*/

void print_nodes(char *filename)
{
  FILE *o;
  int i;
  NODE node;
  DPOINT dp;

    if (!strcmp(filename, "stdout")) {
	o = stdout;
    } else {
	o = fopen(filename, "w");
    }
    if (!o) {
	fprintf( stderr, "node.c:print_nodes.  Couldn't open output file\n" );
	return;
    }

    for (node = Nlnodes; node; node = node->next) {
	dp = (DPOINT)node->taps;
	fprintf(o, "%d\t%s\t(%g,%g)(%d,%d) :%d:num=%d netnum=%d\n",
		node->nodenum, 
		node->netname,
		// legacy:  print only the first point
		dp->x, dp->y, dp->gridx, dp->gridy,
		node->netnum, node->numnodes, node->netnum );
		 
	/* need to print the routes to this node (deprecated)
	for (i = 0 ; i < g->nodes; i++) {
	    fprintf(o, "%s(%g,%g) ", g->node[i], *(g->x[i]), *(g->y[i]));
	}
	*/
    }
    fclose(o);

} /* void print_nodes() */

/*--------------------------------------------------------------*/
/*C print_nlnets - show the nets				*/
/*         ARGS: filename to print to
        RETURNS: nothing
   SIDE EFFECTS: none
AUTHOR and DATE: steve beccue      Tue Aug 04  2003
\*--------------------------------------------------------------*/

void print_nlnets( char *filename )
{
  FILE *o;
  int i;
  NODELIST nl;
  NET net;

    if (!strcmp(filename, "stdout")) {
	o = stdout;
    } else {
	o = fopen(filename, "w");
    }
    if (!o) {
	fprintf(stderr, "node.c:print_nlnets.  Couldn't open output file\n");
	return;
    }

    for (net = Nlnets; net; net = net->next) {
	fprintf(o, "%d\t#=%d\t%s   \t\n", net->netnum, 
		 net->numnodes, net->netname);

	for (nl = net->netnodes; nl; nl = nl->next) {
	   fprintf(o, "%d ", nl->node->nodenum);
	}
    }

    fprintf(o, "%d nets and %d nodes\n", Numnets, Numnodes);
    fflush(o);

} /* void print_nlnets() */

/*--------------------------------------------------------------*/
/* create_obstructions_from_variable_pitch()			*/
/*								*/
/*  Although it would be nice to have an algorithm that would	*/
/*  work with any arbitrary pitch, qrouter will work around	*/
/*  having larger pitches on upper metal layers by selecting	*/
/*  1 out of every N tracks for routing, and placing 		*/
/*  obstructions in the interstices.  This makes the possibly	*/
/*  unwarranted assumption that the contact down to the layer	*/
/*  below does not cause spacing violations to neighboring	*/
/*  tracks.  If that assumption fails, this routine will have	*/
/*  to be revisited.						*/
/*--------------------------------------------------------------*/

void create_obstructions_from_variable_pitch()
{
   int l, o, vnum, hnum, x, y;
   double vpitch, hpitch;

   for (l = 0; l < Num_layers; l++) {
      o = LefGetRouteOrientation(l);
      if (o == 1) {	// Horizontal route
	 vpitch = LefGetRoutePitch(l);
	 hpitch = LefGetRouteWidth(l) + LefGetRouteSpacing(l);
      }
      else {		// Vertical route
	 hpitch = LefGetRoutePitch(l);
	 vpitch = LefGetRouteWidth(l) + LefGetRouteSpacing(l);
      }

      vnum = 1;
      while (vpitch > PitchY[l]) {
	 vpitch /= 2.0;
	 vnum++;
      }
      hnum = 1;
      while (hpitch > PitchX[l]) {
	 hpitch /= 2.0;
	 hnum++;
      }
      if (vnum > 1 || hnum > 1) {
	 for (x = 0; x < NumChannelsX[l]; x++) {
	    if (x % hnum == 0) continue;
	    for (y = 0; y < NumChannelsY[l]; y++) {
	       if (y % vnum == 0) continue;
	       Obs[l][OGRID(x, y, l)] = NO_NET;
	    }
	 }
      }
   }
}

/*--------------------------------------------------------------*/
/* check_obstruct()---						*/
/*	Called from create_obstructions_from_gates(), this	*/
/* 	routine takes a grid point at (gridx, gridy) (physical	*/
/* 	position (dx, dy)) and an obstruction defined by the	*/
/*	rectangle "ds", and sets flags and fills the Obsinfo	*/
/*	array to reflect how the obstruction affects routing to	*/
/*	the grid position.					*/
/*--------------------------------------------------------------*/

void
check_obstruct(int gridx, int gridy, DSEG ds, double dx, double dy)
{
    int *obsptr;
    float dist;

    obsptr = &(Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]);
    dist = Obsinfo[ds->layer][OGRID(gridx, gridy, ds->layer)];

    // Grid point is inside obstruction + halo.
    *obsptr |= NO_NET;

    // Completely inside obstruction?
    if (dy > ds->y1 && dy < ds->y2 && dx > ds->x1 && dx < ds->x2)
       *obsptr |= OBSTRUCT_MASK;

    else {

       // Make more detailed checks in each direction

       if (dy < ds->y1) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_N)) == 0) {
	     if ((dist == 0) || ((ds->y1 - dy) < dist))
		Obsinfo[ds->layer][OGRID(gridx, gridy, ds->layer)] = ds->y1 - dy;
	     *obsptr |= OBSTRUCT_N;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       else if (dy > ds->y2) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_S)) == 0) {
	     if ((dist == 0) || ((dy - ds->y2) < dist))
		Obsinfo[ds->layer][OGRID(gridx, gridy, ds->layer)] = dy - ds->y2;
	     *obsptr |= OBSTRUCT_S;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       if (dx < ds->x1) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_E)) == 0) {
	     if ((dist == 0) || ((ds->x1 - dx) < dist))
		Obsinfo[ds->layer][OGRID(gridx, gridy, ds->layer)] = ds->x1 - dx;
             *obsptr |= OBSTRUCT_E;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       else if (dx > ds->x2) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_W)) == 0) {
	     if ((dist == 0) || ((dx - ds->x2) < dist))
		Obsinfo[ds->layer][OGRID(gridx, gridy, ds->layer)] = dx - ds->x2;
	     *obsptr |= OBSTRUCT_W;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
   }
}

/*--------------------------------------------------------------*/
/* create_obstructions_from_gates()				*/
/*								*/
/*  Fills in the Obs[][] grid from obstructions that were	*/
/*  defined for each macro in the technology LEF file and	*/
/*  translated into a list of grid coordinates in each		*/
/*  instance.							*/
/*								*/
/*  Also, fills in the Obs[][] grid with obstructions that	*/
/*  are defined by nodes of the gate that are unconnected in	*/
/*  this netlist.						*/
/*--------------------------------------------------------------*/

void create_obstructions_from_gates()
{
    GATE g;
    DSEG ds;
    int i, gridx, gridy, no_net, *obsptr;
    double dx, dy, delta[MAX_LAYERS];
    float dist;

    // Get, and store, information for each layer on how much we
    // need to expand an obstruction layer to account for spacing
    // and route width to avoid creating a DRC error with a route. 

    for (i = 0; i < Num_layers; i++) {
	delta[i] = LefGetRouteKeepout(i);
    }

    // Give a single net number to all obstructions, over the range of the
    // number of known nets, so these positions cannot be routed through.
    // If a grid position is not wholly inside an obstruction, then we
    // maintain the direction of the nearest obstruction in Obs and the
    // distance to it in Obsinfo.  This indicates that a route can avoid
    // the obstruction by moving away from it by the amount in Obsinfo
    // plus spacing clearance.  If another obstruction is found that
    // prevents such a move, then all direction flags will be set, indicating
    // that the position is not routable under any condition. 

    for (g = Nlgates; g; g = g->next) {
       for (ds = g->obs; ds; ds = ds->next) {

	  gridx = (int)((ds->x1 - Xlowerbound - delta[ds->layer])
			/ PitchX[ds->layer]) - 1;
	  while (1) {
	     dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
	     if (dx > (ds->x2 + delta[ds->layer])
			|| gridx >= NumChannelsX[ds->layer]) break;
	     else if (dx >= (ds->x1 - delta[ds->layer]) && gridx >= 0) {
	        gridy = (int)((ds->y1 - Ylowerbound - delta[ds->layer])
			/ PitchY[ds->layer]) - 1;
	        while (1) {
		   dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
	           if (dy > (ds->y2 + delta[ds->layer])
				|| gridy >= NumChannelsY[ds->layer]) break;
		   if (dy >= (ds->y1 - delta[ds->layer]) && gridy >= 0)
		      check_obstruct(gridx, gridy, ds, dx, dy);

		   gridy++;
		}
	     }
	     gridx++;
	  }
       }

       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] == 0) {	/* Unconnected node */
	     // Diagnostic, and power bus handling
	     no_net = NO_NET;
	     if (g->node[i]) {
		if (!strncasecmp(g->node[i], "vdd", 3))
		   no_net |= OBS_VDD;
		else if (!strncasecmp(g->node[i], "gnd", 3))
		   no_net |= OBS_VSS;
		else if (!strncasecmp(g->node[i], "vss", 3))
		   no_net |= OBS_VSS;
		else
		   fprintf(stdout, "Gate instance %s unconnected node %s\n",
				g->gatename, g->node[i]);
	     }
	     else
	        fprintf(stdout, "Gate instance %s unconnected node (%d)\n",
			g->gatename, i);
             for (ds = g->taps[i]; ds; ds = ds->next) {

		gridx = (int)((ds->x1 - Xlowerbound - delta[ds->layer])
			/ PitchX[ds->layer]) - 1;
		while (1) {
		   dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		   if (dx > (ds->x2 + delta[ds->layer])
				|| gridx >= NumChannelsX[ds->layer]) break;
		   else if (dx >= (ds->x1 - delta[ds->layer]) && gridx >= 0) {
		      gridy = (int)((ds->y1 - Ylowerbound - delta[ds->layer])
				/ PitchY[ds->layer]) - 1;
		      while (1) {
		         dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		         if (dy > (ds->y2 + delta[ds->layer])
					|| gridy >= NumChannelsY[ds->layer]) break;
		         if (dy >= (ds->y1 - delta[ds->layer]) && gridy >= 0)
			    check_obstruct(gridx, gridy, ds, dx, dy);

		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }
	  }
       }
    }

    // Create additional obstructions from the UserObs list
    // These obstructions are not considered to be metal layers,
    // so we don't compute a distance measure.  However, we need
    // to compute a boundary of 1/2 route width to avoid having
    // the route overlapping the obstruction area.

    for (i = 0; i < Num_layers; i++) {
	delta[i] = LefGetRouteWidth(i) / 2.0;
    }

    for (ds = UserObs; ds; ds = ds->next) {
	gridx = (int)((ds->x1 - Xlowerbound - delta[ds->layer])
			/ PitchX[ds->layer]) - 1;
	while (1) {
	    dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
	    if (dx > (ds->x2 + delta[ds->layer])
			|| gridx >= NumChannelsX[ds->layer]) break;
	    else if (dx >= (ds->x1 - delta[ds->layer]) && gridx >= 0) {
		gridy = (int)((ds->y1 - Ylowerbound - delta[ds->layer])
				/ PitchY[ds->layer]) - 1;
		while (1) {
		    dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		    if (dy > (ds->y2 + delta[ds->layer])
				|| gridy >= NumChannelsY[ds->layer]) break;
		    if (dy >= (ds->y1 - delta[ds->layer]) && gridy >= 0) {
		        check_obstruct(gridx, gridy, ds, dx, dy);
		    }
		    gridy++;
		}
	    }
	    gridx++;
	}
    }
}

/*--------------------------------------------------------------*/
/* create_obstructions_from_nodes()				*/
/*								*/
/*  Fills in the Obs[][] grid from the position of each node	*/
/*  (net terminal), which may have multiple unconnected		*/
/*  positions.							*/
/*								*/
/*  Also fills in the Nodeloc[] grid with the node number,	*/
/*  which causes the router to put a premium on			*/
/*  routing other nets over or under this position, to		*/
/*  discourage boxing in a pin position and making it 		*/
/*  unroutable.							*/
/*								*/
/*  ARGS: none.							*/
/*  RETURNS: nothing						*/
/*  SIDE EFFECTS: none						*/
/*  AUTHOR:  Tim Edwards, June 2011, based on code by Steve	*/
/*	Beccue.							*/
/*--------------------------------------------------------------*/

void create_obstructions_from_nodes()
{
    NODE node, n2;
    GATE g;
    DPOINT dp;
    DSEG ds;
    u_int dir, k;
    int i, gx, gy, gridx, gridy, net;
    double dx, dy, delta[MAX_LAYERS];
    float dist, xdist;

    for (i = 0; i < Num_layers; i++) {
	delta[i] = LefGetRouteKeepout(i);
    }

    // For each node terminal (gate pin), mark each grid position with the
    // net number.  This overrides any obstruction that may be placed at that
    // point.

    // For each pin position, we also find the "keepout" area around the
    // pin where we may not place an unrelated route.  For this, we use a
    // flag bit, so that the position can be ignored when routing the net
    // associated with the pin.  Normal obstructions take precedence.

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] != 0) {

	     // Get the node record associated with this pin.
	     node = g->noderec[i];

	     // First mark all areas inside node geometry boundary.

             for (ds = g->taps[i]; ds; ds = ds->next) {
		gridx = (int)((ds->x1 - Xlowerbound) / PitchX[ds->layer]) - 1;
		while (1) {
		   dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		   if (dx > ds->x2 || gridx >= NumChannelsX[ds->layer]) break;
		   else if (dx >= ds->x1 && gridx >= 0) {
		      gridy = (int)((ds->y1 - Ylowerbound) / PitchY[ds->layer]) - 1;
		      while (1) {
		         dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		         if (dy > ds->y2 || gridy >= NumChannelsY[ds->layer]) break;

			 // Area inside defined pin geometry

			 if (dy > ds->y1 && gridy >= 0) {
			     int orignet = Obs[ds->layer][OGRID(gridx,
					gridy, ds->layer)];

			     if ((orignet & ~PINOBSTRUCTMASK) == (u_int)node->netnum) {

				// Duplicate tap point.   Don't re-process it. (?)
				gridy++;
				continue;
			     }

			     if ((orignet & ~PINOBSTRUCTMASK) != (u_int)0) {
				// Net was assigned to other net, but is inside
				// this pin's geometry.  Declare point to be
				// unroutable, as it is too close to both pins.
				// NOTE:  This is a conservative rule and could
				// potentially make a pin unroutable.
				// Another note:  By setting Obs[] to
				// OBSTRUCT_MASK as well as NO_NET, we ensure
				// that it falls through on all subsequent
				// processing.

			        Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= (u_int)(NO_NET | OBSTRUCT_MASK);
			        Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= NULL;
			        Nodesav[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= NULL;
			        Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= 0.0;
			     }
			     else if (!(orignet & NO_NET)) {

				// A grid point that is within 1/2 route width
				// of a tap rectangle corner can violate metal
				// width rules, and so should declare a stub.
				
				dir = 0;
				dist = 0.0;
			        xdist = 0.5 * LefGetRouteWidth(ds->layer);

				if (dx >= ds->x2 - xdist) {
				   if (dy >= ds->y2 - xdist) {
				      // Check northeast corner

				      if ((ds->x2 - dx) > (ds->y2 - dy)) {
					 // West-pointing stub
					 dir = STUBROUTE_EW;
					 dist = ds->x2 - dx - 2.0 * xdist;
				      }
				      else {
					 // South-pointing stub
					 dir = STUBROUTE_NS;
					 dist = ds->y2 - dy - 2.0 * xdist;
				      }

				   }
				   else if (dy <= ds->y1 + xdist) {
				      // Check southeast corner

				      if ((ds->x2 - dx) > (dy - ds->y1)) {
					 // West-pointing stub
					 dir = STUBROUTE_EW;
					 dist = ds->x2 - dx - 2.0 * xdist;
				      }
				      else {
					 // North-pointing stub
					 dir = STUBROUTE_NS;
					 dist = ds->y1 - dy + 2.0 * xdist;
				      }
				   }
				}
				else if (dx <= ds->x1 + xdist) {
				   if (dy >= ds->y2 - xdist) {
				      // Check northwest corner

				      if ((dx - ds->x1) > (ds->y2 - dy)) {
					 // East-pointing stub
					 dir = STUBROUTE_EW;
					 dist = ds->x1 - dx + 2.0 * xdist;
				      }
				      else {
					 // South-pointing stub
					 dir = STUBROUTE_NS;
					 dist = ds->y2 - dy - 2.0 * xdist;
				      }

				   }
				   else if (dy <= ds->y1 + xdist) {
				      // Check southwest corner

				      if ((dx - ds->x2) > (dy - ds->y1)) {
					 // East-pointing stub
					 dir = STUBROUTE_EW;
					 dist = ds->x1 - dx + 2.0 * xdist;
				      }
				      else {
					 // North-pointing stub
					 dir = STUBROUTE_NS;
					 dist = ds->y1 - dy + 2.0 * xdist;
				      }
				   }
				}

			        Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= (u_int)node->netnum | dir;
			        Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
			        Nodesav[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
			        Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= dist;

			     }
			     else if ((orignet & NO_NET) && ((orignet & OBSTRUCT_MASK)
					!= OBSTRUCT_MASK)) {
				double sdist = LefGetRouteKeepout(ds->layer);

				// If a cell is positioned off-grid, then a grid
				// point may be inside a pin and still be unroutable.
				// The Obsinfo[] array tells where an obstruction is,
				// if there was only one obstruction in one direction
				// blocking the grid point.  If so, then we set the
				// Stub[] distance to move the tap away from the
				// obstruction to resolve the DRC error.

				if (orignet & OBSTRUCT_N) {
			           Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= -(sdist - Obsinfo[ds->layer]
					[OGRID(gridx, gridy, ds->layer)]);
			           Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					|= (STUBROUTE_NS | OFFSET_TAP);
				}
				else if (orignet & OBSTRUCT_S) {
			           Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= sdist - Obsinfo[ds->layer]
					[OGRID(gridx, gridy, ds->layer)];
			           Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					|= (STUBROUTE_NS | OFFSET_TAP);
				}
				else if (orignet & OBSTRUCT_E) {
			           Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= -(sdist - Obsinfo[ds->layer]
					[OGRID(gridx, gridy, ds->layer)]);
			           Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					|= (STUBROUTE_EW | OFFSET_TAP);
				}
				else if (orignet & OBSTRUCT_W) {
			           Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= sdist - Obsinfo[ds->layer]
					[OGRID(gridx, gridy, ds->layer)];
			           Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					|= (STUBROUTE_EW | OFFSET_TAP);
				}

				// Diagnostic, to be removed
				fprintf(stderr, "Port overlaps obstruction at"
					" grid %d %d, position %g %g\n",
					gridx, gridy, dx, dy);
			     }
			 }

			 // Check that we have not created a PINOBSTRUCT
			 // route directly over this point.
			 if (ds->layer < Num_layers - 1) {
			      k = Obs[ds->layer + 1][OGRID(gridx, gridy,
					ds->layer + 1)];
			      if (k & PINOBSTRUCTMASK) {
			         if ((k & ~PINOBSTRUCTMASK) != (u_int)node->netnum) {
				    Obs[ds->layer + 1][OGRID(gridx, gridy,
						ds->layer + 1)] = 0;
				    Nodeloc[ds->layer + 1][OGRID(gridx, gridy,
						ds->layer + 1)] = (NODE)NULL;
				    Nodesav[ds->layer + 1][OGRID(gridx, gridy,
						ds->layer + 1)] = (NODE)NULL;
				    Stub[ds->layer + 1][OGRID(gridx, gridy,
						ds->layer + 1)] = (float)0.0;
				 }
			      }
			 }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }

	     // Repeat this whole exercise for areas in the halo outside
	     // the node geometry.  We have to do this after enumerating
	     // all inside areas because the tap rectangles often overlap,
	     // and one rectangle's halo may be inside another tap.

             for (ds = g->taps[i]; ds; ds = ds->next) {
		gridx = (int)((ds->x1 - Xlowerbound - delta[ds->layer])
			/ PitchX[ds->layer]) - 1;
		while (1) {
		   dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		   if (dx > (ds->x2 + delta[ds->layer]) ||
				gridx >= NumChannelsX[ds->layer]) break;
		   else if (dx >= (ds->x1 - delta[ds->layer]) && gridx >= 0) {
		      gridy = (int)((ds->y1 - Ylowerbound - delta[ds->layer])
				/ PitchY[ds->layer]) - 1;
		      while (1) {
		         dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		         if (dy > (ds->y2 + delta[ds->layer]) ||
				gridy >= NumChannelsY[ds->layer]) break;
		         if (dy >= (ds->y1 - delta[ds->layer]) && gridy >= 0) {
			    xdist = 0.5 * LefGetRouteWidth(ds->layer);

			    // Area inside halo around defined pin geometry.
			    // Exclude areas already processed (areas inside
			    // some pin geometry have been marked with netnum)

			    // Also check that we are not about to define a
			    // route position for a pin on a layer above 0 that
			    // blocks a pin underneath it.

			    n2 = NULL;
			    if (ds->layer > 0)
			       n2 = Nodeloc[ds->layer - 1][OGRID(gridx, gridy,
					ds->layer - 1)];
			    if (n2 == NULL)
			       n2 = Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)];

			    k = Obs[ds->layer][OGRID(gridx, gridy, ds->layer)];

			    // In case of a port that is inaccessible from a grid
			    // point, or not completely overlapping it, the
			    // stub information will show how to adjust the
			    // route position to cleanly attach to the port.

			    dir = STUBROUTE_X;
			    dist = 0.0;

			    if (((k & ~PINOBSTRUCTMASK) != (u_int)node->netnum) &&
					(n2 == NULL)) {

				if ((k & OBSTRUCT_MASK) != 0) {
				   float sdist = Obsinfo[ds->layer][OGRID(gridx,
						gridy, ds->layer)];

				   // If the point is marked as close to an
				   // obstruction, we can declare this an
				   // offset tap if we are not on a corner.
				   // Because we cannot define both an offset
				   // and a stub simultaneously, if the distance
				   // to clear the obstruction does not make the
				   // route reach the tap, then we mark the grid
				   // position as unroutable.

				   if (dy >= (ds->y1 - xdist) &&
						dy <= (ds->y2 + xdist)) {
				      if ((dx >= ds->x2) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_E)) {
				         dist = sdist - LefGetRouteKeepout(ds->layer);
					 if ((dx - ds->x2 + dist) < xdist)
				 	    dir = STUBROUTE_EW | OFFSET_TAP;
				      }
				      else if ((dx <= ds->x1) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_W)) {
				         dist = LefGetRouteKeepout(ds->layer) - sdist;
					 if ((ds->x1 - dx - dist) < xdist)
				            dir = STUBROUTE_EW | OFFSET_TAP;
				      }
			 	   }	
				   if (dx >= (ds->x1 - xdist) &&
						dx <= (ds->x2 + xdist)) {
				      if ((dy >= ds->y2) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_N)) {
				         dist = sdist - LefGetRouteKeepout(ds->layer);
					 if ((dy - ds->y2 + dist) < xdist)
				            dir = STUBROUTE_NS | OFFSET_TAP;
				      }
				      else if ((dy <= ds->y1) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_S)) {
				         dist = LefGetRouteKeepout(ds->layer) - sdist;
					 if ((ds->y1 - dy - dist) < xdist)
				            dir = STUBROUTE_NS | OFFSET_TAP;
				      }
				   }
				   // Otherwise, dir is left as STUBROUTE_X
				}
				else {

				   // Cleanly unobstructed area.  Define stub
				   // route from point to tap, with a route width
				   // overlap if necessary to avoid a DRC width
				   // violation.

				   if ((dx >= ds->x2) &&
					((dx - ds->x2) > (dy - ds->y2)) &&
					((dx - ds->x2) > (ds->y1 - dy))) {
				      // West-pointing stub
				      if ((dy - ds->y2) <= xdist &&
					  (ds->y1 - dy) <= xdist) {
					 // Within reach of tap rectangle
					 dir = STUBROUTE_EW;
					 dist = ds->x2 - dx;
					 if (dy < (ds->y2 - xdist) &&
						dy > (ds->y1 + xdist)) {
					    if (dx < ds->x2 + xdist) dist = 0.0;
					 }
					 else {
					    dist -= 2.0 * xdist;
					 }
				      }
				   }
				   else if ((dx <= ds->x1) &&
					((ds->x1 - dx) > (dy - ds->y2)) &&
					((ds->x1 - dx) > (ds->y1 - dy))) {
				      // East-pointing stub
				      if ((dy - ds->y2) <= xdist &&
					  (ds->y1 - dy) <= xdist) {
					 // Within reach of tap rectangle
					 dir = STUBROUTE_EW;
					 dist = ds->x1 - dx;
					 if (dy < (ds->y2 - xdist) &&
						dy > (ds->y1 + xdist)) {
					    if (dx > ds->x1 - xdist) dist = 0.0;
					 }
					 else {
					    dist += 2.0 * xdist;
					 }
				      }
				   }
				   else if ((dy >= ds->y2) &&
					((dy - ds->y2) > (dx - ds->x2)) &&
					((dy - ds->y2) > (ds->x1 - dx))) {
				      // South-pointing stub
				      if ((dx - ds->x2) <= xdist &&
					  (ds->x1 - dx) <= xdist) {
					 // Within reach of tap rectangle
					 dir = STUBROUTE_NS;
					 dist = ds->y2 - dy;
					 if (dx < (ds->x2 - xdist) &&
						dx > (ds->x1 + xdist)) {
					    if (dy < ds->y2 + xdist) dist = 0.0;
					 }
					 else {
					    dist -= 2.0 * xdist;
					 }
				      }
				   }
				   else if ((dy <= ds->y1) &&
					((ds->y1 - dy) > (dx - ds->x2)) &&
					((ds->y1 - dy) > (ds->x1 - dx))) {
				      // North-pointing stub
				      if ((dx - ds->x2) <= xdist &&
					  (ds->x1 - dx) <= xdist) {
					 // Within reach of tap rectangle
					 dir = STUBROUTE_NS;
					 dist = ds->y1 - dy;
					 if (dx < (ds->x2 - xdist) &&
						dx > (ds->x1 + xdist)) {
					    if (dy > ds->y1 - xdist) dist = 0.0;
					 }
					 else {
					    dist += 2.0 * xdist;
					 }
				      }
				   }

				   if (dir == STUBROUTE_X) {

				      // Outside of pin at a corner.  First, if one
				      // direction is too far away to connect to a
				      // pin, then we must route the other direction.

				      if (dx < ds->x1 - xdist || dx > ds->x2 + xdist) {
				         if (dy >= ds->y1 - xdist &&
							dy <= ds->y2 + xdist) {
				            dir = STUBROUTE_EW;
				            dist = (float)(((ds->x1 + ds->x2) / 2.0)
							- dx);
					 }
				      }
				      else if (dy < ds->y1 - xdist ||
							dy > ds->y2 + xdist) {
				         dir = STUBROUTE_NS;
				         dist = (float)(((ds->y1 + ds->y2) / 2.0) - dy);
				      }

				      // Otherwise we are too far away at a diagonal
				      // to reach the pin by moving in any single
				      // direction.  To be pedantic, we could define
				      // some jogged stub, but for now, we just call
				      // the point unroutable (leave dir = STUBROUTE_X)
				   }
				}

				// Stub distances of <= 1/2 route width are useless
				if (dir == STUBROUTE_NS || dir == STUBROUTE_EW)
				   if (fabs(dist) < (xdist + 1e-4)) {
				      dir = 0;
				      dist = 0.0;
				   }

				if ((k < Numnets) && (dir != STUBROUTE_X)) {
				   Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= (u_int)g->netnum[i] | dir; 
				   Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
				   Nodesav[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
				}
				else {
				   // Keep showing an obstruction, but add the
				   // direction info and log the stub distance.
				   Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					|= dir;
				}
				Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= dist;
			    }
			    else {
			       int othernet = (k & ~PINOBSTRUCTMASK);
			       if (othernet != 0 && othernet != (u_int)node->netnum) {

			          // This location is too close to two different
				  // node terminals and should not be used
				  // (NOTE:  To be thorough, we should check
				  // if othernet could be routed using a tap
				  // offset.  By axing it we might lose the
				  // only route point to one of the pins.)

				  Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= NO_NET;
				  Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= (NODE)NULL;
				  Nodesav[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= (NODE)NULL;
				  Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= (float)0.0;
			       }
			    }
		         }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }
	  }
       }
    }

} /* void create_obstructions_from_nodes( void ) */

/*--------------------------------------------------------------*/
/* adjust_stub_lengths()					*/
/*								*/
/*  Makes an additional pass through the tap and obstruction	*/
/*  databases, checking geometry against the potential stub	*/
/*  routes for DRC spacing violations.  Adjust stub routes as	*/
/*  necessary to resolve the DRC error(s).			*/
/*								*/
/*  ARGS: none.							*/
/*  RETURNS: nothing						*/
/*  SIDE EFFECTS: none						*/
/*  AUTHOR:  Tim Edwards, April 2013				*/
/*--------------------------------------------------------------*/

void adjust_stub_lengths()
{
    NODE node, n2;
    GATE g;
    DPOINT dp;
    DSEG ds, ds2;
    struct dseg_ dt, de;
    u_int dir, k;
    int i, gx, gy, gridx, gridy, net, orignet;
    double dx, dy, w, s;
    float dist;
    u_char errbox;

    // For each node terminal (gate pin), look at the surrounding grid points.
    // If any define a stub route or an offset, check if the stub geometry
    // or offset geometry would create a DRC spacing violation.  If so, adjust
    // the stub route to resolve the error.  If the error cannot be resolved,
    // mark the position as unroutable.  If it is the ONLY grid point accessible
    // to the pin, keep it as-is and flag a warning.

    // Unlike blockage-finding routines, which look in an area of a size equal
    // to the DRC interaction distance around a tap rectangle, this routine looks
    // out one grid pitch in each direction, to catch information about stubs that
    // may terminate within a DRC interaction distance of the tap rectangle.

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] != 0) {

	     // Get the node record associated with this pin.
	     node = g->noderec[i];

	     // Work through each rectangle in the tap geometry

             for (ds = g->taps[i]; ds; ds = ds->next) {
		w = 0.5 * LefGetRouteWidth(ds->layer);
		s = LefGetRouteSpacing(ds->layer);
		gridx = (int)((ds->x1 - Xlowerbound - PitchX[ds->layer])
			/ PitchX[ds->layer]) - 1;
		while (1) {
		   dx = (gridx * PitchX[ds->layer]) + Xlowerbound;
		   if (dx > (ds->x2 + PitchX[ds->layer]) ||
				gridx >= NumChannelsX[ds->layer]) break;
		   else if (dx >= (ds->x1 - PitchX[ds->layer]) && gridx >= 0) {
		      gridy = (int)((ds->y1 - Ylowerbound - PitchY[ds->layer])
				/ PitchY[ds->layer]) - 1;
		      while (1) {
		         dy = (gridy * PitchY[ds->layer]) + Ylowerbound;
		         if (dy > (ds->y2 + PitchY[ds->layer]) ||
				gridy >= NumChannelsY[ds->layer]) break;
		         if (dy >= (ds->y1 - PitchY[ds->layer]) && gridy >= 0) {

			     orignet = Obs[ds->layer][OGRID(gridx, gridy, ds->layer)];

			     // Ignore this location if it is assigned to another
			     // net, or is assigned to NO_NET.

			     if ((orignet & ~PINOBSTRUCTMASK) != node->netnum) {
				gridy++;
				continue;
			     }

			     // STUBROUTE_X are unroutable;  leave them alone
			     if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_X) {
				gridy++;
				continue;
			     }

			     // define a route box around the grid point

			     errbox = FALSE;
			     dt.x1 = dx - w;
			     dt.x2 = dx + w;
			     dt.y1 = dy - w;
			     dt.y2 = dy + w;

			     dist = Stub[ds->layer][OGRID(gridx, gridy, ds->layer)];

			     // adjust the route box according to the stub
			     // or offset geometry

			     if (orignet & OFFSET_TAP) {
				if (orignet & STUBROUTE_EW) {
				   dt.x1 += dist;
				   dt.x2 += dist;
				}
				else if (orignet & STUBROUTE_NS) {
				   dt.y1 += dist;
				   dt.y2 += dist;
				}
			     }
			     else if (orignet & PINOBSTRUCTMASK) {
				if (orignet & STUBROUTE_EW) {
				   if (dist > 0)
				      dt.x2 = dx + dist;
				   else
				      dt.x1 = dx + dist;
				}
				else if (orignet & STUBROUTE_NS) {
				   if (dist > 0)
				      dt.y2 = dy + dist;
				   else
				      dt.y1 = dy + dist;
				}
			     }
			     de = dt;

			     // check for DRC spacing interactions between
			     // the tap box and the route box

			     if ((dt.y1 - ds->y2) > 0 && (dt.y1 - ds->y2) < s) {
				if (ds->x2 > (dt.x1 - s) && ds->x1 < (dt.x2 + s)) {
				   de.y2 = dt.y1;
				   de.y1 = ds->y2;
				   errbox = TRUE;
				}
			     }
			     else if ((ds->y1 - dt.y2) > 0 && (ds->y1 - dt.y2) < s) {
				if (ds->x2 > (dt.x1 - s) && ds->x1 < (dt.x2 + s)) {
				   de.y1 = dt.y2;
				   de.y2 = ds->y1;
				   errbox = TRUE;
				}
			     }

			     if ((dt.x1 - ds->x2) > 0 && (dt.x1 - ds->x2) < s) {
				if (ds->y2 > (dt.y1 - s) && ds->y1 < (dt.y2 + s)) {
				   de.x2 = dt.x1;
				   de.x1 = ds->x2;
				   errbox = TRUE;
				}
			     }
			     else if ((ds->x1 - dt.x2) > 0 && (ds->x1 - dt.x2) < s) {
				if (ds->y2 > (dt.y1 - s) && ds->y1 < (dt.y2 + s)) {
				   de.x1 = dt.x2;
				   de.x2 = ds->x1;
				   errbox = TRUE;
				}
			     }
	
			     if (errbox == TRUE) {
			        // Chop areas off the error box that are covered by
			        // other taps of the same port.

			        for (ds2 = g->taps[i]; ds2; ds2 = ds2->next) {
				   if (ds2 == ds) continue;

				   if (ds2->x1 <= de.x1 && ds2->x2 >= de.x2 &&
					ds2->y1 <= de.y1 && ds2->y2 >= de.y2) {
				      errbox = FALSE;	// Completely covered
				      break;
				   }

				   if (ds2->x1 < de.x2 && ds2->x2 > de.x1) {
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1) {
					 // Partial coverage
					 if (ds2->x1 < de.x1 && ds2->x2 < de.x2)
					    de.x1 = ds2->x2;
					 if (ds2->x2 > de.x2 && ds2->x1 > de.x1)
					    de.x2 = ds2->x1;
					 if (ds2->y1 < de.y1 && ds2->y2 < de.y2)
					    de.y1 = ds2->y2;
					 if (ds2->y2 > de.y2 && ds2->y1 > de.y1)
					    de.y2 = ds2->y1;
				      }
				   }
				}
			     }

			     // Any area left over is a potential DRC error.

			     if ((de.x2 <= de.x1) || (de.y2 <= de.y1))
				errbox = FALSE;
		
			     if (errbox == TRUE) {

				// Create stub route to cover error box, or
				// if possible, stretch existing stub route
				// to cover error box.

				// Allow EW stubs to be changed to NS stubs and
				// vice versa if the original stub length was less
				// than a route width.  This means the grid position
				// makes contact without the stub.  Moving the stub
				// to another side should not create an error.

				// NOTE:  Changed 4/29/13;  direction of stub will
				// be changed even though it might create an error
				// in the other direction;  it can't do worse.
				// But, the case should be re-run to check (to-do)

				if (de.x2 > dt.x2) {
				   if ((orignet & PINOBSTRUCTMASK) == 0) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_EW;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x2 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_EW
						&& (dist > 0)) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x2 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_NS
						// && (fabs(dist) < (2 * w))
						) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						&= ~STUBROUTE_NS;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_EW;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x2 - dx;
				      errbox = FALSE;
				   }
				}
				else if (de.x1 < dt.x1) {
				   if ((orignet & PINOBSTRUCTMASK) == 0) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_EW;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x1 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_EW
						&& (dist < 0)) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x1 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_NS
						// && (fabs(dist) < (2 * w))
						) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						&= ~STUBROUTE_NS;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_EW;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.x1 - dx;
				      errbox = FALSE;
				   }
				}
				else if (de.y2 > dt.y2) {
				   if ((orignet & PINOBSTRUCTMASK) == 0) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_NS;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y2 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_NS
						&& (dist > 0)) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y2 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_EW
						// && (fabs(dist) < (2 * w))
						) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						&= ~STUBROUTE_EW;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_NS;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y2 - dy;
				      errbox = FALSE;
				   }
				}
				else if (de.y1 < dt.y1) {
				   if ((orignet & PINOBSTRUCTMASK) == 0) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_NS;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y1 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_NS
						&& (dist < 0)) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y1 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & PINOBSTRUCTMASK) == STUBROUTE_EW
						// && (fabs(dist) < (2 * w))
						) {
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						&= ~STUBROUTE_EW;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= STUBROUTE_NS;
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= de.y1 - dy;
				      errbox = FALSE;
				   }
				}

				if (errbox == TRUE) {
				   fprintf(stderr, "DRC error potential in route tap\n");
				}
			     }
		         }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }
	  }
       }
    }

} /* void adjust_stub_lengths() */

/* node.c */
