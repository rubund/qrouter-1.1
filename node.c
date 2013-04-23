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
   u_int no_net;

   no_net = (u_int)Numnets + 1;

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
	       Obs[l][OGRID(x, y, l)] = no_net;
	    }
	 }
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
    int i, gridx, gridy;
    double dx, dy, delta[MAX_LAYERS];
    u_int no_net;

    // Get, and store, information for each layer on how much we
    // need to expand an obstruction layer to account for spacing
    // and route width to avoid creating a DRC error with a route. 

    for (i = 0; i < Num_layers; i++) {
	delta[i] = LefGetRouteKeepout(i);
    }

    no_net = (u_int)Numnets + 1;

    // Give a single net number to all obstructions, over the range of the
    // number of known nets, so these positions cannot be routed through.

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
		   if (dy >= (ds->y1 - delta[ds->layer]) && gridy >= 0) {
		      Obs[ds->layer][OGRID(gridx, gridy, ds->layer)] = no_net;
		   }
		   gridy++;
		}
	     }
	     gridx++;
	  }
       }

       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] == 0) {	/* Unconnected node */
	     // Diagnostic
	     if (g->node[i]) {
		if (strncasecmp(g->node[i], "vdd", 3)
			&& strncasecmp(g->node[i], "gnd", 3)
			&& strncasecmp(g->node[i], "vss", 3))
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
		         if (dy >= (ds->y1 - delta[ds->layer]) && gridy >= 0) {
			    Obs[ds->layer][OGRID(gridx, gridy, ds->layer)] = no_net;
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
			Obs[ds->layer][OGRID(gridx, gridy, ds->layer)] = no_net;
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
    u_int dir, k, no_net;
    int i, gx, gy, gridx, gridy, net;
    double dx, dy, delta[MAX_LAYERS];
    float dist;

    no_net = (u_int)Numnets + 1;

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

	     // First mark all areas inside node geometry.

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

			     // Don't reprocess this node in case of
			     // duplicate tap points.

			     if ((orignet & ~PINOBSTRUCTMASK) == (u_int)node->netnum) {
				gridy++;
				continue;
			     }

			     Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= (u_int)node->netnum;
			     Nodeloc[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
			     Nodesav[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= node;
			     Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= 0.0;

			     if (orignet > Numnets) {
				double sdist = 0.5 * LefGetRouteWidth(ds->layer);

				// If a cell is positioned off-grid, then a grid
				// point may be inside a pin and still be unroutable.
				// For DRC-clean standard cells, this can only happen
				// if the grid point is < 1/2 route width away from a
				// pin edge.  So find which pin edge is too close,
				// and force an offset.

				if (dy - ds->y1 < sdist) {
				   if (ds->y2 - dy >= sdist) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= ds->y1 + sdist - dy;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= (STUBROUTE_NS | OFFSET_TAP);
				   }
				   else {
				      // Pin is narrow and we have no reliable
				      // place to put a route.
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= orignet;
				   }
				}
				else if (ds->y2 - dy < sdist) {
			           Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= ds->y2 - sdist - dy;
			           Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					|= (STUBROUTE_NS | OFFSET_TAP);
				}

				/* At least for now, there is no way to specify
				 * separate offsets in the X and Y directions.
				 * So if we have defined a NS offset, we cannot
				 * set an EW offset, since the offset distances
				 * may be different.
				 */

				if ((Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					& STUBROUTE_NS) == 0) {

				   if (dx - ds->x1 < sdist) {
				      if (ds->x2 - dx >= sdist) {
			                 Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
						= ds->x1 + sdist - dx;
			                 Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
						|= (STUBROUTE_EW | OFFSET_TAP);
				      }
				      else {
				         // Pin is narrow and we have no reliable
				         // place to put a route.
			                 Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					   	= orignet;
				      }
				   }
				   else if (ds->x2 - dx < sdist) {
			              Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					   = ds->x2 - sdist - dx;
			              Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					   |= (STUBROUTE_EW | OFFSET_TAP);
				   }
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

			    // Define stub routes over obstructions.  In case of
			    // a port that is inaccessible from a grid point, the
			    // stub information will show how to adjust the
			    // route position to cleanly attach to the port.

			    if (((k & ~PINOBSTRUCTMASK) != (u_int)node->netnum) &&
					(n2 == NULL)) {
				dir = STUBROUTE_X;
				if (dy >= ds->y1 && dy <= ds->y2) {
				   dir = STUBROUTE_EW;
				   dist = (float)(((ds->x1 + ds->x2) / 2.0) - dx);
				}
				if (dx >= ds->x1 && dx <= ds->x2) {
				   dir = STUBROUTE_NS;
				   dist = (float)(((ds->y1 + ds->y2) / 2.0) - dy);
				}

				// Limit stub distance so that we do not route
				// from the edge to the center of a long, narrow
				// tap.  It might make more sense just to
				// calculate the distance to the tap's near edge.

				if (dist > delta[ds->layer])
				    dist = delta[ds->layer];
				else if (dist < -delta[ds->layer])
				    dist = -delta[ds->layer];

				if (k < Numnets) {
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
					|= (dir | OFFSET_TAP); 
				}
				Stub[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= dist;
			    }
			    else if (k & PINOBSTRUCTMASK) {
			       if ((k & ~PINOBSTRUCTMASK) != (u_int)node->netnum) {

			          // This location is too close to two different
				  // node terminals and should not be used

				  Obs[ds->layer][OGRID(gridx, gridy, ds->layer)]
					= no_net;
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

/* node.c */
